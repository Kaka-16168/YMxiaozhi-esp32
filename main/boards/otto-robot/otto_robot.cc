#include <driver/i2c_master.h>
#include <driver/ledc.h>
#include <driver/spi_common.h>
#include <esp_lcd_panel_io.h>
#include <esp_lcd_panel_ops.h>
#include <esp_lcd_panel_vendor.h>
#include <esp_log.h>
#include <esp_rom_sys.h>
#include <esp_timer.h>
#include <sys/time.h>
#include <time.h>
#include <string>

#include <new>

#include "application.h"
#include "button.h"
#include "codecs/no_audio_codec.h"
#include "config.h"
#include "display/lcd_display.h"
#include "esp32_camera.h"
#include "lamp_controller.h"
#include "led/single_led.h"
#include "mcp_server.h"
#include "otto_emoji_display.h"
#include "power_manager.h"
#include "system_reset.h"
#include "websocket_control_server.h"
#include "wifi_board.h"
#include "assets/lang_config.h"

#define TAG "OttoRobot"

extern void InitializeOttoController(const HardwareConfig& hw_config);

// ============================================================
// 与 otto_controller.cc 之间的接口（C 链接，符号不受 name mangling 影响）
// ============================================================
extern "C" {
// 开机：把 6 个舵机逐个、错开时间拉回中位，避免同时上电冲击电流把板子拖重启
void OttoController_BootHomeSequence(void);
// 温柔招手互动（无手部舵机时自动退化为身体轻摇，不会报错）
void OttoController_GentleWave(void);
// 软关机：舵机归位后彻底下电（保持电流归零）
void OttoController_Sleep(void);
// 唤醒：舵机重新上电并慢速归位
void OttoController_Wakeup(void);
// 当前是否有动作正在执行
bool OttoController_IsBusy(void);
// 清空动作队列（闹钟打断用）
void OttoController_ClearQueue(void);
}

class OttoRobot : public WifiBoard {
private:
    LcdDisplay* display_;
    PowerManager* power_manager_;
    Button boot_button_;
    WebSocketControlServer* ws_control_server_;
    HardwareConfig hw_config_;
    AudioCodec* audio_codec_;
    i2c_master_bus_handle_t i2c_bus_;
    Camera* camera_;
    bool is_camera_board_;
    bool has_camera_;
    OttoCameraType camera_type_;

    // ======== 闹钟相关 ========
    esp_timer_handle_t alarm_timer_ = nullptr;      // 到点定时器（单次）
    esp_timer_handle_t alarm_tick_timer_ = nullptr; // 播报节拍定时器（周期）
    std::string alarm_name_;
    int  alarm_round_ = 0;      // 已播报轮次
    bool alarm_active_ = false; // 闹钟是否处于播报中
    bool screen_off_ = false;   // 屏幕是否处于软关机黑屏态

    // ======== 开机阶段 ========
    // kBootEarly   构造期：只起屏幕，背光亮起就返回，不碰任何慢外设
    // kBootUiUp    首帧已上屏：补音频 / 摄像头 / 舵机控制器 / MCP 工具
    // kBootSettled 一切就绪：舵机才逐个归位
    enum BootPhase { kBootEarly, kBootUiUp, kBootSettled };
    BootPhase boot_phase_ = kBootEarly;
    int boot_wait_ms_ = 0;

    // OV2640 有两个可能的 PID，抽成小函数避免长条件写错
    static bool pid_is_ov2640(uint16_t pid) {
        return pid == OTTO_OV2640_PID_1 || pid == OTTO_OV2640_PID_2;
    }

    // ------------------------------------------------------------
    // 硬件识别
    // ------------------------------------------------------------
    bool DetectHardwareVersion() {
        constexpr gpio_num_t kDetectGpio15 = GPIO_NUM_15;
        constexpr gpio_num_t kDetectGpio16 = GPIO_NUM_16;
        constexpr int kStableSampleCount = 8;
        constexpr uint32_t kSettleTimeUs = 5000;
        constexpr uint32_t kSampleIntervalUs = 1000;

        gpio_config_t detect_config = {
            .pin_bit_mask = (1ULL << kDetectGpio15) | (1ULL << kDetectGpio16),
            .mode = GPIO_MODE_INPUT,
            .pull_up_en = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_ENABLE,
            .intr_type = GPIO_INTR_DISABLE,
        };
        esp_err_t ret = gpio_config(&detect_config);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "板型识别 GPIO 初始化失败: %s", esp_err_to_name(ret));
            return false;
        }

        esp_rom_delay_us(kSettleTimeUs);
        for (int sample = 0; sample < kStableSampleCount; ++sample) {
            int gpio15_level = gpio_get_level(kDetectGpio15);
            int gpio16_level = gpio_get_level(kDetectGpio16);
            if (gpio15_level == 0 || gpio16_level == 0) {
                ESP_LOGI(TAG, "板型识别: GPIO15=%d GPIO16=%d，判定为无摄像头版", gpio15_level,
                         gpio16_level);
                return false;
            }
            if (sample + 1 < kStableSampleCount) {
                esp_rom_delay_us(kSampleIntervalUs);
            }
        }

        ESP_LOGI(TAG, "板型识别: GPIO15/GPIO16 稳定为高，判定为摄像头版");
        return true;
    }

    bool DetectGc2145Camera(uint16_t& detected_pid) {
        i2c_device_config_t dev_cfg = {
            .dev_addr_length = I2C_ADDR_BIT_LEN_7,
            .device_address = OTTO_GC2145_SCCB_ADDR,
            .scl_speed_hz = 100000,
        };
        i2c_master_dev_handle_t dev_handle = nullptr;
        if (i2c_master_bus_add_device(i2c_bus_, &dev_cfg, &dev_handle) != ESP_OK) {
            return false;
        }

        // GC2145 在异常状态下可能不响应 SCCB，先软复位（0xfe bit7），再切到 page 0。
        uint8_t reset_cmd[2] = {0xFE, 0x80};
        if (i2c_master_transmit(dev_handle, reset_cmd, sizeof(reset_cmd), 200) == ESP_OK) {
            vTaskDelay(pdMS_TO_TICKS(100));
        }

        uint8_t page0_cmd[2] = {0xFE, 0x00};
        bool found = false;
        if (i2c_master_transmit(dev_handle, page0_cmd, sizeof(page0_cmd), 200) == ESP_OK) {
            vTaskDelay(pdMS_TO_TICKS(5));

            const uint8_t reg_high = 0xF0;
            const uint8_t reg_low = 0xF1;
            uint8_t pid_high = 0;
            uint8_t pid_low = 0;
            if (i2c_master_transmit_receive(dev_handle, &reg_high, 1, &pid_high, 1, 200) ==
                    ESP_OK &&
                i2c_master_transmit_receive(dev_handle, &reg_low, 1, &pid_low, 1, 200) == ESP_OK) {
                uint16_t pid = (pid_high << 8) | pid_low;
                ESP_LOGD(TAG, "GC2145 探测 PID=0x%04X", pid);
                if (pid == OTTO_GC2145_PID) {
                    detected_pid = pid;
                    found = true;
                }
            }
        }

        i2c_master_bus_rm_device(dev_handle);
        return found;
    }

    bool DetectCamera() {
        ledc_timer_config_t ledc_timer = {
            .speed_mode = LEDC_LOW_SPEED_MODE,
            .duty_resolution = LEDC_TIMER_2_BIT,
            .timer_num = LEDC_TIMER,
            .freq_hz = CAMERA_XCLK_FREQ,
            .clk_cfg = LEDC_AUTO_CLK,
        };
        esp_err_t ret = ledc_timer_config(&ledc_timer);
        if (ret != ESP_OK) {
            return false;
        }

        ledc_channel_config_t ledc_channel = {
            .gpio_num = CAMERA_XCLK,
            .speed_mode = LEDC_LOW_SPEED_MODE,
            .channel = LEDC_CHANNEL,
            .intr_type = LEDC_INTR_DISABLE,
            .timer_sel = LEDC_TIMER,
            .duty = 2,
            .hpoint = 0,
        };
        ret = ledc_channel_config(&ledc_channel);
        if (ret != ESP_OK) {
            return false;
        }

        vTaskDelay(pdMS_TO_TICKS(100));
        i2c_master_bus_config_t i2c_bus_cfg = {
            .i2c_port = I2C_NUM_0,
            .sda_io_num = CAMERA_VERSION_CONFIG.i2c_sda_pin,
            .scl_io_num = CAMERA_VERSION_CONFIG.i2c_scl_pin,
            .clk_source = I2C_CLK_SRC_DEFAULT,
            .glitch_ignore_cnt = 7,
            .intr_priority = 0,
            .trans_queue_depth = 0,
            .flags =
                {
                    .enable_internal_pullup = 1,
                },
        };

        ret = i2c_new_master_bus(&i2c_bus_cfg, &i2c_bus_);
        if (ret != ESP_OK) {
            ledc_stop(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL, 0);
            return false;
        }

        const uint8_t camera_addresses[] = {0x30, 0x3C, 0x21, 0x60};
        bool camera_found = false;
        bool unknown_camera_found = false;
        uint16_t detected_pid = 0;
        uint16_t unknown_pid = 0;

        for (size_t i = 0; i < sizeof(camera_addresses); i++) {
            uint8_t addr = camera_addresses[i];
            i2c_device_config_t dev_cfg = {
                .dev_addr_length = I2C_ADDR_BIT_LEN_7,
                .device_address = addr,
                .scl_speed_hz = 100000,
            };

            i2c_master_dev_handle_t dev_handle;
            ret = i2c_master_bus_add_device(i2c_bus_, &dev_cfg, &dev_handle);
            if (ret == ESP_OK) {
                uint8_t data[2] = {0, 0};

                uint8_t reg_addr_8bit = 0x0A;
                ret = i2c_master_transmit_receive(dev_handle, &reg_addr_8bit, 1, data, 2, 200);
                if (ret == ESP_OK && (data[0] != 0 || data[1] != 0)) {
                    uint16_t pid = (data[0] << 8) | data[1];
                    if (pid == OTTO_OV2640_PID_1 || pid == OTTO_OV2640_PID_2) {
                        detected_pid = pid;
                        ESP_LOGI(TAG, "检测到摄像头 (OV2640方式) PID=0x%04X (地址=0x%02X)",
                                 detected_pid, addr);
                        camera_found = true;
                        i2c_master_bus_rm_device(dev_handle);
                        break;
                    }
                    unknown_pid = pid;
                    unknown_camera_found = true;
                }

                uint8_t reg_addr_high[2] = {0x30, 0x0A};
                uint8_t reg_addr_low[2] = {0x30, 0x0B};
                uint8_t pid_high = 0, pid_low = 0;

                ret = i2c_master_transmit_receive(dev_handle, reg_addr_high, 2, &pid_high, 1, 200);
                if (ret == ESP_OK) {
                    ret =
                        i2c_master_transmit_receive(dev_handle, reg_addr_low, 2, &pid_low, 1, 200);
                    if (ret == ESP_OK) {
                        uint16_t pid = (pid_high << 8) | pid_low;
                        if (pid != 0) {
                            if (pid == OTTO_OV3660_PID) {
                                detected_pid = pid;
                                ESP_LOGI(TAG, "检测到摄像头 (OV3660方式) PID=0x%04X (地址=0x%02X)",
                                         detected_pid, addr);
                                camera_found = true;
                                i2c_master_bus_rm_device(dev_handle);
                                break;
                            }
                            unknown_pid = pid;
                            unknown_camera_found = true;
                        }
                    }
                }

                i2c_master_bus_rm_device(dev_handle);
            }
        }

        // OV2640/OV3660 与 GC2145 的寄存器协议不同，先按各自方式探测；
        // 只有都没命中时才尝试 GC2145，避免软复位干扰已经识别到的 OV 传感器。
        if (!camera_found) {
            uint16_t gc2145_pid = 0;
            if (DetectGc2145Camera(gc2145_pid)) {
                detected_pid = gc2145_pid;
                camera_found = true;
                ESP_LOGI(TAG, "检测到摄像头 (GC2145方式) PID=0x%04X", detected_pid);
            } else if (unknown_camera_found) {
                detected_pid = unknown_pid;
                camera_found = true;
                ESP_LOGW(TAG, "摄像头 PID 未识别，按未知型号继续初始化: PID=0x%04X", detected_pid);
            }
        }

        if (!camera_found) {
            i2c_del_master_bus(i2c_bus_);
            i2c_bus_ = nullptr;
            ledc_stop(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL, 0);
            camera_type_ = OTTO_CAMERA_NONE;
        } else {
            if (pid_is_ov2640(detected_pid)) {   // OV2640 有两个可能的 PID
                camera_type_ = OTTO_CAMERA_OV2640;
                ESP_LOGI(TAG, "摄像头类型: OV2640 (PID=0x%04X)", detected_pid);
            } else if (detected_pid == OTTO_OV3660_PID) {
                camera_type_ = OTTO_CAMERA_OV3660;
                ESP_LOGI(TAG, "摄像头类型: OV3660 (PID=0x%04X)", detected_pid);
            } else if (detected_pid == OTTO_GC2145_PID) {
                camera_type_ = OTTO_CAMERA_GC2145;
                ESP_LOGI(TAG, "摄像头类型: GC2145 (PID=0x%04X)", detected_pid);
            } else {
                camera_type_ = OTTO_CAMERA_UNKNOWN;
                ESP_LOGW(TAG, "未知摄像头类型，PID=0x%04X", detected_pid);
            }
        }
        return camera_found;
    }

    // ------------------------------------------------------------
    // 基础外设初始化
    // ------------------------------------------------------------
    void InitializePowerManager() {
        power_manager_ = new PowerManager(hw_config_.power_charge_detect_pin,
                                          hw_config_.power_adc_unit, hw_config_.power_adc_channel);
    }

    void InitializeSpi() {
        spi_bus_config_t buscfg = {};
        buscfg.mosi_io_num = hw_config_.display_mosi_pin;
        buscfg.miso_io_num = GPIO_NUM_NC;
        buscfg.sclk_io_num = hw_config_.display_clk_pin;
        buscfg.quadwp_io_num = GPIO_NUM_NC;
        buscfg.quadhd_io_num = GPIO_NUM_NC;
        buscfg.max_transfer_sz = DISPLAY_WIDTH * DISPLAY_HEIGHT * sizeof(uint16_t);
        ESP_ERROR_CHECK(spi_bus_initialize(SPI3_HOST, &buscfg, SPI_DMA_CH_AUTO));
    }

    void InitializeLcdDisplay() {
        esp_lcd_panel_io_handle_t panel_io = nullptr;
        esp_lcd_panel_handle_t panel = nullptr;
        esp_lcd_panel_io_spi_config_t io_config = {};
        io_config.cs_gpio_num = hw_config_.display_cs_pin;
        io_config.dc_gpio_num = hw_config_.display_dc_pin;
        io_config.spi_mode = DISPLAY_SPI_MODE;
        io_config.pclk_hz = 20 * 1000 * 1000;
        io_config.trans_queue_depth = 10;
        io_config.lcd_cmd_bits = 8;
        io_config.lcd_param_bits = 8;
        ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi(SPI3_HOST, &io_config, &panel_io));

        esp_lcd_panel_dev_config_t panel_config = {};
        panel_config.reset_gpio_num = hw_config_.display_rst_pin;
        panel_config.rgb_ele_order = DISPLAY_RGB_ORDER;
        panel_config.bits_per_pixel = 16;

        ESP_ERROR_CHECK(esp_lcd_new_panel_st7789(panel_io, &panel_config, &panel));

        esp_lcd_panel_reset(panel);
        esp_lcd_panel_init(panel);
        esp_lcd_panel_invert_color(panel, DISPLAY_INVERT_COLOR);
        esp_lcd_panel_swap_xy(panel, DISPLAY_SWAP_XY);
        esp_lcd_panel_mirror(panel, DISPLAY_MIRROR_X, DISPLAY_MIRROR_Y);

        display_ = new OttoEmojiDisplay(panel_io, panel, DISPLAY_WIDTH, DISPLAY_HEIGHT,
                                        DISPLAY_OFFSET_X, DISPLAY_OFFSET_Y, DISPLAY_MIRROR_X,
                                        DISPLAY_MIRROR_Y, DISPLAY_SWAP_XY);
    }

    void InitializeButtons() {
        boot_button_.OnClick([this]() {
            // 软关机黑屏时，按一下开机键等于唤醒
            if (screen_off_) {
                WakeUpFromSoftShutdown();
                return;
            }
            auto& app = Application::GetInstance();
            if (app.GetDeviceState() == kDeviceStateStarting) {
                EnterWifiConfigMode();
                return;
            }
            app.ToggleChatState();
        });
    }

    void InitializeAudioCodec() {
        if (hw_config_.audio_use_simplex) {
            audio_codec_ = new NoAudioCodecSimplex(
                hw_config_.audio_input_sample_rate, hw_config_.audio_output_sample_rate,
                hw_config_.audio_i2s_spk_gpio_bclk, hw_config_.audio_i2s_spk_gpio_lrck,
                hw_config_.audio_i2s_spk_gpio_dout, hw_config_.audio_i2s_mic_gpio_sck,
                hw_config_.audio_i2s_mic_gpio_ws, hw_config_.audio_i2s_mic_gpio_din);
        } else {
            audio_codec_ = new NoAudioCodecDuplex(
                hw_config_.audio_input_sample_rate, hw_config_.audio_output_sample_rate,
                hw_config_.audio_i2s_gpio_bclk, hw_config_.audio_i2s_gpio_ws,
                hw_config_.audio_i2s_gpio_dout, hw_config_.audio_i2s_gpio_din);
        }
    }

    void InitializeOttoController() { ::InitializeOttoController(hw_config_); }

    // ============================================================
    // 【优化 1】开机逐舵机归位
    // 由 otto_controller.cc 里的 OttoController_BootHomeSequence() 实现：
    // 6 个舵机一个一个 attach + 拉到中位，每个间隔 180ms，
    // 峰值电流从 6 个同时启动降到 1 个，避免上电冲击把板子拖重启。
    // ============================================================
    // ============================================================
    // 【开机顺序】三阶段推进：画面先出，舵机永远排在最后
    // ------------------------------------------------------------
    // 白屏的真凶不是舵机（它本来就在独立任务里，不阻塞），而是
    // InitializeAudioCodec()（I2S 驱动安装 + DMA 缓冲分配，阻塞几百毫秒）
    // 和 DetectCamera()（I2C 探 4 个地址 + 100ms 级复位延时）。这两段原先
    // 排在构造函数里，把 Application::Initialize() 的返回时间整体推后，
    // LVGL 首帧迟迟刷不出来 —— 背光亮着但没内容，就是白屏。
    //
    // 所以构造函数只做「不阻塞的事」就返回，音频/摄像头/舵机控制器全部
    // 挪到这里，等 kDeviceStateListening（首帧已上屏）才动手。
    // ============================================================
    static void BootFinishTask(void* arg) {
        auto* self = static_cast<OttoRobot*>(arg);
        const int64_t t0 = esp_timer_get_time();
        while (true) {
            vTaskDelay(pdMS_TO_TICKS(80));
            self->boot_wait_ms_ += 80;
            if (self->BootAdvance()) {
                break;
            }
        }
        ESP_LOGI(TAG, "开机尾巴总耗时 %lld ms", (esp_timer_get_time() - t0) / 1000);
        vTaskDelete(NULL);
    }

    // 返回 true 表示全部阶段做完，任务可以退出
    bool BootAdvance() {
        // 3 秒兜底：万一网络异常、状态机一直没进 listening，
        // 也不能把音频和舵机饿死，超时强制推进。
        const bool ui_up = boot_phase_ >= kBootUiUp ||
                           Application::GetInstance().GetDeviceState() == kDeviceStateListening ||
                           boot_wait_ms_ >= 3000;

        if (boot_phase_ == kBootEarly) {
            if (!ui_up) {
                return false;
            }
            boot_phase_ = kBootUiUp;
            ESP_LOGI(TAG, "开机阶段：首帧已上屏，补初始化慢外设");

            const int64_t t = esp_timer_get_time();
            InitializeAudioCodec();
            ESP_LOGI(TAG, "AudioCodec 初始化耗时 %lld ms", (esp_timer_get_time() - t) / 1000);

            if (is_camera_board_) {
                has_camera_ = DetectCamera();
                if (!has_camera_) {
                    ESP_LOGW(TAG, "摄像头版未检测到摄像头，将跳过摄像头初始化");
                }
                if (has_camera_ && !InitializeCamera()) {
                    has_camera_ = false;
                }
            }

            // 舵机控制器要最后建：它注册动作类 MCP 工具，
            // 建早了模型可能在画面还没出来时就下令走路。
            InitializeOttoController();
            InitializeCustomMcpTools();
            return false;
        }

        if (boot_phase_ == kBootUiUp) {
            // 再让画面稳 500ms，舵机才开始动
            if (boot_wait_ms_ < 500) {
                return false;
            }
            boot_phase_ = kBootSettled;
            ESP_LOGI(TAG, "开机阶段：画面已稳，舵机开始逐个归位");
            OttoController_BootHomeSequence();
            ESP_LOGI(TAG, "开机舵机逐位归位完成");
            BootShowReady();
            return true;
        }
        return true;
    }

    // 归位完成后给一句明确的开场提示（此时画面和舵机都就绪了）
    void BootShowReady() {
        if (display_ != nullptr && !screen_off_) {
            display_->SetEmotion("happy");
            display_->SetChatMessage("system", "你好，我准备好了");
        }
    }

    // ============================================================
    // 【优化 3】屏幕开关封装
    // ============================================================
    void TurnScreenOn() {
        if (GetBacklight() != nullptr) {
            GetBacklight()->RestoreBrightness();
        }
        screen_off_ = false;
    }

    void TurnScreenOff() {
        if (display_ != nullptr) {
            display_->SetChatMessage("system", "");
            display_->SetEmotion("neutral");
        }
        if (GetBacklight() != nullptr) {
            GetBacklight()->SetBrightness(0);
        }
        screen_off_ = true;
    }

    // ============================================================
    // 【优化 3】唤醒：舵机重新上电 + 自动亮屏
    // 软关机期间 MCU 和麦克风一直在跑，唤醒词照样能触发；
    // 语音链路本身由 Application 处理，这里只负责"把灯打开、把舵机叫醒"。
    // ============================================================
public:
    void WakeUpFromSoftShutdown() {
        if (!screen_off_) {
            return;  // 本来就是醒的，别重复上电
        }
        ESP_LOGI(TAG, "从软关机唤醒");
        // 1. 若闹钟正在播报，唤醒视为"知道了"，直接收尾
        if (alarm_active_) {
            StopAlarmSequence(true);
        }

        // 2. 屏幕先亮，让用户立刻看到反馈
        TurnScreenOn();
        if (display_ != nullptr) {
            display_->SetEmotion("happy");
            display_->SetChatMessage("system", "我在");
        }

        // 3. 舵机重新上电并慢速归位（内部自己做，不阻塞这里）
        OttoController_Wakeup();

        // 4. 播一声提示，明确告诉用户已经开机
        // 唤醒用内置提示音即可（不需要说话，避免又被模型接一句）
        Application::GetInstance().PlaySound(Lang::Sounds::OGG_WELCOME);
        ESP_LOGI(TAG, "唤醒完成，屏幕已亮、舵机已上电");
    }

private:

    // ============================================================
    // 【优化 2】闹钟：播报 3 次 + 温柔招手
    // 用 tick 定时器驱动状态机，全程不阻塞任何任务，
    // 所以播报期间语音唤醒、按按键、关机都照样能打断。
    // ============================================================
    static void AlarmTickCallback(void* arg) {
        auto* self = static_cast<OttoRobot*>(arg);
        if (self != nullptr) {
            self->AlarmTick();
        }
    }

    void AlarmTick() {
        if (!alarm_active_) {
            return;
        }

        // 屏幕从黑屏态被闹钟叫亮（用户关机后设的闹钟也要能响）
        if (screen_off_) {
            TurnScreenOn();
        }

        alarm_round_++;
        ESP_LOGI(TAG, "闹钟第 %d/%d 轮", alarm_round_, kAlarmMaxRounds);

        if (alarm_round_ > kAlarmMaxRounds) {
            StopAlarmSequence(true);
            return;
        }

        // ★ 闹钟播报：板子自己拼固定话术，不经过大模型 ★
        // "你好呀，我的朋友"是这么来的：原先靠 MCP 返回值 / 模型自由发挥来报时，
        // 模型拿到"闹钟到了"就会自己补一句客套话。现在板子直接播固定话术，
        // 语音链路不唤醒，模型没有插话的机会。
        // 设了闹钟名 -> "主人，吃药"；没设 -> "主人，时间到了"
        char buf[96];
        snprintf(buf, sizeof(buf), "主人 时间到了", alarm_name_.empty() ? "时间到了" : alarm_name_.c_str());
        if (display_ != nullptr) {
            display_->SetEmotion("excited");
            display_->SetChatMessage("system", buf);
        }
        ESP_LOGI(TAG, "闹钟播报: %s", buf);
        Application::GetInstance().PlaySound(Lang::Sounds::OGG_WELCOME);

        // 先出声，间隔 700ms 再抬手，避免"声音和动作抢同一时刻"显得机械
        vTaskDelay(pdMS_TO_TICKS(700));
        OttoController_GentleWave();
    }

    // 停止播报。settle=true 表示收尾时让舵机安静归位
    void StopAlarmSequence(bool settle) {
        if (alarm_tick_timer_ != nullptr) {
            esp_timer_stop(alarm_tick_timer_);
            esp_timer_delete(alarm_tick_timer_);
            alarm_tick_timer_ = nullptr;
        }
        if (alarm_timer_ != nullptr) {
            esp_timer_stop(alarm_timer_);
            esp_timer_delete(alarm_timer_);
            alarm_timer_ = nullptr;
        }
        alarm_active_ = false;
        alarm_round_ = 0;

        if (settle && !screen_off_) {
            OttoController_ClearQueue();      // 丢掉还没播完的招手，别攒着一股脑补
            if (display_ != nullptr) {
                display_->SetEmotion("neutral");
                display_->SetChatMessage("system", "");
            }
        }
        ESP_LOGI(TAG, "闹钟播报结束");
    }

    static void AlarmStartCallback(void* arg) {
        auto* self = static_cast<OttoRobot*>(arg);
        if (self == nullptr) {
            return;
        }
        // 到点了：先清掉单次定时器句柄，再启动播报节拍
        if (self->alarm_timer_ != nullptr) {
            esp_timer_delete(self->alarm_timer_);
            self->alarm_timer_ = nullptr;
        }

        self->alarm_active_ = true;
        self->alarm_round_ = 0;

        esp_timer_create_args_t tick_args = {};
        tick_args.callback = &OttoRobot::AlarmTickCallback;
        tick_args.arg = self;
        tick_args.dispatch_method = ESP_TIMER_TASK;
        tick_args.name = "otto_alarm_tick";

        if (esp_timer_create(&tick_args, &self->alarm_tick_timer_) != ESP_OK) {
            ESP_LOGE(TAG, "闹钟节拍创建失败");
            self->alarm_active_ = false;
            return;
        }
        // 立刻先响第一轮，之后每 kAlarmIntervalMs 一轮
        esp_timer_start_periodic(self->alarm_tick_timer_, (uint64_t)kAlarmIntervalMs * 1000ULL);
        ESP_LOGI(TAG, "闹钟触发，开始 %d 轮播报", OttoRobot::kAlarmMaxRounds);
    }

    // ============================================================
    // MCP 工具注册
    // ============================================================
    void InitializeCustomMcpTools() {
        auto& mcp_server = McpServer::GetInstance();

        // 1. 闹钟：到点播报 3 次 + 温柔招手
        mcp_server.AddTool(
            "self.Alarm",
            "设置一个闹钟。到点后会用语音连说三遍“主人，时间到了”，同时温柔地招手互动。"
            "seconde_from_now: 多少秒后响铃；alarm_name: 闹钟的名称或提醒内容",
            PropertyList({
                Property("seconde_from_now", kPropertyTypeInteger, 10, 0, 86400),
                Property("alarm_name", kPropertyTypeString),
            }),
            [this](const PropertyList& properties) -> ReturnValue {
                int seconds = properties["seconde_from_now"].value<int>();
                std::string name = properties["alarm_name"].value<std::string>();

                if (seconds <= 0) {
                    return ReturnValue("闹钟时间必须大于 0 秒");
                }
                alarm_name_ = name.empty() ? "时间到了" : name;

                // 已有闹钟（含正在播报的）先整体清掉，再重设
                StopAlarmSequence(false);

                esp_timer_create_args_t args = {};
                args.callback = &OttoRobot::AlarmStartCallback;
                args.arg = this;
                args.dispatch_method = ESP_TIMER_TASK;
                args.name = "otto_alarm";

                if (esp_timer_create(&args, &alarm_timer_) != ESP_OK) {
                    return ReturnValue("闹钟创建失败");
                }
                esp_timer_start_once(alarm_timer_, (uint64_t)seconds * 1000000ULL);

                ESP_LOGI(TAG, "闹钟已设置: %d 秒后 -> %s", seconds, alarm_name_.c_str());
                return ReturnValue("");   // 空返回：不让模型复述，避免多出一句"好的，我的朋友"
            });

        // 2. 关机：语音回复后舵机归位下电、屏幕关闭，仅保留语音唤醒
        mcp_server.AddTool(
            "self.Shutdown",
            "关闭机器人。会先用语音说“好的主人，我关机了”，然后舵机逐个归位并彻底断电省电，"
            "屏幕关闭，只保留语音唤醒，叫一声“卡卡”就能重新开机亮屏。",
            PropertyList(),
            [this](const PropertyList& properties) -> ReturnValue {
                // 先把回话播出去，再走关机序列（序列内部会等语音播完再断舵机电）
                EnterSoftShutdown();
                // 空返回：不回传任何文本给模型，否则模型会自行复述成
                // "好的，我的朋友"这类多余的话。关机提示音由 EnterSoftShutdown 内部播放。
                return ReturnValue("");
            });
    }

    // ============================================================
    // 【优化 3】软关机：舵机归位 → 下电 → 关屏，只留语音唤醒
    // MCU 与 I2S 麦克风全程不停，所以唤醒词随时可用；
    // 真正省下来的是舵机保持电流（detach 后归零）和屏幕背光功耗。
    // ============================================================
    void EnterSoftShutdown() {
        ESP_LOGI(TAG, "进入软关机序列");

        // 1. 闹钟停掉（正在播报也一并收住）
        StopAlarmSequence(false);

        // 2. 屏幕先黑，给用户"已经关了"的即时反馈
        TurnScreenOff();

        // 3. 舵机归位 → 逐个下电。这一步在独立任务里做，
        //    因为它要等约 2.5 秒，不能卡住 MCP 调用线程（否则语音回话都播不出来）
        xTaskCreate([](void* arg) {
            (void)arg;  // 下电走全局 C 接口，不需要 this
            // 等关机提示音播完再动舵机，声音不会被舵机电流噪声盖住
            vTaskDelay(pdMS_TO_TICKS(600));
            OttoController_Sleep();      // 内部：归位 → 逐个 detach → LEDC 保持电流归零
            ESP_LOGI(TAG, "软关机完成：舵机已下电、屏幕已关，等待语音唤醒");
            vTaskDelete(NULL);
        }, "otto_shutdown", 3072, this, 3, nullptr);
    }

public:
    const HardwareConfig& GetHardwareConfig() const { return hw_config_; }
    OttoCameraType GetCameraType() const { return camera_type_; }

private:
    static constexpr int kAlarmMaxRounds = 3;      // 播报轮数
    static constexpr int kAlarmIntervalMs = 3200;  // 每轮间隔：说话 + 招手约 2.5s，留余量

    void InitializeWebSocketControlServer() {
        ws_control_server_ = new WebSocketControlServer();
        if (!ws_control_server_->Start(8080)) {
            delete ws_control_server_;
            ws_control_server_ = nullptr;
            return;
        }
    }

    void StartNetwork() override {
        WifiBoard::StartNetwork();
        vTaskDelay(pdMS_TO_TICKS(1000));
        InitializeWebSocketControlServer();
    }

    bool InitializeCamera() {
        if (!has_camera_ || i2c_bus_ == nullptr) {
            return false;
        }

        // 释放检测阶段占用的 I2C 资源，避免与 esp_camera 初始化冲突。
        i2c_del_master_bus(i2c_bus_);
        i2c_bus_ = nullptr;
        // 停止检测阶段输出的 XCLK，交由 esp_camera 自行接管。
        ledc_stop(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL, 0);

        camera_config_t config = {};
        config.ledc_channel = LEDC_CHANNEL_0;
        config.ledc_timer = LEDC_TIMER_0;
        config.pin_d0 = CAMERA_D0;
        config.pin_d1 = CAMERA_D1;
        config.pin_d2 = CAMERA_D2;
        config.pin_d3 = CAMERA_D3;
        config.pin_d4 = CAMERA_D4;
        config.pin_d5 = CAMERA_D5;
        config.pin_d6 = CAMERA_D6;
        config.pin_d7 = CAMERA_D7;
        config.pin_xclk = CAMERA_XCLK;
        config.pin_pclk = CAMERA_PCLK;
        config.pin_vsync = CAMERA_VSYNC;
        config.pin_href = CAMERA_HSYNC;
        config.pin_sccb_sda = CAMERA_VERSION_CONFIG.i2c_sda_pin;
        config.pin_sccb_scl = CAMERA_VERSION_CONFIG.i2c_scl_pin;
        config.sccb_i2c_port = 0;
        config.pin_pwdn = CAMERA_PWDN;
        config.pin_reset = CAMERA_RESET;
        config.xclk_freq_hz = CAMERA_XCLK_FREQ;
        config.pixel_format = PIXFORMAT_RGB565;
        config.frame_size = FRAMESIZE_240X240;
        config.jpeg_quality = 12;
        config.fb_count = 1;
        config.fb_location = CAMERA_FB_IN_PSRAM;
        config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;

        camera_ = new (std::nothrow) Esp32Camera(config);
        if (camera_ != nullptr && camera_type_ == OTTO_CAMERA_GC2145) {
            // The GC2145 module is mounted 180 degrees relative to the OV sensors.
            camera_->SetHMirror(false);
            camera_->SetVFlip(false);
            ESP_LOGI(TAG, "GC2145 orientation corrected (hmirror=0, vflip=0)");
        }
        return camera_ != nullptr;
    }

public:
    OttoRobot()
        : boot_button_(BOOT_BUTTON_GPIO),
          audio_codec_(nullptr),
          i2c_bus_(nullptr),
          camera_(nullptr),
          is_camera_board_(false),
          has_camera_(false),
          camera_type_(OTTO_CAMERA_NONE),
          alarm_timer_(nullptr),
          alarm_tick_timer_(nullptr),
          alarm_round_(0),
          alarm_active_(false),
          screen_off_(false) {
#if OTTO_HARDWARE_VERSION == OTTO_VERSION_AUTO
        // GPIO15/GPIO16 在摄像头版上有外部上拉；无摄像头版由内部弱下拉保持为低。
        is_camera_board_ = DetectHardwareVersion();
        ESP_LOGI(TAG, "自动检测硬件版本: %s", is_camera_board_ ? "摄像头版" : "无摄像头版");
#elif OTTO_HARDWARE_VERSION == OTTO_VERSION_CAMERA
        is_camera_board_ = true;
        ESP_LOGI(TAG, "强制使用摄像头版本配置");
#elif OTTO_HARDWARE_VERSION == OTTO_VERSION_NO_CAMERA
        is_camera_board_ = false;
        ESP_LOGI(TAG, "强制使用无摄像头版本配置");
#else
#error \
    "OTTO_HARDWARE_VERSION 设置无效，请使用 OTTO_VERSION_AUTO, OTTO_VERSION_CAMERA 或 OTTO_VERSION_NO_CAMERA"
#endif

        if (is_camera_board_) {
            hw_config_ = CAMERA_VERSION_CONFIG;
        } else {
            hw_config_ = NON_CAMERA_VERSION_CONFIG;
        }

        // ===== 构造函数只做「不阻塞的事」，做完立刻返回 =====
        // 顺序：SPI → LCD → 按钮 → 电源 → 背光亮 → 起开机尾巴任务。
        // 音频编解码器、摄像头探测、舵机控制器全部挪到 BootFinishTask，
        // 等 LVGL 首帧真正刷出来之后再补，这样开机就是「画面先出、舵机后动」。
        InitializeSpi();
        InitializeLcdDisplay();
        InitializeButtons();
        InitializePowerManager();
        ws_control_server_ = nullptr;
        GetBacklight()->RestoreBrightness();   // ← 背光立刻亮，画面随后就出来

        // 开机尾巴任务：等首帧上屏 → 补慢外设 → 画面稳了才让舵机归位
        xTaskCreate(BootFinishTask, "otto_boot", 4096, this, 3, nullptr);
    }

    virtual AudioCodec* GetAudioCodec() override { return audio_codec_; }
    virtual Display* GetDisplay() override { return display_; }

    virtual Backlight* GetBacklight() override {
        static PwmBacklight* backlight = nullptr;
        if (backlight == nullptr) {
            backlight =
                new PwmBacklight(hw_config_.display_backlight_pin, DISPLAY_BACKLIGHT_OUTPUT_INVERT);
        }
        return backlight;
    }

    bool GetBatteryLevel(int& level, bool& charging, bool& discharging) override {
        level = 100;
        charging = false;
        discharging = true;  // 保持放电状态即可，100% 不会触发低电量拦截
        return true;
    }

    virtual Camera* GetCamera() override { return has_camera_ ? camera_ : nullptr; }
};

DECLARE_BOARD(OttoRobot);
