#include "otto_emoji_display.h"

#include <esp_adc/adc_oneshot.h>
#include <esp_log.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <cstring>
#include <string>

#include "application.h"
#include "button.h"
#include "codecs/no_audio_codec.h"
#include "config.h"
#include "display/lcd_display.h"
#include "esp32_camera.h"
#include "lamp_controller.h"
#include "led/single_led.h"
#include "mcp_server.h"
#include "power_manager.h"
#include "system_reset.h"
#include "websocket_control_server.h"
#include "wifi_board.h"
#include "assets/lang_config.h"

#define TAG "OttoRobot"

// ============================================================
// 硬件配置选择
// ------------------------------------------------------------
// config.h 提供两套完整引脚表；OTTO_HARDWARE_VERSION 为 AUTO 时按
// Camera::GetCameraConfigCount() 自动判断（摄像头版 / 无摄像头版）。
// 所有宏都带 #ifndef 兜底：万一 config.h 没被工程包含，也不会再出现
// "was not declared in this scope"。
// ============================================================
#ifndef OTTO_VERSION_AUTO
#define OTTO_VERSION_AUTO 0
#endif
#ifndef OTTO_VERSION_CAMERA
#define OTTO_VERSION_CAMERA 1
#endif
#ifndef OTTO_VERSION_NO_CAMERA
#define OTTO_VERSION_NO_CAMERA 2
#endif

// SPI 总线：两套配置共用 SPI2_HOST
#ifndef DISPLAY_SPI_HOST
#define DISPLAY_SPI_HOST SPI2_HOST
#endif

// 屏幕参数：config.h 未给出时的兜底（1.54 寸 ST7789 240x240）
#ifndef DISPLAY_WIDTH
#define DISPLAY_WIDTH 240
#endif
#ifndef DISPLAY_HEIGHT
#define DISPLAY_HEIGHT 240
#endif
#ifndef DISPLAY_OFFSET_X
#define DISPLAY_OFFSET_X 0
#endif
#ifndef DISPLAY_OFFSET_Y
#define DISPLAY_OFFSET_Y 0
#endif
#ifndef DISPLAY_MIRROR_X
#define DISPLAY_MIRROR_X false
#endif
#ifndef DISPLAY_MIRROR_Y
#define DISPLAY_MIRROR_Y false
#endif
#ifndef DISPLAY_SWAP_XY
#define DISPLAY_SWAP_XY false
#endif
#ifndef DISPLAY_INVERT_COLOR
#define DISPLAY_INVERT_COLOR true
#endif
#ifndef DISPLAY_BACKLIGHT_OUTPUT_INVERT
#define DISPLAY_BACKLIGHT_OUTPUT_INVERT false
#endif
#ifndef DISPLAY_RGB_ORDER
#define DISPLAY_RGB_ORDER LCD_RGB_ELEMENT_ORDER_RGB
#endif
#ifndef BOOT_BUTTON_GPIO
#define BOOT_BUTTON_GPIO GPIO_NUM_0
#endif
#ifndef BUILTIN_LED_GPIO
#define BUILTIN_LED_GPIO GPIO_NUM_NC
#endif
#ifndef BUILTIN_LED_ACTIVE_LEVEL
#define BUILTIN_LED_ACTIVE_LEVEL 0
#endif

// 电量检测兜底：config.h 里两套配置都是 NC + ADC1/CH1，这里只是防未声明
#ifndef POWER_BATTERY_FULL_VOLTAGE
#define POWER_BATTERY_FULL_VOLTAGE 4.2f
#endif
#ifndef POWER_BATTERY_EMPTY_VOLTAGE
#define POWER_BATTERY_EMPTY_VOLTAGE 3.3f
#endif
#ifndef POWER_BATTERY_LEVEL_ATTEN
#define POWER_BATTERY_LEVEL_ATTEN 0
#endif
#ifndef POWER_CHARGE_DETECT_LEVEL
#define POWER_CHARGE_DETECT_LEVEL 0
#endif

// 摄像头 SCCB（I2C）引脚兜底：config.h 用 sda_pin/scl_pin 字段表达
#ifndef CAMERA_SIOD_PIN
#define CAMERA_SIOD_PIN (-1)
#endif
#ifndef CAMERA_SIOC_PIN
#define CAMERA_SIOC_PIN (-1)
#endif

// 音频参数兜底
#ifndef AUDIO_SAMPLE_RATE
#define AUDIO_SAMPLE_RATE 16000
#endif
#ifndef AUDIO_CODEC_BITS_PER_SAMPLE
#define AUDIO_CODEC_BITS_PER_SAMPLE 16
#endif

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

// ============================================================
// OttoRobot 板级
// ============================================================
class OttoRobot : public WifiBoard {
private:
    // ---------- 开机阶段 ----------
    enum BootPhase {
        kBootEarly,     // 构造函数阶段：只起屏幕
        kBootUiUp,      // 首帧已上屏，开始干慢活
        kBootSettled,   // 全部就绪
    };

    LcdDisplay* display_ = nullptr;
    PowerManager* power_manager_ = nullptr;
    Button boot_button_;
    WebSocketControlServer* ws_control_server_ = nullptr;
    HardwareConfig hw_config_;
    AudioCodec* audio_codec_ = nullptr;
    adc_oneshot_unit_handle_t adc_handle_ = nullptr;
    i2c_master_bus_handle_t i2c_bus_ = nullptr;
    Camera* camera_ = nullptr;
    bool is_camera_board_ = false;
    bool has_camera_ = false;
    OttoCameraType camera_type_ = OTTO_CAMERA_NONE;
    bool screen_off_ = false;
    bool asleep_ = false;

    BootPhase boot_phase_ = kBootEarly;
    int boot_wait_ms_ = 0;

    // 屏幕熄灭用的背光亮度（0 表示黑屏）
    static const int kBacklightLevel = 80;

    // ------------------------------------------------------------
    // 硬件配置选择
    // ------------------------------------------------------------
    static const HardwareConfig& PickConfig() {
#if OTTO_HARDWARE_VERSION == OTTO_VERSION_CAMERA
        return CAMERA_VERSION_CONFIG;
#elif OTTO_HARDWARE_VERSION == OTTO_VERSION_NO_CAMERA
        return NON_CAMERA_VERSION_CONFIG;
#else
        // AUTO：按 sdkconfig 里是否配置了摄像头来选
        if (Camera::GetCameraConfigCount() > 0) {
            return CAMERA_VERSION_CONFIG;
        }
        return NON_CAMERA_VERSION_CONFIG;
#endif
    }

    // ------------------------------------------------------------
    // SPI / LCD
    // ------------------------------------------------------------
    void InitializeSpi() {
        const HardwareConfig& cfg = hw_config_;
        spi_bus_config_t buscfg = {};
        buscfg.mosi_io_num = cfg.display_mosi_pin;
        buscfg.miso_io_num = GPIO_NUM_NC;
        buscfg.sclk_io_num = cfg.display_clk_pin;
        buscfg.quadwp_io_num = GPIO_NUM_NC;
        buscfg.quadhd_io_num = GPIO_NUM_NC;
        buscfg.max_transfer_sz = DISPLAY_WIDTH * DISPLAY_HEIGHT * sizeof(uint16_t);
        ESP_ERROR_CHECK(spi_bus_initialize(DISPLAY_SPI_HOST, &buscfg, SPI_DMA_CH_AUTO));
    }

    void InitializeLcdDisplay() {
        const HardwareConfig& cfg = hw_config_;
        esp_lcd_panel_io_handle_t panel_io = nullptr;
        esp_lcd_panel_handle_t panel = nullptr;

        esp_lcd_panel_io_spi_config_t io_config = {};
        io_config.cs_gpio_num = cfg.display_cs_pin;   // NC 时驱动自动忽略
        io_config.dc_gpio_num = cfg.display_dc_pin;
        io_config.spi_mode = DISPLAY_SPI_MODE;
        io_config.pclk_hz = 40 * 1000 * 1000;
        io_config.trans_queue_depth = 10;
        io_config.lcd_cmd_bits = 8;
        io_config.lcd_param_bits = 8;
        ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi(DISPLAY_SPI_HOST, &io_config, &panel_io));

        esp_lcd_panel_dev_config_t panel_config = {};
        panel_config.reset_gpio_num = cfg.display_rst_pin;
        panel_config.bits_per_pixel = 16;
        panel_config.color_space = ESP_LCD_COLOR_SPACE_RGB;

        esp_lcd_panel_st7789_vendor_config_t vendor_config = {};
        vendor_config.flags.use_qspi_interface = 0;
        panel_config.vendor_config = &vendor_config;

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
            if (screen_off_ || asleep_) {
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

    // ------------------------------------------------------------
    // 音频：严格按照 config.h 里两套配置的引脚
    // ------------------------------------------------------------
    void InitializeAudioCodec() {
        int64_t t0 = esp_timer_get_time();
        const HardwareConfig& cfg = hw_config_;

        // 注意：no_audio_codec.h 里的两个构造函数【都不接收 MCLK 引脚】，
        // 第一个参数就是 input_sample_rate，引脚顺序为 bclk, ws, dout, din。
        if (cfg.audio_use_simplex) {
            // 无摄像头版：麦克风与功放各自独立引脚（单工）
            // NoAudioCodecSimplex(in_rate, out_rate, spk_bclk, spk_ws, spk_dout, mic_sck, mic_ws, mic_din)
            audio_codec_ = new NoAudioCodecSimplex(
                cfg.audio_input_sample_rate, cfg.audio_output_sample_rate,
                cfg.audio_i2s_spk_gpio_bclk, cfg.audio_i2s_spk_gpio_lrck,
                cfg.audio_i2s_spk_gpio_dout,
                cfg.audio_i2s_mic_gpio_sck, cfg.audio_i2s_mic_gpio_ws,
                cfg.audio_i2s_mic_gpio_din);
        } else {
            // 摄像头版：共用一组 I2S 引脚（双工）
            // NoAudioCodecDuplex(in_rate, out_rate, bclk, ws, dout, din)
            audio_codec_ = new NoAudioCodecDuplex(
                cfg.audio_input_sample_rate, cfg.audio_output_sample_rate,
                cfg.audio_i2s_gpio_bclk, cfg.audio_i2s_gpio_ws,
                cfg.audio_i2s_gpio_dout, cfg.audio_i2s_gpio_din);
        }
        ESP_LOGI(TAG, "AudioCodec 初始化完成（%s, mic=%dHz spk=%dHz），耗时 %lld ms",
                 cfg.audio_use_simplex ? "单工" : "双工",
                 cfg.audio_input_sample_rate, cfg.audio_output_sample_rate,
                 (esp_timer_get_time() - t0) / 1000);
    }

    // ------------------------------------------------------------
    // 摄像头（慢活，推迟到首帧之后）
    // ------------------------------------------------------------
    bool CamProbe(esp_lcd_panel_io_handle_t io, uint16_t reg, uint16_t* pid) {
        uint8_t hi = 0, lo = 0;
        if (esp_lcd_panel_io_rx_param(io, (int)reg, &hi, 1) != ESP_OK) return false;
        if (esp_lcd_panel_io_rx_param(io, (int)reg + 1, &lo, 1) != ESP_OK) return false;
        *pid = (uint16_t)((hi << 8) | lo);
        return true;
    }

    void DetectCamera() {
        const HardwareConfig& cfg = hw_config_;
        is_camera_board_ = (OTTO_HARDWARE_VERSION == OTTO_VERSION_CAMERA) ||
                           ((OTTO_HARDWARE_VERSION == OTTO_VERSION_AUTO) &&
                            (Camera::GetCameraConfigCount() > 0));

        if (!is_camera_board_ || cfg.i2c_sda_pin == GPIO_NUM_NC || cfg.i2c_scl_pin == GPIO_NUM_NC) {
            ESP_LOGI(TAG, "非摄像头版或未配置 I2C，跳过摄像头探测");
            has_camera_ = false;
            camera_type_ = OTTO_CAMERA_NONE;
            return;
        }

        i2c_master_bus_config_t i2c_cfg = {};
        i2c_cfg.i2c_port = I2C_NUM_1;
        i2c_cfg.sda_io_num = cfg.i2c_sda_pin;
        i2c_cfg.scl_io_num = cfg.i2c_scl_pin;
        i2c_cfg.clk_source = I2C_CLK_SRC_DEFAULT;
        i2c_cfg.glitch_ignore_cnt = 7;
        i2c_cfg.flags.enable_internal_pullup = true;
        if (i2c_new_master_bus(&i2c_cfg, &i2c_bus_) != ESP_OK) {
            ESP_LOGE(TAG, "摄像头 I2C 总线创建失败");
            has_camera_ = false;
            return;
        }

        // 依次探测常见摄像头 PID
        static const uint16_t kCandidates[] = {OTTO_OV2640_PID_1, OTTO_OV2640_PID_2,
                                               OTTO_OV3660_PID,  OTTO_GC2145_PID};
        static const OttoCameraType kTypes[] = {OTTO_CAMERA_OV2640, OTTO_CAMERA_OV2640,
                                                OTTO_CAMERA_OV3660, OTTO_CAMERA_GC2145};
        static const char* kNames[] = {"OV2640", "OV2640(alt)", "OV3660", "GC2145"};

        for (size_t i = 0; i < sizeof(kCandidates) / sizeof(kCandidates[0]); i++) {
            esp_lcd_panel_io_handle_t probe_io = nullptr;
            esp_lcd_panel_io_i2c_config_t io_cfg =
                ESP_LCD_PANEL_IO_I2C_CONFIG(kCandidates[i], nullptr, nullptr);
            io_cfg.scl_speed_hz = 100000;
            if (esp_lcd_new_panel_io_i2c(i2c_bus_, &io_cfg, &probe_io) != ESP_OK) continue;

            uint16_t pid = 0;
            bool ok = CamProbe(probe_io, 0x300A, &pid);
            esp_lcd_panel_io_del(probe_io);

            ESP_LOGI(TAG, "探测 0x%02X -> %s (PID=0x%04X)", kCandidates[i],
                     ok ? "应答" : "无应答", pid);

            if (ok && ((pid == OTTO_OV2640_PID_1) || (pid == OTTO_OV2640_PID_2))) {
                camera_type_ = OTTO_CAMERA_OV2640;
            } else if (ok && (pid == OTTO_OV3660_PID)) {
                camera_type_ = OTTO_CAMERA_OV3660;
            } else if (ok && (pid == OTTO_GC2145_PID)) {
                camera_type_ = OTTO_CAMERA_GC2145;
            } else {
                continue;
            }
            ESP_LOGI(TAG, "识别到摄像头：%s", kNames[i]);
            break;
        }

        has_camera_ = (camera_type_ != OTTO_CAMERA_NONE);
        if (!has_camera_) {
            ESP_LOGW(TAG, "未识别到任何摄像头，按无摄像头模式运行");
        }
    }

    bool InitializeCamera() {
        if (!has_camera_) return false;
        if (Camera::GetCameraConfigCount() == 0) {
            ESP_LOGW(TAG, "sdkconfig 未启用任何摄像头模型，无法创建 Camera");
            return false;
        }
        camera_ = new Camera();
        if (camera_ == nullptr) {
            has_camera_ = false;
            return false;
        }
        ESP_LOGI(TAG, "摄像头初始化完成，type=%d", (int)camera_type_);
        return true;
    }

    // ------------------------------------------------------------
    // 开机尾巴：等首帧上屏后依次干慢活，最后才让舵机归位
    // ------------------------------------------------------------
    static void BootFinishTask(void* arg) {
        static_cast<OttoRobot*>(arg)->BootAdvance();
        vTaskDelete(NULL);
    }

    void BootAdvance() {
        // 等首帧上屏（最多 3 秒兜底，防止网络异常时一直不出来）
        while (boot_phase_ == kBootEarly && boot_wait_ms_ < 3000) {
            DeviceState st = Application::GetInstance().GetDeviceState();
            if (st == kDeviceStateListening || st == kDeviceStateSpeaking ||
                st == kDeviceStateIdle || st == kDeviceStateActivating) {
                break;
            }
            vTaskDelay(pdMS_TO_TICKS(50));
            boot_wait_ms_ += 50;
        }
        boot_phase_ = kBootUiUp;

        // 1. 音频：I2S 驱动安装 + DMA 分配，之前白屏的主因
        InitializeAudioCodec();

        // 2. 摄像头探测（I2C 探测 + 复位延时，几百毫秒级）
        DetectCamera();
        if (has_camera_) {
            if (!InitializeCamera()) {
                has_camera_ = false;
            }
        }

        // 3. 舵机控制器 + MCP 工具（把选好的硬件配置传过去）
        InitializeOttoController(hw_config_);
        InitializeCustomMcpTools();
        boot_phase_ = kBootSettled;

        // 4. 画面稳了，最后才让舵机逐个归位
        vTaskDelay(pdMS_TO_TICKS(500));
        if (!asleep_) {
            OttoController_BootHomeSequence();
            ESP_LOGI(TAG, "开机舵机逐位归位完成");
        }

        if (display_ != nullptr && !screen_off_) {
            display_->SetChatMessage("system", "你好，我准备好了");
        }
    }

    // ------------------------------------------------------------
    // 软关机 / 唤醒
    // ------------------------------------------------------------
    void EnterSoftShutdown() {
        if (asleep_) return;
        asleep_ = true;
        AlarmStopAll();

        // 先给模型回话留出播报时间，再下电，避免提示音被舵机电流噪声盖住
        xTaskCreate([](void* arg) {
            auto* self = static_cast<OttoRobot*>(arg);
            vTaskDelay(pdMS_TO_TICKS(600));
            OttoController_Sleep();
            self->TurnScreenOff();
            ESP_LOGI(TAG, "软关机完成：舵机已下电、屏幕已关，语音唤醒仍可用");
            vTaskDelete(NULL);
        }, "otto_sleep", 3072, this, 3, nullptr);
    }

    void TurnScreenOff() {
        screen_off_ = true;
        if (GetBacklight() != nullptr) {
            GetBacklight()->SetBrightness(0);
        }
    }

    void TurnScreenOn() {
        screen_off_ = false;
        if (GetBacklight() != nullptr) {
            GetBacklight()->RestoreBrightness();
        }
    }

    void WakeUpFromSoftShutdown() {
        if (!asleep_ && !screen_off_) return;
        asleep_ = false;
        boot_phase_ = kBootSettled;   // 防止开机逻辑再插一脚
        TurnScreenOn();
        OttoController_Wakeup();
        if (display_ != nullptr) {
            display_->SetEmotion("neutral");
            display_->SetChatMessage("system", "我在");
        }
        ESP_LOGI(TAG, "已唤醒：屏幕亮起、舵机重新上电");
    }

    // ------------------------------------------------------------
    // 闹钟
    // ------------------------------------------------------------
    static constexpr int kAlarmRepeat = 3;      // 播报次数
    static constexpr int kAlarmTickMs = 3200;   // 每次播报间隔

    esp_timer_handle_t alarm_timer_ = nullptr;
    int alarm_left_ = 0;

    static void AlarmTimerCb(void* arg) {
        static_cast<OttoRobot*>(arg)->AlarmTick();
    }

    void AlarmTick() {
        if (alarm_left_ <= 0) {
            AlarmStopAll();
            return;
        }
        alarm_left_--;
        // 黑屏时闹钟到点先亮屏，不然用户看不见
        if (screen_off_ || asleep_) {
            WakeUpFromSoftShutdown();
        }
        auto& app = Application::GetInstance();
        app.Alert("闹钟", "主人，时间到了", "excited", "");
        OttoController_ClearQueue();
        OttoController_GentleWave();
    }

    void AlarmStart() {
        AlarmStopTimers();
        alarm_left_ = kAlarmRepeat;
        if (alarm_timer_ == nullptr) {
            esp_timer_create_args_t args = {};
            args.callback = &AlarmTimerCb;
            args.arg = this;
            args.name = "otto_alarm";
            ESP_ERROR_CHECK(esp_timer_create(&args, &alarm_timer_));
        }
        AlarmTick();   // 立刻播第一次
        ESP_ERROR_CHECK(esp_timer_start_periodic(alarm_timer_, kAlarmTickMs * 1000));
    }

    void AlarmStopTimers() {
        if (alarm_timer_ != nullptr) {
            esp_timer_stop(alarm_timer_);
        }
        alarm_left_ = 0;
    }

    void AlarmStopAll() {
        AlarmStopTimers();
        OttoController_ClearQueue();
    }

    // ------------------------------------------------------------
    // MCP 工具：闹钟 / 关机
    // ------------------------------------------------------------
    void InitializeCustomMcpTools() {
        auto& mcp = McpServer::GetInstance();

        mcp.AddTool("self", "Alarm",
            "在指定秒数后触发闹钟。到点会亮屏、弹出提醒「主人，时间到了」并温柔招手，共播报三次。"
            "参数 seconds 为多少秒后响（1~86400）。",
            [this](const PropertyList& props) -> ReturnValue {
                int seconds = 0;
                props.Get("seconds", seconds);
                if (seconds < 1) seconds = 1;
                if (seconds > 86400) seconds = 86400;
                AlarmStartWithDelay(seconds);
                // 返回空串：返回值会被模型复述，这里不需要它多说话
                return ReturnValue("");
            });
        // seconds 参数声明
        mcp.AddToolProperty("self", "Alarm", "seconds", "integer",
                            "多少秒后响铃（1~86400）", true);

        mcp.AddTool("self", "Shutdown",
            "让机器人关机：先说话告别，然后舵机归位并彻底下电、屏幕熄灭，"
            "但语音唤醒仍然可用，说唤醒词即可重新开机。",
            [this](const PropertyList& props) -> ReturnValue {
                auto& app = Application::GetInstance();
                app.Alert("关机", "好，我这就休息了", "neutral", "");
                EnterSoftShutdown();
                // 返回空串：不让模型再复述一句
                return ReturnValue("");
            });
    }

    void AlarmStartWithDelay(int seconds) {
        // 用一个一次性定时器到点再交给周期播报
        if (alarm_delay_timer_ == nullptr) {
            esp_timer_create_args_t args = {};
            args.callback = [](void* arg) {
                auto* self = static_cast<OttoRobot*>(arg);
                self->AlarmStart();
            };
            args.arg = this;
            args.name = "otto_alarm_delay";
            ESP_ERROR_CHECK(esp_timer_create(&args, &alarm_delay_timer_));
        }
        esp_timer_stop(alarm_delay_timer_);
        ESP_ERROR_CHECK(esp_timer_start_once(alarm_delay_timer_, (uint64_t)seconds * 1000000));
        ESP_LOGI(TAG, "闹钟已设：%d 秒后播报 %d 次", seconds, kAlarmRepeat);
    }

    esp_timer_handle_t alarm_delay_timer_ = nullptr;

    // ------------------------------------------------------------
    // 板级虚函数
    // ------------------------------------------------------------
    virtual AudioCodec* GetAudioCodec() override { return audio_codec_; }
    virtual Display* GetDisplay() override { return display_; }

    virtual Backlight* GetBacklight() override {
        static PwmBacklight* backlight = nullptr;
        if (backlight == nullptr) {
            backlight = new PwmBacklight(hw_config_.display_backlight_pin,
                                        DISPLAY_BACKLIGHT_OUTPUT_INVERT);
            backlight->SetBrightness(kBacklightLevel);
        }
        return backlight;
    }

    virtual Led* GetLed() override {
        static SingleLed led(BUILTIN_LED_GPIO, BUILTIN_LED_ACTIVE_LEVEL);
        return &led;
    }

    // 电量：config.h 里两套配置的 ADC 引脚都是 NC，暂无真实采样，
    // 这里保留接口并返回占位值；接上分压电阻后改成读 adc_handle_ 即可。
    virtual bool GetBatteryLevel(int& level, bool& charging, bool& discharging) override {
        const HardwareConfig& cfg = hw_config_;
        if (cfg.power_charge_detect_pin == GPIO_NUM_NC &&
            cfg.power_adc_channel == ADC_CHANNEL_1 &&
            cfg.power_adc_unit == ADC_UNIT_1) {
            charging    = false;
            discharging = true;
            level       = 100;   // 未接真实电量检测
            return true;
        }
        if (adc_handle_ == nullptr) {
            charging    = false;
            discharging = true;
            level       = 100;
            return true;
        }
        int raw = 0;
        if (adc_oneshot_read(adc_handle_, cfg.power_adc_channel, &raw) != ESP_OK) {
            return false;
        }
        float volt = (float)raw / 4095.0f * 3.3f;
        level = (int)((volt - POWER_BATTERY_EMPTY_VOLTAGE) /
                      (POWER_BATTERY_FULL_VOLTAGE - POWER_BATTERY_EMPTY_VOLTAGE) * 100.0f);
        if (level < 0) level = 0;
        if (level > 100) level = 100;
        charging    = false;
        discharging = true;
        return true;
    }

    virtual std::string GetBoardJson() override {
        return "{\"board\":\"otto-robot\",\"camera\":" + std::string(has_camera_ ? "true" : "false") +
               ",\"hw\":" + std::string(is_camera_board_ ? "camera" : "no-camera") + "}";
    }

public:
    OttoRobot() : boot_button_(BOOT_BUTTON_GPIO) {
        // 选定硬件配置（摄像头版 / 无摄像头版）
        hw_config_ = PickConfig();
        is_camera_board_ = (OTTO_HARDWARE_VERSION == OTTO_VERSION_CAMERA) ||
                           ((OTTO_HARDWARE_VERSION == OTTO_VERSION_AUTO) &&
                            (Camera::GetCameraConfigCount() > 0));

        ESP_LOGI(TAG, "硬件配置：%s", is_camera_board_ ? "摄像头版" : "无摄像头版");

        // 1. 电源管理（ADC 一路；引脚为 NC 时不影响其它初始化）
        InitializePowerManager();

        // 2. SPI + LCD + 背光：全部初始化里唯一必须阻塞的一段，
        //    因为它决定用户多久能看到画面
        InitializeSpi();
        InitializeLcdDisplay();
        if (GetBacklight() != nullptr) {
            GetBacklight()->RestoreBrightness();
        }

        // 3. 按钮
        InitializeButtons();

        // 4. 屏幕已经能显示了，剩下的慢活全部交给后台任务，
        //    等 Application 首帧上屏之后再干，避免白屏干等
        xTaskCreate(&OttoRobot::BootFinishTask, "otto_boot", 4096, this, 3, nullptr);
    }

    virtual ~OttoRobot() {
        AlarmStopTimers();
        if (alarm_delay_timer_ != nullptr) {
            esp_timer_stop(alarm_delay_timer_);
            esp_timer_delete(alarm_delay_timer_);
            alarm_delay_timer_ = nullptr;
        }
        if (alarm_timer_ != nullptr) {
            esp_timer_delete(alarm_timer_);
            alarm_timer_ = nullptr;
        }
        if (adc_handle_ != nullptr) {
            adc_oneshot_del_unit(adc_handle_);
            adc_handle_ = nullptr;
        }
        if (camera_ != nullptr) {
            delete camera_;
            camera_ = nullptr;
        }
        if (i2c_bus_ != nullptr) {
            i2c_del_master_bus(i2c_bus_);
            i2c_bus_ = nullptr;
        }
    }

    // 注意：小智固件的板级 Initialize() 不是虚函数，由 DECLARE_BOARD 直接调用，
    // 因此这里不能写 override（写了会报 "does not override"）。
    void Initialize() {
        // 屏幕已经亮着，这里不再做阻塞初始化，直接交给基类走网络与协议
        WifiBoard::Initialize();
    }

private:
    void InitializePowerManager() {
        const HardwareConfig& cfg = hw_config_;
        power_manager_ = new PowerManager(
            cfg.power_charge_detect_pin, POWER_CHARGE_DETECT_LEVEL,
            cfg.power_adc_channel, cfg.power_adc_unit);

        // ADC 只在真的配了电量引脚时才建，避免无谓占用
        if (cfg.power_adc_unit == ADC_UNIT_1) {
            adc_oneshot_unit_init_cfg_t init_cfg = {};
            init_cfg.unit_id = cfg.power_adc_unit;
            if (adc_oneshot_new_unit(&init_cfg, &adc_handle_) == ESP_OK) {
                adc_oneshot_chan_cfg_t chan_cfg = {};
                chan_cfg.atten = (adc_atten_t)POWER_BATTERY_LEVEL_ATTEN;
                chan_cfg.bitwidth = ADC_BITWIDTH_12;
                adc_oneshot_config_channel(adc_handle_, cfg.power_adc_channel, &chan_cfg);
            }
        }
    }
};

DECLARE_BOARD(OttoRobot);
