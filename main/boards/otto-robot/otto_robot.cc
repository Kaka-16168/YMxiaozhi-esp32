#include "otto_emoji_display.h"

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
    LcdDisplay* display_ = nullptr;
    PowerManager* power_manager_ = nullptr;
    Button boot_button_;
    WebSocketControlServer* ws_control_server_ = nullptr;
    HardwareConfig hw_config_;
    AudioCodec* audio_codec_ = nullptr;
    i2c_master_bus_handle_t i2c_bus_ = nullptr;
    Camera* camera_ = nullptr;
    bool is_camera_board_ = false;
    bool has_camera_ = false;
    OttoCameraType camera_type_ = OTTO_CAMERA_NONE;

    // ======== 闹钟相关 ========
    esp_timer_handle_t alarm_timer_ = nullptr;      // 到点定时器（单次）
    esp_timer_handle_t alarm_tick_timer_ = nullptr; // 播报节拍定时器（周期）
    std::string alarm_name_;
    int  alarm_round_ = 0;      // 已播报轮次
    bool alarm_active_ = false; // 闹钟是否处于播报中
    bool screen_off_ = false;   // 屏幕是否处于软关机黑屏态

    // ======== 开机阶段 ========
    // kBootEarly   : 构造函数阶段，只把屏幕点起来
    // kBootUiUp    : 首帧已上屏，可以开始做重活（音频/摄像头/舵机）
    // kBootSettled : 全部就绪，舵机逐个归位
    enum BootPhase { kBootEarly, kBootUiUp, kBootSettled };
    BootPhase boot_phase_ = kBootEarly;
    int       boot_wait_ms_ = 0;

    // OV2640 有两个可能的 PID，抽成小函数避免长条件写错
    static bool pid_is_ov2640(uint16_t pid) {
        return pid == OTTO_OV2640_PID_1 || pid == OTTO_OV2640_PID_2;
    }

    // ------------------------------------------------------------
    // 屏幕
    // ------------------------------------------------------------
    void TurnScreenOff() {
        screen_off_ = true;
        if (GetBacklight() != nullptr) {
            GetBacklight()->SetBrightness(0);
        }
        if (display_ != nullptr) {
            display_->SetChatMessage("system", "");
        }
    }

    void TurnScreenOn() {
        screen_off_ = false;
        if (GetBacklight() != nullptr) {
            GetBacklight()->RestoreBrightness();
        }
    }

    // ------------------------------------------------------------
    // SPI / LCD
    // ------------------------------------------------------------
    void InitializeSpi() {
        spi_bus_config_t buscfg = {};
        buscfg.mosi_io_num = DISPLAY_MOSI_PIN;
        buscfg.miso_io_num = GPIO_NUM_NC;
        buscfg.sclk_io_num = DISPLAY_CLK_PIN;
        buscfg.quadwp_io_num = GPIO_NUM_NC;
        buscfg.quadhd_io_num = GPIO_NUM_NC;
        buscfg.max_transfer_sz = DISPLAY_WIDTH * DISPLAY_HEIGHT * sizeof(uint16_t);
        ESP_ERROR_CHECK(spi_bus_initialize(DISPLAY_SPI_HOST, &buscfg, SPI_DMA_CH_AUTO));
    }

    void InitializeLcdDisplay() {
        esp_lcd_panel_io_handle_t panel_io = nullptr;
        esp_lcd_panel_handle_t panel = nullptr;

        esp_lcd_panel_io_spi_config_t io_config = {};
        io_config.cs_gpio_num = DISPLAY_CS_PIN;
        io_config.dc_gpio_num = DISPLAY_DC_PIN;
        io_config.spi_mode = 0;
        io_config.pclk_hz = 40 * 1000 * 1000;
        io_config.trans_queue_depth = 10;
        io_config.lcd_cmd_bits = 8;
        io_config.lcd_param_bits = 8;
        ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi(DISPLAY_SPI_HOST, &io_config, &panel_io));

        esp_lcd_panel_dev_config_t panel_config = {};
        panel_config.reset_gpio_num = DISPLAY_RST_PIN;
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

    // ------------------------------------------------------------
    // 音频
    // ------------------------------------------------------------
    void InitializeAudioCodec() {
        int64_t t0 = esp_timer_get_time();
        audio_codec_ = new NoAudioCodecDuplex(
            AUDIO_I2S_GPIO_BCLK, AUDIO_I2S_GPIO_WS,
            AUDIO_I2S_GPIO_DIN, AUDIO_I2S_GPIO_DOUT,
            AUDIO_I2S_GPIO_MCLK, AUDIO_SAMPLE_RATE, AUDIO_CODEC_BITS_PER_SAMPLE);
        ESP_LOGI(TAG, "AudioCodec 初始化耗时 %lld ms", (esp_timer_get_time() - t0) / 1000);
    }

    // ------------------------------------------------------------
    // 摄像头（慢活，推迟到首帧之后）
    // ------------------------------------------------------------
    static bool CamProbe(esp_lcd_panel_io_handle_t io, uint16_t reg, uint16_t* pid) {
        uint8_t hi = 0, lo = 0;
        if (esp_lcd_panel_io_rx_param(io, (int)reg, &hi, 1) != ESP_OK) return false;
        if (esp_lcd_panel_io_rx_param(io, (int)reg + 1, &lo, 1) != ESP_OK) return false;
        *pid = (uint16_t)((hi << 8) | lo);
        return true;
    }

    void DetectCamera() {
        if (Camera::GetCameraConfigCount() == 0) {
            ESP_LOGW(TAG, "未配置任何摄像头，跳过探测");
            has_camera_ = false;
            return;
        }

        const int sio_c = CAMERA_SIOD_PIN;
        const int sio_d = CAMERA_SIOC_PIN;
        i2c_master_bus_config_t i2c_cfg = {};
        i2c_cfg.i2c_port = I2C_NUM_1;
        i2c_cfg.sda_io_num = sio_d;
        i2c_cfg.scl_io_num = sio_c;
        i2c_cfg.clk_source = I2C_CLK_SRC_DEFAULT;
        i2c_cfg.glitch_ignore_cnt = 7;
        i2c_cfg.flags.enable_internal_pullup = true;
        if (i2c_new_master_bus(&i2c_cfg, &i2c_bus_) != ESP_OK) {
            ESP_LOGE(TAG, "摄像头 I2C 总线创建失败");
            has_camera_ = false;
            return;
        }

        static const uint16_t kCandidates[] = {0x2640, 0x2642, 0x3660, 0x2145};
        static const char*    kNames[]      = {"OV2640", "OV2640(alt)", "OV3660", "GC2145"};

        for (size_t i = 0; i < sizeof(kCandidates) / sizeof(kCandidates[0]); i++) {
            esp_lcd_panel_io_handle_t probe_io = nullptr;
            esp_lcd_panel_io_i2c_config_t io_cfg = ESP_LCD_PANEL_IO_I2C_CONFIG(
                kCandidates[i], nullptr, nullptr);
            io_cfg.scl_speed_hz = 100000;
            if (esp_lcd_new_panel_io_i2c(i2c_bus_, &io_cfg, &probe_io) != ESP_OK) continue;

            uint16_t pid = 0;
            bool ok = CamProbe(probe_io, 0x300A, &pid);
            esp_lcd_panel_io_del(probe_io);

            ESP_LOGI(TAG, "探测 0x%02X -> %s (PID=0x%04X)", kCandidates[i],
                     ok ? "应答" : "无应答", pid);

            if (ok && pid_is_ov2640(pid)) {
                camera_type_ = OTTO_CAMERA_OV2640;
                has_camera_  = true;
                break;
            }
            if (ok && pid == 0x3660) {
                camera_type_ = OTTO_CAMERA_OV3660;
                has_camera_  = true;
                break;
            }
            if (ok && pid == 0x2145) {
                camera_type_ = OTTO_CAMERA_GC2145;
                has_camera_  = true;
                break;
            }
            // GC2145 的 PID 寄存器布局与 OV 不同，0x300A 读不到时按型号兜底
            if (!ok && kCandidates[i] == 0x2145) {
                camera_type_ = OTTO_CAMERA_GC2145;
                has_camera_  = true;
                break;
            }
            (void)kNames;
        }

        ESP_LOGI(TAG, "摄像头探测结果: %s", has_camera_ ? "已发现" : "未发现");
    }

    bool InitializeCamera() {
        esp_cam_sensor_config_t cam_config{};
        cam_config.scl_io_num = (gpio_num_t)CAMERA_SIOC_PIN;
        cam_config.sda_io_num = (gpio_num_t)CAMERA_SIOD_PIN;
        cam_config.detect_pixel_format = 0;
        cam_config.fb_location = CAMERA_FB_IN_PSRAM;
        cam_config.magnification = 0;

        camera_ = new Esp32Camera(cam_config);
        if (!camera_->Start()) {
            ESP_LOGE(TAG, "摄像头启动失败");
            delete camera_;
            camera_ = nullptr;
            return false;
        }
        ESP_LOGI(TAG, "摄像头启动成功");
        return true;
    }

    // ------------------------------------------------------------
    // 开机尾巴：首帧上屏之后再干重活
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

        // 3. 舵机控制器 + MCP 工具
        InitializeOttoController();
        InitializeCustomMcpTools();
        boot_phase_ = kBootSettled;

        // 4. 画面稳了，最后才让舵机逐个归位
        vTaskDelay(pdMS_TO_TICKS(500));
        OttoController_BootHomeSequence();
        ESP_LOGI(TAG, "开机舵机逐位归位完成");

        if (display_ != nullptr && !screen_off_) {
            display_->SetChatMessage("system", "你好，我准备好了");
        }
    }

    // ============================================================
    // 闹钟：播报 3 次「主人，时间到了」+ 温柔招手
    // 用周期定时器驱动状态机，全程不阻塞任何任务
    // ============================================================
    void AlarmStopTimers() {
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
    }

    // 收尾：播够 3 次或被用户打断
    void StopAlarmSequence(bool by_user) {
        AlarmStopTimers();
        bool was_active = alarm_active_;
        alarm_active_ = false;
        alarm_round_  = 0;

        OttoController_ClearQueue();  // 丢掉没播完的招手
        if (was_active && !by_user) {
            OttoController_GentleWave();  // 最后一轮让身体回到舒展姿态
        }
        if (by_user) {
            ESP_LOGI(TAG, "闹钟被打断");
        } else {
            ESP_LOGI(TAG, "闹钟播报结束");
        }
    }

    static void AlarmTickCb(void* arg) {
        static_cast<OttoRobot*>(arg)->AlarmTick();
    }

    void AlarmTick() {
        if (!alarm_active_) {
            return;
        }
        alarm_round_++;
        if (alarm_round_ > 3) {
            StopAlarmSequence(false);
            return;
        }

        // 黑屏时先亮屏，不然用户看不见
        if (screen_off_) {
            TurnScreenOn();
        }

        auto& app = Application::GetInstance();
        // ★ 闹钟播报入口 ★
        // 屏幕弹「主人，时间到了」+ 表情，这是本地弹窗，不需要服务端 TTS
        app.Alert("闹钟", alarm_name_.empty() ? "主人，时间到了" : alarm_name_.c_str(),
                  "excited", "");
        ESP_LOGI(TAG, "闹钟播报第 %d/3 次", alarm_round_);

        // 700ms 后招手，让"提醒"和"动作"错开，观感更像互动
        vTaskDelay(pdMS_TO_TICKS(700));
        if (alarm_active_) {
            OttoController_GentleWave();
        }
    }

    static void AlarmStartCb(void* arg) {
        static_cast<OttoRobot*>(arg)->AlarmStart();
    }

    void AlarmStart() {
        ESP_LOGI(TAG, "闹钟到点，开始播报");
        alarm_active_ = true;
        alarm_round_  = 0;

        esp_timer_create_args_t args = {};
        args.callback = &OttoRobot::AlarmTickCb;
        args.arg      = this;
        args.dispatch_method = ESP_TIMER_TASK;
        args.name     = "otto_alarm_tick";
        if (esp_timer_create(&args, &alarm_tick_timer_) != ESP_OK) {
            ESP_LOGE(TAG, "闹钟节拍定时器创建失败");
            alarm_active_ = false;
            return;
        }
        // 立刻先播一次，之后每 3.2 秒一轮
        AlarmTick();
        if (alarm_active_) {
            esp_timer_start_periodic(alarm_tick_timer_, 3200000ULL);
        }
    }

    // ============================================================
    // 软关机 / 唤醒
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
            // 等语音回话的提示音播完，避免舵机电流噪声盖住声音
            vTaskDelay(pdMS_TO_TICKS(600));
            OttoController_Sleep();
            ESP_LOGI(TAG, "软关机完成：舵机已下电、屏幕已关，等待语音唤醒");
            vTaskDelete(NULL);
        }, "otto_shutdown", 3072, nullptr, 3, nullptr);
    }

    // ============================================================
    // 唤醒：舵机重新上电 + 自动亮屏
    // 软关机期间 MCU 和麦克风一直在跑，唤醒词照样能触发；
    // 语音链路本身由 Application 处理，这里只负责"把灯打开、把舵机叫醒"。
    // ============================================================
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
        }

        // 3. 舵机重新上电并慢速归位（独立任务，不阻塞按键回调）
        xTaskCreate([](void* arg) {
            (void)arg;
            OttoController_Wakeup();
            ESP_LOGI(TAG, "唤醒完成，屏幕已亮、舵机已上电");
            vTaskDelete(NULL);
        }, "otto_wakeup", 3072, nullptr, 3, nullptr);
    }

    // ============================================================
    // MCP 工具：闹钟 / 关机
    // ============================================================
    void InitializeCustomMcpTools() {
        auto& mcp_server = McpServer::GetInstance();

        // 1. 闹钟：到点后屏幕弹「主人，时间到了」+ 温柔招手，连播 3 次
        mcp_server.AddTool(
            "self.Alarm",
            "设置一个闹钟。到点后屏幕会弹出提醒「主人，时间到了」并温柔招手互动，"
            "连续提醒 3 次，期间说任何话或通过按键都会立刻停止。",
            PropertyList({
                Property("seconds", kPropertyTypeInteger, 10, 1, 86400),
                Property("name", kPropertyTypeString, "主人，时间到了"),
            }),
            [this](const PropertyList& properties) -> ReturnValue {
                int seconds = properties["seconds"].value<int>();
                if (properties.count("name") && !properties["name"].value<std::string>().empty()) {
                    alarm_name_ = properties["name"].value<std::string>();
                } else {
                    alarm_name_ = "主人，时间到了";
                }

                // 已有闹钟在跑就先撤掉
                AlarmStopTimers();
                alarm_active_ = false;
                alarm_round_  = 0;

                esp_timer_create_args_t args = {};
                args.callback = &OttoRobot::AlarmStartCb;
                args.arg      = this;
                args.dispatch_method = ESP_TIMER_TASK;
                args.name     = "otto_alarm";
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
            "让机器人进入省电休眠：舵机归位并彻底下电、屏幕关闭，但语音唤醒始终可用，"
            "说唤醒词或按开机键即可立刻恢复。",
            PropertyList({}),
            [this](const PropertyList&) -> ReturnValue {
                EnterSoftShutdown();
                return ReturnValue("");   // 空返回：不让模型复述
            });
    }

    // ------------------------------------------------------------
    // 板级虚函数
    // ------------------------------------------------------------
    virtual AudioCodec* GetAudioCodec() override { return audio_codec_; }
    virtual Display* GetDisplay() override { return display_; }

    virtual Backlight* GetBacklight() override {
        static PwmBacklight* backlight = nullptr;
        if (backlight == nullptr) {
            backlight = new PwmBacklight(DISPLAY_BACKLIGHT_PIN, DISPLAY_BACKLIGHT_OUTPUT_INVERT);
            backlight->SetBrightness(80);
        }
        return backlight;
    }

    virtual Led* GetLed() override {
        static SingleLed led(BUILTIN_LED_GPIO, BUILTIN_LED_ACTIVE_LEVEL);
        return &led;
    }

    virtual bool GetBatteryLevel(int& level, bool& charging, bool& discharging) override {
        charging    = false;
        discharging = true;
        level       = 100;   // 暂未接真实电量检测
        return true;
    }

    virtual std::string GetBoardJson() override {
        return "{\"board\":\"otto-robot\",\"camera\":" + std::string(has_camera_ ? "true" : "false") + "}";
    }

public:
    OttoRobot() : boot_button_(BOOT_BUTTON_GPIO) {
        // 1. 电源管理（必须最先，很多板级初始化依赖它）
        InitializePowerManager();

        // 2. SPI + LCD + 背光：这一小段是全部初始化里唯一必须阻塞的部分，
        //    因为它决定了用户多久能看到画面
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
        if (camera_ != nullptr) {
            delete camera_;
            camera_ = nullptr;
        }
        if (i2c_bus_ != nullptr) {
            i2c_del_master_bus(i2c_bus_);
            i2c_bus_ = nullptr;
        }
    }

    virtual void Initialize() override {
        // 屏幕已经亮着，这里不再做阻塞初始化，直接交给基类走网络与协议
        WifiBoard::Initialize();
    }

private:
    void InitializePowerManager() {
        power_manager_ = new PowerManager(
            POWER_CHARGE_DETECT_PIN, POWER_CHARGE_DETECT_LEVEL,
            POWER_BATTERY_LEVEL_PIN, POWER_BATTERY_LEVEL_CHANNEL,
            POWER_BATTERY_LEVEL_ATTEN, POWER_BATTERY_FULL_VOLTAGE,
            POWER_BATTERY_EMPTY_VOLTAGE);
    }
};

DECLARE_BOARD(OttoRobot);
