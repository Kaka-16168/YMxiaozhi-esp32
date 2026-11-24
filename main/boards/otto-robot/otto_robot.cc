#include <driver/i2c_master.h>
#include <driver/ledc.h>
#include <driver/spi_common.h>
#include <esp_lcd_panel_io.h>
#include <esp_lcd_panel_ops.h>
#include <esp_lcd_panel_vendor.h>
#include <esp_log.h>
#include <esp_rom_sys.h>
#include <esp_timer.h>

#include <new>
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
#include "otto_emoji_display.h"
#include "power_manager.h"
#include "system_reset.h"
#include "websocket_control_server.h"
#include "wifi_board.h"
#include "assets/lang_config.h"

#define TAG "OttoRobot"

extern void InitializeOttoController(const HardwareConfig& hw_config);

extern "C" {
void OttoController_BootHomeSequence(void);
void OttoController_GentleWave(void);
void OttoController_Sleep(void);
void OttoController_Wakeup(void);
bool OttoController_IsBusy(void);
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

    // 闹钟相关
    esp_timer_handle_t alarm_timer_ = nullptr;
    esp_timer_handle_t alarm_tick_timer_ = nullptr;
    std::string alarm_name_;
    int alarm_round_ = 0;
    bool alarm_active_ = false;
    bool screen_off_ = false;

    static constexpr int kAlarmMaxRounds = 3;
    static constexpr int kAlarmIntervalMs = 3200;

    // 移除了危险的 GPIO15/16 硬件检测，改为在构造函数中强依赖宏定义
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
            if (i2c_master_transmit_receive(dev_handle, &reg_high, 1, &pid_high, 1, 200) == ESP_OK &&
                i2c_master_transmit_receive(dev_handle, &reg_low, 1, &pid_low, 1, 200) == ESP_OK) {
                uint16_t pid = (pid_high << 8) | pid_low;
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
        if (ret != ESP_OK) return false;

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
        if (ret != ESP_OK) return false;

        vTaskDelay(pdMS_TO_TICKS(100));
        i2c_master_bus_config_t i2c_bus_cfg = {
            .i2c_port = I2C_NUM_0,
            .sda_io_num = CAMERA_VERSION_CONFIG.i2c_sda_pin,
            .scl_io_num = CAMERA_VERSION_CONFIG.i2c_scl_pin,
            .clk_source = I2C_CLK_SRC_DEFAULT,
            .glitch_ignore_cnt = 7,
            .intr_priority = 0,
            .trans_queue_depth = 0,
            .flags = { .enable_internal_pullup = 1 },
        };

        ret = i2c_new_master_bus(&i2c_bus_cfg, &i2c_bus_);
        if (ret != ESP_OK) {
            ledc_stop(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL, 0);
            return false;
        }

        const uint8_t camera_addresses[] = {0x30, 0x3C, 0x21, 0x60};
        bool camera_found = false;
        uint16_t detected_pid = 0;

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
                        camera_found = true;
                        i2c_master_bus_rm_device(dev_handle);
                        break;
                    }
                }
                i2c_master_bus_rm_device(dev_handle);
            }
        }

        if (!camera_found) {
            uint16_t gc2145_pid = 0;
            if (DetectGc2145Camera(gc2145_pid)) {
                detected_pid = gc2145_pid;
                camera_found = true;
            }
        }

        if (!camera_found) {
            i2c_del_master_bus(i2c_bus_);
            i2c_bus_ = nullptr;
            ledc_stop(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL, 0);
            camera_type_ = OTTO_CAMERA_NONE;
        } else {
            camera_type_ = OTTO_CAMERA_UNKNOWN; // 这里可根据PID细化
        }
        return camera_found;
    }

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
        io_config.pclk_hz = 20 * 1000 * 1000; // 如果花屏可降至 10M
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

    static void BootHomeTask(void* arg) {
        OttoRobot* self = static_cast<OttoRobot*>(arg);
        vTaskDelay(pdMS_TO_TICKS(200)); // 等屏幕出图
        OttoController_BootHomeSequence();
        ESP_LOGI(TAG, "开机舵机逐位归位完成");
        vTaskDelete(NULL);
    }

    void TurnScreenOn() {
        if (GetBacklight() != nullptr) GetBacklight()->RestoreBrightness();
        screen_off_ = false;
    }

    void TurnScreenOff() {
        if (display_ != nullptr) {
            display_->SetChatMessage("system", "");
            display_->SetEmotion("neutral");
        }
        if (GetBacklight() != nullptr) GetBacklight()->SetBrightness(0);
        screen_off_ = true;
    }

public:
    void WakeUpFromSoftShutdown() {
        if (!screen_off_) return;
        ESP_LOGI(TAG, "从软关机唤醒");

        if (alarm_active_) StopAlarmSequence(true);

        TurnScreenOn();
        if (display_ != nullptr) {
            display_->SetEmotion("happy");
            display_->SetChatMessage("system", "我在");
        }

        OttoController_Wakeup();
        Application::GetInstance().PlaySound(Lang::Sounds::OGG_WELCOME);
    }

private:
    static void AlarmTickCallback(void* arg) {
        auto* self = static_cast<OttoRobot*>(arg);
        if (self != nullptr) self->AlarmTick();
    }

    void AlarmTick() {
        if (!alarm_active_) return;
        if (screen_off_) TurnScreenOn();

        alarm_round_++;
        if (alarm_round_ > kAlarmMaxRounds) {
            StopAlarmSequence(true);
            return;
        }

        if (display_ != nullptr) {
            display_->SetEmotion("excited");
            display_->SetChatMessage("system", alarm_name_.empty() ? "时间到了" : alarm_name_.c_str());
        }

        Application::GetInstance().PlaySound(Lang::Sounds::OGG_WELCOME);
        vTaskDelay(pdMS_TO_TICKS(700));
        OttoController_GentleWave();
    }

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
            OttoController_ClearQueue();
            if (display_ != nullptr) {
                display_->SetEmotion("neutral");
                display_->SetChatMessage("system", "");
            }
        }
    }

    static void AlarmStartCallback(void* arg) {
        auto* self = static_cast<OttoRobot*>(arg);
        if (self == nullptr) return;
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
            self->alarm_active_ = false;
            return;
        }
        esp_timer_start_periodic(self->alarm_tick_timer_, (uint64_t)kAlarmIntervalMs * 1000ULL);
    }

    void InitializeCustomMcpTools() {
        auto& mcp_server = McpServer::GetInstance();

        mcp_server.AddTool("self.Alarm", "设置闹钟。到点后语音提示，并招手互动。",
            PropertyList({
                Property("seconde_from_now", kPropertyTypeInteger, 10, 0, 86400),
                Property("alarm_name", kPropertyTypeString),
            }),
            [this](const PropertyList& properties) -> ReturnValue {
                int seconds = properties["seconde_from_now"].value<int>();
                std::string name = properties["alarm_name"].value<std::string>();

                if (seconds <= 0) return ReturnValue("闹钟时间必须大于 0 秒");
                alarm_name_ = name.empty() ? "时间到了" : name;
                StopAlarmSequence(false);

                esp_timer_create_args_t args = {};
                args.callback = &OttoRobot::AlarmStartCallback;
                args.arg = this;
                args.dispatch_method = ESP_TIMER_TASK;
                args.name = "otto_alarm";

                if (esp_timer_create(&args, &alarm_timer_) != ESP_OK) return ReturnValue("闹钟创建失败");
                esp_timer_start_once(alarm_timer_, (uint64_t)seconds * 1000000ULL);
                return ReturnValue(""); // 防止模型废话
            });

        mcp_server.AddTool("self.Shutdown", "关闭机器人。舵机归位断电，屏幕关闭。",
            PropertyList(),
            [this](const PropertyList& properties) -> ReturnValue {
                EnterSoftShutdown();
                return ReturnValue("");
            });
    }

    void EnterSoftShutdown() {
        ESP_LOGI(TAG, "进入软关机序列");
        StopAlarmSequence(false);
        TurnScreenOff();

        // 注意：这里的 3072 可能不够，如果底层调用了复杂的串口打印，建议加到 4096
        xTaskCreate([](void* arg) {
            OttoRobot* self = static_cast<OttoRobot*>(arg);
            vTaskDelay(pdMS_TO_TICKS(600)); // 等语音播完
            OttoController_Sleep();
            ESP_LOGI(TAG, "软关机完成：舵机已下电、屏幕已关");
            vTaskDelete(NULL);
        }, "otto_shutdown", 4096, this, 3, nullptr); 
    }

public:
    const HardwareConfig& GetHardwareConfig() const { return hw_config_; }
    OttoCameraType GetCameraType() const { return camera_type_; }

private:
    void InitializeWebSocketControlServer() {
        ws_control_server_ = new WebSocketControlServer();
        if (!ws_control_server_->Start(8080)) {
            delete ws_control_server_;
            ws_control_server_ = nullptr;
        }
    }

    void StartNetwork() override {
        WifiBoard::StartNetwork();
        vTaskDelay(pdMS_TO_TICKS(1000));
        InitializeWebSocketControlServer();
    }

    bool InitializeCamera() {
        if (!has_camera_ || i2c_bus_ == nullptr) return false;

        i2c_del_master_bus(i2c_bus_);
        i2c_bus_ = nullptr;
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
            camera_->SetHMirror(false);
            camera_->SetVFlip(false);
        }
        return camera_ != nullptr;
    }

public:
    OttoRobot()
        : boot_button_(BOOT_BUTTON_GPIO) {
        
        // ============================================================
        // 修复：直接使用宏定义强制指定版本，彻底避开 GPIO15/16 冲突
        // ============================================================
#if OTTO_HARDWARE_VERSION == OTTO_VERSION_AUTO
        // 如果用户没定义版本，默认当作无摄像头版（因为你已经硬接了无摄像头版的引脚）
        is_camera_board_ = false;
        ESP_LOGI(TAG, "未指定版本，默认使用无摄像头版本配置");
#elif OTTO_HARDWARE_VERSION == OTTO_VERSION_CAMERA
        is_camera_board_ = true;
        ESP_LOGI(TAG, "强制使用摄像头版本配置");
#elif OTTO_HARDWARE_VERSION == OTTO_VERSION_NO_CAMERA
        is_camera_board_ = false;
        ESP_LOGI(TAG, "强制使用无摄像头版本配置");
#endif

        if (is_camera_board_) {
            hw_config_ = CAMERA_VERSION_CONFIG;
        } else {
            hw_config_ = NON_CAMERA_VERSION_CONFIG;
        }

        InitializeSpi();
        InitializeLcdDisplay();
        ws_control_server_ = nullptr;
        GetBacklight()->RestoreBrightness();

        xTaskCreate(BootHomeTask, "otto_boot", 4096, this, 3, nullptr); 

        InitializeButtons();
        InitializePowerManager();
        InitializeAudioCodec();

        if (is_camera_board_) {
            has_camera_ = DetectCamera();
            if (!has_camera_) {
                ESP_LOGW(TAG, "摄像头版未检测到摄像头，将跳过摄像头初始化");
            }
        }
        if (has_camera_) {
            if (!InitializeCamera()) {
                has_camera_ = false;
            }
        }

        InitializeOttoController();
        InitializeCustomMcpTools();
    }

    virtual AudioCodec* GetAudioCodec() override { return audio_codec_; }
    virtual Display* GetDisplay() override { return display_; }

    virtual Backlight* GetBacklight() override {
        static PwmBacklight* backlight = nullptr;
        if (backlight == nullptr) {
            backlight = new PwmBacklight(hw_config_.display_backlight_pin, DISPLAY_BACKLIGHT_OUTPUT_INVERT);
        }
        return backlight;
    }

    bool GetBatteryLevel(int& level, bool& charging, bool& discharging) override {
        level = 100;
        charging = false;
        discharging = true; 
        return true;
    }

    virtual Camera* GetCamera() override { return has_camera_ ? camera_ : nullptr; }
};

DECLARE_BOARD(OttoRobot);
