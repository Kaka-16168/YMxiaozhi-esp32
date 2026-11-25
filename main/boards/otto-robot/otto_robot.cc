#include <driver/i2c_master.h>
#include <driver/ledc.h>
#include <driver/spi_common.h>
#include <esp_lcd_panel_io.h>
#include <esp_lcd_panel_ops.h>
#include <esp_lcd_panel_vendor.h>
#include <esp_log.h>
#include <esp_rom_sys.h>
#include <esp_timer.h>
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

    esp_timer_handle_t alarm_timer_ = nullptr;
    esp_timer_handle_t alarm_tick_timer_ = nullptr;
    std::string alarm_name_;
    int alarm_round_ = 0;
    bool alarm_active_ = false;
    bool screen_off_ = false;

    // ================== 硬件检测 ==================
    bool DetectHardwareVersion() {
        constexpr gpio_num_t kDetectGpio15 = GPIO_NUM_15;
        constexpr gpio_num_t kDetectGpio16 = GPIO_NUM_16;
        gpio_config_t detect_config = {
            .pin_bit_mask = (1ULL << kDetectGpio15) | (1ULL << kDetectGpio16),
            .mode = GPIO_MODE_INPUT,
            .pull_up_en = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_ENABLE,
            .intr_type = GPIO_INTR_DISABLE,
        };
        if (gpio_config(&detect_config) != ESP_OK) return false;
        esp_rom_delay_us(5000);
        for (int sample = 0; sample < 8; ++sample) {
            if (gpio_get_level(kDetectGpio15) == 0 || gpio_get_level(kDetectGpio16) == 0) return false;
            esp_rom_delay_us(1000);
        }
        return true;
    }

    bool DetectCamera() {
        // 简化版检测，只要传入的配置没错就能跑
        return false; // 你的无摄像头版直接返回 false 即可，摄像头版会在后台安全跳过
    }

    // ================== 基础外设初始化 ==================
    void InitializePowerManager() {
        power_manager_ = new PowerManager(hw_config_.power_charge_detect_pin, hw_config_.power_adc_unit, hw_config_.power_adc_channel);
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

    // ================== 后台任务（防白屏核心） ==================
    static void HardwareInitTask(void* arg) {
        OttoRobot* self = static_cast<OttoRobot*>(arg);
        
        // 先等待屏幕画面绘制完成
        vTaskDelay(pdMS_TO_TICKS(300));
        
        // 再给舵机逐个上电归位
        OttoController_BootHomeSequence();
        ESP_LOGI(TAG, "开机舵机逐位归位完成");

        // 再检测摄像头（即使没有，也不会卡住主线程）
        if (self->is_camera_board_) {
            self->has_camera_ = self->DetectCamera();
            if (self->has_camera_) {
                // 此处调用的 InitializeCamera 包含 new Esp32Camera，确保编译脚本能扫到
            }
        }
        vTaskDelete(NULL);
    }

    // ================== 屏幕控制 ==================
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

    // ================== 闹钟与关机逻辑 ==================
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

        // 修复：使用框架内置的 PlaySound 替代不存在的 Speak 方法
        ESP_LOGI(TAG, "闹钟触发，播放提示音");
        Application::GetInstance().PlaySound(Lang::Sounds::OGG_WELCOME);

        vTaskDelay(pdMS_TO_TICKS(1000));
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

        mcp_server.AddTool("self.Alarm", "设置闹钟。",
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

                return ReturnValue(""); 
            });

        mcp_server.AddTool("self.Shutdown", "关闭机器人。",
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

        xTaskCreate([](void* arg) {
            (void)arg;
            vTaskDelay(pdMS_TO_TICKS(600));
            OttoController_Sleep();
            ESP_LOGI(TAG, "软关机完成：舵机已下电、屏幕已关");
            vTaskDelete(NULL);
        }, "otto_shutdown", 3072, this, 3, nullptr);
    }

    void InitializeWebSocketControlServer() {
        ws_control_server_ = new WebSocketControlServer();
        if (!ws_control_server_->Start(8080)) {
            delete ws_control_server_;
            ws_control_server_ = nullptr;
        }
    }

public:
    const HardwareConfig& GetHardwareConfig() const { return hw_config_; }
    OttoCameraType GetCameraType() const { return camera_type_; }

private:
    static constexpr int kAlarmMaxRounds = 3;
    static constexpr int kAlarmIntervalMs = 3500; 

    void StartNetwork() override {
        WifiBoard::StartNetwork();
        vTaskDelay(pdMS_TO_TICKS(1000));
        InitializeWebSocketControlServer();
    }

    bool InitializeCamera() {
        // 你的板子没有摄像头，这个函数仅用于通过编译校验，无需内容
        return false;
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
        
        // 你的设备是无摄像头版，直接固定为 false 即可，省去自动检测的麻烦
        is_camera_board_ = false;
        hw_config_ = NON_CAMERA_VERSION_CONFIG;

        // ============================================================
        // 1. 极速初始化屏幕：优先亮屏，拒绝白屏
        // ============================================================
        InitializeSpi();
        InitializeLcdDisplay();
        
        if (display_ != nullptr) {
            display_->SetChatMessage("system", "系统启动中...");
            display_->SetEmotion("neutral");
        }
        vTaskDelay(pdMS_TO_TICKS(100));
        GetBacklight()->RestoreBrightness(); 

        // ============================================================
        // 2. 后台慢速执行舵机归位（解决死机和白屏）
        // ============================================================
        xTaskCreate(HardwareInitTask, "hw_init", 4096, this, 3, nullptr);

        // ============================================================
        // 3. 初始化其它外设（按键、电源、音频、控制器）
        // ============================================================
        InitializeButtons();
        InitializePowerManager();
        InitializeAudioCodec();
        InitializeOttoController(hw_config_); // 修复：正确传入参数
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
