#include <cJSON.h>
#include <esp_log.h>
#include <cstdlib>
#include <cstring>

#include "application.h"
#include "board.h"
#include "config.h"
#include "mcp_server.h"
#include "otto_movements.h"
#include "power_manager.h"
#include "sdkconfig.h"
#include "settings.h"
#include <wifi_manager.h>

#define TAG "OttoController"

// 动作限速与温柔化参数
static const int kServoLimitDegPerSec = 90;    // 限制舵机角速度，大幅降低启动电流
static const int kBootStaggerMs = 350;         // 开机各舵机逐个上电的间隔
static const int kBootServoSpeedMs = 2000;     // 开机单舵机缓慢归位时间
static const int kHomeSettleSpeedMs = 1800;    // 收尾归位时间

static const int kGaitSpeedMin = 1500;         // 动作最小耗时（1500ms，强制放慢）
static const int kGaitSpeedMax = 3000;
static const int kGaitAmountMax = 25;          // 动作幅度限制
static const int kStepCountMax = 3;            // 单次动作步数上限

class OttoController {
private:
    Otto otto_;
    TaskHandle_t action_task_handle_ = nullptr;
    QueueHandle_t action_queue_;
    bool has_hands_ = false;
    bool is_action_in_progress_ = false;
    bool is_asleep_ = false;

    struct OttoActionParams {
        int action_type;
        int steps;
        int speed;
        int direction;
        int amount;
        char servo_sequence_json[512];
    };

    enum ActionType {
        ACTION_WALK = 1,
        ACTION_TURN = 2,
        ACTION_JUMP = 3,
        ACTION_SWING = 4,
        ACTION_MOONWALK = 5,
        ACTION_BEND = 6,
        ACTION_SHAKE_LEG = 7,
        ACTION_RADIO_CALISTHENICS = 26,
        ACTION_MAGIC_CIRCLE = 27,
        ACTION_UPDOWN = 8,
        ACTION_TIPTOE_SWING = 9,
        ACTION_JITTER = 10,
        ACTION_ASCENDING_TURN = 11,
        ACTION_CRUSAITO = 12,
        ACTION_FLAPPING = 13,
        ACTION_HANDS_UP = 14,
        ACTION_HANDS_DOWN = 15,
        ACTION_HAND_WAVE = 16,
        ACTION_WINDMILL = 20,
        ACTION_TAKEOFF = 21,
        ACTION_FITNESS = 22,
        ACTION_GREETING = 23,
        ACTION_SHY = 24,
        ACTION_SIT = 25,
        ACTION_SHOWCASE = 28,
        ACTION_HOME = 17,
        ACTION_SERVO_SEQUENCE = 18,
        ACTION_WHIRLWIND_LEG = 19
    };

    static int ClampGaitSpeed(int speed) {
        if (speed < kGaitSpeedMin) return kGaitSpeedMin;
        if (speed > kGaitSpeedMax) return kGaitSpeedMax;
        return speed;
    }
    static int ClampGaitAmount(int amount) {
        if (amount < 0) return 0;
        if (amount > kGaitAmountMax) return kGaitAmountMax;
        return amount;
    }
    static int ClampStepCount(int steps) {
        if (steps < 1) return 1;
        if (steps > kStepCountMax) return kStepCountMax;
        return steps;
    }

    static void ActionTask(void* arg) {
        OttoController* controller = static_cast<OttoController*>(arg);
        OttoActionParams params;

        while (true) {
            if (xQueueReceive(controller->action_queue_, &params, pdMS_TO_TICKS(1000)) == pdTRUE) {
                ESP_LOGI(TAG, "执行动作: %d", params.action_type);
                PowerManager::PauseBatteryUpdate();
                controller->is_action_in_progress_ = true;

                if (params.action_type == ACTION_SERVO_SEQUENCE) {
                    cJSON* json = cJSON_Parse(params.servo_sequence_json);
                    if (json != nullptr) {
                        cJSON_Delete(json);
                    }
                } else {
                    switch (params.action_type) {
                        case ACTION_WALK:
                            controller->otto_.Walk(ClampStepCount(params.steps), ClampGaitSpeed(params.speed), params.direction, ClampGaitAmount(params.amount));
                            break;
                        case ACTION_TURN:
                            controller->otto_.Turn(ClampStepCount(params.steps), ClampGaitSpeed(params.speed), params.direction, ClampGaitAmount(params.amount) / 2);
                            break;
                        case ACTION_JUMP:
                            controller->otto_.Jump(params.steps, ClampGaitSpeed(params.speed));
                            break;
                        case ACTION_SWING:
                            controller->otto_.Swing(params.steps, ClampGaitSpeed(params.speed), ClampGaitAmount(params.amount));
                            break;
                        case ACTION_MOONWALK:
                            controller->otto_.Moonwalker(params.steps, ClampGaitSpeed(params.speed), ClampGaitAmount(params.amount), params.direction);
                            break;
                        case ACTION_BEND:
                            controller->otto_.Bend(params.steps, ClampGaitSpeed(params.speed), params.direction);
                            break;
                        case ACTION_SHAKE_LEG:
                            controller->otto_.ShakeLeg(params.steps, ClampGaitSpeed(params.speed), params.direction);
                            break;
                        case ACTION_SIT:
                            controller->otto_.Sit();
                            break;
                        case ACTION_RADIO_CALISTHENICS:
                            if (controller->has_hands_) controller->otto_.RadioCalisthenics();
                            break;
                        case ACTION_MAGIC_CIRCLE:
                            if (controller->has_hands_) controller->otto_.MagicCircle();
                            break;
                        case ACTION_SHOWCASE:
                            controller->otto_.Showcase();
                            break;
                        case ACTION_UPDOWN:
                            controller->otto_.UpDown(params.steps, ClampGaitSpeed(params.speed), ClampGaitAmount(params.amount));
                            break;
                        case ACTION_TIPTOE_SWING:
                            controller->otto_.TiptoeSwing(params.steps, ClampGaitSpeed(params.speed), ClampGaitAmount(params.amount));
                            break;
                        case ACTION_JITTER:
                            controller->otto_.Jitter(params.steps, ClampGaitSpeed(params.speed), ClampGaitAmount(params.amount));
                            break;
                        case ACTION_ASCENDING_TURN:
                            controller->otto_.AscendingTurn(params.steps, ClampGaitSpeed(params.speed), ClampGaitAmount(params.amount));
                            break;
                        case ACTION_CRUSAITO:
                            controller->otto_.Crusaito(params.steps, ClampGaitSpeed(params.speed), ClampGaitAmount(params.amount), params.direction);
                            break;
                        case ACTION_FLAPPING:
                            controller->otto_.Flapping(params.steps, ClampGaitSpeed(params.speed), ClampGaitAmount(params.amount), params.direction);
                            break;
                        case ACTION_WHIRLWIND_LEG:
                            controller->otto_.WhirlwindLeg(params.steps, ClampGaitSpeed(params.speed), ClampGaitAmount(params.amount));
                            break;
                        case ACTION_HANDS_UP:
                            if (controller->has_hands_) controller->otto_.HandsUp(ClampGaitSpeed(params.speed), params.direction);
                            break;
                        case ACTION_HANDS_DOWN:
                            if (controller->has_hands_) controller->otto_.HandsDown(ClampGaitSpeed(params.speed), params.direction);
                            break;
                        case ACTION_HAND_WAVE:
                            if (controller->has_hands_) controller->otto_.HandWave(params.direction);
                            break;
                        case ACTION_WINDMILL:
                            if (controller->has_hands_) controller->otto_.Windmill(params.steps, ClampGaitSpeed(params.speed), ClampGaitAmount(params.amount));
                            break;
                        case ACTION_TAKEOFF:
                            if (controller->has_hands_) controller->otto_.Takeoff(params.steps, ClampGaitSpeed(params.speed), ClampGaitAmount(params.amount));
                            break;
                        case ACTION_FITNESS:
                            if (controller->has_hands_) controller->otto_.Fitness(params.steps, ClampGaitSpeed(params.speed), ClampGaitAmount(params.amount));
                            break;
                        case ACTION_GREETING:
                            if (controller->has_hands_) controller->otto_.Greeting(params.direction, params.steps);
                            break;
                        case ACTION_SHY:
                            if (controller->has_hands_) controller->otto_.Shy(params.direction, params.steps);
                            break;
                        case ACTION_HOME:
                            controller->otto_.Home(true);
                            break;
                    }
                    if (params.action_type != ACTION_SIT) {
                        if (params.action_type != ACTION_HOME && params.action_type != ACTION_SERVO_SEQUENCE) {
                            UBaseType_t pending_actions = uxQueueMessagesWaiting(controller->action_queue_);
                            if (pending_actions == 0) {
                                controller->otto_.Home(params.action_type != ACTION_HANDS_UP);
                            }
                        }
                    }
                }

                controller->is_action_in_progress_ = false;
                PowerManager::ResumeBatteryUpdate();

                // 动作结束后强制停顿 400ms，避免连续动作累计浪涌导致重启
                vTaskDelay(pdMS_TO_TICKS(400));
            }
        }
    }

    void StartActionTaskIfNeeded() {
        if (action_task_handle_ == nullptr) {
            xTaskCreate(ActionTask, "otto_action", 1024 * 4, this, configMAX_PRIORITIES - 1, &action_task_handle_);
        }
    }

    void QueueAction(int action_type, int steps, int speed, int direction, int amount) {
        if ((action_type >= ACTION_HANDS_UP && action_type <= ACTION_HAND_WAVE) ||
            (action_type == ACTION_WINDMILL) || (action_type == ACTION_TAKEOFF) ||
            (action_type == ACTION_FITNESS) || (action_type == ACTION_GREETING) ||
            (action_type == ACTION_SHY) || (action_type == ACTION_RADIO_CALISTHENICS) ||
            (action_type == ACTION_MAGIC_CIRCLE)) {
            if (!has_hands_) return;
        }
        OttoActionParams params = {action_type, steps, speed, direction, amount, ""};
        xQueueSend(action_queue_, &params, portMAX_DELAY);
        StartActionTaskIfNeeded();
    }

    void LoadTrimsFromNVS() {
        Settings settings("otto_trims", false);
        otto_.SetTrims(
            settings.GetInt("left_leg", 0), settings.GetInt("right_leg", 0),
            settings.GetInt("left_foot", 0), settings.GetInt("right_foot", 0),
            settings.GetInt("left_hand", 0), settings.GetInt("right_hand", 0)
        );
    }

public:
    OttoController(const HardwareConfig& hw_config) {
        otto_.Init(
            hw_config.left_leg_pin, hw_config.right_leg_pin,
            hw_config.left_foot_pin, hw_config.right_foot_pin,
            hw_config.left_hand_pin, hw_config.right_hand_pin
        );

        has_hands_ = (hw_config.left_hand_pin != GPIO_NUM_NC && hw_config.right_hand_pin != GPIO_NUM_NC);
        otto_.EnableServoLimit(kServoLimitDegPerSec);
        LoadTrimsFromNVS();

        action_queue_ = xQueueCreate(10, sizeof(OttoActionParams));
        // 此处不调用 BootUpSequence，避免卡死阻塞屏幕及系统初始化
        RegisterMcpTools();
    }

    void RequestHome() { QueueAction(ACTION_HOME, 1, 1000, 1, 0); }

    void RequestGentleWave(int direction) {
        if (!has_hands_) {
            QueueAction(ACTION_SWING, 1, 1800, 0, 10);
            return;
        }
        QueueAction(ACTION_GREETING, 2, 1500, direction == 0 ? 1 : direction, 0);
    }

    void Sleep() {
        xQueueReset(action_queue_);
        is_action_in_progress_ = false;
        is_asleep_ = true;
        otto_.DetachServos();
        otto_.SetRestState(false);
        ESP_LOGI(TAG, "舵机已下电（DetachServos），保持电流归零");
    }

    void Wakeup() {
        if (!is_asleep_) {
            QueueAction(ACTION_HOME, 1, 1000, 1, 0);
            return;
        }
        is_asleep_ = false;
        otto_.AttachServos();
        StartActionTaskIfNeeded();
        QueueAction(ACTION_HOME, 1, kHomeSettleSpeedMs, 1, 0);
        ESP_LOGI(TAG, "舵机重新上电，已入队归位动作");
    }

    bool HasHands() const { return has_hands_; }

    void BootHomeSequence() {
        static const int kHomeAngle[SERVO_COUNT] = {90, 90, 90, 90, 45, 135};
        ESP_LOGI(TAG, "开机舵机逐位归位开始");
        otto_.AttachServos();

        for (int i = 0; i < SERVO_COUNT; i++) {
            if ((i == LEFT_HAND || i == RIGHT_HAND) && !has_hands_) continue;
            otto_.MoveSingle(kHomeAngle[i], i);
            vTaskDelay(pdMS_TO_TICKS(kBootStaggerMs));
        }

        otto_.SetRestState(true);
        vTaskDelay(pdMS_TO_TICKS(150));
        ESP_LOGI(TAG, "开机舵机逐位归位结束");
    }

    void GentleWave() { RequestGentleWave(1); }

    bool IsBusy() const { return is_action_in_progress_; }
    void ClearActionQueue() { if (action_queue_ != nullptr) xQueueReset(action_queue_); }
    bool IsAsleep() const { return is_asleep_; }

    void RegisterMcpTools() {
        auto& mcp_server = McpServer::GetInstance();
        mcp_server.AddTool("self.otto.stop", "立即停止所有动作并复位", PropertyList(), [this](const PropertyList& properties) -> ReturnValue {
            if (action_task_handle_ != nullptr) { vTaskDelete(action_task_handle_); action_task_handle_ = nullptr; }
            is_action_in_progress_ = false;
            PowerManager::ResumeBatteryUpdate();
            xQueueReset(action_queue_);
            StartActionTaskIfNeeded();
            QueueAction(ACTION_HOME, 1, 1000, 1, 0);
            return true;
        });
        mcp_server.AddTool("self.otto.sleep", "舵机下电省电。", PropertyList(), [this](const PropertyList& properties) -> ReturnValue {
            QueueAction(ACTION_HOME, 1, 1000, 1, 0);
            Sleep();
            return "舵机已下电";
        });
        mcp_server.AddTool("self.otto.wake", "舵机重新上电。", PropertyList(), [this](const PropertyList& properties) -> ReturnValue {
            Wakeup();
            return "舵机已上电";
        });
    }

    ~OttoController() {
        if (action_task_handle_ != nullptr) vTaskDelete(action_task_handle_);
        vQueueDelete(action_queue_);
    }
};

OttoController* g_otto_controller = nullptr;

void InitializeOttoController(const HardwareConfig& hw_config) {
    if (g_otto_controller == nullptr) {
        g_otto_controller = new OttoController(hw_config);
    }
}

extern "C" void OttoController_RequestHome(void) { if (g_otto_controller) g_otto_controller->RequestHome(); }
extern "C" void OttoController_BootHomeSequence(void) { if (g_otto_controller) g_otto_controller->BootHomeSequence(); }
extern "C" void OttoController_GentleWave(void) { if (g_otto_controller) g_otto_controller->GentleWave(); }
extern "C" void OttoController_Sleep(void) { if (g_otto_controller) g_otto_controller->Sleep(); }
extern "C" void OttoController_Wakeup(void) { if (g_otto_controller) g_otto_controller->Wakeup(); }
extern "C" bool OttoController_IsBusy(void) { return g_otto_controller ? g_otto_controller->IsBusy() : false; }
extern "C" void OttoController_ClearQueue(void) { if (g_otto_controller) g_otto_controller->ClearActionQueue(); }
extern "C" bool OttoController_IsAsleep(void) { return g_otto_controller ? g_otto_controller->IsAsleep() : true; }
