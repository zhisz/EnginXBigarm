#include "enginchas_logic.h"

#include "EF_com.h"
#define CHASSIS_SPEED_X_MAX 1.0f
#define CHASSIS_SPEED_Y_MAX 1.0f
#define CHASSIS_SPEED_W_MAX 1.0f

#define SENSITIVE 220
#define LIFT_MAX_SPEED 2

static INTF_Chassis_HandleTypeDef *chassis;
extern TIM_HandleTypeDef htim8;

static INTF_Motor_HandleTypeDef *m_rise;    // 侧方
static INTF_Motor_HandleTypeDef *m_lift;    // 后方
static INTF_Motor_HandleTypeDef *m_handup;  // 抬手

static RC_ctrl_t *rc_ctrl;

static osThreadId Enginchas_Logic_MainLoopTaskHandle;

static void Enginchas_Logic_MainLoop() {
    extern uint8_t g_dr16_is_connected;
    uint32_t DWT_CNT = DWT_GetCNT();
    float speed_x, speed_y, speed_w;

    while (true) {
        float dt = DWT_GetDeltaT(&DWT_CNT);

        if (g_dr16_is_connected) {
            // 抬升
            switch (rc_ctrl[0].rc.switch_right) {
            case RC_SW_UP:
                m_rise->set_angle(m_rise, m_rise->target_angle - dt * 2 * PI * LIFT_MAX_SPEED * rc_ctrl->rc.rocker_r1 / 660);

                m_handup->set_mode(m_handup, MOTOR_MODE_TORQUE);
                m_handup->set_torque(m_handup, 0);

                break;
            case RC_SW_DOWN:
                m_lift->set_angle(m_lift, m_lift->target_angle + dt * 2 * PI * LIFT_MAX_SPEED * rc_ctrl->rc.rocker_r1 / 660);

                break;
            default:
                m_handup->set_mode(m_handup, MOTOR_MODE_ANGLE);
                m_handup->set_angle(m_handup, m_handup->target_angle + dt * 2 * PI * LIFT_MAX_SPEED * rc_ctrl->rc.rocker_r1 / 660);
                break;
            }

            switch (rc_ctrl[0].rc.switch_left) {
            case RC_SW_MID:
                // HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET);
                __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_4, 1000);
                break;
            case RC_SW_DOWN:
                // HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_RESET);
                __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_4, 1500);
                break;

            default:
                break;
            }

            // 前后左右
            switch (rc_ctrl[0].rc.switch_right) {
            case RC_SW_UP:
            case RC_SW_DOWN:
                speed_x = rc_ctrl[0].rc.rocker_l_ / 660.0f * 1;
                speed_y = rc_ctrl[0].rc.rocker_l1 / 660.0f * 1;
                speed_w = -rc_ctrl[0].rc.dial / 660.0f * 1;

                if (abs(rc_ctrl->rc.rocker_r_) > 500) {
                    speed_w = rc_ctrl[0].rc.rocker_r_ / 660.0f * 0.2f;
                }

                chassis->target_speed_x = speed_x;
                chassis->target_speed_y = speed_y;
                chassis->target_speed_w = speed_w;

                extern void enginchs_set_wheel(float speed_x, float speed_y, float speed_w);
                enginchs_set_wheel(speed_x, speed_y, speed_w);
                break;
            case RC_SW_MID:
                // 底盘控制
                speed_x = rc_ctrl[0].rc.rocker_l_ / 660.0f * CHASSIS_SPEED_X_MAX;
                speed_y = rc_ctrl[0].rc.rocker_l1 / 660.0f * CHASSIS_SPEED_Y_MAX;
                speed_w = -rc_ctrl[0].rc.dial / 660.0f * CHASSIS_SPEED_W_MAX;

                chassis->target_speed_x = speed_x;
                chassis->target_speed_y = speed_y;
                chassis->target_speed_w = speed_w;

                extern void enginchs_set_wheel(float speed_x, float speed_y, float speed_w);
                enginchs_set_wheel(speed_x, speed_y, speed_w);
                break;
            default:
                break;
            }
        }
        EF_send_float(&m_rise->target_angle, 1);
        osDelay(2);
    }
}

void Enginchas_Logic_Init() {
    chassis = Bus_SharePtr("chassis", sizeof(INTF_Chassis_HandleTypeDef));
    rc_ctrl = Bus_SharePtr("DR16", sizeof(INTF_Chassis_HandleTypeDef));

    m_rise = Bus_SharePtr("/motor/EN_rise", sizeof(INTF_Motor_HandleTypeDef));
    m_lift = Bus_SharePtr("/motor/EN_lift", sizeof(INTF_Motor_HandleTypeDef));
    m_handup = Bus_SharePtr("/motor/EN_hand_up", sizeof(INTF_Motor_ModeTypeDef));

    extern void enginchas_wheel_init();
    enginchas_wheel_init();

    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_4);
    __HAL_TIM_SET_PRESCALER(&htim8, 84 - 1);
    __HAL_TIM_SET_AUTORELOAD(&htim8, 20000 - 1);

    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_4, 1500);

    osThreadDef(Enginchas_Logic_MainLoopTask, Enginchas_Logic_MainLoop, osPriorityLow, 0, 512);
    Enginchas_Logic_MainLoopTaskHandle = osThreadCreate(osThread(Enginchas_Logic_MainLoopTask), NULL);
}