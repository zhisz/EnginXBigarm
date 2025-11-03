//
// Created by Ukua on 2024/1/20.
//

#include "C610.h"

#include "GMmotors.h"

extern GM_BufferTypeDef GM_Buffer[GM_BUFFER_NUM];
osThreadId C610_MainLoopTaskHandle;

// index:[motor->motor_id-1]
INTF_Motor_HandleTypeDef *g_c610_motors[GM_BUFFER_NUM][C610_MOTOR_NUM] = {0};

int16_t C610_Torque2Current(float torque) {
    if (torque > 10 * C610_TORQUE_CONST) {
        return C610_CURRENT_MAX;
    } else if (torque < -10 * C610_TORQUE_CONST) {
        return -C610_CURRENT_MAX;
    } else {
        return (int16_t)roundf(torque / C610_TORQUE_CONST / 10.0f * C610_CURRENT_MAX);
    }
}

float C610_Current2Torque(int16_t current) {
    return current * 10.0f / C610_CURRENT_MAX * C610_TORQUE_CONST;
}

void C610_SetCurrent(uint8_t id, int16_t current, Bus_TopicHandleTypeDef *can_tx_topic) {
    // 限幅
    if (current > C610_CURRENT_MAX) {
        current = C610_CURRENT_MAX;
    } else if (current < -C610_CURRENT_MAX) {
        current = -C610_CURRENT_MAX;
    }

    GM_BufferTypeDef *buffer = NULL;
    for (int i = 0; i < GM_BUFFER_NUM; ++i) {
        if (GM_Buffer[i].can_tx_topic == can_tx_topic) {
            buffer = &GM_Buffer[i];
        }
    }
    if (buffer == NULL) {
        return;
    }
    if (id > 4) {
        buffer->buffer_0x1FF[(id - 5) * 2] = current >> 8;
        buffer->buffer_0x1FF[(id - 5) * 2 + 1] = current;
    } else {
        buffer->buffer_0x200[(id - 1) * 2] = current >> 8;
        buffer->buffer_0x200[(id - 1) * 2 + 1] = current;
    }
}

void C610_CAN_CallBack(void *message, Bus_TopicHandleTypeDef *topic) {
    INTF_CAN_MessageTypeDef *msg = (INTF_CAN_MessageTypeDef *)message;
    if (msg->id_type == BSP_CAN_ID_EXT || msg->can_id < 0x201 || msg->can_id > 0x208) {
        return;
    }
    uint8_t id = msg->can_id - C610_ID_BASE - 1;
    GM_BufferTypeDef *buffer = NULL;
    uint16_t buffer_id = -1;
    for (int i = 0; i < GM_BUFFER_NUM; i++) {
        if (GM_Buffer[i].can_rx_topic == topic) {
            buffer = &GM_Buffer[i];
            buffer_id = i;
        }
    }
    if (buffer == NULL) {
        return;
    }
    if (g_c610_motors[buffer_id][id] == NULL) {
        return;
    }
    INTF_Motor_HandleTypeDef *m = g_c610_motors[buffer_id][id];

    // 确认电机在正常运行
    if (m->motor_state != MOTOR_STATE_RUNNING) {
        m->motor_state = MOTOR_STATE_RUNNING;
    }

    m->update_time = HAL_GetTick();

    int16_t ecd = (int16_t)((int16_t)(msg->data[0] << 8 | msg->data[1]) *
                            sign(m->direction));
    m->real_speed = LowPassFilter(m->real_speed,
                                  (int16_t)(msg->data[2] << 8 | msg->data[3]) *
                                      m->direction * 2.0f * M_PI / 60,
                                  0.9f);  // RPM->rad/s
    m->real_torque = LowPassFilter(m->real_torque,
                                   C610_Current2Torque(
                                       (int16_t)(msg->data[4] << 8 | msg->data[5])) *
                                       m->direction,
                                   0.9f);  // Nm

    C610_ResDataTypeDef *priv = m->private_data;
    if (ecd - priv->last_ecd > 4096)
        priv->total_rounds--;
    else if (ecd - priv->last_ecd < -4096)
        priv->total_rounds++;
    m->real_angle = LowPassFilter(m->real_angle,
                                  priv->total_rounds * 2.0f * M_PI +
                                      ecd * 2.0f * M_PI / C610_ANGLE_MAX + m->angle_offset,
                                  0.9f);
    priv->last_ecd = ecd;
}

void C610_PIDCalc() {
    for (int j = 0; j < GM_BUFFER_NUM; ++j) {
        for (int i = 0; i < C610_MOTOR_NUM; ++i) {
            INTF_Motor_HandleTypeDef *m = g_c610_motors[j][i];
            if (m == NULL) {
                continue;
            }
            if (m->motor_state != MOTOR_STATE_RUNNING) {
                continue;  // 确认电机在正常运行
            }

            // 电机离线检测
            if (HAL_GetTick() - m->update_time > 30) {
                m->motor_state = MOTOR_STATE_ERROR;
            }

            float PID_out = 0;  // 临时存放PID的输出
            C610_ResDataTypeDef *priv = m->private_data;

            switch (m->motor_mode) {
                case MOTOR_MODE_ANGLE:
                    PID_out = m->target_angle;
                    break;
                case MOTOR_MODE_SPEED:
                    PID_out = m->target_speed;
                    break;
                case MOTOR_MODE_TORQUE:
                    PID_out = m->target_torque;
                    break;
            }

            float angle_measure, speed_measure;
            // 注意：下面的switch没有break，PID逐级计算
            switch (m->motor_mode) {
                case MOTOR_MODE_ANGLE:
                    if (priv->other_feedback_of_angle == NULL) {
                        angle_measure = m->real_angle;
                    } else {
                        angle_measure = *(priv->other_feedback_of_angle);
                    }
                    PID_out = PIDCalculate(&(priv->angle_pid), angle_measure, PID_out);
                    [[fallthrough]];
                case MOTOR_MODE_SPEED:
                    if (priv->other_feedback_of_speed == NULL) {
                        speed_measure = m->real_speed;
                    } else {
                        speed_measure = *(priv->other_feedback_of_speed);
                    }
                    PID_out = PIDCalculate(&(priv->speed_pid), speed_measure, PID_out);
                    [[fallthrough]];
                case MOTOR_MODE_TORQUE:
                    // 前馈
                    PID_out = priv->torque_feed_forward * PID_out +
                              PIDCalculate(&(priv->torque_pid), m->real_torque, PID_out);
                    // 限幅，防止爆uint16_t
                    PID_out = float_constrain(PID_out, -C610_CURRENT_MAX, C610_CURRENT_MAX);
                    break;
            }
            C610_SetCurrent(i + 1, (int16_t)roundf(PID_out * m->direction), priv->can_tx_topic);
        }
    }
}

void C610_Setmode_t(struct INTF_Motor_Handle *self, INTF_Motor_ModeTypeDef mode) {
    self->motor_mode = mode;
}

void C610_SetSpeed_t(struct INTF_Motor_Handle *self, float speed) {
    self->target_speed = speed;
}

void C610_SetAngle_t(struct INTF_Motor_Handle *self, float angle) {
    self->target_angle = angle;
}

void C610_SetTorque_t(struct INTF_Motor_Handle *self, float torque) {
    self->target_torque = torque;
}

INTF_Motor_HandleTypeDef *C610_Register(C610_ConfigTypeDef *config) {
    INTF_Motor_HandleTypeDef *motor = Bus_SharePtr(config->motor_ptr_name, sizeof(INTF_Motor_HandleTypeDef));
    motor->motor_id = config->motor_id;
    motor->motor_mode = config->motor_mode;
    motor->motor_state = MOTOR_STATE_INIT;
    motor->target_speed = 0.0f;
    motor->real_speed = 0.0f;
    motor->target_angle = 0.0f;
    motor->real_angle = 0.0f;
    motor->target_torque = 0.0f;
    motor->real_torque = 0.0f;
    motor->direction = config->direction;

    motor->angle_offset = config->angle_offset;

    motor->private_data = JUST_MALLOC(sizeof(C610_ResDataTypeDef));
    memset(motor->private_data, 0, sizeof(C610_ResDataTypeDef));
    C610_ResDataTypeDef *priv = motor->private_data;
    priv->can_rx_topic = Bus_SubscribeFromName(config->can_rx_topic_name, C610_CAN_CallBack);
    priv->can_tx_topic = Bus_TopicRegister(config->can_tx_topic_name);

    priv->other_feedback_of_angle = config->other_feedback_of_angle;
    priv->other_feedback_of_speed = config->other_feedback_of_speed;

    PIDInit(&priv->angle_pid, config->angle_pid_config);
    PIDInit(&priv->speed_pid, config->speed_pid_config);
    PIDInit(&priv->torque_pid, config->torque_pid_config);

    motor->set_mode = C610_Setmode_t;
    motor->set_speed = C610_SetSpeed_t;
    motor->set_angle = C610_SetAngle_t;
    motor->set_torque = C610_SetTorque_t;

    for (int i = 0; i < GM_BUFFER_NUM; ++i) {
        if (GM_Buffer[i].can_tx_topic == priv->can_tx_topic) {
            g_c610_motors[i][motor->motor_id - 1] = motor;
        }
    }

    for (int i = 0; i < GM_BUFFER_NUM; ++i) {
        if (GM_Buffer[i].can_tx_topic == priv->can_tx_topic) {
            if (motor->motor_id > 4) {
                GM_Buffer[i].buffer_0x1FF_not_null = 1;
            } else {
                GM_Buffer[i].buffer_0x200_not_null = 1;
            }
            break;
        }
    }

    return motor;
}

void C610_Init() {
       PID_Init_Config_s angle_pid = {
               .Kp=5.3f,
               .Ki=0.0031f,
               .Kd=0.0011f,
               .MaxOut=10000.0f,
               .DeadBand = 0.0f,
               .Improve=PID_Integral_Limit ,
       };
       PID_Init_Config_s speed_pid = {
               .Kp=0.065f,
               .Ki=0.00002f,
               .Kd=0.0003f,
               .MaxOut=6.0f,
               .DeadBand = 0.5f,
               .IntegralLimit = 3.0f,
               .Improve=PID_Integral_Limit,
       };
       PID_Init_Config_s torque_pid = {
               .Kp=2000.0f,
               .Ki=5000.0f,
               .Kd=0.0f,
               .MaxOut=C610_CURRENT_MAX,
               .DeadBand = 0.0f,
               .Improve=PID_Integral_Limit,
               .IntegralLimit=100.0f,
       };
       C610_ConfigTypeDef config_1 = {            //大疆电机的速度模式和角度模式是干涉的，想调角度要先把速度调好再改

               .motor_id = 1,//3,
               .motor_ptr_name = "/motor/big_arm_1",
               .motor_mode = MOTOR_MODE_ANGLE,
               .direction = 1.0f,
               //.torque_feed_forward = C610_Torque2Current(1.0f),//未测试
               .angle_pid_config = &angle_pid,
               .speed_pid_config = &speed_pid,
               .torque_pid_config = &torque_pid,
               .can_rx_topic_name = "/CAN1/RX",
               .can_tx_topic_name = "/CAN1/TX",
       };
       C610_Register(&config_1);
        
       C610_ConfigTypeDef config_2 = {
               .motor_id = 6,//3,
               .motor_ptr_name = "/motor/big_arm_2",
               .motor_mode = MOTOR_MODE_ANGLE,
               .direction = 1.0f,
               //.torque_feed_forward = C610_Torque2Current(1.0f),//未测试
               .angle_pid_config = &angle_pid,
               .speed_pid_config = &speed_pid,
               .torque_pid_config = &torque_pid,
               .can_rx_topic_name = "/CAN1/RX",
               .can_tx_topic_name = "/CAN1/TX",
       };
       C610_Register(&config_2);
}