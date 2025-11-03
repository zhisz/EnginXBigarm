//
// Created by Ukua on 2024/1/20.
//

#include "C620.h"

#include "GMmotors.h"

extern GM_BufferTypeDef GM_Buffer[GM_BUFFER_NUM];
osThreadId C620_MainLoopTaskHandle;

// index:[motor->motor_id-1]
INTF_Motor_HandleTypeDef *g_c620_motors[GM_BUFFER_NUM][C620_MOTOR_NUM] = {0};

int16_t C620_Torque2Current(float torque) {
    if (torque > 20 * C620_TORQUE_CONST) {
        return C620_CURRENT_MAX;
    } else if (torque < -20 * C620_TORQUE_CONST) {
        return -C620_CURRENT_MAX;
    } else {
        return (int16_t)roundf(torque / C620_TORQUE_CONST / 20.0f * C620_CURRENT_MAX);
    }
}

float C620_Current2Torque(int16_t current) {
    return current * 20.0f / C620_CURRENT_MAX * C620_TORQUE_CONST;
}

void C620_SetCurrent(uint8_t id, int16_t current, Bus_TopicHandleTypeDef *can_tx_topic) {
    // 限幅
    if (current > C620_CURRENT_MAX) {
        current = C620_CURRENT_MAX;
    } else if (current < -C620_CURRENT_MAX) {
        current = -C620_CURRENT_MAX;
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

void C620_CAN_CallBack(void *message, Bus_TopicHandleTypeDef *topic) {
    INTF_CAN_MessageTypeDef *msg = (INTF_CAN_MessageTypeDef *)message;
    if (msg->id_type == BSP_CAN_ID_EXT || msg->can_id < 0x201 || msg->can_id > 0x208) {
        return;
    }
    uint8_t id = msg->can_id - C620_ID_BASE - 1;

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
    if (g_c620_motors[buffer_id][id] == NULL) {
        return;
    }

    INTF_Motor_HandleTypeDef *m = g_c620_motors[buffer_id][id];
    // 确认电机在正常运行
    if (m->motor_state != MOTOR_STATE_RUNNING) {
        m->motor_state = MOTOR_STATE_RUNNING;
    }

    m->update_time = HAL_GetTick();

    C620_ResDataTypeDef *priv = m->private_data;

    int32_t ecd = (int16_t)((int16_t)(msg->data[0] << 8 | msg->data[1]) *
                            sign(m->direction));  // 0~360°
    m->real_speed = LowPassFilter(m->real_speed,
                                  (int16_t)(msg->data[2] << 8 | msg->data[3]) *
                                      m->direction * 2.0f * PI / 60 / priv->gear_ratio,
                                  0.9f);  // RPM->rad/s
    m->real_torque = LowPassFilter(m->real_torque,
                                   C620_Current2Torque(
                                       (int16_t)(msg->data[4] << 8 | msg->data[5])) *
                                       m->direction,
                                   0.9f);  // Nm

    if (ecd - priv->last_ecd > 4096)
        priv->total_rounds--;
    else if (ecd - priv->last_ecd < -4096)
        priv->total_rounds++;
    m->real_angle = LowPassFilter(m->real_angle,
                                  (priv->total_rounds * 2.0f * PI +
                                   ecd * 2.0f * PI / C620_ANGLE_MAX + m->angle_offset) /
                                      priv->gear_ratio,
                                  0.9f);
    priv->last_ecd = ecd;
}

void C620_PIDCalc() {
    for (int j = 0; j < GM_BUFFER_NUM; ++j) {
        for (int i = 0; i < C620_MOTOR_NUM; ++i) {
            INTF_Motor_HandleTypeDef *m = g_c620_motors[j][i];
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
            C620_ResDataTypeDef *priv = m->private_data;

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
                PID_out = float_constrain(PID_out, -C620_CURRENT_MAX, C620_CURRENT_MAX);
            }
            C620_SetCurrent(i + 1, (int16_t)roundf(PID_out * m->direction), priv->can_tx_topic);
        }
    }
}

void C620_Setmode_t(struct INTF_Motor_Handle *self, INTF_Motor_ModeTypeDef mode) {
    self->motor_mode = mode;
}

void C620_SetSpeed_t(struct INTF_Motor_Handle *self, float speed) {
    self->target_speed = speed;
}

void C620_SetAngle_t(struct INTF_Motor_Handle *self, float angle) {
    self->target_angle = angle;
}

void C620_SetTorque_t(struct INTF_Motor_Handle *self, float torque) {
    self->target_torque = torque;
}

INTF_Motor_HandleTypeDef *C620_Register(C620_ConfigTypeDef *config) {
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

    motor->private_data = JUST_MALLOC(sizeof(C620_ResDataTypeDef));
    memset(motor->private_data, 0, sizeof(C620_ResDataTypeDef));
    C620_ResDataTypeDef *priv = motor->private_data;

    if (config->gear_ratio != 0) {
        priv->gear_ratio = config->gear_ratio;
    } else {
        priv->gear_ratio = 19.203f;
    }

    priv->can_rx_topic = Bus_SubscribeFromName(config->can_rx_topic_name, C620_CAN_CallBack);
    priv->can_tx_topic = Bus_TopicRegister(config->can_tx_topic_name);

    priv->other_feedback_of_angle = config->other_feedback_of_angle;
    priv->other_feedback_of_speed = config->other_feedback_of_speed;

    priv->torque_feed_forward = config->torque_feed_forward;

    PIDInit(&priv->angle_pid, config->angle_pid_config);
    PIDInit(&priv->speed_pid, config->speed_pid_config);
    PIDInit(&priv->torque_pid, config->torque_pid_config);

    motor->set_mode = C620_Setmode_t;
    motor->set_speed = C620_SetSpeed_t;
    motor->set_angle = C620_SetAngle_t;
    motor->set_torque = C620_SetTorque_t;
    for (int i = 0; i < GM_BUFFER_NUM; ++i) {
        if (GM_Buffer[i].can_tx_topic == priv->can_tx_topic) {
            g_c620_motors[i][motor->motor_id - 1] = motor;
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

void C620_Init() {
    PID_Init_Config_s angle_pid = {
            .Kp=7.0f,
            .Ki=0.001f,
            .Kd=1.5f,
            .MaxOut=10.0f,
            .DeadBand = 0.0f,
            .Improve=PID_Integral_Limit,
    };
    PID_Init_Config_s speed_pid = {
            .Kp=0.9f,
            .Ki=0.001f,
            .Kd=0.048f,
            .MaxOut=6.0f,
            .DeadBand = 0.0f,
            .Output_LPF_RC=0.1f,
            .Improve=PID_Integral_Limit | PID_OutputFilter,
            .IntegralLimit=3.0f,
    };
    //  PID_Init_Config_s speed_pid = {

    //         .Kp=0.2f,
    //         .Ki=0.02f,
    //         .Kd=0.001f,
    //         .MaxOut=6.0f,
    //         .DeadBand = 0.0f,
    //         .Output_LPF_RC=0.1f,
    //         .Improve=PID_Integral_Limit | PID_OutputFilter,
    //         .IntegralLimit=1.0f,
    // };
    PID_Init_Config_s torque_pid = {
            .Kp=1000.0f,
            .Ki=5000.0f,
            .Kd=0.0f,
            .MaxOut=C620_CURRENT_MAX,
            .DeadBand = 0.0f,
            .Improve=PID_Integral_Limit,
            .IntegralLimit=500.0f,
    };
    C620_ConfigTypeDef config = {
            .motor_id=3,
            .motor_ptr_name="/motor/big_arm_yaw",
            .motor_mode=MOTOR_MODE_ANGLE,
            .direction=1.0f,
           //.torque_feed_forward = C620_Torque2Current(1.0f),//未测试
            .angle_pid_config=&angle_pid,
            .speed_pid_config=&speed_pid,
            .torque_pid_config=&torque_pid,
            .can_rx_topic_name="/CAN1/RX",
            .can_tx_topic_name="/CAN1/TX",
    };
    C620_Register(&config);

    C620_ConfigTypeDef config2 = {        //若要给大臂整体加抬升要用的参数（如果是3508）//没调
        .motor_id=2,
        .motor_ptr_name="/motor/big_arm_up",
        .motor_mode=MOTOR_MODE_ANGLE,
        .direction=1.0f,
        // .torque_feed_forward = C620_Torque2Current(1.0f),//未测试
        .angle_pid_config=&angle_pid,
        .speed_pid_config=&speed_pid,
        .torque_pid_config=&torque_pid,
        .can_rx_topic_name="/CAN1/RX",
        .can_tx_topic_name="/CAN1/TX",
};
    C620_Register(&config2);
    // /* 工程 */
    // // 底盘PID
    // PID_Init_Config_s angle_pid = {
    //     .Kp = 0.0f,
    //     .Ki = 0.0f,
    //     .Kd = 0.0f,
    //     .MaxOut = 1000.0f,
    //     .DeadBand = 0.0f,
    //     .Improve = PID_Integral_Limit,
    // };
    // PID_Init_Config_s speed_pid = {
    //     .Kp = 0.12f,
    //     .Ki = 0.00f,
    //     .Kd = 0.005f,
    //     .MaxOut = 6.0f,
    //     .DeadBand = 0.0f,
    //     .Output_LPF_RC = 0.1f,
    //     .Improve = PID_Integral_Limit | PID_OutputFilter,
    //     .IntegralLimit = 3.0f,
    // };
    // PID_Init_Config_s torque_pid = {
    //     .Kp = 1000.0f,
    //     .Ki = 5000.0f,
    //     .Kd = 0.0f,
    //     .MaxOut = C620_CURRENT_MAX,
    //     .DeadBand = 0.0f,
    //     .Improve = PID_Integral_Limit,
    //     .IntegralLimit = 500.0f,
    // };

    // C620_ConfigTypeDef config = {
    //     .motor_id = 4,
    //     //    .motor_ptr_name="/motor/OW_F_R",
    //     .motor_ptr_name = "/motor/MW_F_R",
    //     .motor_mode = MOTOR_MODE_SPEED,
    //     .direction = 1.0f,
    //     .torque_feed_forward = C620_Torque2Current(1.0f),  // 未测试
    //     .angle_pid_config = &angle_pid,
    //     .speed_pid_config = &speed_pid,
    //     .torque_pid_config = &torque_pid,
    //     .can_rx_topic_name = "/CAN1/RX",
    //     .can_tx_topic_name = "/CAN1/TX",
    // };
    // C620_Register(&config);

    // config.motor_id = 1;  // 3
    // config.motor_ptr_name = "/motor/MW_F_L";

    // C620_Register(&config);

    // config.motor_id = 3;  // 4
    // config.motor_ptr_name = "/motor/MW_B_L";
    // C620_Register(&config);

    // config.motor_id = 2;  // 2
    // config.motor_ptr_name = "/motor/MW_B_R";
    // C620_Register(&config);

    // // 工程底部小轮子
    // config.can_rx_topic_name = "/CAN2/RX";
    // config.can_tx_topic_name = "/CAN2/TX";
    // config.motor_ptr_name = "/motor/EN_S_L";
    // config.motor_id = 1;
    // C620_Register(&config);

    // config.motor_ptr_name = "/motor/EN_S_R";
    // config.motor_id = 2;
    // C620_Register(&config);

    // config.can_rx_topic_name = "/CAN1/RX";
    // config.can_tx_topic_name = "/CAN1/TX";

    // // 中间轮子抬升
    // PID_Init_Config_s angle_pid1 = {
    //     .Kp = 30.0f,
    //     .Ki = 0.0f,
    //     .Kd = 0.0f,
    //     .MaxOut = 1000.0f,
    //     .DeadBand = 0.0f,
    //     .Improve = PID_Integral_Limit,
    // };
    // PID_Init_Config_s speed_pid1 = {
    //     .Kp = 0.8f,
    //     .Ki = 0.000f,
    //     .Kd = 0.05f,
    //     .MaxOut = 6.0f,
    //     .DeadBand = 0.0f,
    //     .Output_LPF_RC = 0.1f,
    //     .Improve = PID_Integral_Limit | PID_OutputFilter,
    //     .IntegralLimit = 3.0f,
    // };
    // PID_Init_Config_s torque_pid1 = {
    //     .Kp = 1000.0f,
    //     .Ki = 5000.0f,
    //     .Kd = 0.0f,
    //     .MaxOut = C620_CURRENT_MAX,
    //     .DeadBand = 0.0f,
    //     .Improve = PID_Integral_Limit,
    //     .IntegralLimit = 500.0f,
    // };
    // config.speed_pid_config = &speed_pid1;
    // config.angle_pid_config = &angle_pid1;
    // config.torque_pid_config = &torque_pid1;

    // config.motor_mode = MOTOR_MODE_ANGLE;
    // config.can_rx_topic_name = "/CAN2/RX";
    // config.can_tx_topic_name = "/CAN2/TX";
    // config.motor_id = 5;
    // config.motor_ptr_name = "/motor/EN_rise";
    // C620_Register(&config);

    // // 中间轮子抬升
    // PID_Init_Config_s angle_pid2 = {
    //     .Kp = 20.0f,
    //     .Ki = 0.0f,
    //     .Kd = 0.0f,
    //     .MaxOut = 1000.0f,
    //     .DeadBand = 0.0f,
    //     .Improve = PID_Integral_Limit,
    // };
    // PID_Init_Config_s speed_pid2 = {
    //     .Kp = 0.8f,
    //     .Ki = 0.000f,
    //     .Kd = 0.05f,
    //     .MaxOut = 6.0f,
    //     .DeadBand = 0.0f,
    //     .Output_LPF_RC = 0.1f,
    //     .Improve = PID_Integral_Limit | PID_OutputFilter,
    //     .IntegralLimit = 3.0f,
    // };
    // PID_Init_Config_s torque_pid2 = {
    //     .Kp = 1000.0f,
    //     .Ki = 5000.0f,
    //     .Kd = 0.0f,
    //     .MaxOut = C620_CURRENT_MAX,
    //     .DeadBand = 0.0f,
    //     .Improve = PID_Integral_Limit,
    //     .IntegralLimit = 500.0f,
    // };
    // config.speed_pid_config = &speed_pid2;
    // config.angle_pid_config = &angle_pid2;
    // config.torque_pid_config = &torque_pid2;

    // config.motor_mode = MOTOR_MODE_ANGLE;
    // config.can_rx_topic_name = "/CAN1/RX";
    // config.can_tx_topic_name = "/CAN1/TX";
    // config.motor_ptr_name = "/motor/EN_lift";
    // config.motor_id = 5;

    // C620_Register(&config);

    // // 中间轮子抬升
    // PID_Init_Config_s angle_pid3 = {
    //     .Kp = 20.0f,
    //     .Ki = 0.0f,
    //     .Kd = 0.0f,
    //     .MaxOut = 1000.0f,
    //     .DeadBand = 0.0f,
    //     .Improve = PID_Integral_Limit,
    // };
    // PID_Init_Config_s speed_pid3 = {
    //     .Kp = 0.8f,
    //     .Ki = 0.000f,
    //     .Kd = 0.05f,
    //     .MaxOut = 6.0f,
    //     .DeadBand = 0.0f,
    //     .Output_LPF_RC = 0.1f,
    //     .Improve = PID_Integral_Limit | PID_OutputFilter,
    //     .IntegralLimit = 3.0f,
    // };
    // PID_Init_Config_s torque_pid3 = {
    //     .Kp = 1000.0f,
    //     .Ki = 5000.0f,
    //     .Kd = 0.0f,
    //     .MaxOut = C620_CURRENT_MAX,
    //     .DeadBand = 0.0f,
    //     .Improve = PID_Integral_Limit,
    //     .IntegralLimit = 500.0f,
    // };
    // config.speed_pid_config = &speed_pid3;
    // config.angle_pid_config = &angle_pid3;
    // config.torque_pid_config = &torque_pid3;
    // config.direction = -1;

    // config.motor_mode = MOTOR_MODE_ANGLE;
    // config.can_rx_topic_name = "/CAN1/RX";
    // config.can_tx_topic_name = "/CAN1/TX";
    // config.motor_ptr_name = "/motor/EN_hand_up";
    // config.motor_id = 6;

    // C620_Register(&config);

    // PID_Init_Config_s angle_pid3 = {
    //     .Kp = 20.0f,
    //     .Ki = 0.0f,
    //     .Kd = 0.0f,
    //     .MaxOut = 1000.0f,
    //     .DeadBand = 0.0f,
    //     .Improve = PID_Integral_Limit,
    // };

    // PID_Init_Config_s speed_pid3 = {
    //     .Kp = 0.6f,
    //     .Ki = 0.001f,
    //     .Kd = 0.001f,
    //     .MaxOut = 6.0f,
    //     .DeadBand = 0.0f,
    //     .Output_LPF_RC = 0.1f,
    //     .Improve = PID_Integral_Limit | PID_OutputFilter,
    //     .IntegralLimit = 3.0f,
    // };

    // PID_Init_Config_s torque_pid3 = {
    //     .Kp = 1000.0f,
    //     .Ki = 5000.0f,
    //     .Kd = 0.0f,
    //     .MaxOut = C620_CURRENT_MAX,
    //     .DeadBand = 0.0f,
    //     .Improve = PID_Integral_Limit,
    //     .IntegralLimit = 500.0f,
    // };

    // C620_ConfigTypeDef config = {
    //     .motor_id = 1,
    //     //    .motor_ptr_name="/motor/OW_F_R",
    //     .motor_ptr_name = "/motor/town",
    //     .motor_mode = MOTOR_MODE_SPEED,
    //     .direction = 1.0f,
    //     .torque_feed_forward = C620_Torque2Current(1.0f),  // 未测试
    //     .angle_pid_config = &angle_pid3,
    //     .speed_pid_config = &speed_pid3,
    //     .torque_pid_config = &torque_pid3,
    //     .can_rx_topic_name = "/CAN1/RX",
    //     .can_tx_topic_name = "/CAN1/TX",
    // };

    // C620_Register(&config);
}
