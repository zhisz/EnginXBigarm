//
// Created by Ukua on 2023/10/2.
//

#include "MF9025.h"

// Bus_TopicHandleTypeDef *g_mf9025_can_tx;
// Bus_TopicHandleTypeDef *g_mf9025_usb_tx;

osThreadId MF9025_MainLoopTaskHandle;

static uint16_t idx = 0;
INTF_Motor_HandleTypeDef *g_mf9025_motors[MF9025_MOTOR_NUM] = {0};

void MF9025_Enable_All(Bus_TopicHandleTypeDef *can_tx_topic) {
    INTF_CAN_MessageTypeDef msg;
    msg.id_type = BSP_CAN_ID_STD;
    msg.can_id = MF9025_ID_BROADCAST_MIX;
    memset(msg.data, 0, 8);
    msg.data[0] = MF9025_CMD_ENABLE;
    msg.data[2] = MF9025_CMD_ENABLE;
    msg.data[4] = MF9025_CMD_ENABLE;
    msg.data[6] = MF9025_CMD_ENABLE;
    Bus_Publish(can_tx_topic, &msg);
}

void MF9025_Disable_All(Bus_TopicHandleTypeDef *can_tx_topic) {
    INTF_CAN_MessageTypeDef msg;
    msg.id_type = BSP_CAN_ID_STD;
    msg.can_id = MF9025_ID_BROADCAST_MIX;
    memset(msg.data, 0, 8);
    msg.data[0] = MF9025_CMD_DISABLE;
    msg.data[2] = MF9025_CMD_DISABLE;
    msg.data[4] = MF9025_CMD_DISABLE;
    msg.data[6] = MF9025_CMD_DISABLE;
    Bus_Publish(can_tx_topic, &msg);
}

void MF9025_SetSpeed_All(uint16_t speed1, uint16_t speed2, uint16_t speed3, uint16_t speed4,
                         Bus_TopicHandleTypeDef *can_tx_topic) {
    INTF_CAN_MessageTypeDef msg;
    msg.id_type = BSP_CAN_ID_STD;
    msg.can_id = MF9025_ID_BROADCAST_SPEED;
    memset(msg.data, 0, 8);
    msg.data[0] = speed1 >> 8;
    msg.data[1] = speed1;
    msg.data[2] = speed2 >> 8;
    msg.data[3] = speed2;
    msg.data[4] = speed3 >> 8;
    msg.data[5] = speed3;
    msg.data[6] = speed4 >> 8;
    msg.data[7] = speed4;
    Bus_Publish(can_tx_topic, &msg);
}

/**
 * @brief 设置MF9025的速度
 * @param id:电机的ID(1~32)
 * @param speed:速度(单位degree/min)
 **/
void MF9025_SetSpeed(uint8_t id, int32_t speed, Bus_TopicHandleTypeDef *can_tx_topic) {
    INTF_CAN_MessageTypeDef msg;
    msg.id_type = BSP_CAN_ID_STD;
    msg.can_id = MF9025_ID_BASE + id;
    memset(msg.data, 0, 8);
    msg.data[0] = MF9025_CMD_SPEED_CLOSE;
    msg.data[4] = *(uint8_t *)&speed;
    msg.data[5] = *((uint8_t *)&speed + 1);
    msg.data[6] = *((uint8_t *)&speed + 2);
    msg.data[7] = *((uint8_t *)&speed + 3);
    Bus_Publish(can_tx_topic, &msg);
}

/**
 * @brief 设置MF9025的速度
 * @param id:电机的ID(1~32)
 * @param iq:电流(+-2000对应+-32A)
 **/
void MF9025_SetTorque(uint8_t id, int16_t iq, Bus_TopicHandleTypeDef *can_tx_topic) {
    INTF_CAN_MessageTypeDef msg;
    msg.id_type = BSP_CAN_ID_STD;
    msg.can_id = MF9025_ID_BASE + id;
    memset(msg.data, 0, 8);
    msg.data[0] = MF9025_CMD_TORQUE_CLOSE;
    msg.data[4] = *(uint8_t *)&iq;
    msg.data[5] = *((uint8_t *)&iq + 1);
    Bus_Publish(can_tx_topic, &msg);
}

/**
 * @brief 使能MF9025
 * @param id:电机的ID(1~32)
 **/
void MF9025_Enable(uint8_t id, Bus_TopicHandleTypeDef *can_tx_topic) {
    INTF_CAN_MessageTypeDef msg;
    msg.id_type = BSP_CAN_ID_STD;
    msg.can_id = MF9025_ID_BASE + id;
    memset(msg.data, 0, 8);
    msg.data[0] = MF9025_CMD_ENABLE;
    Bus_Publish(can_tx_topic, &msg);
}

void MF9025_CAN_CallBack(void *message, Bus_TopicHandleTypeDef *topic) {
    INTF_CAN_MessageTypeDef *msg = (INTF_CAN_MessageTypeDef *)message;
    if (msg->id_type == BSP_CAN_ID_EXT || msg->can_id < 0x140 + 1 || msg->can_id > 0x140 + 32) {
        return;
    }

    uint8_t id = msg->can_id - 0x140;
    INTF_Motor_HandleTypeDef *m = NULL;
    int i;
    for (i = 0; i < MF9025_MOTOR_NUM; i++) {
        if (g_mf9025_motors[i]->motor_id == id &&
            ((MF9025_ResDataTypeDef *)g_mf9025_motors[i]->private_data)->can_rx_topic->topic == topic) {
            m = g_mf9025_motors[i];
            break;
        }
        if (i == MF9025_MOTOR_NUM - 1) {
            return;
        }
    }
    if (m != NULL) {
        //        m->real_angle = ;
        m->real_speed =
            ((float)((int16_t)((msg->data[5]) << 8 | msg->data[4]))) / 360.0f * 2.0f * 3.1415926f * m->direction;
        //        m->real_speed = (1 - SPEED_SMOOTH_COEF) * m->real_speed +
        //                               SPEED_SMOOTH_COEF * ((float)((int16_t)((msg->data[5])<<8 | msg->data[4])))/360.0*2.0*3.1415926*m->direction;;
        //        m->real_torque = ;
    }
}

void MF9025_Setmode_t(struct INTF_Motor_Handle *self, INTF_Motor_ModeTypeDef mode) {
    self->motor_mode = mode;
    return;
}

void MF9025_SetSpeed_t(struct INTF_Motor_Handle *self, float speed) {
    self->target_speed = speed;
}

void MF9025_SetAngle_t(struct INTF_Motor_Handle *self, float angle) {
    self->target_angle = angle;
}

void MF9025_SetTorque_t(struct INTF_Motor_Handle *self, float torque) {
    self->target_torque = torque;
}

void MF9025_MainLoop() {
    osDelay(1000);
    for (int i = 0; i < MF9025_MOTOR_NUM; ++i) {
        MF9025_Enable(g_mf9025_motors[i]->motor_id,
                      ((MF9025_ResDataTypeDef *)g_mf9025_motors[i]->private_data)->can_tx_topic);
    }
    osDelay(1000);
    while (1) {
        for (int i = 0; i < MF9025_MOTOR_NUM; ++i) {
            INTF_Motor_HandleTypeDef m = *g_mf9025_motors[i];
            switch (m.motor_mode) {
                case MOTOR_MODE_TORQUE:
                    MF9025_SetTorque(m.motor_id, (int16_t)((m.target_torque / MF9025_TORQUE_CONSTANT * (2000.0f / 32)) * m.direction),
                                     ((MF9025_ResDataTypeDef *)m.private_data)->can_tx_topic);
                    break;
                case MOTOR_MODE_SPEED:
                    MF9025_SetSpeed(m.motor_id, (int32_t)(m.target_speed / (2 * 3.14f) * 360.0f * 60 * m.direction),
                                    ((MF9025_ResDataTypeDef *)m.private_data)->can_tx_topic);
                    break;
                case MOTOR_MODE_ANGLE:
                    // TODO: 补全角度模式
                    break;
            }
        }
        osDelay(1);
    }
}

void MF9025_Register(MF9025_ConfigTypeDef *config) {
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
    motor->direction = 1.0f;
    motor->private_data = JUST_MALLOC(sizeof(MF9025_ResDataTypeDef));
    memset(motor->private_data, 0, sizeof(MF9025_ResDataTypeDef));
    ((MF9025_ResDataTypeDef *)motor->private_data)->can_rx_topic = Bus_SubscribeFromName(config->can_rx_topic_name,
                                                                                         MF9025_CAN_CallBack);
    ((MF9025_ResDataTypeDef *)motor->private_data)->can_tx_topic = Bus_TopicRegister(config->can_tx_topic_name);
    g_mf9025_motors[idx++] = motor;
}

void MF9025_Init() {
    MF9025_ConfigTypeDef leg_ml = {
        .motor_id = 1,
        .motor_ptr_name = "/motor/leg-ML",
        .direction = -1.0f,
        .motor_mode = MOTOR_MODE_TORQUE,
        .can_rx_topic_name = "/CAN1/RX",
        .can_tx_topic_name = "/CAN1/TX",
    };
    MF9025_Register(&leg_ml);

    //    Bus_SubscribeFromName("CAN_RX", MF9025_CAN_CallBack);
    //    MF9025_Register(1, "leg-ML");
    //    MF9025_Register(2, "leg-MR");

    //    g_mf9025_motors[0]->direction = -1.0f;
    //    g_mf9025_motors[0]->set_torque=0.3f;

    osThreadDef(MF9025_MainLoopTask, MF9025_MainLoop, osPriorityLow, 0, 256);
    MF9025_MainLoopTaskHandle = osThreadCreate(osThread(MF9025_MainLoopTask), NULL);
}