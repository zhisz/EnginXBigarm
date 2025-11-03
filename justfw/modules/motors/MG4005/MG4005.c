#include "MG4005.h"

//Bus_TopicHandleTypeDef *g_mg4005_can_tx;
//Bus_TopicHandleTypeDef *g_mg4005_usb_tx;

osThreadId MG4005_MainLoopTaskHandle;

static uint16_t idx = 0;
INTF_Motor_HandleTypeDef *g_mg4005_motors[MG4005_MOTOR_NUM] = {0};

void MG4005_Enable_All(Bus_TopicHandleTypeDef *can_tx_topic) {
    INTF_CAN_MessageTypeDef msg;
    msg.id_type = BSP_CAN_ID_STD;
    msg.can_id = MG4005_ID_BROADCAST_MIX;
    memset(msg.data, 0, 8);
    msg.data[0] = MG4005_CMD_ENABLE;
    msg.data[2] = MG4005_CMD_ENABLE;
    msg.data[4] = MG4005_CMD_ENABLE;
    msg.data[6] = MG4005_CMD_ENABLE;
    Bus_Publish(can_tx_topic, &msg);
}

void MG4005_Disable_All(Bus_TopicHandleTypeDef *can_tx_topic) {
    INTF_CAN_MessageTypeDef msg;
    msg.id_type = BSP_CAN_ID_STD;
    msg.can_id = MG4005_ID_BROADCAST_MIX;
    memset(msg.data, 0, 8);
    msg.data[0] = MG4005_CMD_DISABLE;
    msg.data[2] = MG4005_CMD_DISABLE;
    msg.data[4] = MG4005_CMD_DISABLE;
    msg.data[6] = MG4005_CMD_DISABLE;
    Bus_Publish(can_tx_topic, &msg);
}

void MG4005_SetSpeed_All(uint16_t speed1, uint16_t speed2, uint16_t speed3, uint16_t speed4,
                         Bus_TopicHandleTypeDef *can_tx_topic) {
    INTF_CAN_MessageTypeDef msg;
    msg.id_type = BSP_CAN_ID_STD;
    msg.can_id = MG4005_ID_BROADCAST_SPEED;
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
void MG4005_SetSpeed(uint8_t id, int32_t speed, Bus_TopicHandleTypeDef *can_tx_topic) {
    INTF_CAN_MessageTypeDef msg;
    msg.id_type = BSP_CAN_ID_STD;
    msg.can_id = MG4005_ID_BASE + id;
    memset(msg.data, 0, 8);
    msg.data[0] = MG4005_CMD_SPEED_CLOSE;
    msg.data[4] = *(uint8_t *) &speed;
    msg.data[5] = *((uint8_t *) &speed + 1);
    msg.data[6] = *((uint8_t *) &speed + 2);
    msg.data[7] = *((uint8_t *) &speed + 3);
    Bus_Publish(can_tx_topic, &msg);
}

/**
 * @brief 设置MF9025的速度
 * @param id:电机的ID(1~32)
 * @param iq:电流(单位degree/min)
**/
void MG4005_SetTorque(uint8_t id, int16_t iq, Bus_TopicHandleTypeDef *can_tx_topic) {
    INTF_CAN_MessageTypeDef msg;
    msg.id_type = BSP_CAN_ID_STD;
    msg.can_id = MG4005_ID_BASE + id;
    memset(msg.data, 0, 8);
    msg.data[0] = MG4005_CMD_TORQUE_CLOSE;
    msg.data[4] = *(uint8_t *) &iq;
    msg.data[5] = *((uint8_t *) &iq + 1);
    Bus_Publish(can_tx_topic, &msg);
}

void MG4005_SetAngle(uint8_t id, int16_t angle, Bus_TopicHandleTypeDef *can_tx_topic)
{INTF_CAN_MessageTypeDef msg;
    msg.id_type = BSP_CAN_ID_STD;
    msg.can_id = MG4005_ID_BASE + id;
    memset(msg.data, 0, 8);
    msg.data[0]=MG4005_CMD_POSITION_CLOSE2;
    msg.data[4] = (uint8_t)(angle & 0xFF);
    msg.data[5] = (uint8_t)((angle >> 8) & 0xFF);
    Bus_Publish(can_tx_topic, &msg);
}

/**
 * @brief 使能MF9025
 * @param id:电机的ID(1~32)
**/
void MG4005_Enable(uint8_t id, Bus_TopicHandleTypeDef *can_tx_topic) {
    INTF_CAN_MessageTypeDef msg;
    msg.id_type = BSP_CAN_ID_STD;
    msg.can_id = MG4005_ID_BASE + id;
    memset(msg.data, 0, 8);
    msg.data[0] = MG4005_CMD_ENABLE;
    Bus_Publish(can_tx_topic, &msg);
}

void MG4005_CAN_CallBack(void *message, Bus_TopicHandleTypeDef *topic) {
    INTF_CAN_MessageTypeDef *msg = (INTF_CAN_MessageTypeDef *) message;
    if (msg->id_type == BSP_CAN_ID_EXT || msg->can_id < 0x140 + 1 || msg->can_id > 0x140 + 32) {
        return;
    }

    uint8_t id = msg->can_id - 0x140;
    INTF_Motor_HandleTypeDef *m = NULL;
    int i;
    for (i = 0; i < MG4005_MOTOR_NUM; i++) {
        if (g_mg4005_motors[i]->motor_id == id &&
            ((MG4005_ResDataTypeDef *) g_mg4005_motors[i]->private_data)->can_rx_topic == topic) {

            m = g_mg4005_motors[i];
            break;
        }
        if (i == MG4005_MOTOR_NUM - 1) {
            return;
        }
    }
    if (m != NULL) {

        m->real_angle =  ((int16_t)((msg->data[1]) << 8 | msg->data[0]));
        m->real_speed =
                ((float) ((int16_t) ((msg->data[5]) << 8 | msg->data[4]))) / 360.0f * 2.0f * 3.1415926f * m->direction;
//        m->real_speed = (1 - SPEED_SMOOTH_COEF) * m->real_speed +
//                               SPEED_SMOOTH_COEF * ((float)((int16_t)((msg->data[5])<<8 | msg->data[4])))/360.0*2.0*3.1415926*m->direction;;
//        m->real_torque = ;
    }
}

void MG4005_Setmode_t(struct INTF_Motor_Handle *self, INTF_Motor_ModeTypeDef mode) {
    return;
}

void MG4005_SetSpeed_t(struct INTF_Motor_Handle *self, float speed) {
    self->target_speed = speed;
    //Cybergear_Trigger(self);
}

void MG4005_SetAngle_t(struct INTF_Motor_Handle *self, float angle) {
    self->target_angle = angle;
    //Cybergear_Trigger(self);
}

void MG4005_SetTorque_t(struct INTF_Motor_Handle *self, float torque) {
    self->target_torque = torque;
    //Cybergear_Trigger(self);
}


void MG4005_MainLoop() {

    osDelay(1000);
    for (int i = 0; i < MG4005_MOTOR_NUM; ++i) {
        MG4005_Enable(g_mg4005_motors[i]->motor_id,
                      ((MG4005_ResDataTypeDef *) g_mg4005_motors[i]->private_data)->can_tx_topic);
    }
    osDelay(1000);
    while (1) {
        for (int i = 0; i < MG4005_MOTOR_NUM; ++i) {
            INTF_Motor_HandleTypeDef m = *g_mg4005_motors[i];
            switch (m.motor_mode) {
                case MOTOR_MODE_TORQUE:
                    MG4005_SetTorque(m.motor_id, (int16_t) ((m.target_torque / 0.81f * (2000.0f / 32)) *
                                                            m.direction),
                                     ((MG4005_ResDataTypeDef *) m.private_data)->can_tx_topic);//扭矩常数 16匝为0.32 35匝为0.81
                    break;
                case MOTOR_MODE_SPEED:
                    MG4005_SetSpeed(m.motor_id,(int16_t)(m.target_speed),
                                    ((MG4005_ResDataTypeDef *) m.private_data)->can_tx_topic);
                    break;
                case MOTOR_MODE_ANGLE:
                    MG4005_SetAngle(m.motor_id, (int16_t)((m.target_angle*57.29f*10.0f*100.0f)  ) ,
                                     ((MG4005_ResDataTypeDef *) m.private_data)->can_tx_topic);
                    break;
            }
        }
        osDelay(1);
    }
}

void MG4005_Register(MG4005_ConfigTypeDef *config) {
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
    motor->private_data = malloc(sizeof(MG4005_ResDataTypeDef));
    memset(motor->private_data, 0, sizeof(MG4005_ResDataTypeDef));
    ((MG4005_ResDataTypeDef *) motor->private_data)->can_rx_topic = Bus_SubscribeFromName(config->can_rx_topic_name,
                                                                                          MG4005_CAN_CallBack);
    ((MG4005_ResDataTypeDef *) motor->private_data)->can_tx_topic = Bus_TopicRegister(config->can_tx_topic_name);
    g_mg4005_motors[idx++] = motor;
}

void MG4005_Init() {

    Bus_SubscribeFromName("CAN_RX", MG4005_CAN_CallBack);

    MG4005_ConfigTypeDef config = {
            .motor_id=1,
            .motor_ptr_name="joint4",
            .direction=-1.0f,
            .motor_mode=MOTOR_MODE_ANGLE,
            .can_rx_topic_name="/CAN1/RX",
            .can_tx_topic_name="/CAN1/TX",
    };
    MG4005_Register(&config);

//    Bus_SubscribeFromName("CAN_RX", MF9025_CAN_CallBack);
//    MF9025_Register(1, "leg-ML");
//    MF9025_Register(2, "leg-MR");

//    g_mf9025_motors[0]->direction = -1.0f;
//    g_mf9025_motors[0]->set_torque=0.3f;

    osThreadDef(MG4005_MainLoopTask, MG4005_MainLoop, osPriorityLow, 0, 256);
    MG4005_MainLoopTaskHandle = osThreadCreate(osThread(MG4005_MainLoopTask), NULL);
}
