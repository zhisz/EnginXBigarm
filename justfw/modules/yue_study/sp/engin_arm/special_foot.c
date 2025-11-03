// //
// // Created by lingyue on 2025/3/22.
// //

// #include "special_foot.h"
// static uint16_t idx = 0;
// INTF_Motor_HandleTypeDef *g_DM_J3519_motors[DM_J3519_MOTOR_NUM] = {0};

// osThreadId DM_J3519_MainLoopTaskHandle;

// // #define OFFSET_ANGLE 0.396692792f

// #define P_MIN (-12.5f)
// #define P_MAX 12.5f
// #define V_MIN (-45.0f)
// #define V_MAX 45.0f
// #define KP_MIN 0.0f
// #define KP_MAX 500.0f
// #define KD_MIN 0.0f
// #define KD_MAX 5.0f
// #define T_MIN (-20.0f)       //力矩
// #define T_MAX 20.0f

// #define A_MIN (-4 * 3.1415926535)    //角度
// #define A_MAX (4 * 3.1415926535)

// #define MASTER_ID 0x00    //主机ID



// #define DRAT_RT_SENSITIVITY 100          //定义摇杆灵敏度阈值



// struct DM_J3519_feedback_frame {
//    uint8_t id_with_error : 8;  // 电机id和错误信息 ID|ERR<<4
//    uint16_t POS : 16;          // 位置
//    uint16_t VEL : 12;          // 速度
//    uint16_t T : 12;            // 力矩
//    uint8_t T_MOS : 8;          // MOS温度 单位摄氏度
//    uint8_t T_Rotor : 8;        // 电机内部线圈平均温度 单位摄氏度
// };

// // MIT控制帧
// struct DM_J3519_MIT_frame {
//    uint16_t P_des : 16;  // 位置给定
//    uint16_t V_des : 12;  // 速度给定
//    uint16_t Kp : 12;     // kp 0~500
//    uint16_t Kd : 12;     // kd 0~5
//    uint16_t T_ff : 12;   // 力矩给定
// };
// void DM_J3519_Enable(uint8_t id, Bus_TopicHandleTypeDef *can_tx_topic) {
//    INTF_CAN_MessageTypeDef msg = {

//        //构建CAN消息
//        .id_type = BSP_CAN_ID_STD,
//        .can_id = id,
//    };
//    //填充使能帧数据
//    msg.data[0] = 0xFF;
//    msg.data[1] = 0xFF;
//    msg.data[2] = 0xFF;
//    msg.data[3] = 0xFF;
//    msg.data[4] = 0xFF;
//    msg.data[5] = 0xFF;
//    msg.data[6] = 0xFF;
//    msg.data[7] = 0xFC;  // 使能帧

//    //发送CAN消息
//    Bus_Publish(can_tx_topic, &msg);
// }

// //设置电机控制模式
// void DM_J3519_ControlMode(uint8_t id, float torque, float MechPosition, float speed, float kp, float kd,
//    Bus_TopicHandleTypeDef *can_tx_topic) {
// struct DM_J3519_MIT_frame frame = {
// .P_des = float_to_uint(MechPosition, P_MIN, P_MAX, 16),
// .V_des = float_to_uint(speed, V_MIN, V_MAX, 12),
// .Kp = float_to_uint(kp, KP_MIN, KP_MAX, 12),
// .Kd = float_to_uint(kd, KD_MIN, KD_MAX, 12),
// .T_ff = float_to_uint(torque, T_MIN, T_MAX, 12),  //给定值转整数
// };

// INTF_CAN_MessageTypeDef msg = {
// .id_type = BSP_CAN_ID_STD,
// .can_id = id};
// //    memcpy(msg.data, &frame, 8);h
// msg.data[0] = (frame.P_des >> 8);
// msg.data[1] = frame.P_des;
// msg.data[2] = (frame.V_des >> 4);
// msg.data[3] = ((frame.V_des & 0xF) << 4) | (frame.Kp >> 8);
// msg.data[4] = frame.Kp;
// msg.data[5] = (frame.Kd >> 4);
// msg.data[6] = ((frame.Kd & 0xF) << 4) | (frame.T_ff >> 8);
// msg.data[7] = frame.T_ff;
// Bus_Publish(can_tx_topic, &msg);
// }

// void DM_J3519_Setmode_t(struct INTF_Motor_Handle *self, INTF_Motor_ModeTypeDef mode) {
//    return;
// }

// void DM_J3519_SetSpeed_t(struct INTF_Motor_Handle *self, float speed) {
//    self->target_speed = speed;
// }

// void DM_J3519_SetAngle_t(struct INTF_Motor_Handle *self, float angle) {
//    self->target_angle = angle;
// }

// void DM_J3519_SetTorque_t(struct INTF_Motor_Handle *self, float torque) {
//    self->target_torque = torque;
// }

// void DM_J3519_Reset_t(struct INTF_Motor_Handle *self) {
// }

// void DM_J3519_Enable_t(struct INTF_Motor_Handle *self) {
// }

// void DM_J3519_Disable_t(struct INTF_Motor_Handle *self) {
// }

// void DM_J3519_MainLoop() {

//    osDelay(1000);
//     for (int i = 0; i < DM_J3519_MOTOR_NUM; i++) {
//         if (g_DM_J3519_motors[i] == NULL)
//             continue;
//         DM_J3519_Enable(g_DM_J3519_motors[i]->motor_id, ((DM_J3519_ResDataTypeDef *)g_DM_J3519_motors[i]->private_data)->can_tx_topic);
//         osDelay(100);
//     }
//    osDelay(1000);
//    while (1) {
//        for (int i = 0; i < DM_J3519_MOTOR_NUM; i++) {

//            // 电机离线检测
//            INTF_Motor_HandleTypeDef *m = g_DM_J3519_motors[i];
//            DM_J3519_ResDataTypeDef *priv = (DM_J3519_ResDataTypeDef *)m->private_data;

//            m->motor_state = MOTOR_STATE_RUNNING;

//            DM_J3519_ControlMode(m->motor_id,
//                                 m->target_torque * m->direction,
//                                 (m->target_angle - priv->angle_offset) * m->direction,  // 修正offset后的角度值
//                                 m->target_speed * m->direction,
//                                 priv->kp,
//                                 priv->kd,
//                                 priv->can_tx_topic);

//            osDelay(0);
//        }

//        osDelay(5);
//    }
// }

// void DM_J3519_CAN_CallBack(void *message, Bus_TopicHandleTypeDef *topic) {
//    INTF_CAN_MessageTypeDef *msg = (INTF_CAN_MessageTypeDef *)message;

//    if (msg->id_type == BSP_CAN_ID_EXT || msg->can_id != MASTER_ID) {
//        return;
//    }

//    INTF_Motor_HandleTypeDef *m = NULL;
//    int i;
//    for (i = 0; i < DM_J3519_MOTOR_NUM; i++) {
//        // 由于达妙的神奇id设计，只能通过id的低四位来匹配电机
//        if ((g_DM_J3519_motors[i]->motor_id & 0xF) == (msg->data[0] & 0xF) &&
//            ((DM_J3519_ResDataTypeDef *)g_DM_J3519_motors[i]->private_data)->can_rx_topic->topic == topic) {
//            m = g_DM_J3519_motors[i];
//            break;
//        }

//        // 未匹配，直接返回
//        if (i == DM_J3519_MOTOR_NUM - 1) {
//            return;
//        }
//    }
//    struct DM_J3519_feedback_frame frame;
//    frame.POS = (msg->data[1] << 8) | msg->data[2];
//    frame.VEL = (msg->data[3] << 4) | (msg->data[4] >> 4);
//    frame.T = ((msg->data[4] & 0xF) << 8) | msg->data[5];
//    m->real_angle = m->direction * uint_to_float(frame.POS, P_MIN, P_MAX, 16) + m->angle_offset;
//    m->real_speed = m->direction * uint_to_float(frame.VEL, V_MIN, V_MAX, 12);
//    m->real_torque = m->direction * uint_to_float(frame.T, T_MIN, T_MAX, 12);
// }

// void DM_J3519_Register(DM_J3519_ConfigTypeDef *config) {
//    INTF_Motor_HandleTypeDef *motor = Bus_SharePtr(config->motor_ptr_name, sizeof(INTF_Motor_HandleTypeDef));
//    motor->motor_id = config->motor_id;
//    motor->motor_mode = config->motor_mode;
//    motor->motor_state = MOTOR_STATE_INIT;
//    motor->target_speed = 0.0f;
//    motor->real_speed = 0.0f;
//    motor->target_angle = 0.0f;
//    motor->real_angle = 0.0f;
//    motor->target_torque = 0.0f;
//    motor->real_torque = 0.0f;
//    motor->angle_offset = config->angle_offset;

//    motor->position = config->speed_position;  ////个人

//    motor->direction = config->direction;

//    motor->set_torque = DM_J3519_SetTorque_t;
//    motor->set_speed = DM_J3519_SetSpeed_t;
//    motor->set_angle = DM_J3519_SetAngle_t;
//    motor->set_mode = DM_J3519_Setmode_t;

//    motor->private_data = JUST_MALLOC(sizeof(DM_J3519_ResDataTypeDef));
//    memset(motor->private_data, 0, sizeof(DM_J3519_ResDataTypeDef));
//    ((DM_J3519_ResDataTypeDef *)motor->private_data)->can_rx_topic = Bus_SubscribeFromName(config->can_rx_topic_name,
//                                                                                           DM_J3519_CAN_CallBack);
//    ((DM_J3519_ResDataTypeDef *)motor->private_data)->can_tx_topic = Bus_TopicRegister(config->can_tx_topic_name);
//    ((DM_J3519_ResDataTypeDef *)motor->private_data)->kp = config->kp;
//    ((DM_J3519_ResDataTypeDef *)motor->private_data)->kd = config->kd;
//    g_DM_J3519_motors[idx++] = motor;
// }


// void DM_J3519_Init() {
//    // 注册电机开始
//    #define OFFSET_ANGLE 12.98f * DEG2RAD
//    #define OFFSET_SPEED 1.0f

//        DM_J3519_ConfigTypeDef leglf_config = {
//            .motor_id = 0x01,
//            .motor_ptr_name = "/motor/leg-l-f",//电机身份标识

//            .direction = 1.0f,                        //旋转方向正(-1.0f为负)
//            .motor_mode = MOTOR_MODE_TORQUE,          //工作模式 扭矩控制
//            .can_rx_topic_name = "/CAN1/RX",
//            .can_tx_topic_name = "/CAN1/TX",          //can
//            .kp = 0.0f,
//            .kd = 0.0f,
//        };
//        DM_J3519_Register(&leglf_config);

//        DM_J3519_ConfigTypeDef leglb_config = {
//            .motor_id = 0x03,
//            .motor_ptr_name = "/motor/leg-l-b",

//            .direction = 1.0f,
//            .motor_mode = MOTOR_MODE_TORQUE,
//            .can_rx_topic_name = "/CAN1/RX",
//            .can_tx_topic_name = "/CAN1/TX",
//            .kp = 0.0f,
//            .kd = 0.0f,
//        };
//        DM_J3519_Register(&leglb_config);

//        DM_J3519_ConfigTypeDef legrb_config = {
//            .motor_id = 0x04,
//            .motor_ptr_name = "/motor/leg-r-b",

//            .direction = -1.0f,
//            .motor_mode = MOTOR_MODE_TORQUE,
//            .can_rx_topic_name = "/CAN1/RX",
//            .can_tx_topic_name = "/CAN1/TX",
//            .kp = 0.0f,
//            .kd = 0.0f,
//        };
//        DM_J3519_Register(&legrb_config);

//        DM_J3519_ConfigTypeDef legrf_config = {
//            .motor_id = 0x02,
//            .motor_ptr_name = "/motor/leg-r-f",

//            .direction = -1.0f,
//            .motor_mode = MOTOR_MODE_TORQUE,
//            .can_rx_topic_name = "/CAN1/RX",
//            .can_tx_topic_name = "/CAN1/TX",
//            .kp = 0.0f,
//            .kd = 0.0f,
//        };
//        DM_J3519_Register(&legrf_config);


//        DM_J3519_ConfigTypeDef spade_config = {
//            .motor_id = 0x05,
//            .motor_ptr_name = "/motor/spade",
//            //.angle_offset = 0,//1.622606f - OFFSET_ANGLE,  //-3.83211327f+OFFSET_ANGLE+2.06f,
//            .direction = -1.0f,
//            .motor_mode = MOTOR_MODE_SPEED,
//            .can_rx_topic_name = "/CAN1/RX",
//            .can_tx_topic_name = "/CAN1/TX",
//            .kp = 0.0f,
//            .kd = 0.5f,
//            //.speed_position = 0,
//        };
//        DM_J3519_Register(&spade_config);


//        // 注册电机结束

//        osThreadDef(DM_J3519_MainLoopTask, DM_J3519_MainLoop, osPriorityLow, 0, 512);
//        DM_J3519_MainLoopTaskHandle = osThreadCreate(osThread(DM_J3519_MainLoopTask), NULL);
//    }

