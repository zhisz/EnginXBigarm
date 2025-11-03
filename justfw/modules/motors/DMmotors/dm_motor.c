 #include "dm_motor.h"

 #define DM_CONNECT_TIME_OUT 1000  // 1s

 #define DM_P_MIN (-12.5f)      //位置
 #define DM_P_MAX 12.5f
 #define DM_V_MIN (-45.0f)      //速度
 #define DM_V_MAX 45.0f
 #define DM_KP_MIN 0.0f         //比例增益
 #define DM_KP_MAX 500.0f
 #define DM_KD_MIN 0.0f         //微分增益
 #define DM_KD_MAX 5.0f
 #define DM_T_MIN (-20.0f)      //扭矩
 #define DM_T_MAX 20.0f

 #define DM_A_MIN (-4 * 3.1415926535)   //角度
 #define DM_A_MAX (4 * 3.1415926535)

 #define DM_MASTER_ID 0x00             //定义主设备ID

 static uint16_t idx = 0;              //电机索引
 INTF_Motor_HandleTypeDef *g_DM_motors[DM_MOTOR_NUM] = {0};    //电机句柄数组

 osThreadId DM_motor_MainLoopTaskHandle;   //电机主循环任务句柄

 //定义电机反馈数据结构
 struct DM_Motor_feedback_frame {
     uint8_t id_with_error : 8;     //ID和错误信息
     uint16_t POS : 16;             //位置
     uint16_t VEL : 12;             //速度
     uint16_t T : 12;               //扭矩
     uint8_t T_MOS : 8;             //MOS温度
     uint8_t T_Rotor : 8;           //转子温度
 };

 // MIT 控制帧
 struct DM_Motor_MIT_frame {
     uint16_t P_des : 16; //目标值
     uint16_t V_des : 12;
     uint16_t Kp : 12;    // kp 0 ~ 500
     uint16_t Kd : 12;    // kd 0 ~ 5
     uint16_t T_ff : 12;  // 力矩给定
 };

 //电机使能函数
 void DM_Motor_Enable(uint8_t id, Bus_TopicHandleTypeDef *can_tx_topic) {
     INTF_CAN_MessageTypeDef msg = {
         .id_type = BSP_CAN_ID_STD,
         .can_id = id,
     };
     for (uint8_t i = 0; i < 8; i++) {
         msg.data[i] = 0xFF;
     }

     msg.data[7] = 0xFC;
     Bus_Publish(can_tx_topic, &msg);
 }

 void DM_motor_Control_MIT(uint8_t id, float torque, float MechPosition, float speed, float kp, float kd,
     Bus_TopicHandleTypeDef *can_tx_topic) {
 struct DM_Motor_MIT_frame frame = {
 .P_des = float_to_uint(MechPosition, DM_P_MIN, DM_P_MAX, 16),
 .V_des = float_to_uint(speed, DM_V_MIN, DM_V_MAX, 12),
 .Kp = float_to_uint(kp, DM_KP_MIN, DM_KP_MAX, 12),
 .Kd = float_to_uint(kd, DM_KD_MIN, DM_KD_MAX, 12),
 .T_ff = float_to_uint(torque, DM_T_MIN, DM_T_MAX, 12)};

 INTF_CAN_MessageTypeDef msg = {
 .id_type = BSP_CAN_ID_STD,
 .can_id = id};

 msg.data[0] = (frame.P_des >> 8);
 msg.data[1] = frame.P_des;
 msg.data[2] = (frame.V_des >> 4);
 msg.data[3] = ((frame.V_des & 0xF) << 4) | (frame.Kp >> 8);
 msg.data[4] = frame.Kp;
 msg.data[5] = (frame.Kd >> 4);
 msg.data[6] = ((frame.Kd & 0xF) << 4) | (frame.T_ff >> 8);
 msg.data[7] = frame.T_ff;
 Bus_Publish(can_tx_topic, &msg);
 }

 void DM_Motor_Setmode_t(struct INTF_Motor_Handle *self, INTF_Motor_ModeTypeDef mode) {
     return;
 }

 void DM_Motor_SetSpeed_t(struct INTF_Motor_Handle *self, float speed) {
     self->target_speed = speed;
 }

 void DM_Motor_SetAngle_t(struct INTF_Motor_Handle *self, float angle) {
     self->target_angle = angle;
 }

 void DM_Motor_SetTorque_t(struct INTF_Motor_Handle *self, float torque) {
     self->target_torque = torque;
 }

 void DM_Motor_Reset_t(struct INTF_Motor_Handle *self) {
 }

 void DM_Motor_Enable_t(struct INTF_Motor_Handle *self) {
 }

 void DM_Motor_Disable_t(struct INTF_Motor_Handle *self) {
 }

 void DM_Motor_MainLoop() {
     osDelay(1000);
     // for (int i = 0; i < DM_MOTOR_NUM; i++) {
     //     if (g_DM_motors[i] == NULL)
     //         continue;
     //     DM_Motor_Enable(g_DM_motors[i]->motor_id, ((DM_Motor_ResDataTypeDef *)g_DM_motors[i]->private_data)->can_tx_topic);
     //     osDelay(100);
     // }

     while (1) {
         for (int i = 0; i < DM_MOTOR_NUM; i++) {
             if (g_DM_motors[i] == NULL)
                 continue;

             if (g_DM_motors[i]->motor_state == MOTOR_STATE_INIT) {
                 DM_Motor_Enable(g_DM_motors[i]->motor_id, ((DM_Motor_ResDataTypeDef *)g_DM_motors[i]->private_data)->can_tx_topic);
                 continue;
             }

             // 电机离线检测
             INTF_Motor_HandleTypeDef *m = g_DM_motors[i];
             DM_Motor_ResDataTypeDef *priv = (DM_Motor_ResDataTypeDef *)m->private_data;

             m->motor_state = MOTOR_STATE_RUNNING;

             DM_motor_Control_MIT(m->motor_id, m->target_torque * m->direction,
                                  (m->target_angle - m->angle_offset) * m->direction,  // 修正offset后的角度值
                                  m->target_speed * m->direction,
                                  priv->kp,
                                  priv->kd,
                                  priv->can_tx_topic);

             osDelay(0);
         }

         osDelay(5);
     }
 }

 void DM_Motor_CAN_CallBack(void *message, Bus_TopicHandleTypeDef *topic) {
     INTF_CAN_MessageTypeDef *msg = (INTF_CAN_MessageTypeDef *)message;

      INTF_Motor_HandleTypeDef *m = NULL;
     int i;
     for (i = 0; i < DM_MOTOR_NUM; i++) {
         // 由于达妙的神奇id设计，只能通过id的低四位来匹配电机
         // if ((g_DM_motors[i]->motor_id & 0xF) == (msg->data[0] & 0xF) &&
         //     ((DM_Motor_ResDataTypeDef *)g_DM_motors[i]->private_data)->can_rx_topic->topic == topic) {
         //     m = g_DM_motors[i];
         //     ((DM_Motor_ResDataTypeDef *)m->private_data)->last_tick = HAL_GetTick();

         //     break;
         // }

         // // 未匹配，直接返回
         // if (i == DM_MOTOR_NUM - 1) {
         //     return;
         // }

         if ((g_DM_motors[i]->motor_id | 0x01 << 4) == (msg->data[0]) &&
             ((DM_Motor_ResDataTypeDef *)g_DM_motors[i]->private_data)->can_rx_topic->topic == topic) {
             m = g_DM_motors[i];
             m->motor_state = MOTOR_STATE_RUNNING;
             break;
         } else {
             m->motor_state = MOTOR_STATE_INIT;
         }
         // 未匹配，直接返回
         if (i == DM_MOTOR_NUM - 1) {
             return;
         }
     }

     struct DM_Motor_feedback_frame frame;
     frame.POS = (msg->data[1] << 8) | msg->data[2];
     frame.VEL = (msg->data[3] << 4) | (msg->data[4] >> 4);
     frame.T = ((msg->data[4] & 0xF) << 8) | msg->data[5];
     m->real_angle = m->direction * uint_to_float(frame.POS, DM_P_MIN, DM_P_MAX, 16) + m->angle_offset;
     m->real_speed = m->direction * uint_to_float(frame.VEL, DM_V_MIN, DM_V_MAX, 12);
     m->real_torque = m->direction * uint_to_float(frame.T, DM_T_MIN, DM_T_MAX, 12);
 }

 void DM_Motor_Register(DM_Motor_ConfigTypeDef *config) {
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
     motor->angle_offset = config->angle_offset;
     motor->direction = config->direction;

     motor->set_torque = DM_Motor_SetTorque_t;
     motor->set_speed = DM_Motor_SetSpeed_t;
     motor->set_angle = DM_Motor_SetAngle_t;
     motor->set_mode = DM_Motor_Setmode_t;

     motor->private_data = JUST_MALLOC(sizeof(DM_Motor_ResDataTypeDef));
     memset(motor->private_data, 0, sizeof(DM_Motor_ResDataTypeDef));
     ((DM_Motor_ResDataTypeDef *)motor->private_data)->can_rx_topic = Bus_SubscribeFromName(config->can_rx_topic_name, DM_Motor_CAN_CallBack);
     ((DM_Motor_ResDataTypeDef *)motor->private_data)->can_tx_topic = Bus_TopicRegister(config->can_tx_topic_name);
     ((DM_Motor_ResDataTypeDef *)motor->private_data)->kp = config->kp;
     ((DM_Motor_ResDataTypeDef *)motor->private_data)->kd = config->kd;
     ((DM_Motor_ResDataTypeDef *)motor->private_data)->last_tick = HAL_GetTick();

     g_DM_motors[idx++] = motor;
 }

 void DM_Motor_Init() {
     DM_Motor_ConfigTypeDef config = {
         .motor_ptr_name = "/motor/arm_6220",
         .angle_offset = 0.855211,
         .can_tx_topic_name = "/CAN1/TX",       
         .can_rx_topic_name = "/CAN1/RX",
         .motor_mode = MOTOR_MODE_MIT,
         .direction = 1.0f,
         .kp = 2.12f,
         .kd = 0.14f,
         .motor_id = 0x01};
     DM_Motor_Register(&config);

    DM_Motor_ConfigTypeDef config_l = {
         .motor_ptr_name = "/motor/arm_l_86",
         .angle_offset = -2.050001,//-2,3
         .can_tx_topic_name = "/CAN1/TX",
         .can_rx_topic_name = "/CAN1/RX",
         .motor_mode = MOTOR_MODE_ANGLE,
         .direction = -1.0f,
         .kp = 17.0,//40.0f,//10.0
         .kd =3.8,//2.3f,//3.0
         .motor_id = 0x13};
     DM_Motor_Register(&config_l);

    DM_Motor_ConfigTypeDef config_r = {
         .motor_ptr_name = "/motor/arm_r_86",
         .angle_offset = 1.055211,//(同步带脱带需重新调)
         .can_tx_topic_name = "/CAN1/TX",       
         .can_rx_topic_name = "/CAN1/RX",
         .motor_mode = MOTOR_MODE_ANGLE,
         .direction = 1.0f,//-1.0f,
         .kp = 17.0,//20.0f,//17
         .kd = 3.8,//4.1f,//3.8
         .motor_id = 0x12};
     DM_Motor_Register(&config_r);
     // config.motor_ptr_name = "/motor/test2";
     // config.direction = -1.0f;
     // config.motor_id = 2;
     // DM_Motor_Register(&config);

     // config.motor_ptr_name = "/motor/test3";
     // config.direction = 1.0f;
     // config.motor_id = 3;
     // DM_Motor_Register(&config);

     osThreadDef(DM_MOTOR_MainLoopTask, DM_Motor_MainLoop, osPriorityLow, 0, 256);
     DM_motor_MainLoopTaskHandle = osThreadCreate(osThread(DM_MOTOR_MainLoopTask), NULL);
 }
