// //
// // Created by Ukua on 2024/2/23.
// //

// #include "stepper_motor.h"

// //
// // Created by Ukua on 2024/1/20.
// //

// #include "C610.h"
// #include "GMmotors.h"

// INTF_Motor_HandleTypeDef *g_stepper_motors[C610_MOTOR_NUM] = {0};

// osThreadId StepperMotor_MainLoopTaskHandle;

// void C610_Setmode_t(struct INTF_Motor_Handle *self, INTF_Motor_ModeTypeDef mode) {
//    self->motor_mode=mode;
// }

// void C610_SetSpeed_t(struct INTF_Motor_Handle *self, float speed) {
//    self->target_speed = speed;
// }

// void C610_SetAngle_t(struct INTF_Motor_Handle *self, float angle) {
//    self->target_angle = angle;
// }

// void C610_SetTorque_t(struct INTF_Motor_Handle *self, float torque) {
//    self->target_torque = torque;
// }

// INTF_Motor_HandleTypeDef *StepperMotor_Register(C610_ConfigTypeDef *config) {
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
//    motor->direction = config->direction;

// //    motor->set_torque = Cybergear_SetTorque_t;
// //    motor->set_speed = Cybergear_SetSpeed_t;
// //    motor->set_angle = Cybergear_SetAngle_t;
// //    motor->set_mode = Cybergear_Setmode_t;

//    motor->private_data = JUST_MALLOC(sizeof(C610_ResDataTypeDef));
//    memset(motor->private_data, 0, sizeof(C610_ResDataTypeDef));
//    C610_ResDataTypeDef *priv = motor->private_data;
//    priv->can_rx_topic = Bus_SubscribeFromName(config->can_rx_topic_name,C610_CAN_CallBack);
//    priv->can_tx_topic = Bus_TopicRegister(config->can_tx_topic_name);

//    PIDInit(&priv->angle_pid, config->angle_pid_config);
//    PIDInit(&priv->speed_pid, config->speed_pid_config);
//    PIDInit(&priv->torque_pid, config->torque_pid_config);

//    motor->set_mode=C610_Setmode_t;
//    motor->set_speed=C610_SetSpeed_t;
//    motor->set_angle=C610_SetAngle_t;
//    motor->set_torque=C610_SetTorque_t;

//    g_c610_motors[motor->motor_id - 1] = motor;


//    for (int i = 0; i < GM_BUFFER_NUM; ++i) {
//        if(GM_Buffer[i].can_tx_topic==priv->can_tx_topic){
//            if(motor->motor_id>4){
//                GM_Buffer[i].buffer_0x1FF_not_null=1;
//            }else{
//                GM_Buffer[i].buffer_0x200_not_null=1;
//            }
//            break;
//        }
//    }

//    return motor;
// }

// void StepperMotor_MainLoop() {
//    while (1){

//        osDelay(5);
//    }
// }

// void StepperMotor_Init() {
//    StepperMotor_Register(&config);
//    osThreadDef(StepperMotor_MainLoopTask, StepperMotor_MainLoop, osPriorityLow, 0, 256);
//    StepperMotor_MainLoopTaskHandle = osThreadCreate(osThread(StepperMotor_MainLoopTask), NULL);
// }
