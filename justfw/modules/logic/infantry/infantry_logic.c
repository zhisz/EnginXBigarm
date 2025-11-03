// //
// // Created by Ukua on 2024/3/23.
// //

// #include "infantry_logic.h"

// osThreadId Infantry_Logic_MainLoopTaskHandle;

// #include "referee.h"//裁判系统通信模块

// referee_data_t *g_if_logic_referee;//裁判系统数据指针

// INTF_Chassis_HandleTypeDef *g_if_logic_chassis;  //地盘控制接口

// INTF_Gimbal_HandleTypeDef *g_if_logic_gimbal;    //云台控制接口

// RC_ctrl_t *if_logic_rc_ctrl;              //遥控器数据结构体指针

// /*地盘运动参数 */
// #define CHASSIS_SPEED_X_MAX 10.0f     
// #define CHASSIS_SPEED_Y_MAX 10.0f
// #define CHASSIS_SPEED_W_MAX 10.0f        //旋转角速度最大值
// /*云台控制灵敏度 */
// #define GIMBAL_YAW_SENSITIVITY 0.0003f   //偏航轴灵敏度
// #define GIMBAL_PITCH_SENSITIVITY 0.0002f  //俯仰轴灵敏度

// INTF_Motor_HandleTypeDef *m_rise;  // 侧方
// INTF_Motor_HandleTypeDef *m_lift;  // 后方

// PIDInstance chassis_follow_gimbal;  //底盘跟随云台的PID实例

// extern TIM_HandleTypeDef htim1;//pwm定时器句柄

// //摩擦轮控制函数
// void fric_off(void) {//关闭摩擦轮
//     __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, 1000);  //设置pwm占空比（1000对应0°）
//     __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, 1000);
// }
// void fric_on(uint16_t cmd) {//开启摩擦轮
//     __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, cmd);  //通过参数设置占空比
//     __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, cmd);
// }

// /**
//  * @brief magazine_on
//  * @author king
//  * @note 开启弹仓盖，0.5ms--0°   500   2.5ms --180°   2500
//  *                  1.5ms--90°
//  */
// void magazine_on(void) {  //开启弹舱（2270对应约180）
//     __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3, 2270);
//     __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_4, 2270);
// }

// /**
//  * @brief magazine_off
//  * @author king
//  * @note 关闭弹仓盖，0.5ms--0°   500   2.5ms --180°   2500
//  */
// void magazine_off(void) {  //关闭弹舱（1300对应约0）
//     __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3, 1300);
//     __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_4, 1300);
// }

// /*鼠标速度滤波 */
// extKalman_t kalman_mouse_x_speed, kalman_mouse_y_speed; // 卡尔曼滤波器实测
// float mouse_x_speed_buffer[10] = {0};   //鼠标X轴速度滑动窗口
// float mouse_y_speed_buffer[10] = {0};   //鼠标y轴速度滑动窗口

// void Infantry_Logic_MainLoop() {
//     osDelay(5000);  // 等待其他模块初始化

//     //    HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);  //启动pwm通道
//     //    HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_2);
//     //    HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_3);
//     //    HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_4);
//     //    fric_off();  //初始化关闭摩擦轮
//     //    g_if_logic_gimbal->set_mode(g_if_logic_gimbal,GIMBAL_MODE_FOLLOW_GYRO);//设置云台模式
//     //    g_if_logic_gimbal->set_pitch(g_if_logic_gimbal,0);

//     while (1) {
//         extern uint8_t g_dr16_is_connected;  //遥控器连接状态
//         if (g_dr16_is_connected) {  //当遥控器连接时
            
//             // 底盘控制
//             //将摇杆值（-660 ~ +660）映射到目标速度
//             g_if_logic_chassis->target_speed_x = if_logic_rc_ctrl[0].rc.rocker_l_ / 660.0f * CHASSIS_SPEED_X_MAX;
//             g_if_logic_chassis->target_speed_y = if_logic_rc_ctrl[0].rc.rocker_l1 / 660.0f * CHASSIS_SPEED_Y_MAX;
//             g_if_logic_chassis->target_speed_w = if_logic_rc_ctrl[0].rc.rocker_r_ / 660.0f * CHASSIS_SPEED_W_MAX;

//             // 抬升机构控制
//             switch (if_logic_rc_ctrl[0].rc.switch_right) {  //右拨杆状态
//             case RC_SW_UP:  //上拨：控制侧方电机
//                 m_rise->set_speed(m_rise, if_logic_rc_ctrl[0].rc.rocker_r1 / 660.0f * 5.0f);
//                 break;
//             case RC_SW_DOWN:  //下拨：控制后方电机
//                 m_lift->set_speed(m_lift, if_logic_rc_ctrl[0].rc.rocker_r1 / 660.0f * 5.0f);
//                 break;
//             case RC_SW_MID:  //中间：控制步进电机
//                 if (if_logic_rc_ctrl[0].rc.rocker_r1 > 220) {
//                     stepper_start(1);//正传
//                 } else if (if_logic_rc_ctrl[0].rc.rocker_r1 < -220) {
//                     stepper_start(0);//反转
//                 } else {
//                     stepper_stop();//停止
//                 }
//             }

//             //    float vy = (if_logic_rc_ctrl[0].keyboard.w - if_logic_rc_ctrl[0].keyboard.s)*10.0f;
//             //    float vx = (if_logic_rc_ctrl[0].keyboard.a - if_logic_rc_ctrl[0].keyboard.d)*10.0f;
//             //    g_if_logic_chassis->target_speed_y = vy * sinf(g_if_logic_gimbal->real_yaw) - vx * cosf(g_if_logic_gimbal->real_yaw);
//             //    g_if_logic_chassis->target_speed_x = vy * cosf(g_if_logic_gimbal->real_yaw) + vx * sinf(g_if_logic_gimbal->real_yaw);

//             //        g_if_logic_chassis->target_speed_w= if_logic_rc_ctrl[0].keyboard.q*10.0f-if_logic_rc_ctrl[0].keyboard.e*10.0f;
//             //            g_if_logic_chassis->target_speed_w = PIDCalculate(&chassis_follow_gimbal, g_if_logic_gimbal->real_yaw, 0);

//             //        g_if_logic_gimbal->target_yaw-=if_logic_rc_ctrl[0].mouse.x*GIMBAL_YAW_SENSITIVITY;
//             //        g_if_logic_gimbal->target_pitch-=if_logic_rc_ctrl[0].mouse.y*GIMBAL_PITCH_SENSITIVITY;


//             /*鼠标输入滤波处理 */
//             float speed_x = AverageFilter(KalmanFilter(&kalman_mouse_x_speed, if_logic_rc_ctrl[0].mouse.x),
//                                           mouse_x_speed_buffer, 10);  
//             float speed_y = AverageFilter(KalmanFilter(&kalman_mouse_y_speed, if_logic_rc_ctrl[0].mouse.y),
//                                           mouse_y_speed_buffer, 10);  

//             //            g_if_logic_gimbal->target_yaw -= speed_x * GIMBAL_YAW_SENSITIVITY;
//             //            g_if_logic_gimbal->target_pitch -= speed_y * GIMBAL_PITCH_SENSITIVITY;

//             //    g_if_logic_gimbal->target_yaw+=if_logic_rc_ctrl[0].rc.rocker_l_/660.0f*0.01f;
//             //    g_if_logic_gimbal->target_pitch+=if_logic_rc_ctrl[0].rc.rocker_l1/660.0f*0.05f;

//             //     if(if_logic_rc_ctrl[0].keyboard.shift){
//             //        g_if_logic_chassis->target_speed_w = sinf(HAL_GetTick()/1000.0f)*5.0f;
//             //    } else{
//             //      g_if_logic_chassis->target_speed_w = sinf(HAL_GetTick()/1000.0f)*5.0f;
//             //    }
//         }
//         osDelay(10);}
// }

// void Infantry_Logic_Init() {
//     g_if_logic_chassis = Bus_SharePtr("chassis", sizeof(INTF_Chassis_HandleTypeDef));
//     //    g_if_logic_gimbal = Bus_SharePtr("gimbal",sizeof(INTF_Gimbal_HandleTypeDef));
//     if_logic_rc_ctrl = Bus_SharePtr("DR16", sizeof(INTF_Chassis_HandleTypeDef));

//     //获取端基控制接口
//     m_rise = Bus_SharePtr("/motor/EN_rise", sizeof(INTF_Motor_HandleTypeDef));
//     m_lift = Bus_SharePtr("/motor/EN_lift", sizeof(INTF_Motor_HandleTypeDef));

//     //    PID_Init_Config_s follow_gimbal_config = {
//     //            .Kp=8.0f,
//     //            .Ki=0.0f,
//     //            .Kd=0.0f,
//     //            .MaxOut=5.0f * RPM2RPS,
//     //            .DeadBand = 0.0f,
//     //            .Improve=PID_OutputFilter|PID_Integral_Limit,//|PID_Derivative_On_Measurement,
//     //            .IntegralLimit=1.0f,
//     //    };
//     //    PIDInit(&chassis_follow_gimbal,&follow_gimbal_config);

//     //初始化滤波器
//     KalmanCreate(&kalman_mouse_x_speed, 1, 60);
//     KalmanCreate(&kalman_mouse_y_speed, 1, 60);
//     /********按键映射***********/
//     Bus_SubscribeFromName("/signal/keyboard", NULL);

//     osThreadDef(Infantry_Logic_MainLoopTask, Infantry_Logic_MainLoop, osPriorityLow, 0, 512);
//     Infantry_Logic_MainLoopTaskHandle = osThreadCreate(osThread(Infantry_Logic_MainLoopTask), NULL);
// }//步兵
