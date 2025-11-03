//
// Created by Ukua on 2024/3/28.
//

#include "hero_logic.h"

osThreadId Hero_Logic_MainLoopTaskHandle;  //FreeRTOS

#include "referee.h"                       //裁判系统通信相关

referee_data_t *g_hero_logic_referee;      //裁判系统数据指针

INTF_Chassis_HandleTypeDef *g_hero_logic_chassis;   //底盘控制接口

INTF_Gimbal_HandleTypeDef *g_hero_logic_gimbal;     //云台控制接口

RC_ctrl_t *hero_logic_rc_ctrl;            //遥控器数据结构指针

#define CHASSIS_SPEED_X_MAX 10.0f         //X轴最大速度
#define CHASSIS_SPEED_Y_MAX 10.0f         //Y轴最大速度
#define CHASSIS_SPEED_W_MAX 10.0f         //旋转角速度

#define GIMBAL_YAW_SENSITIVITY 0.0003f    //偏航轴灵敏度
#define GIMBAL_PITCH_SENSITIVITY 0.0002f  //俯仰轴灵敏度

// INTF_Motor_HandleTypeDef *test_m;

// PIDInstance chassis_follow_gimbal;

//总线主题声明（射击控制相关）
Bus_TopicHandleTypeDef *hero_shoot_one, *hero_feeder_back, *hero_fric_on, *hero_fric_off;

//卡尔曼滤波相关
extKalman_t hero_kalman_mouse_x_speed, hero_kalman_mouse_y_speed;  //鼠标速度滤波
float hero_mouse_x_speed_buffer[10] = {0};     //鼠标X手速度缓冲区
float hero_mouse_y_speed_buffer[10] = {0};     //鼠标Y轴速度缓冲区


void Hero_Logic_MainLoop() {
    osDelay(5000);  // 等待其他模块初始化

    //初始化云台模式
    g_hero_logic_gimbal->set_mode(g_hero_logic_gimbal, GIMBAL_MODE_FOLLOW_GYRO);  //陀螺仪跟随模式
    g_hero_logic_gimbal->set_pitch(g_hero_logic_gimbal, 0);   //俯仰角归零

    static uint8_t last_sw_l = 0;    //记录上一次左拨杆状态

    while (1) {
        extern uint8_t g_dr16_is_connected;
        if (g_dr16_is_connected) {//确保遥控器连接
            // 底盘控制
            //
            if (hero_logic_rc_ctrl[0].rc.switch_right == 2) { //右拨杆上位模式
                //键盘控制模式（WASD）
                float vy = (hero_logic_rc_ctrl[0].keyboard.w - hero_logic_rc_ctrl[0].keyboard.s) * 10.0f;
                float vx = (hero_logic_rc_ctrl[0].keyboard.a - hero_logic_rc_ctrl[0].keyboard.d) * 10.0f;
                
                //云台坐标系转换
                g_hero_logic_chassis->target_speed_y = vy * sinf(g_hero_logic_gimbal->real_yaw) - vx * cosf(g_hero_logic_gimbal->real_yaw);
                g_hero_logic_chassis->target_speed_x = vy * cosf(g_hero_logic_gimbal->real_yaw) + vx * sinf(g_hero_logic_gimbal->real_yaw);

                //        g_if_logic_chassis->target_speed_w= if_logic_rc_ctrl[0].keyboard.q*10.0f-if_logic_rc_ctrl[0].keyboard.e*10.0f;
                //            g_if_logic_chassis->target_speed_w = PIDCalculate(&chassis_follow_gimbal, g_if_logic_gimbal->real_yaw, 0);

                //云台控制（摇杆加鼠标）
                g_hero_logic_gimbal->target_yaw -= hero_logic_rc_ctrl[0].rc.rocker_r_ / 660.0f * 0.01f;    //右侧摇杆控制
                g_hero_logic_gimbal->target_yaw -= hero_logic_rc_ctrl[0].mouse.x * GIMBAL_YAW_SENSITIVITY;   //鼠标X轴
                g_hero_logic_gimbal->target_pitch -= hero_logic_rc_ctrl[0].mouse.y * GIMBAL_PITCH_SENSITIVITY; //鼠标Y轴

                //鼠标速度滤波处理
                float speed_x = AverageFilter(KalmanFilter(&hero_kalman_mouse_x_speed, hero_logic_rc_ctrl[0].mouse.x),
                                              hero_mouse_x_speed_buffer, 10);
                float speed_y = AverageFilter(KalmanFilter(&hero_kalman_mouse_y_speed, hero_logic_rc_ctrl[0].mouse.y),
                                              hero_mouse_y_speed_buffer, 10);

                 //应用滤波后的速度
                g_hero_logic_gimbal->target_yaw -= speed_x * GIMBAL_YAW_SENSITIVITY;
                g_hero_logic_gimbal->target_pitch -= speed_y * GIMBAL_PITCH_SENSITIVITY;

                //                    g_if_logic_gimbal->target_yaw+=if_logic_rc_ctrl[0].rc.rocker_l_/660.0f*0.01f;
                //                    g_if_logic_gimbal->target_pitch+=if_logic_rc_ctrl[0].rc.rocker_l1/660.0f*0.05f;

                //            if(if_logic_rc_ctrl[0].keyboard.shift){
                //                g_hero_logic_chassis->target_speed_w = sinf(HAL_GetTick()/1000.0f)*5.0f;
                //            } else{
                //                g_hero_logic_chassis->target_speed_w = sinf(HAL_GetTick()/1000.0f)*5.0f;
                //            }
            } else { //右拨杆非上位模式（常规遥控器控制）
                //底盘摇杆直接控制
                g_hero_logic_chassis->target_speed_x = hero_logic_rc_ctrl[0].rc.rocker_l_ / 660.0f * CHASSIS_SPEED_X_MAX;
                g_hero_logic_chassis->target_speed_y = hero_logic_rc_ctrl[0].rc.rocker_l1 / 660.0f * CHASSIS_SPEED_Y_MAX;
                g_hero_logic_chassis->target_speed_w = -hero_logic_rc_ctrl[0].rc.dial / 660.0f * CHASSIS_SPEED_W_MAX;

                //云台摇杆控制
                g_hero_logic_gimbal->target_yaw += hero_logic_rc_ctrl[0].rc.rocker_r_ / 660.0f * 0.01f;
                g_hero_logic_gimbal->target_pitch += hero_logic_rc_ctrl[0].rc.rocker_r1 / 660.0f * 0.05f;

                //拨杆触发控制
                if (last_sw_l == 3 && hero_logic_rc_ctrl[0].rc.switch_left == 1) {
                    Bus_Publish(hero_shoot_one, NULL); //左拨杆下位触发单发
                }
                if (last_sw_l == 3 && hero_logic_rc_ctrl[0].rc.switch_left == 2) {
                    Bus_Publish(hero_feeder_back, NULL); //中位触发退弹
                }
                if (hero_logic_rc_ctrl[0].rc.switch_right == 1) {
                    Bus_Publish(hero_fric_on, NULL);//右拨杆下位开启摩擦轮
                } else {
                    Bus_Publish(hero_fric_off, NULL);//否则关闭
                }
                last_sw_l = hero_logic_rc_ctrl[0].rc.switch_left;//记录拨杆状态
            }
        }
        osDelay(10);
    }
}

void Hero_Logic_Init() {
    //获取共享资源指针
    g_hero_logic_chassis = Bus_SharePtr("chassis", sizeof(INTF_Chassis_HandleTypeDef));
    g_hero_logic_gimbal = Bus_SharePtr("gimbal", sizeof(INTF_Gimbal_HandleTypeDef));
    hero_logic_rc_ctrl = Bus_SharePtr("DR16", sizeof(INTF_Chassis_HandleTypeDef));

    //注册总线主题
    hero_shoot_one = Bus_TopicRegister("/signal/shoot_one");
    hero_feeder_back = Bus_TopicRegister("/signal/feeder_back");
    hero_fric_on = Bus_TopicRegister("/signal/fric_on");
    hero_fric_off = Bus_TopicRegister("/signal/fric_off");

    //    test_m = Bus_SharePtr("/motor/test",sizeof(INTF_Motor_HandleTypeDef));
    //
    //    PID_Init_Config_s follow_gimbal_config = {
    //            .Kp=8.0f,
    //            .Ki=0.0f,
    //            .Kd=0.0f,
    //            .MaxOut=5.0f * RPM2RPS,
    //            .DeadBand = 0.0f,
    //            .Improve=PID_OutputFilter|PID_Integral_Limit,//|PID_Derivative_On_Measurement,
    //            .IntegralLimit=1.0f,
    //    };
    //    PIDInit(&chassis_follow_gimbal,&follow_gimbal_config);
    //
    //    KalmanCreate(&kalman_mouse_x_speed,1,60);
    //    KalmanCreate(&kalman_mouse_y_speed,1,60);
    /********按键映射***********/
    Bus_SubscribeFromName("/signal/keyboard", NULL);


    //创建FreeRTOS任务
    osThreadDef(Hero_Logic_MainLoopTask, Hero_Logic_MainLoop, osPriorityLow, 0, 512);
    Hero_Logic_MainLoopTaskHandle = osThreadCreate(osThread(Hero_Logic_MainLoopTask), NULL);
}
