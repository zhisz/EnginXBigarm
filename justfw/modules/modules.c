//
// Created by Ukua on 2023/9/18.
//

#include "modules.h"

void Modules_Init() {
    // 初始化开始，禁用中断
    __disable_irq();

    /**********核心模块**********/
    /**
     * TinyBus软总线，实现模块间通信基础通信
     * 包括:pub-sub模式的消息传递，共享指针
     **/
    Bus_Init();
    /**
     * 守护进程，可以用来检测模块掉线
     **/
    Daemon_Init();
    /**
     * DWT计时器，用于高精度计时
     **/
    DWT_Init(168);
    /**
     * 阻塞计时器，提供微秒/毫秒级别的阻塞延时
     **/
    delay_init();

    /**********BSP封装**********/
    /**
     * CAN通信封装
     * 输出话题 /CAN1/RX /CAN2/RX
     * 输入话题 /CAN1/TX /CAN2/TX
     **/
    BSP_CAN_Init();
    /**
     * USB通信封装
     * 输出话题 /USB/RX
     * 输入话题 /USB/TX
     **/
    BSP_USB_Init();
    /**
     * 串口通信封装
     **/
    BSP_UART_Init();
    /**
     * 蜂鸣器封装
     **/
    BSP_Buzzer_Init();
    /**
     * BMI088 IMU传感器封装
     **/
    // BSP_bmi088_Init();
    BSP_LED_Init();
    BSP_button_init();
    BSP_IT_init();
    /**********电机驱动**********/
    /**
     * 小米Cybergear电机驱动
     **/
    //  Cybergear_Init();
    /**
     * lkmotor 系列电机驱动(MF9025)
     **/
    //    MF9025_Init();
    /**
     * RM官方电机驱动合集
     **/
    GM_Init();
    /**
     * DMJ8006电机驱动
     **/
    //DM_J8006_Init();
    DM_Motor_Init();

    //  MG4005_Init();

    /**********其他模块**********/
    /**
     * 日志模块，输出日志到串口
     * 输入话题 LOG
     **/
    // LOG_Init();
    /**
     * DR16遥控器解析
     * 共享指针 DR16
     * 输入话题 /DBUS/RX
     * 输出话题 /signal/DR16/disconnected /signal/DR16/connected /signal/DR16/updated
     * 以及键盘按键按下/释放signal
     **/
    DR16_Init();
    // Leg_Init();
    // MW_Chassis_Init();
    //    OW_Chassis_Init();
    //    Gimbal_Init();
    //    Shooter_Init();
    //    Launcher_Init();
    //    Vision_Com_Init();
    //    Freeretos_Debug_Start();
    //    Navigation_Com_Init();
    // Referee_Init();
    // Infantry_Logic_Init();
    //    PM01_Init();
    //    Hero_Logic_Init();
    //    Unknown_SuperCap_Init();
    //Bu_Init();
    //Yue_Init();
    //GM6020_Init();
//    DM_J3519_Init();
//    SP_logic_Init();
 NRF24L01_Init(); // 初始化NRF24L01

    Engin_Arm_Init();
    // Arm_Logic_Init();
    // test_logic_init();
    // StepperMotor_Init();
    // Enginchas_Logic_Init();
    //  DM_J8006_Init();
    // 初始化完成开启中断
    __enable_irq();
}