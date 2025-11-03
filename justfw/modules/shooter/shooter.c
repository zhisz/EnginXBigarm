//
// Created by Ukua on 2024/1/23.
//

#include "shooter.h"

osThreadId Shooter_MainLoopTaskHandle;
INTF_Shooter_HandleTypeDef g_shooter = {0};

float fric_speed = 120.0f;

RC_ctrl_t *shooter_rc_ctrl;

///@brief 拨盘供弹
///@param i 拨弹的个数
void Shoot(int i) {
    g_shooter.motor_feeder->set_angle(g_shooter.motor_feeder, g_shooter.motor_feeder->target_angle - 15 * i);
}

///@brief 拨盘抖动函数，解决卡弹
void Vibrating() {
    float old = g_shooter.motor_feeder->real_angle;
    for (int i = 0; i < 5; ++i) {
        g_shooter.motor_feeder->set_angle(g_shooter.motor_feeder, g_shooter.motor_feeder->real_angle - 15);
        osDelay(100);
        g_shooter.motor_feeder->set_angle(g_shooter.motor_feeder, g_shooter.motor_feeder->real_angle + 15);
        osDelay(100);
    }
    g_shooter.motor_feeder->set_angle(g_shooter.motor_feeder, old);

}
void shoot_one(){
    g_shooter.motor_feeder->target_angle += 2 * PI / 2 * 27.0f;
}

void feeder_back(){
    g_shooter.motor_feeder->target_angle -= 2 * PI / 4 * 27.0f;
}

void fric_on(){
    g_shooter.motor_left->target_speed = 300;
    g_shooter.motor_right->target_speed = 300;
}

void fric_off(){
    g_shooter.motor_left->target_speed = 0;
    g_shooter.motor_right->target_speed = 0;
}

void Shooter_MainLoop() {
    int last_switch_left = 0;
    while (1) {
//        if (last_switch_left == 3 && shooter_rc_ctrl[0].rc.switch_left == 1) {
//
//        }
//        if (last_switch_left == 3 && shooter_rc_ctrl[0].rc.switch_left == 2) {
//            g_shooter.motor_feeder->target_angle -= 2 * PI / 4 * 27.0f;
//            /*g_shooter.motor_feeder->target_speed = 0;*/
//        }
//        if (shooter_rc_ctrl[0].rc.switch_right == 1) {
//            g_shooter.motor_left->target_speed = 300;
//            g_shooter.motor_right->target_speed = 300;
//        } else {
//            g_shooter.motor_left->target_speed = 0;
//            g_shooter.motor_right->target_speed = 0;
//        }
//        last_switch_left = shooter_rc_ctrl[0].rc.switch_left;
//        osDelay(5);
    }
}

void Shooter_Init() {
    shooter_rc_ctrl = Bus_SharePtr("DR16", sizeof(RC_ctrl_t));

    g_shooter.motor_left = Bus_SharePtr("/motor/shooter_left", sizeof(INTF_Motor_HandleTypeDef));
    g_shooter.motor_right = Bus_SharePtr("/motor/shooter_right", sizeof(INTF_Motor_HandleTypeDef));
    g_shooter.motor_feeder = Bus_SharePtr("/motor/shooter_feeder", sizeof(INTF_Motor_HandleTypeDef));

    Bus_SubscribeFromName("/signal/shoot_one",shoot_one);
    Bus_SubscribeFromName("/signal/feeder_back",feeder_back);
    Bus_SubscribeFromName("/signal/fric_off",fric_off);
    Bus_SubscribeFromName("/signal/fric_on",fric_on);

    osThreadDef(Shooter_MainLoopTask, Shooter_MainLoop, osPriorityLow, 0, 256);
    Shooter_MainLoopTaskHandle = osThreadCreate(osThread(Shooter_MainLoopTask), NULL);
}

