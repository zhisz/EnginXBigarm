//
// Created by Ukua on 2023/11/4.
//

#ifndef JUSTFW_INTF_LEG_H
#define JUSTFW_INTF_LEG_H

#include <interface.h>
#include "PID.h"
#include "PID_classic.h"

/**
 *  ___x  MB     MF
 * |  E 1  _____  4 A
 * |y     /     \
 *    D 2 \     / 3 B
 *         \   /
 *          \./
 *           5 C
**/
typedef struct INTF_Leg_Handle {
    INTF_Motor_HandleTypeDef *motor_b;// 1
    INTF_Motor_HandleTypeDef *motor_f;// 4

    INTF_Motor_HandleTypeDef *wheel;//驱动轮

    float set_len;   //单位m
    float real_len;   //单位m
    //float set_theta;   //单位rad
    float theta;   //单位rad
    float set_torque;  //单位N*m
    float real_torque;  //单位N*m
    float _set_force;  //单位N
    float real_force;  //单位N
    float set_speed;  //单位rad/s
    float height;  //单位m



    float xB;
    float yB;
    float xD;
    float yD;
    float xC;
    float yC;
    float A0;
    float B0;
    float phi1;
    float phi2;
    float phi3;
    float phi4;
    float phi5;

    float phi1_pred;
    float phi4_pred;

    float legd;
    float phi2_w;
    float phi5_w;
    float theta_w;

    float wheel_w;//修正轮速

    float T_wheel;//驱动轮输出扭矩
    float T_hip;//髋关节输出扭矩

    float fn;//法向支持力

    PID_HandleTypeDef *len_pid;  //腿长pid
} INTF_Leg_HandleTypeDef;

#endif //JUSTFW_INTF_LEG_H
