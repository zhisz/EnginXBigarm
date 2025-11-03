//
// Created by Ukua on 2024/1/22.
//

#ifndef JUSTFW_INTF_GIMBAL_H
#define JUSTFW_INTF_GIMBAL_H


#include "stdint.h"
#include "intf_motor.h"

typedef enum Gimbal_Mode{
    GIMBAL_MODE_NORMAL = 0,  //开环角度控制
    GIMBAL_MODE_FOLLOW_GYRO, //闭环陀螺仪角度
}Gimbal_ModeTypeDef;

typedef struct INTF_Gimbal_Handle{
    Gimbal_ModeTypeDef mode;

    float target_yaw;//目标yaw轴角度（rad）
    float target_pitch;//目标pitch轴角度（rad）

    float real_yaw;//单位rad，逆时针为正
    float real_pitch;//单位rad，以水平为0，向上为正

    float yaw_limit_max;//单位rad
    float yaw_limit_min;//单位rad
    float pitch_limit_max;//单位rad
    float pitch_limit_min;//单位rad

    INTF_Motor_HandleTypeDef *motor_yaw;
    INTF_Motor_HandleTypeDef *motor_pitch;

    /**
     * @brief 设置pitch轴角度
     * @param self 底盘句柄
     * @param target_pitch 欲设置pitch轴角度（rad）
     */
    void (*set_pitch)(struct INTF_Gimbal_Handle *self, float target_pitch); //设置pitch轴角度
    /**
    * @brief 设置yaw轴角度
    * @param self 底盘句柄
    * @param target_yaw 欲设置yaw轴角度（rad）
    */
    void (*set_yaw)(struct INTF_Gimbal_Handle *self, float target_yaw); //设置yaw轴角度

    /**
    * @brief 设置云台控制模式
    * @param self 底盘句柄
    * @param mode 欲设置的云台模式
    */
    void (*set_mode)(struct INTF_Gimbal_Handle *self, Gimbal_ModeTypeDef mode); //设置yaw轴角度

    void *private_data;
}INTF_Gimbal_HandleTypeDef;





#endif //JUSTFW_INTF_GIMBAL_H
