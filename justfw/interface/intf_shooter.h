//
// Created by Ukua on 2024/1/22.
//

#ifndef JUSTFW_INTF_SHOOTER_H
#define JUSTFW_INTF_SHOOTER_H


#include "stdint.h"
#include "intf_motor.h"

typedef struct INTF_Shooter_Handle{
    INTF_Motor_HandleTypeDef *motor_left;
    INTF_Motor_HandleTypeDef *motor_right;
    INTF_Motor_HandleTypeDef *motor_feeder;

    /**
     * @brief 设置底盘速度
     * @param self 底盘句柄
     * @param speed_x 欲设置的底盘x轴速度（m/s）
     * @param speed_y 欲设置的底盘y轴速度（m/s）
     * @param speed_w 欲设置的底盘w轴速度（rad/s）
     */
    void (*set_speed)(struct INTF_Chassis_Handle *self, float speed_x,float speed_y,float speed_w); //设置底盘速度

    void *private_data;
}INTF_Shooter_HandleTypeDef;





#endif //JUSTFW_INTF_SHOOTER_H
