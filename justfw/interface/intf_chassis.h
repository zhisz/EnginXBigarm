//
// Created by Ukua on 2024/1/22.
//

#ifndef JUSTFW_INTF_CHASSIS_H
#define JUSTFW_INTF_CHASSIS_H


#include "stdint.h"

/*
 * ↑ y (车头方向)
 * |
 * |     x
 * ------>
 */
typedef struct INTF_Chassis_Handle{
    float target_speed_x;
    float target_speed_y;
    float target_speed_w;
    float real_speed_x;
    float real_speed_y;
    float real_speed_w;

    /**
     * @brief 设置底盘速度
     * @param self 底盘句柄
     * @param speed_x 欲设置的底盘x轴速度（m/s）
     * @param speed_y 欲设置的底盘y轴速度（m/s）
     * @param speed_w 欲设置的底盘w轴速度（rad/s）
     */
    void (*set_speed)(struct INTF_Chassis_Handle *self, float speed_x,float speed_y,float speed_w); //设置底盘速度

    void *private_data;
}INTF_Chassis_HandleTypeDef;





#endif //JUSTFW_INTF_CHASSIS_H
