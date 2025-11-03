//
// Created by Ukua on 2024/2/23.
//

#ifndef JUSTFW_LAUNCHER_H
#define JUSTFW_LAUNCHER_H

#include "interface.h"
#include "tinybus.h"

typedef struct Launcher_Config {
    INTF_Motor_HandleTypeDef *motor_f_r;
    INTF_Motor_HandleTypeDef *motor_f_l;
    INTF_Motor_HandleTypeDef *motor_b_l;
    INTF_Motor_HandleTypeDef *motor_b_r;
} Launcher_ConfigTypeDef;

void Launcher_Init();

#endif  // JUSTFW_LAUNCHER_H
