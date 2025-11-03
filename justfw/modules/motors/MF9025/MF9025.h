//
// Created by Ukua on 2023/10/2.
//

#ifndef JUSTFW_MF9025_H
#define JUSTFW_MF9025_H

#include "tinybus.h"
#include "interface.h"
#include "cmsis_os.h"
#include "MF9025_defines.h"

#define MF9025_TORQUE_CONSTANT 0.81f //扭矩常数 16匝为0.32 35匝为0.81

#define MF9025_MOTOR_NUM 2

typedef struct MF9025_Config {
    uint32_t motor_id;
    char *motor_ptr_name;//共享指针名
    float angle_offset;
    float direction;//电机方向（电机角度、输出乘以该系数，设置-1反向）
    INTF_Motor_ModeTypeDef motor_mode; //运行模式
    char *can_rx_topic_name;
    char *can_tx_topic_name;
}MF9025_ConfigTypeDef;

typedef struct MF9025_ResData{
    Bus_SubscriberTypeDef *can_rx_topic;
    Bus_TopicHandleTypeDef *can_tx_topic;
}MF9025_ResDataTypeDef;

void MF9025_Init();

#endif //JUSTFW_MF9025_H
