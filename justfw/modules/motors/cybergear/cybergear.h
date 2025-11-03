//
// Created by Ukua on 2023/9/24.
//

#ifndef JUSTFW_CYBERGEAR_H
#define JUSTFW_CYBERGEAR_H

#include "cybergear_defines.h"
#include "cybergear_config.h"
#include "tinybus.h"
#include "interface.h"
#include "cmsis_os.h"

#include "stdio.h"

#define CYBERGEAR_MOTOR_NUM 4 //电机数量

typedef struct Cybergear_Config {
    uint32_t motor_id;
    char *motor_ptr_name;//共享指针名
    float angle_offset;
    float direction;//电机方向（电机角度、输出乘以该系数，设置-1反向）
    INTF_Motor_ModeTypeDef motor_mode; //运行模式
    char *can_rx_topic_name;
    char *can_tx_topic_name;
    float kp;//运控模式kp
    float kd;//运控模式kd
}Cybergear_ConfigTypeDef;


typedef struct Cybergear_ResData{
    float angle_offset;
    Bus_SubscriberTypeDef *can_rx_topic;
    Bus_TopicHandleTypeDef *can_tx_topic;
    float kp;//运控模式kp
    float kd;//运控模式kd
}Cybergear_ResDataTypeDef;

void Cybergear_Init();

#endif //JUSTFW_CYBERGEAR_H
