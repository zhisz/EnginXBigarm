//
// Created by 3050 on 2024/2/18.
//

#ifndef JUSTFW_MG4005_H
#define JUSTFW_MG4005_H
#include "tinybus.h"
#include "interface.h"
#include "cmsis_os.h"
#include "MG4005_defines.h"

#define MG4005_MOTOR_NUM 1

typedef struct MG4005_Config {
    uint32_t motor_id;
    char *motor_ptr_name;//共享指针名
    float angle_offset;
    float direction;//电机方向（电机角度、输出乘以该系数，设置-1反向）
    INTF_Motor_ModeTypeDef motor_mode; //运行模式
    char *can_rx_topic_name;
    char *can_tx_topic_name;
}MG4005_ConfigTypeDef;

typedef struct MG4005_ResData{
    Bus_SubscriberTypeDef *can_rx_topic;
    Bus_TopicHandleTypeDef *can_tx_topic;
}MG4005_ResDataTypeDef;

void MG4005_Init();
#endif //JUSTFW_MG4005_H
