//
// Created by Ukua on 2024/1/20.
//

#ifndef JUSTFW_GM6020_H
#define JUSTFW_GM6020_H

#include "tinybus.h"

#include "interface.h"


#define GM6020_MOTOR_NUM 7 //电机数量

//-16384~16384 = -3A~3A
#define GM6020_CURRENT_MAX 16384 //+-3A
#define GM6020_VOLTAGE_MAX 25000 //未知单位
#define GM6020_ANGLE_MAX 8191 //0~360°


//转矩常数
#define GM6020_TORQUE_CONST 0.741f

#define GM6020_ID_BASE 0x204

typedef struct GM6020_Config {
    uint32_t motor_id;
    char *motor_ptr_name;//共享指针名
    float angle_offset;
    float direction;//电机方向（电机角度、输出乘以该系数，设置-1反向）
    INTF_Motor_ModeTypeDef motor_mode; //运行模式
    char *can_rx_topic_name;
    char *can_tx_topic_name;
    float *other_feedback_of_angle;//其他的角度源(用于替换PID闭环)为空时默认使用电机本身传感器
    float *other_feedback_of_speed;//其他的速度源(用于替换PID闭环)为空时默认使用电机本身传感器

    float torque_feed_forward;//力矩环前馈参数
    PID_Init_Config_s *speed_pid_config;
    PID_Init_Config_s *angle_pid_config;
    PID_Init_Config_s *torque_pid_config;
}GM6020_ConfigTypeDef;


typedef struct GM6020_ResData{
    Bus_SubscriberTypeDef *can_rx_topic;
    Bus_TopicHandleTypeDef *can_tx_topic;
    PIDInstance speed_pid;
    PIDInstance angle_pid;
    PIDInstance torque_pid;
    float *other_feedback_of_angle;
    float *other_feedback_of_speed;
    int32_t total_rounds;
    float offset_angle;
    int16_t last_ecd;
    float torque_feed_forward;//力矩环前馈参数
}GM6020_ResDataTypeDef;

void GM6020_Init();
void GM6020_PIDCalc();

#endif //JUSTFW_GM6020_H
