//
// Created by Ukua on 2024/1/20.
//

#ifndef JUSTFW_C610_H
#define JUSTFW_C610_H

#include "tinybus.h"

#include "interface.h"


#define C610_MOTOR_NUM 8 //电机数量

//-16384~16384 = -20A~20A
#define C610_CURRENT_MAX 10000 //+-10A
#define C610_ANGLE_MAX 8191 //0~360°


//转矩常数
#define C610_TORQUE_CONST 0.18f

#define C610_ID_BASE 0x200

typedef struct C610_Config {
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
}C610_ConfigTypeDef;


typedef struct C610_ResData{
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
}C610_ResDataTypeDef;

void C610_Init();
void C610_PIDCalc();

int16_t C610_Torque2Current(float torque);

void C610_CAN_CallBack();

#endif //JUSTFW_C610_H
