// //
// // Created by Ukua on 2024/3/25.
// //

// #ifndef JUSTFW_DM_J8006_H
// #define JUSTFW_DM_J8006_H

// #include "interface.h"
// #include "tinybus.h"

// #define DM_J8006_MOTOR_NUM 4

// typedef struct DM_J8006_Config {
//     uint32_t motor_id;
//     char *motor_ptr_name;  // 共享指针名
//     float angle_offset;
//     float direction;                    // 电机方向（电机角度、输出乘以该系数，设置-1反向）
//     INTF_Motor_ModeTypeDef motor_mode;  // 运行模式
//     char *can_rx_topic_name;
//     char *can_tx_topic_name;
//     float kp;  // 运控模式kp
//     float kd;  // 运控模式kd
// } DM_J8006_ConfigTypeDef;

// typedef struct DM_J8006_ResData {
//     float angle_offset;
//     Bus_SubscriberTypeDef *can_rx_topic;
//     Bus_TopicHandleTypeDef *can_tx_topic;
//     float kp;  // MIT模式kp
//     float kd;  // MIT模式kd
// } DM_J8006_ResDataTypeDef;

// void DM_J8006_Init();

// #endif  // JUSTFW_DM_J8006_H
