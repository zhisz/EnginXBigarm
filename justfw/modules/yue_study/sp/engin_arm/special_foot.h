// //
// // Created by lingyue on 2025/3/22.
// //

// #ifndef SPECIAL_FOOT_H
// #define SPECIAL_FOOT_H

// #include "interface.h"
// #include "tinybus.h"


// #define DM_J3519_MOTOR_NUM 5

// typedef struct DM_Motor_Config {
//    uint32_t motor_id;
//    char *motor_ptr_name;  // 共享指针名
//    float angle_offset;
//    float speed_offset;
//    float speed_position;    //  (个人)速度累计

//    float direction;                    // 电机方向（电机角度、输出乘以该系数，设置-1反向）
//    INTF_Motor_ModeTypeDef motor_mode;  // 运行模式
//    char *can_rx_topic_name;
//    char *can_tx_topic_name;
//    float kp;  // 运控模式kp
//    float kd;  // 运控模式kd
// } DM_J3519_ConfigTypeDef;

// typedef struct DM_Motor_ResData {
//    float angle_offset;
//    Bus_SubscriberTypeDef *can_rx_topic;
//    Bus_TopicHandleTypeDef *can_tx_topic;
//    float kp;  // MIT模式kp
//    float kd;  // MIT模式kd
//    uint32_t last_tick;
// } DM_J3519_ResDataTypeDef;

// void DM_J3519_Init();


// #endif
