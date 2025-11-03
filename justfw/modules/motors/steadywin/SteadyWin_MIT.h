#ifndef __STEADY_WIN_H
#define __STEADY_WIN0_H
#include "interface.h"

#define STEADYWIN_MOTOR_MAX_NUM 10  // 这里定义储存电机句柄最大数量，不需要更改

typedef enum {
    STEADYWIN_MIT_COMMAND_GET_ERR,
    STEADYWIN_MIT_COMMAND_CLEAR_ERR,
    STEADYWIN_MIT_COMMAND_ENABLE,
    STEADYWIN_MIT_COMMAND_DISEABLE,
    STEADYWIN_MIT_COMMAND_SET_ZERO,
    STEADYWIN_MIT_COMMAND_CONTRO,
    STEADYWIN_MIT_COMMAND_NONE,
} STEADYWIN_MIT_COMMAND;

typedef enum {
    STEADYWIN_MIT_ERR_NONE = 0x00,           // 无异常
    STEADYWIN_MIT_ERR_FOC_FREQ_HIGH = 0x01,  // FoC频率太高
    STEADYWIN_MIT_ERR_OVERVOLTAGE = 0x02,    // 过压
    STEADYWIN_MIT_ERR_UNDERVOLTAGE = 0x04,   // 欠压
    STEADYWIN_MIT_ERR_OVERTEMP = 0x08,       // 过温
    STEADYWIN_MIT_ERR_START_FAIL = 0x10,     // 启动失败
    STEADYWIN_MIT_ERR_OVERCURRENT = 0x40,    // 过流
    STEADYWIN_MIT_ERR_SOFTWARE = 0x80        // 软件异常
} STEADYWIN_MIT_ERR;

typedef struct SteadyWin_MIT_Config {
    uint8_t motor_id;
    char *motor_name;
    float angle_offset;
    float direction;
    char *can_rx_topic_name;
    char *can_tx_topic_name;
    float kp;  // 运控模式kp
    float kd;  // 运控模式kd
} SteadyWin_MIT_ConfigTyepdef;

typedef struct SteadyWin_MIT_ResData {
    float angle_offset;
    Bus_SubscriberTypeDef *can_rx_topic;
    Bus_TopicHandleTypeDef *can_tx_topic;
    float kp;
    float kd;
    bool recived;
    STEADYWIN_MIT_ERR err;
    STEADYWIN_MIT_COMMAND last_command;
} SteadyWin_MIT_ResDataTypedef;

void SteadyWin_Init();

#endif