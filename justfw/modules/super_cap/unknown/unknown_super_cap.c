//
// Created by Ukua on 2024/3/29.
//

#include "unknown_super_cap.h"

osThreadId Unknown_SuperCapTaskHandle;

struct SuperCap_State {
    float input_voltage;  // 0.01V
    float cap_voltage;    // 0.01V
    float input_current;  // 0.01A
    float target_power;   // 0.01W
} supercap_state;

Bus_SubscriberTypeDef *sc_can;

void Unknown_CAN_CallBack(void *message, Bus_TopicHandleTypeDef *topic) {
    INTF_CAN_MessageTypeDef *msg = (INTF_CAN_MessageTypeDef *)message;

    if (msg->id_type == BSP_CAN_ID_EXT || msg->can_id != 0x211) {
        return;
    }
    uint16_t *pPowerdata = (uint16_t *)msg->data;                 // CAN收到的8个字节的数组
    supercap_state.input_voltage = (float)pPowerdata[0] / 100.f;  // 输入电压
    supercap_state.cap_voltage = (float)pPowerdata[1] / 100.f;    // 电容电压
    supercap_state.input_current = (float)pPowerdata[2] / 100.f;  // 输入电流
    supercap_state.target_power = (float)pPowerdata[3] / 100.f;   // 设定功率
}

void Unknown_SuperCap_SetPower(uint16_t power) {
    if (power < 3000) power = 3000;
    if (power < 13000) power = 13000;
    INTF_CAN_MessageTypeDef msg = {
        .rtr_type = BSP_CAN_RTR_DATA,
        .id_type = BSP_CAN_ID_STD,
        .can_id = 0x210,
    };
    msg.data[0] = power >> 8;
    msg.data[1] = power;
    Bus_Publish(sc_can->topic, &msg);
}

void Unknown_SuperCap_MainLoop() {
    while (1) {
        Unknown_SuperCap_SetPower(8000);
        osDelay(100);
    }
}

void Unknown_SuperCap_Init() {
    sc_can = Bus_SubscribeFromName("/CAN1/RX", Unknown_CAN_CallBack);

    osThreadDef(Unknown_SuperCap_MainLoopTask, Unknown_SuperCap_MainLoop, osPriorityLow, 0, 512);
    Unknown_SuperCapTaskHandle = osThreadCreate(osThread(Unknown_SuperCap_MainLoopTask), NULL);
}
//关于超级电容的控制