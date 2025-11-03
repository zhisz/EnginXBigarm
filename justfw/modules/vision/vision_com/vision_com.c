//
// Created by Ukua on 2024/3/12.
//

#include "vision_com.h"

#include "crc8_crc16.h"

osThreadId Vision_Com_MainLoopTaskHandle;

AutoAim_M2S_PacketTypeDef g_auto_aim_m2s = {0};
AutoAim_S2M_PacketTypeDef g_auto_aim_s2m = {0};

Bus_TopicHandleTypeDef *g_autoaim_tx;

INTF_Chassis_HandleTypeDef *g_vision_chassis;
INTF_Gimbal_HandleTypeDef *g_vision_gimbal;

//上位机->下位机
union AutoAim_M2S_Union {
    AutoAim_M2S_PacketTypeDef auto_aim_m2s;
    uint8_t raw[sizeof(AutoAim_M2S_PacketTypeDef)];
};

//下位机->上位机
union AutoAim_S2M_Union {
    AutoAim_S2M_PacketTypeDef auto_aim_s2m;
    uint8_t raw[sizeof(AutoAim_S2M_PacketTypeDef)];
};

void Vision_Com_Solve(void *message, Bus_TopicHandleTypeDef *topic) {
    INTF_UART_MessageTypeDef *msg = (INTF_UART_MessageTypeDef *) message;
    if (msg->len != sizeof(AutoAim_M2S_PacketTypeDef)) {
        return;//长度不对丢包
    }
    union AutoAim_M2S_Union auto_aim_m2s_union;
    memcpy(auto_aim_m2s_union.raw, msg->data, sizeof(AutoAim_M2S_PacketTypeDef));
    if (auto_aim_m2s_union.auto_aim_m2s.header == 0xA5) {
        if (verify_CRC16_check_sum(auto_aim_m2s_union.raw, sizeof(AutoAim_M2S_PacketTypeDef))) { //校验不对丢包{
            memcpy(&g_auto_aim_m2s, auto_aim_m2s_union.raw, sizeof(AutoAim_M2S_PacketTypeDef));
        }
    }
}

void AutoAim_Transmit() {
    union AutoAim_S2M_Union auto_aim_s2m_union;
    auto_aim_s2m_union.auto_aim_s2m.header = 0x5A;
    auto_aim_s2m_union.auto_aim_s2m.pitch = g_vision_gimbal->real_yaw;
    auto_aim_s2m_union.auto_aim_s2m.yaw = g_vision_gimbal->real_pitch;

    append_CRC16_check_sum(auto_aim_s2m_union.raw, sizeof(AutoAim_S2M_PacketTypeDef));

    INTF_UART_MessageTypeDef msg;
    msg.data = auto_aim_s2m_union.raw;
    msg.len = sizeof(AutoAim_S2M_PacketTypeDef);
    Bus_Publish(g_autoaim_tx, &msg);
}

void Vision_Com_MainLoop() {
    while (1) {
        AutoAim_Transmit();
        osDelay(5);
    }
}

void Vision_Com_Init(void) {
    Bus_SubscribeFromName("USB_RX", Vision_Com_Solve);
    g_autoaim_tx = Bus_TopicRegister("USB_TX");

    g_vision_chassis = Bus_SharePtr("chassis", sizeof(INTF_Chassis_HandleTypeDef));
    g_vision_gimbal = Bus_SharePtr("gimbal", sizeof(INTF_Chassis_HandleTypeDef));

    osThreadDef(Vision_Com_MainLoopTask, Vision_Com_MainLoop, osPriorityLow, 0, 256);
    Vision_Com_MainLoopTaskHandle = osThreadCreate(osThread(Vision_Com_MainLoopTask), NULL);
}