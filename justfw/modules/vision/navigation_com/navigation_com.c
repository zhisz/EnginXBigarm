//
// Created by Konodoki on 2024-03-21.
//

#include "navigation_com.h"
#include "crc8_crc16.h"
osThreadId Navigation_Com_MainLoopTaskHandle;

Navigation_M2S_PacketTypeDef g_navigation_com_m2s = {0};

Bus_TopicHandleTypeDef *g_navigation_tx;
Bus_TopicHandleTypeDef *g_navigation_rx;

//上位机->下位机
union Navigation_M2S_Union {
    Navigation_M2S_PacketTypeDef navigation_com_m2s;
    uint8_t raw[sizeof(Navigation_M2S_PacketTypeDef)];
};

//下位机->上位机
union Navigation_S2M_Union {
    Navigation_S2M_PacketTypeDef navigation_com_s2m;
    uint8_t raw[sizeof(Navigation_S2M_PacketTypeDef)];
};

void Navigation_Com_Solve(void *message, Bus_TopicHandleTypeDef *topic) {
    INTF_UART_MessageTypeDef *msg = (INTF_UART_MessageTypeDef *) message;
    if (msg->len != sizeof(Navigation_M2S_PacketTypeDef)) {
        return;//长度不对丢包
    }
    union Navigation_M2S_Union navigation_com_m2s_union;
    memcpy(navigation_com_m2s_union.raw, msg->data, sizeof(Navigation_M2S_PacketTypeDef));
    if (navigation_com_m2s_union.navigation_com_m2s.header == 0xA6) {
        if (verify_CRC16_check_sum(navigation_com_m2s_union.raw, sizeof(Navigation_M2S_PacketTypeDef))) { //校验不对丢包{
            memcpy(&g_navigation_com_m2s, navigation_com_m2s_union.raw, sizeof(Navigation_M2S_PacketTypeDef));
            Bus_Publish(g_navigation_rx,&g_navigation_com_m2s);
        }
    }

}

void NavigationData_Transmit() {
    union Navigation_S2M_Union navigation_com_s2m_union ={0};
    navigation_com_s2m_union.navigation_com_s2m.header = 0x6A;

    append_CRC16_check_sum(navigation_com_s2m_union.raw, sizeof(Navigation_S2M_PacketTypeDef));

    INTF_UART_MessageTypeDef msg;
    msg.data = navigation_com_s2m_union.raw;
    msg.len = sizeof(Navigation_S2M_PacketTypeDef);
    Bus_Publish(g_navigation_tx, &msg);
}

void Navigation_Com_MainLoop() {
    while (1) {
        NavigationData_Transmit();
        osDelay(1);
    }
}

void Navigation_Com_Init(void) {
    Bus_SubscribeFromName("USB_RX", Navigation_Com_Solve);

    g_navigation_tx = Bus_TopicRegister("USB_TX");
    g_navigation_rx = Bus_TopicRegister("Navigation_RX");

    osThreadDef(Navigation_Com_MainLoopTask, Navigation_Com_MainLoop, osPriorityLow, 0, 256);
    Navigation_Com_MainLoopTaskHandle = osThreadCreate(osThread(Navigation_Com_MainLoopTask), NULL);
}