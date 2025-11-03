//
// Created by Ukua on 2024/1/21.
//

#include "GMmotors.h"

GM_BufferTypeDef GM_Buffer[GM_BUFFER_NUM]={0};

osThreadId GM_MainLoopTaskHandle;

void GM_ControlSend(){
    for (int i = 0; i < GM_BUFFER_NUM; ++i) {
        INTF_CAN_MessageTypeDef msg = {
                .id_type = BSP_CAN_ID_STD,
        };

        if(GM_Buffer[i].buffer_0x200_not_null) {
            msg.can_id = 0x200;
            memcpy(msg.data, GM_Buffer[i].buffer_0x200, 8);
            Bus_Publish(GM_Buffer[i].can_tx_topic, &msg);
        }
        if(GM_Buffer[i].buffer_0x1FF_not_null) {
            msg.can_id=0x1FF;
            memcpy(msg.data, GM_Buffer[i].buffer_0x1FF, 8);
            Bus_Publish(GM_Buffer[i].can_tx_topic,&msg);
        }
        if(GM_Buffer[i].buffer_0x2FF_not_null) {
            msg.can_id = 0x2FF;
            memcpy(msg.data, GM_Buffer[i].buffer_0x2FF, 8);
            Bus_Publish(GM_Buffer[i].can_tx_topic, &msg);
        }
    }
}

void GM_MainLoop(){
    osDelay(2000);//等待电机启动
    while (1){
        C620_PIDCalc();
        C610_PIDCalc();
        GM6020_PIDCalc();
        GM_ControlSend();
        osDelay(2);
    }
}

void GM_Init() {
    GM_Buffer[0].can_tx_topic = Bus_TopicRegister("/CAN1/TX");
    GM_Buffer[1].can_tx_topic = Bus_TopicRegister("/CAN2/TX");

    //注册接收话题，仅用于比较话题地址，不发布消息
    GM_Buffer[0].can_rx_topic = Bus_TopicRegister("/CAN1/RX");
    GM_Buffer[1].can_rx_topic = Bus_TopicRegister("/CAN2/RX");

    C620_Init();
    C610_Init();
    GM6020_Init();

    osThreadDef(GM_MainLoopTask, GM_MainLoop, osPriorityNormal, 0, 512);
    GM_MainLoopTaskHandle = osThreadCreate(osThread(GM_MainLoopTask), NULL);
}

