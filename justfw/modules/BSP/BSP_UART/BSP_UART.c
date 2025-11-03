//
// Created by Ukua on 2023/11/2.
//

#include "BSP_UART.h"

osThreadId BSP_UART_MainloopTaskHandle;

/* UART service instance, modules' info would be recoreded here using UARTRegister() */
/* UART服务实例,所有注册了UART的模块信息会被保存在这里 */
static uint8_t idx;
static UART_InstanceTypeDef *UART_instance[DEVICE_UART_CNT] = {NULL};

void UART_TX_CallBack(void *message, Bus_TopicHandleTypeDef *topic) {
    INTF_UART_MessageTypeDef *msg = (INTF_UART_MessageTypeDef *)message;
    for (int i = 0; i < DEVICE_UART_CNT; ++i) {
        if (UART_instance[i]->tx_topic->topic == topic) {
            // 防止高速发送信息时爆炸
            if (fifo_free(UART_instance[i]->tx_fifo) <= 0) {
                continue;
            }
            // 防止发长信息爆炸
            if (xPortGetFreeHeapSize() < msg->len + sizeof(INTF_UART_MessageTypeDef)) {
                continue;
            }
            INTF_UART_MessageTypeDef *new_msg = JUST_MALLOC(sizeof(INTF_UART_MessageTypeDef));
            new_msg->data = JUST_MALLOC(msg->len);
            new_msg->len = msg->len;
            memcpy(new_msg->data, msg->data, msg->len);

            UART_BufferTypeDef buffer;
            buffer.message = new_msg;
            buffer.uart_instance = UART_instance[i];

            fifo_put(UART_instance[i]->tx_fifo, &buffer);
            return;
        }
    }
}

/**
 * @brief 启动串口服务,会在每个实例注册之后自动启用接收,当前实现为DMA接收,后续可能添加IT和BLOCKING接收
 *
 * @todo 串口服务会在每个实例注册之后自动启用接收,当前实现为DMA接收,后续可能添加IT和BLOCKING接收
 *       可能还要将此函数修改为extern,使得module可以控制串口的启停
 *
 * @param _instance instance owned by module,模块拥有的串口实例
 */
void BSP_UART_Instance_Init(UART_InstanceTypeDef *_instance) {
    HAL_UARTEx_ReceiveToIdle_DMA(_instance->uart_handle, _instance->recv_buff, _instance->recv_buff_size);
    // 关闭dma half transfer中断防止两次进入HAL_UARTEx_RxEventCallback()
    // 这是HAL库的一个设计失误,发生DMA传输完成/半完成以及串口IDLE中断都会触发HAL_UARTEx_RxEventCallback()
    // 我们只希望处理第一种和第三种情况,因此直接关闭DMA半传输中断
    __HAL_DMA_DISABLE_IT(_instance->uart_handle->hdmarx, DMA_IT_HT);
}

UART_InstanceTypeDef *BSP_UART_Register(UART_InstanceConfigTypeDef *config) {
    UART_InstanceTypeDef *instance = (UART_InstanceTypeDef *)JUST_MALLOC(sizeof(UART_InstanceTypeDef));
    memset(instance, 0, sizeof(UART_InstanceTypeDef));

    instance->uart_handle = config->UART_handle;
    instance->recv_buff_size = config->recv_buff_size;
    instance->tx_topic = Bus_SubscribeFromName(config->tx_topic_name, UART_TX_CallBack);
    instance->rx_topic = Bus_TopicRegister(config->rx_topic_name);

    instance->tx_fifo = fifo_create(sizeof(INTF_UART_MessageTypeDef), BSP_UART_TX_FIFO_SIZE);

    UART_instance[idx++] = instance;
    BSP_UART_Instance_Init(instance);
    return instance;
}

/* 串口发送时,gstate会被设为BUSY_TX */
uint8_t UART_IsReady(UART_InstanceTypeDef *_instance) {
    if (_instance->uart_handle->gState == HAL_UART_STATE_READY) {
        return 1;
    } else {
        return 0;
    }
}

/**
 * @brief 每次dma/idle中断发生时，都会调用此函数.对于每个uart实例会调用对应的回调进行进一步的处理
 *        例如:视觉协议解析/遥控器解析/裁判系统解析
 *
 * @note  通过__HAL_DMA_DISABLE_IT(huart->hdmarx,DMA_IT_HT)关闭dma half transfer中断防止两次进入HAL_UARTEx_RxEventCallback()
 *        这是HAL库的一个设计失误,发生DMA传输完成/半完成以及串口IDLE中断都会触发HAL_UARTEx_RxEventCallback()
 *        我们只希望处理，因此直接关闭DMA半传输中断第一种和第三种情况
 *
 * @param huart 发生中断的串口
 * @param Size 此次接收到的总数居量,暂时没用
 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
    for (uint8_t i = 0; i < idx; ++i) {
        if (huart == UART_instance[i]->uart_handle) {
            INTF_UART_MessageTypeDef message;
            message.data = UART_instance[i]->recv_buff;
            message.len = Size;
            Bus_Publish(UART_instance[i]->rx_topic, &message);
            memset(UART_instance[i]->recv_buff, 0, Size);  // 接收结束后清空buffer,对于变长数据是必要的
            HAL_UARTEx_ReceiveToIdle_DMA(UART_instance[i]->uart_handle, UART_instance[i]->recv_buff,
                                         UART_instance[i]->recv_buff_size);
            __HAL_DMA_DISABLE_IT(UART_instance[i]->uart_handle->hdmarx, DMA_IT_HT);
            return;
        }
    }
}

/**
 * @brief 当串口发送/接收出现错误时,会调用此函数,此时这个函数要做的就是重新启动接收
 *
 * @note  最常见的错误:奇偶校验/溢出/帧错误
 *
 * @param huart 发生错误的串口
 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
    for (uint8_t i = 0; i < idx; ++i) {
        if (huart == UART_instance[i]->uart_handle) {
            HAL_UARTEx_ReceiveToIdle_DMA(UART_instance[i]->uart_handle, UART_instance[i]->recv_buff,
                                         UART_instance[i]->recv_buff_size);

            __HAL_DMA_DISABLE_IT(UART_instance[i]->uart_handle->hdmarx, DMA_IT_HT);
            return;
        }
    }
}

void BSP_UART_MainLoop() {
    while (1) {
        UART_BufferTypeDef buffer;
        // TODO:考虑同时处理，当前代码在前一个串口阻塞时会停止处理之后的串口消息发送
        for (int i = 0; i < DEVICE_UART_CNT; ++i) {
            while (!fifo_is_empty(UART_instance[i]->tx_fifo)) {
                fifo_get(UART_instance[i]->tx_fifo, &buffer);
                //                while (!UART_IsReady(UART_instance[i])) {
                //                    osDelay(0);
                //                }
                //                HAL_UART_Transmit_DMA(UART_instance[i]->uart_handle, buffer.message->data, buffer.message->len);
                HAL_UART_Transmit(UART_instance[i]->uart_handle, buffer.message->data, buffer.message->len, 1000);
                JUST_FREE(buffer.message->data);
                JUST_FREE(buffer.message);
            }
        }
        osDelay(0);
    }
}

void BSP_UART_Init() {
    extern UART_HandleTypeDef huart1;
    UART_InstanceConfigTypeDef uart1_config = {
        .UART_handle = &huart1,
        .recv_buff_size = 50,
        .tx_topic_name = "/UART/BLE_TX",
        .rx_topic_name = "/UART/BLE_RX"};
    BSP_UART_Register(&uart1_config);

    extern UART_HandleTypeDef huart2;

    UART_InstanceConfigTypeDef uart2_config = {
        .UART_handle = &huart2,
        .recv_buff_size = 50,
        .tx_topic_name = "/UART_TEST1_TX",
        .rx_topic_name = "/UART_TEST2_RX"};
    BSP_UART_Register(&uart2_config);

    // extern UART_HandleTypeDef huart4;
    // UART_InstanceConfigTypeDef uart4_config = {
    //     .UART_handle = &huart4,
    //     .recv_buff_size = 18,
    //     .tx_topic_name = "/DBUS/TX",
    //     .rx_topic_name = "/DBUS/RX"};
    // BSP_UART_Register(&uart4_config);

    extern UART_HandleTypeDef huart5;
    UART_InstanceConfigTypeDef uart5_config = {
        .UART_handle = &huart5,
        .recv_buff_size = 5,
        .tx_topic_name = "/UART5_TX/TX",
        .rx_topic_name = "/UART5_RX/RX"};
    BSP_UART_Register(&uart5_config);

    // osThreadDef(BSP_UART_MainLoop_Task, BSP_UART_MainLoop, osPriorityNormal, 0, 256);
    // BSP_UART_MainloopTaskHandle = osThreadCreate(osThread(BSP_UART_MainLoop_Task), NULL);
}
