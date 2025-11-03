//
// Created by Ukua on 2023/8/12.
//

#include "BSP_CAN.h"

extern CAN_HandleTypeDef hcan1, hcan2;

Bus_SubscriberTypeDef *g_bsp_can1_tx,
    *g_bsp_can2_tx;

Bus_TopicHandleTypeDef *g_bsp_can1_rx,
    *g_bsp_can2_rx;

fifo_t *g_can1_tx_fifo,
    *g_can1_rx_fifo,
    *g_can2_tx_fifo,
    *g_can2_rx_fifo;

osThreadId BSP_CAN1_TX_LoopTaskHandle,
    BSP_CAN1_RX_LoopTaskHandle,
    BSP_CAN2_TX_LoopTaskHandle,
    BSP_CAN2_RX_LoopTaskHandle;

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
    CAN_RxHeaderTypeDef rx_header;

    INTF_CAN_MessageTypeDef msg;
    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, msg.data);

    if (rx_header.IDE == CAN_ID_STD) {
        msg.id_type = BSP_CAN_ID_STD;
        msg.can_id = rx_header.StdId;
    } else {
        msg.id_type = BSP_CAN_ID_EXT;
        msg.can_id = rx_header.ExtId;
    }

    if (rx_header.RTR == CAN_RTR_DATA) {
        msg.rtr_type = BSP_CAN_RTR_DATA;
    } else {
        msg.rtr_type = BSP_CAN_RTR_REMOTE;
    }

    if (hcan == &_BSP_CAN_PORT1) {
        fifo_put(g_can1_rx_fifo, &msg);
    } else {
        fifo_put(g_can2_rx_fifo, &msg);
    }
}

/**
 * @brief CAN发送函数
 * @param *data:要发送的数据帧(8byte)
 * @param idType:数据帧的ID类型
 * @param id:数据帧的ID
 * @param port:要发送到的can端口
 */
void BSP_CAN_Transmit(uint8_t *data, INTF_CAN_ID_Type idType, INTF_CAN_RTR_Type rtrType, uint32_t id, CAN_HandleTypeDef *port) {
    CAN_TxHeaderTypeDef tx_header;

    tx_header.DLC = 8;
    tx_header.TransmitGlobalTime = DISABLE;

    switch (rtrType) {
    case BSP_CAN_RTR_DATA:
        tx_header.RTR = CAN_RTR_DATA;
        break;
    case BSP_CAN_RTR_REMOTE:
        tx_header.RTR = CAN_RTR_REMOTE;
        break;
    default:
        tx_header.RTR = CAN_RTR_DATA;
        break;
    }

    switch (idType) {
    case BSP_CAN_ID_STD:
        tx_header.IDE = CAN_ID_STD;
        if (id < 0x7FF) {
            tx_header.StdId = id;
        } else {
            tx_header.StdId = 0;
        }
        break;
    case BSP_CAN_ID_EXT:
        tx_header.IDE = CAN_ID_EXT;
        if (id < 0x1FFFFFFF) {
            tx_header.ExtId = id;
        } else {
            tx_header.ExtId = 0;
        }
        break;
    default:
        tx_header.IDE = CAN_ID_STD;
        if (id < 0x7FF) {
            tx_header.StdId = id;
        } else {
            tx_header.StdId = 0;
        }
        break;
    }
    uint32_t tx_mailBox;

    HAL_CAN_AddTxMessage(port, &tx_header, data, &tx_mailBox);
}

/**
 * @brief CAN标准发送话题回调函数
 * @param *message:消息
 * @param *topic:来源话题
 * @note 该函数会将消息储存在fifo中
 */
void BSP_CAN_TX_CallBack(void *message, Bus_TopicHandleTypeDef *topic) {
    INTF_CAN_MessageTypeDef *msg = (INTF_CAN_MessageTypeDef *)message;
    if (topic == g_bsp_can1_tx->topic) {
        fifo_put(g_can1_tx_fifo, msg);
    } else if (topic == g_bsp_can2_tx->topic) {
        fifo_put(g_can2_tx_fifo, msg);
    } else {
        return;
    }
}

void BSP_CAN1_TX_Loop() {
    INTF_CAN_MessageTypeDef msg;
    while (1) {
        while (!fifo_is_empty(g_can1_tx_fifo)) {
            //            if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) == 0) {
            //                break;
            //            }
            fifo_get(g_can1_tx_fifo, &msg);
            if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) == 0) {
                fifo_put(g_can1_tx_fifo, &msg);
                break;
            }
            BSP_CAN_Transmit(msg.data, msg.id_type, msg.rtr_type, msg.can_id, &hcan1);
        }
        osDelay(1);
    }
}

void BSP_CAN1_RX_Loop() {
    INTF_CAN_MessageTypeDef msg;
    while (1) {
        while (!fifo_is_empty(g_can1_rx_fifo)) {
            fifo_get(g_can1_rx_fifo, &msg);
            Bus_Publish(g_bsp_can1_rx, &msg);
        }
        osDelay(1);
    }
}

void BSP_CAN2_TX_Loop() {
    while (1) {
        INTF_CAN_MessageTypeDef msg;
        while (!fifo_is_empty(g_can2_tx_fifo)) {
            fifo_get(g_can2_tx_fifo, &msg);
            if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan2) == 0) {
                fifo_put(g_can2_tx_fifo, &msg);
                break;
            }
            BSP_CAN_Transmit(msg.data, msg.id_type, msg.rtr_type, msg.can_id, &hcan2);
        }
        osDelay(1);
    }
}

void BSP_CAN2_RX_Loop() {
    INTF_CAN_MessageTypeDef msg;
    while (1) {
        while (!fifo_is_empty(g_can2_rx_fifo)) {
            fifo_get(g_can2_rx_fifo, &msg);
            Bus_Publish(g_bsp_can2_rx, &msg);
        }
        osDelay(1);
    }
}

void BSP_CAN_Init() {
    // CAN过滤器配置
    CAN_FilterTypeDef filter = {
        .FilterBank = 0,
        .FilterMode = CAN_FILTERMODE_IDMASK,
        .FilterScale = CAN_FILTERSCALE_32BIT,
        .FilterIdHigh = 0x0000,
        .FilterIdLow = 0x0000,
        .FilterMaskIdHigh = 0x0000,
        .FilterMaskIdLow = 0x0000,
        .FilterFIFOAssignment = CAN_RX_FIFO0,
        .FilterActivation = ENABLE,
    };
    HAL_CAN_ConfigFilter(&hcan1, &filter);
    HAL_CAN_Start(&hcan1);
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);

    filter.SlaveStartFilterBank = 14;
    filter.FilterBank = 14;

    HAL_CAN_ConfigFilter(&hcan2, &filter);
    HAL_CAN_Start(&hcan2);
    HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);

    // 注册话题
    g_bsp_can1_rx = Bus_TopicRegister("/CAN1/RX");
    g_bsp_can2_rx = Bus_TopicRegister("/CAN2/RX");

    // 订阅话题
    g_bsp_can1_tx = Bus_SubscribeFromName("/CAN1/TX", BSP_CAN_TX_CallBack);
    g_bsp_can2_tx = Bus_SubscribeFromName("/CAN2/TX", BSP_CAN_TX_CallBack);

    // FIFO初始化
    g_can1_tx_fifo = fifo_create(sizeof(INTF_CAN_MessageTypeDef), BSP_CAN_TX_FIFO_SIZE);
    g_can1_rx_fifo = fifo_create(sizeof(INTF_CAN_MessageTypeDef), BSP_CAN_RX_FIFO_SIZE);
    g_can2_tx_fifo = fifo_create(sizeof(INTF_CAN_MessageTypeDef), BSP_CAN_TX_FIFO_SIZE);
    g_can2_rx_fifo = fifo_create(sizeof(INTF_CAN_MessageTypeDef), BSP_CAN_RX_FIFO_SIZE);

    // 主循环任务
    osThreadDef(BSP_CAN1_TX_Loop_Task, BSP_CAN1_TX_Loop, osPriorityLow, 0, 256);
    BSP_CAN1_TX_LoopTaskHandle = osThreadCreate(osThread(BSP_CAN1_TX_Loop_Task), NULL);
    osThreadDef(BSP_CAN1_RX_Loop_Task, BSP_CAN1_RX_Loop, osPriorityLow, 0, 256);
    BSP_CAN1_RX_LoopTaskHandle = osThreadCreate(osThread(BSP_CAN1_RX_Loop_Task), NULL);
    osThreadDef(BSP_CAN2_TX_Loop_Task, BSP_CAN2_TX_Loop, osPriorityLow, 0, 256);
    BSP_CAN2_TX_LoopTaskHandle = osThreadCreate(osThread(BSP_CAN2_TX_Loop_Task), NULL);
    osThreadDef(BSP_CAN2_RX_Loop_Task, BSP_CAN2_RX_Loop, osPriorityLow, 0, 256);
    BSP_CAN2_RX_LoopTaskHandle = osThreadCreate(osThread(BSP_CAN2_RX_Loop_Task), NULL);
}
