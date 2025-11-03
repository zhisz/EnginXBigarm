#include "BSP_USB.h"

#include <stdarg.h>

#include "BSP_USB_config.h"
#include "fifo.h"

static osThreadId BSP_USB_MainLoopTaskHandle;

static Bus_SubscriberTypeDef *g_bsp_usb_tx;
static Bus_TopicHandleTypeDef *g_bsp_usb_rx;

static fifo_t *g_bsp_usb_rx_fifo;

static uint8_t rx_delay = 0;
static uint16_t rx_package_index = 0;
static uint8_t rx_package[BSP_USB_RX_PACKAGE_BUFFER_SIZE];

static uint8_t tx_buffer[BSP_USB_TX_BUFFER_SIZE];
static uint16_t tx_buffer_index = 0;

static void BSP_USB_TX_CallBack(void *message, Bus_TopicHandleTypeDef *topic) {
    INTF_UART_MessageTypeDef *msg = (INTF_UART_MessageTypeDef *)message;
    if (tx_buffer_index + msg->len > BSP_USB_TX_BUFFER_SIZE) return;

    memcpy(tx_buffer + tx_buffer_index, msg->data, msg->len);
    tx_buffer_index += msg->len;
}

// USB接收回调
void usbd_cdc_rx_callback(uint8_t *data, uint32_t len) {
    if (rx_delay > 0 || len == BSP_USB_RX_PACKAGE_MAX_SIZE) {
        // 发生了分包
        if (len + rx_package_index < BSP_USB_RX_PACKAGE_BUFFER_SIZE) {
            memcpy(rx_package + rx_package_index, data, len);
            rx_package_index += len;
            if (len == BSP_USB_RX_PACKAGE_MAX_SIZE)
                rx_delay = BSP_USB_RX_PACKAGE_DLAY;
            else
                rx_delay = 0;
        }
    } else {
        INTF_UART_MessageTypeDef message = {
            .data = data,
            .len = len};
        fifo_put(g_bsp_usb_rx_fifo, &message);
    }
}

static void BSP_USB_MainLoop() {
    while (1) {
        INTF_UART_MessageTypeDef msg;

        if (rx_delay == 0 && rx_package_index > 0) {
            msg.data = rx_package;
            msg.len = rx_package_index;
            fifo_put(g_bsp_usb_rx_fifo, &msg);
            rx_package_index = 0;
        } else if (rx_delay > 0)
            rx_delay--;

        while (!fifo_is_empty(g_bsp_usb_rx_fifo)) {
            fifo_get(g_bsp_usb_rx_fifo, &msg);
            Bus_Publish(g_bsp_usb_rx, &msg);
        }

        if (tx_buffer_index > 0) {
            extern uint8_t CDC_Transmit_FS(uint8_t *Buf, uint16_t Len);
            CDC_Transmit_FS(tx_buffer, tx_buffer_index);
            tx_buffer_index = 0;
        }

        osDelay(1);
    }
}

void BSP_USB_Init(void) {
    g_bsp_usb_rx = Bus_TopicRegister(BSP_USB_RX_TOPIC_NAME);
    g_bsp_usb_tx = Bus_SubscribeFromName(BSP_USB_TX_TOPIC_NAME, BSP_USB_TX_CallBack);

    g_bsp_usb_rx_fifo = fifo_create(BSP_USB_RX_FIFO_SIZE, sizeof(INTF_UART_MessageTypeDef));

    osThreadDef(BSP_USB_MainLoopTask, BSP_USB_MainLoop, osPriorityNormal, 0, 256);
    BSP_USB_MainLoopTaskHandle = osThreadCreate(osThread(BSP_USB_MainLoopTask), NULL);
}

void USB_printf(const char *fmt, ...) {
    va_list args;
    va_start(args, fmt);
    size_t len;
    len = vsnprintf((char *)tx_buffer + tx_buffer_index, BSP_USB_TX_BUFFER_SIZE - tx_buffer_index, fmt, args);
    va_end(args);
    tx_buffer_index += len;
}
