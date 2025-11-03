#include "EF_com.h"

#include "EF_conf.h"

// 对外通信
static Bus_TopicHandleTypeDef *tx_topic;
static Bus_SubscriberTypeDef *rx_sub;

// 对内通信
static Bus_TopicHandleTypeDef *int_data_topic;
static Bus_TopicHandleTypeDef *float_data_topic;
static Bus_TopicHandleTypeDef *str_data_topic;
static Bus_TopicHandleTypeDef *custom_data_topic;

static uint8_t tx_buffer[EF_TX_BUFFER_SIZE];

static const char *ef_res_msg = "HI,THIS IS STM32F446_01FE";
static const char *ef_ask_msg = "Hello, STM32!";

static uint8_t get_crc(uint8_t *data, uint16_t len) {
    uint8_t result = 0;
    for (uint8_t i = 0; i < len; i++) {
        result += data[i];
    }

    return result;
}

static void EF_transmit(uint16_t cmd, uint8_t *data, uint16_t len) {
    // 帧头跳过2字节
    *(uint16_t *)(tx_buffer + 2) = cmd;
    *(uint16_t *)(tx_buffer + 4) = len;
    memcpy(tx_buffer + 6, data, len);

    tx_buffer[len + 6] = get_crc(data, len);

    INTF_UART_MessageTypeDef msg = {
        .data = tx_buffer,
        .len = len + 7};

    Bus_Publish(tx_topic, &msg);
}

static void EF_rx_callback(void *message, Bus_TopicHandleTypeDef *topic) {
    INTF_UART_MessageTypeDef *msg = (INTF_UART_MessageTypeDef *)message;
    uint16_t head = *(uint16_t *)(msg->data);
    if (head != EF_DATA_HEAD) return;

    uint16_t msg_cmd = *(uint16_t *)(msg->data + 2);
    uint16_t msg_data_len = *(uint16_t *)(msg->data + 4);
    uint8_t *msg_data = msg->data + 6;

    uint8_t msg_crc = msg->data[msg_data_len + 6];
    uint8_t crc = get_crc(msg_data, msg_data_len);

    if (crc != msg_crc) return;

    switch (msg_cmd) {
        case 0x00:
            for (uint8_t i = 0; i < msg_data_len; i++) {
                if (msg_data[i] != ef_res_msg[i]) {
                    break;
                }
            }

            EF_transmit(0x00, (uint8_t *)ef_ask_msg, sizeof(ef_ask_msg));
            break;
        case 0x01:
            EF_int_data_t int_data = {
                .count = msg_data_len / sizeof(int32_t),
                .data = (int32_t *)msg_data};

            Bus_Publish(int_data_topic, &int_data);
            break;
        case 0x02:
            EF_flat_data_t flat_data = {
                .count = msg_data_len / sizeof(float),
                .data = (float *)msg_data};

            Bus_Publish(float_data_topic, &flat_data);
            break;
        case 0x03:
            EF_str_data_t str_data = {
                .data = (char *)msg_data,
                .size = msg_data_len};

            Bus_Publish(custom_data_topic, &str_data);
            break;

        case 0x05:
            EF_custom_data_t custom_data = {
                .data = msg_data,
                .size = msg_data_len};

            Bus_Publish(custom_data_topic, &custom_data);
            break;
    }
}

void EF_init(void) {
    *(uint16_t *)tx_buffer = EF_DATA_HEAD;

    tx_topic = Bus_TopicRegister(EF_TX_TOPIC_NAME);
    rx_sub = Bus_SubscribeFromName(EF_RX_TOPIC_NAME, EF_rx_callback);

    int_data_topic = Bus_TopicRegister(EF_INT_DATA_TOPIC_NAME);
    float_data_topic = Bus_TopicRegister(EF_FLOAT_DATE_TOPIC_NAME);
    custom_data_topic = Bus_TopicRegister(EF_CUSTOM_DATA_TOPIC_NAME);
}

void EF_send_int(int32_t *arr, uint8_t count) {
    uint8_t data[EF_TX_BUFFER_SIZE];
    memcpy(data, arr, count * sizeof(int32_t));
    EF_transmit(0x01, data, count * sizeof(int32_t));
}

void EF_send_float(float *arr, uint8_t count) {
    uint8_t data[EF_TX_BUFFER_SIZE];
    memcpy(data, arr, count * sizeof(float));
    EF_transmit(0x02, data, count * sizeof(float));
}

void EF_send_char(char *str) {
    EF_transmit(0x03, (uint8_t *)str, strlen(str) + 1);
}

void EF_send_data(uint8_t *data, uint8_t len) {
    EF_transmit(0x05, data, len);
}
