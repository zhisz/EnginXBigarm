# stm32串口封装

当前仅实现了dma方式接收、发送数据

支持不定长度数据接收

注意:对应串口需要在cubeMX中设置为异步模式，配置DMA，并且更改DMA模式为circular

要注册串口，在init函数中添加以下配置
```objectivec
UART_InstanceConfigTypeDef uart3_config = {
        .UART_handle = &huart3,  //对应板子串口号
        .recv_buff_size = 18,    //接收缓冲区大小，当dma接收空闲/缓冲区满时收取数据
        .tx_topic_name = "/DBUS/TX",  //发送数据的topic，要发送的数据发送到这个话题
        .rx_topic_name = "/DBUS/RX"   //接收数据的topic，接收到的数据发送到这个话题
};
BSP_UART_Register(&uart3_config);
```