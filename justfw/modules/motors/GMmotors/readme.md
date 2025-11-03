# GMmotors通用GM电机驱动模块

本模块可以驱动C610、C620电调和M6020电机

使用时在对应电调、电机的实现文件中找到初始化函数，在其中注册电机即可

以C610为例：

```objectivec
void C610_Init() {
    //定义角度环PID参数
    PID_Init_Config_s angle_pid = {
            .Kp=0.0f,
            .Ki=0.0f,
            .Kd=0.0f,
            .MaxOut=1000.0f,
            .DeadBand = 0.0f,
            .Improve=PID_Integral_Limit,
    };
    //定义速度环PID参数
    PID_Init_Config_s speed_pid = {
            .Kp=5.0f,
            .Ki=0.01f,
            .Kd=0.05f,
            .MaxOut=20.0f,
            .DeadBand = 0.0f,
            .Improve=PID_Integral_Limit,
    };
    //定义力矩环PID参数
    PID_Init_Config_s torque_pid = {
            .Kp=100.0f,
            .Ki=0.01f,
            .Kd=1.0f,
            .MaxOut=C610_CURRENT_MAX,
            .DeadBand = 0.0f,
            .Improve=PID_Integral_Limit,
            .IntegralLimit=100.0f,
    };
    //定义电机配置
    C610_ConfigTypeDef config = {
            .motor_id=5, //电机id（对应电调ID，参考官方手册）
            .motor_ptr_name="/motor/shooter_feeder", //电机共享指针名，建议以`/motor/电机名`命名
            .motor_mode=MOTOR_MODE_SPEED, //电机初始状态控制模式
            .direction=-1.0f,//电机方向，参考`intf_motor.h`
            .angle_pid_config=&angle_pid, //角度环pid参数
            .speed_pid_config=&speed_pid, //速度环pid参数
            .torque_pid_config=&torque_pid, //力矩环pid参数
            .can_rx_topic_name="/CAN1/RX", //CAN接收话题名
            .can_tx_topic_name="/CAN1/TX", //CAN发送话题名
    };
    //注册这个电机
    C610_Register(&config);

}
```

