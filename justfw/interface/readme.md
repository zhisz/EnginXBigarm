# 通用接口定义

## [interface.h]:
包含所有通用接口头文件,一般情况下引用本文件而不直接引用某个的接口头文件

## [intf_can.h]:
CAN通信接口定义文件

用于BSP_CAN模块中硬件CAN通信实现的抽象

一个合理的BSP_CAN模块应该在接收到CAN总线消息时把消息解析成`INTF_CAN_Message`，并将其广播；在收到其他模块的发来的`INTF_CAN_Message`包时将其发送到CAN总线
### ::enum INTF_CAN_ID_Type
CAN ID类型描述

| 关键字            | 描述     |
|:---------------|:-------|
| BSP_CAN_ID_STD | CAN标准帧 |
| BSP_CAN_ID_EXT | CAN扩展帧 |

### ::struct INTF_CAN_Message
CAN消息结构体,用于储存要发送的/接收到的CAN消息

| 成员      |        类型        | 描述          |
|:--------|:----------------:|:------------|
| id_type | INTF_CAN_ID_Type | 这条消息的CAN帧类型 |
| id      |     uint32_t     | 这条消息的CAN ID |
| data    |    uint8_t[8]    | 这条消息的内容     |

## [intf_uart.h]:
所有基于串口的通信接口定义文件

### ::struct INTF_UART_Message
串口消息结构体,用于储存要发送的/接收到的串口消息

| 成员   |    类型    | 描述       |
|:-----|:--------:|:---------|
| len  | uint16_t | 这条消息的长度  |
| data | uint8_t* | 指向消息头的指针 |


## [intf_motor.h]:
电机控制接口定义文件

这是一个抽象的电机控制接口，用于描述电机的控制方式

包含控制电机最基本的接口，如电机的启动、停止、设置速度等

当仅使用最基本的电机功能时，使用本接口可以实现电机驱动的任意替代，无需考虑具体电机型号/细节

### ::enum INTF_Motor_Mode
电机控制模式描述

| 关键字               | 描述       |
|:------------------|:---------|
| MOTOR_MODE_TORQUE | 电机力矩控制模式 |
| MOTOR_MODE_SPEED  | 电机速度控制模式 |
| MOTOR_MODE_ANGLE  | 电机角度控制模式 |

### ::enum INTF_Motor_State
电机当前运行状态描述

除`MOTOR_STATE_RUNNING`状态外，其他状态均为错误状态，控制时需要注意

| 关键字                 | 描述     |
|:--------------------|:-------|
| MOTOR_STATE_INIT    | 电机待初始化 |
| MOTOR_STATE_RUNNING | 正常运行   |
| MOTOR_STATE_STUCK   | 电机堵转   |
| MOTOR_STATE_ERROR   | 电机产生错误 |

### ::struct INTF_Motor_Handle
电机控制句柄

包含控制一个电机对象所需的所有信息和方法

控制时应当优先考虑使用`INTF_Motor_Handle`中的方法，而不是直接设置`INTF_Motor_Handle`中的成员

例如，当想要控制电机速度时，使用motor.set_speed(motor,speed)而不是直接设置motor.target_speed

| 成员                       |           类型            | 描述                                                      |
|:-------------------------|:-----------------------:|:--------------------------------------------------------|
| motor_id                 |        uint32_t         | 电机的标识id，不同电机驱动注册的电机对象可能会出现重复id，不一定唯一                    |
| motor_mode               | INTF_Motor_ModeTypeDef  | 电机的控制模式                                                 |
| motor_state              | INTF_Motor_StateTypeDef | 电机的当前状态                                                 |
| target_speed             |          float          | 电机当前设定的目标转速，单位rad/s，以逆时针旋转为正向                           |
| real_speed               |          float          | 电机当前实际速度(一般电机反馈值，也可能为估计值)，单位rad/s,以逆时针旋转为正向             |
| target_angle             |          float          | 电机当前设定的目标角度，单位rad，以逆时针旋转为正向，，例如:初始角度0目标角度4*Pi时，电机正向旋转两圈 |
| real_angle               |          float          | 电机当前实际角度(一般电机反馈值，也可能为估计值)，单位rad，以逆时针旋转为正向，可累计多圈         |
| target_torque            |          float          | 电机当前设定的目标力矩，单位N*m，以逆时针旋转为正向                             |
| real_torque              |          float          | 电机当前实际力矩(一般电机反馈值，也可能为估计值)，单位N*m，以逆时针旋转为正向               |
| direction                |          float          | 当为-1时电机方向相反,为1时电机方向不变，原理为电机的测量值和输出值乘以此系数                |
| *set_mode(self,mode)     |          函数指针           | 电机模式设置函数，将`self`的控制模式设置为'mode'                          |
| *set_speed(self,speed)   |          函数指针           | 电机速度设置函数，将`self`的目标速度设置为'speed',逆时针为正,单位rad/s           |
| *set_angle(self,angle)   |          函数指针           | 电机角度设置函数，将`self`的目标角度设置为'angle',逆时针为正,单位rad，可累计多圈       |
| *set_torque(self,torque) |          函数指针           | 电机力矩设置函数，将`self`的目标力矩设置为'torque',逆时针为正,单位N*m            |
| *reset(self)             |          函数指针           | 电机重置函数，将self重启，用于必要情况下临时排除电机报错                          |
| *enable(self)            |          函数指针           | 电机使能函数，使`self`电机进入运行状态，开始执行控制任务                         |
| *disable(self)           |          函数指针           | 电机失能函数，使`self`电机进入停止状态，不再受控                             |
| *private_data            |          void*          | 无类型指针，一般用于存放电机驱动的一些用户无关私有数据，也可以用于放置通用接口没有定义的私有api函数     |




