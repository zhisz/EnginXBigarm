//
// Created by Ukua on 2023/11/2.
//

#include "DR16.h"
// 遥控器数据
RC_ctrl_t *rc_ctrl;

DaemonInstance *g_rc_daemon_instance;

Bus_SubscriberTypeDef *g_dr16_rx;

Bus_TopicHandleTypeDef *g_dr16_signal_disconnected,  // 遥控器失联
    *g_dr16_signal_connected,                        // 遥控器连接
    *g_dr16_signal_updated;                          // 遥控器数据更新

// 按键按下
Bus_TopicHandleTypeDef *g_dr16_signal_key_w_pressed,
    *g_dr16_signal_key_s_pressed,
    *g_dr16_signal_key_a_pressed,
    *g_dr16_signal_key_d_pressed,
    *g_dr16_signal_key_shift_pressed,
    *g_dr16_signal_key_ctrl_pressed,
    *g_dr16_signal_key_q_pressed,
    *g_dr16_signal_key_e_pressed,
    *g_dr16_signal_key_r_pressed,
    *g_dr16_signal_key_f_pressed,
    *g_dr16_signal_key_g_pressed,
    *g_dr16_signal_key_z_pressed,
    *g_dr16_signal_key_x_pressed,
    *g_dr16_signal_key_c_pressed,
    *g_dr16_signal_key_v_pressed,
    *g_dr16_signal_key_b_pressed;

// 按键释放
Bus_TopicHandleTypeDef *g_dr16_signal_key_w_released,
    *g_dr16_signal_key_s_released,
    *g_dr16_signal_key_a_released,
    *g_dr16_signal_key_d_released,
    *g_dr16_signal_key_shift_released,
    *g_dr16_signal_key_ctrl_released,
    *g_dr16_signal_key_q_released,
    *g_dr16_signal_key_e_released,
    *g_dr16_signal_key_r_released,
    *g_dr16_signal_key_f_released,
    *g_dr16_signal_key_g_released,
    *g_dr16_signal_key_z_released,
    *g_dr16_signal_key_x_released,
    *g_dr16_signal_key_c_released,
    *g_dr16_signal_key_v_released,
    *g_dr16_signal_key_b_released;

uint8_t g_dr16_is_connected = 0;

uint32_t g_last_solve_tick = 0;

/**
 * @brief 矫正遥控器摇杆的值,超过660或者小于-660的值都认为是无效值,置0
 *
 */
static void RectifyRCjoystick() {
    for (uint8_t i = 0; i < 5; ++i) {
        if (abs(*(&rc_ctrl[TEMP].rc.rocker_l_ + i)) > 660)
            *(&rc_ctrl[TEMP].rc.rocker_l_ + i) = 0;
    }
}

/**
 * @brief          remote control protocol resolution
 * @param[in]      sbus_buf: raw data point
 * @param[out]     rc_ctrl: remote control data struct point
 * @retval         none
 */

static void DR16_solve(const uint8_t *sbus_buf) {
    // 遥控器连接signal
    if (!g_dr16_is_connected) {
        g_dr16_is_connected = 1;
        Bus_Publish(g_dr16_signal_connected, NULL);
    }

    // 记录更新时间
    rc_ctrl[TEMP].update_time = HAL_GetTick();

    // 摇杆,直接解算时减去偏置
    rc_ctrl[TEMP].rc.rocker_r_ = ((sbus_buf[0] | (sbus_buf[1] << 8)) & 0x07ff) - RC_CH_VALUE_OFFSET;                               //!< Channel 0
    rc_ctrl[TEMP].rc.rocker_r1 = (((sbus_buf[1] >> 3) | (sbus_buf[2] << 5)) & 0x07ff) - RC_CH_VALUE_OFFSET;                        //!< Channel 1
    rc_ctrl[TEMP].rc.rocker_l_ = (((sbus_buf[2] >> 6) | (sbus_buf[3] << 2) | (sbus_buf[4] << 10)) & 0x07ff) - RC_CH_VALUE_OFFSET;  //!< Channel 2
    rc_ctrl[TEMP].rc.rocker_l1 = (((sbus_buf[4] >> 1) | (sbus_buf[5] << 7)) & 0x07ff) - RC_CH_VALUE_OFFSET;                        //!< Channel 3
    rc_ctrl[TEMP].rc.dial = ((sbus_buf[16] | (sbus_buf[17] << 8)) & 0x07FF) - RC_CH_VALUE_OFFSET;                                  // 左侧拨轮
    RectifyRCjoystick();
    // 开关,0左1右
    rc_ctrl[TEMP].rc.switch_right = ((sbus_buf[5] >> 4) & 0x0003);      //!< Switch right
    rc_ctrl[TEMP].rc.switch_left = ((sbus_buf[5] >> 4) & 0x000C) >> 2;  //!< Switch left

    // 鼠标解析
    rc_ctrl[TEMP].mouse.x = sbus_buf[6] | (sbus_buf[7] << 8);    //!< Mouse X axis
    rc_ctrl[TEMP].mouse.y = sbus_buf[8] | (sbus_buf[9] << 8);    //!< Mouse Y axis
    rc_ctrl[TEMP].mouse.z = sbus_buf[10] | (sbus_buf[11] << 8);  //!< Mouse Z axis
    rc_ctrl[TEMP].mouse.press_l = sbus_buf[12];                  //!< Mouse Left Is Press ?
    rc_ctrl[TEMP].mouse.press_r = sbus_buf[13];                  //!< Mouse Right Is Press ?

    //  位域的按键值解算,直接memcpy即可,注意小端低字节在前,即lsb在第一位,msb在最后. 尚未测试
    *(uint16_t *)&rc_ctrl[TEMP].keyboard = (uint16_t)(sbus_buf[14] | (sbus_buf[15] << 8));

    // 键盘按键值发生变化
    if (memcmp(&rc_ctrl[TEMP], &rc_ctrl[1], sizeof(RC_ctrl_t)) != 0) {
        // 按键按下
        if (rc_ctrl[TEMP].keyboard.w && !rc_ctrl[1].keyboard.w)
            Bus_Publish(g_dr16_signal_key_w_pressed, NULL);
        if (rc_ctrl[TEMP].keyboard.s && !rc_ctrl[1].keyboard.s)
            Bus_Publish(g_dr16_signal_key_s_pressed, NULL);
        if (rc_ctrl[TEMP].keyboard.a && !rc_ctrl[1].keyboard.a)
            Bus_Publish(g_dr16_signal_key_a_pressed, NULL);
        if (rc_ctrl[TEMP].keyboard.d && !rc_ctrl[1].keyboard.d)
            Bus_Publish(g_dr16_signal_key_d_pressed, NULL);
        if (rc_ctrl[TEMP].keyboard.shift && !rc_ctrl[1].keyboard.shift)
            Bus_Publish(g_dr16_signal_key_shift_pressed, NULL);
        if (rc_ctrl[TEMP].keyboard.ctrl && !rc_ctrl[1].keyboard.ctrl)
            Bus_Publish(g_dr16_signal_key_ctrl_pressed, NULL);
        if (rc_ctrl[TEMP].keyboard.q && !rc_ctrl[1].keyboard.q)
            Bus_Publish(g_dr16_signal_key_q_pressed, NULL);
        if (rc_ctrl[TEMP].keyboard.e && !rc_ctrl[1].keyboard.e)
            Bus_Publish(g_dr16_signal_key_e_pressed, NULL);
        if (rc_ctrl[TEMP].keyboard.r && !rc_ctrl[1].keyboard.r)
            Bus_Publish(g_dr16_signal_key_r_pressed, NULL);
        if (rc_ctrl[TEMP].keyboard.f && !rc_ctrl[1].keyboard.f)
            Bus_Publish(g_dr16_signal_key_f_pressed, NULL);
        if (rc_ctrl[TEMP].keyboard.g && !rc_ctrl[1].keyboard.g)
            Bus_Publish(g_dr16_signal_key_g_pressed, NULL);
        if (rc_ctrl[TEMP].keyboard.z && !rc_ctrl[1].keyboard.z)
            Bus_Publish(g_dr16_signal_key_z_pressed, NULL);
        if (rc_ctrl[TEMP].keyboard.x && !rc_ctrl[1].keyboard.x)
            Bus_Publish(g_dr16_signal_key_x_pressed, NULL);
        if (rc_ctrl[TEMP].keyboard.c && !rc_ctrl[1].keyboard.c)
            Bus_Publish(g_dr16_signal_key_c_pressed, NULL);
        if (rc_ctrl[TEMP].keyboard.v && !rc_ctrl[1].keyboard.v)
            Bus_Publish(g_dr16_signal_key_v_pressed, NULL);
        if (rc_ctrl[TEMP].keyboard.b && !rc_ctrl[1].keyboard.b)
            Bus_Publish(g_dr16_signal_key_b_pressed, NULL);

        // 按键释放
        if (!rc_ctrl[TEMP].keyboard.w && rc_ctrl[1].keyboard.w)
            Bus_Publish(g_dr16_signal_key_w_released, NULL);
        if (!rc_ctrl[TEMP].keyboard.s && rc_ctrl[1].keyboard.s)
            Bus_Publish(g_dr16_signal_key_s_released, NULL);
        if (!rc_ctrl[TEMP].keyboard.a && rc_ctrl[1].keyboard.a)
            Bus_Publish(g_dr16_signal_key_a_released, NULL);
        if (!rc_ctrl[TEMP].keyboard.d && rc_ctrl[1].keyboard.d)
            Bus_Publish(g_dr16_signal_key_d_released, NULL);
        if (!rc_ctrl[TEMP].keyboard.shift && rc_ctrl[1].keyboard.shift)
            Bus_Publish(g_dr16_signal_key_shift_released, NULL);
        if (!rc_ctrl[TEMP].keyboard.ctrl && rc_ctrl[1].keyboard.ctrl)
            Bus_Publish(g_dr16_signal_key_ctrl_released, NULL);
        if (!rc_ctrl[TEMP].keyboard.q && rc_ctrl[1].keyboard.q)
            Bus_Publish(g_dr16_signal_key_q_released, NULL);
        if (!rc_ctrl[TEMP].keyboard.e && rc_ctrl[1].keyboard.e)
            Bus_Publish(g_dr16_signal_key_e_released, NULL);
        if (!rc_ctrl[TEMP].keyboard.r && rc_ctrl[1].keyboard.r)
            Bus_Publish(g_dr16_signal_key_r_released, NULL);
        if (!rc_ctrl[TEMP].keyboard.f && rc_ctrl[1].keyboard.f)
            Bus_Publish(g_dr16_signal_key_f_released, NULL);
        if (!rc_ctrl[TEMP].keyboard.g && rc_ctrl[1].keyboard.g)
            Bus_Publish(g_dr16_signal_key_g_released, NULL);
        if (!rc_ctrl[TEMP].keyboard.z && rc_ctrl[1].keyboard.z)
            Bus_Publish(g_dr16_signal_key_z_released, NULL);
        if (!rc_ctrl[TEMP].keyboard.x && rc_ctrl[1].keyboard.x)
            Bus_Publish(g_dr16_signal_key_x_released, NULL);
        if (!rc_ctrl[TEMP].keyboard.c && rc_ctrl[1].keyboard.c)
            Bus_Publish(g_dr16_signal_key_c_released, NULL);
        if (!rc_ctrl[TEMP].keyboard.v && rc_ctrl[1].keyboard.v)
            Bus_Publish(g_dr16_signal_key_v_released, NULL);
        if (!rc_ctrl[TEMP].keyboard.b && rc_ctrl[1].keyboard.b)
            Bus_Publish(g_dr16_signal_key_b_released, NULL);
    }

    // 发布遥控器数据更新signal
    Bus_Publish(g_dr16_signal_updated, NULL);

    DaemonReload(g_rc_daemon_instance);  // 重载守护进程(检查遥控器是否正常工作

    memcpy(&rc_ctrl[1], &rc_ctrl[TEMP], sizeof(RC_ctrl_t));  // 保存上一次的数据,用于按键持续按下和切换的判断
}

void DR16_RX_CallBack(void *message, Bus_TopicHandleTypeDef *topic) {
    INTF_UART_MessageTypeDef *msg = (INTF_UART_MessageTypeDef *)message;
    if (msg->len != 18) {
        return;  // 长度不对，丢包
    }
    DR16_solve(msg->data);
    g_last_solve_tick = HAL_GetTick();
}

void RCLostCallback(void *id) {
    if (g_dr16_is_connected) {
        g_dr16_is_connected = 0;
        Bus_Publish(g_dr16_signal_disconnected, NULL);
    }
}

void DR16_Init() {
    rc_ctrl = Bus_SharePtr("DR16", sizeof(RC_ctrl_t) * 2);

    // 进行守护进程的注册,用于定时检查遥控器是否正常工作
    Daemon_Init_Config_s daemon_conf = {
        .reload_count = 100,  // 100ms未收到数据视为离线,遥控器的接收频率实际上是1000/14Hz(大约70Hz)
        .callback = RCLostCallback,
        .owner_id = NULL,  // 只有1个遥控器,不需要owner_id
    };
    g_rc_daemon_instance = DaemonRegister(&daemon_conf);

    // g_dr16_rx = Bus_SubscribeFromName("/DBUS/RX",DR16_RX_CallBack);
     g_dr16_rx = Bus_SubscribeFromName("/UART/BLE_RX", DR16_RX_CallBack);

    // 遥控器事件
    g_dr16_signal_disconnected = Bus_TopicRegister("/signal/DR16/disconnected");
    g_dr16_signal_connected = Bus_TopicRegister("/signal/DR16/connected");
    g_dr16_signal_updated = Bus_TopicRegister("/signal/DR16/updated");

    // 按键按下signal
    g_dr16_signal_key_w_pressed = Bus_TopicRegister("/signal/control/key_w_pressed");
    g_dr16_signal_key_s_pressed = Bus_TopicRegister("/signal/control/key_s_pressed");
    g_dr16_signal_key_a_pressed = Bus_TopicRegister("/signal/control/key_a_pressed");
    g_dr16_signal_key_d_pressed = Bus_TopicRegister("/signal/control/key_d_pressed");
    g_dr16_signal_key_shift_pressed = Bus_TopicRegister("/signal/control/key_shift_pressed");
    g_dr16_signal_key_ctrl_pressed = Bus_TopicRegister("/signal/control/key_ctrl_pressed");
    g_dr16_signal_key_q_pressed = Bus_TopicRegister("/signal/control/key_q_pressed");
    g_dr16_signal_key_e_pressed = Bus_TopicRegister("/signal/control/key_e_pressed");
    g_dr16_signal_key_r_pressed = Bus_TopicRegister("/signal/control/key_r_pressed");
    g_dr16_signal_key_f_pressed = Bus_TopicRegister("/signal/control/key_f_pressed");
    g_dr16_signal_key_g_pressed = Bus_TopicRegister("/signal/control/key_g_pressed");
    g_dr16_signal_key_z_pressed = Bus_TopicRegister("/signal/control/key_z_pressed");
    g_dr16_signal_key_x_pressed = Bus_TopicRegister("/signal/control/key_x_pressed");
    g_dr16_signal_key_c_pressed = Bus_TopicRegister("/signal/control/key_c_pressed");
    g_dr16_signal_key_v_pressed = Bus_TopicRegister("/signal/control/key_v_pressed");
    g_dr16_signal_key_b_pressed = Bus_TopicRegister("/signal/control/key_b_pressed");

    // 按键释放signal
    g_dr16_signal_key_w_released = Bus_TopicRegister("/signal/control/key_w_released");
    g_dr16_signal_key_s_released = Bus_TopicRegister("/signal/control/key_s_released");
    g_dr16_signal_key_a_released = Bus_TopicRegister("/signal/control/key_a_released");
    g_dr16_signal_key_d_released = Bus_TopicRegister("/signal/control/key_d_released");
    g_dr16_signal_key_shift_released = Bus_TopicRegister("/signal/control/key_shift_released");
    g_dr16_signal_key_ctrl_released = Bus_TopicRegister("/signal/control/key_ctrl_released");
    g_dr16_signal_key_q_released = Bus_TopicRegister("/signal/control/key_q_released");
    g_dr16_signal_key_e_released = Bus_TopicRegister("/signal/control/key_e_released");
    g_dr16_signal_key_r_released = Bus_TopicRegister("/signal/control/key_r_released");
    g_dr16_signal_key_f_released = Bus_TopicRegister("/signal/control/key_f_released");
    g_dr16_signal_key_g_released = Bus_TopicRegister("/signal/control/key_g_released");
    g_dr16_signal_key_z_released = Bus_TopicRegister("/signal/control/key_z_released");
    g_dr16_signal_key_x_released = Bus_TopicRegister("/signal/control/key_x_released");
    g_dr16_signal_key_c_released = Bus_TopicRegister("/signal/control/key_c_released");
    g_dr16_signal_key_v_released = Bus_TopicRegister("/signal/control/key_v_released");
    g_dr16_signal_key_b_released = Bus_TopicRegister("/signal/control/key_b_released");
}
