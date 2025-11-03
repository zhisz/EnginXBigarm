//
// Created by Ukua on 2024/3/23.
//

#ifndef JUSTFW_INTF_DR16_H
#define JUSTFW_INTF_DR16_H

#include "stdint.h"

/* ----------------------- RC Switch Definition----------------------------- */
#define RC_SW_UP ((uint16_t)1)    // 开关向上时的值
#define RC_SW_MID ((uint16_t)3)   // 开关中间时的值
#define RC_SW_DOWN ((uint16_t)2)  // 开关向下时的值

/* ----------------------- Data Struct ------------------------------------- */
// 待测试的位域结构体,可以极大提升解析速度

//估计是遥控器
typedef struct
{
    struct
    {
        int16_t rocker_l_;  // 左水平
        int16_t rocker_l1;  // 左竖直
        int16_t rocker_r_;  // 右水平
        int16_t rocker_r1;  // 右竖直
        int16_t dial;       // 侧边拨轮

        uint8_t switch_left;   // 左侧开关
        uint8_t switch_right;  // 右侧开关
    } rc;
    struct
    {
        int16_t x;
        int16_t y;
        int16_t z;
        uint8_t press_l;
        uint8_t press_r;
    } mouse;

    struct DR16_Keyboard {
        uint8_t w : 1;
        uint8_t s : 1;
        uint8_t a : 1;
        uint8_t d : 1;
        uint8_t shift : 1;
        uint8_t ctrl : 1;
        uint8_t q : 1;
        uint8_t e : 1;
        uint8_t r : 1;
        uint8_t f : 1;
        uint8_t g : 1;
        uint8_t z : 1;
        uint8_t x : 1;
        uint8_t c : 1;
        uint8_t v : 1;
        uint8_t b : 1;
    } keyboard;

    //    Key_t key[3];  // 改为位域后的键盘索引,空间减少8倍,速度增加16~倍
    //
    //    uint8_t key_count[3][16];
    uint32_t update_time;  // 更新时间，单位ms (HAL_GetTick())
} RC_ctrl_t;

/* ------------------------- Internal Data ----------------------------------- */

#endif  // JUSTFW_INTF_DR16_H
