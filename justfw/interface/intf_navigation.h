//
// Created by Konodoki on 2024-03-21.
//

#ifndef JUSTFW_INTF_NAVIGATION_H
#define JUSTFW_INTF_NAVIGATION_H
#include <stdint.h>

typedef struct Navigation_S2M_Packet
{
    uint8_t header;//0x6A
    //  Robot HP （1-英雄 2-工程 3-步兵 4-步兵 5-步兵 7-哨兵 8-前哨站 9-基地）
    // 血量
    float game_time ; //比赛时间
    int red_1_robot_hp;
    int red_2_robot_hp;
    int red_3_robot_hp;
    int red_4_robot_hp;
    int red_5_robot_hp;
    int red_7_robot_hp;
    int red_outpost_hp;
    int red_base_hp;

    int blue_1_robot_hp;
    int blue_2_robot_hp;
    int blue_3_robot_hp;
    int blue_4_robot_hp;
    int blue_5_robot_hp;
    int blue_7_robot_hp;
    int blue_outpost_hp;
    int blue_base_hp;
    uint8_t rfid_patrol_status;
    int robot_id;  // 机器人ID （1~7->红，101~107->蓝）
    int current_hp;

    // v1.6 0x0202 实时功率热量数据
    int shooter_heat;  // 枪口热量
    //1 .6 0x020B 地面机器人位置数据

    float hero_x;
    float hero_y;

    float engineer_x;
    float engineer_y;

    float standard_3_x;
    float standard_3_y;

    float standard_4_x;
    float standard_4_y;

    float standard_5_x;
    float standard_5_y;
    uint16_t checksum;
} __attribute__((packed)) Navigation_S2M_PacketTypeDef;

typedef struct Navigation_M2S_Packet
{
    uint8_t header;//0xA6
    float vx;
    float vy;
    float vw;
    uint16_t checksum;
} __attribute__((packed)) Navigation_M2S_PacketTypeDef;
#endif //JUSTFW_INTF_NAVIGATION_H
