//
// Created by Ukua on 2024/3/12.
//

#ifndef JUSTFW_INTF_VISION_H
#define JUSTFW_INTF_VISION_H

#include <stdint.h>

typedef struct AutoAim_S2M_Packet
{
    uint8_t header;//0x5A
    float pitch;
    float yaw;
    uint16_t checksum;
} __attribute__((packed)) AutoAim_S2M_PacketTypeDef;

typedef struct AutoAim_M2S_Packet
{
    uint8_t header;//0xA5
    uint8_t is_tracking;
    float x;
    float y;
    float z;
    float yaw;
    float vx;
    float vy;
    float vz;
    float v_yaw;
    float r;
    float another_z;
    float imu_pitch;
    float imu_yaw;
    uint16_t checksum;
} __attribute__((packed)) AutoAim_M2S_PacketTypeDef;

#endif //JUSTFW_INTF_VISION_H
