//
// Created by Ukua on 2024/1/21.
//

#ifndef JUSTFW_BSP_BUZZER_H
#define JUSTFW_BSP_BUZZER_H

#include <tinybus.h>

typedef struct{
    const uint16_t *sound;
    const uint16_t *time;
    uint16_t size;
    uint16_t speed;
}Music;

typedef enum{
    D_Do = 262,
    D_Re = 294,
    D_Mi = 330,
    D_Fa = 349,
    D_Sol = 392,
    D_La = 440,
    D_Si = 494,
    Do = 523,
    Re = 587,
    Mi = 659,
    Fa = 698,
    Sol = 784,
    La = 880,
    Si = 988,
    H_Do = 1047,
    H_Re = 1175,
    H_Mi = 1319,
    H_Fa = 1397,
    H_Sol = 1568,
    H_La = 1760,
    H_Si = 1967,
}frequency_sound_e;

typedef enum{
    L_1 =0,
    L_2,
    L_3,
    L_4,
    L_5,
    L_6,
    L_7,
    M_1,
    M_2,
    M_3,
    M_4,
    M_5,
    M_6,
    M_7,
    H_1,
    H_2,
    H_3,
    H_4,
    H_5,
    H_6,
    H_7,
    NONE
}sheet;

void BSP_Buzzer_Init();

#define NO_SOUND 100

#endif //JUSTFW_BSP_BUZZER_H
