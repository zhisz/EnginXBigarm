#ifndef __EF_VISON_COM_H
#define __EF_VISON_COM_H

#include "interface.h"

void EF_send_int(int32_t *arr, uint8_t count);
void EF_send_float(float *arr, uint8_t count);
void EF_send_char(char *str);
void EF_send_data(uint8_t *data, uint8_t count);

#endif