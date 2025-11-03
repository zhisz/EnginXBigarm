#ifndef __INTF_EFCOM_H
#define __INTF_EFCOM_H

#include "stdint.h"

typedef struct
{
    int32_t *data;
    uint8_t count;
} EF_int_data_t;

typedef struct
{
    float *data;
    uint8_t count;
} EF_flat_data_t;

typedef struct
{
    char *data;
    uint8_t size;
} EF_str_data_t;

typedef struct
{
    uint8_t *data;
    uint8_t size;
} EF_custom_data_t;

#endif