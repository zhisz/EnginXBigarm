#include "test.h"

#ifdef TEST_FLASH
typedef struct
{
    char str[20];
    uint8_t data[10];
} flash_test_struct;
#endif

void test_init() {
#ifdef TEST_FLASH

#endif
}