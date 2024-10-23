#include <stdint.h>

#pragma pack(push, 1)
typedef struct {
    uint16_t package_counter;
    int16_t temperature;
    uint16_t humidity;
    int16_t voltage;
} payload_t;
#pragma pack(pop)