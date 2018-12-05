#ifndef PROTOCOL_H
#define PROTOCOL_H

#include <stdint.h>
#include "sensor.h"

#define PROTOCOL_START 0xaabb
#define PROTOCOL_DATA_START 0xaf0f
#define PROTOCOL_END 0xcccc
#define PROTOCOL_PAD_SIZE (1024 - (10 + (SENSOR_PIXEL_COUNT * 2)))

typedef struct __attribute__((__packed__))
{
    uint16_t start;
    uint16_t integrationTime;
    uint16_t averageCount;
    uint16_t end;
} ProtocolCommandFrame;

typedef struct __attribute__((__packed__))
{
    uint16_t start;
    uint16_t integrationTime;
    uint16_t averageCount;
    uint16_t dataStart;
    uint16_t spectrum[SENSOR_PIXEL_COUNT];
    uint16_t end;
    uint8_t pad[PROTOCOL_PAD_SIZE];
} ProtocolResponseFrame;

void protocolInit();
void protocolProcess();

#endif