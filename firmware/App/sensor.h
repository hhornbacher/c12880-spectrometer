#ifndef SENSOR_H
#define SENSOR_H

#include <stdint.h>
#include "stm32l4xx_hal.h"

#define SENSOR_PIXEL_COUNT 288
#define SENSOR_MAX_AVERAGE_COUNT 32
#define SENSOR_CLK_PERIOD_US 1250

void setLedPWM(uint32_t dutyCycle);
int16_t adcRead();
void sensorInit();
void sensorStartScan(int intTimeMS, int avgCount);
int sensorIsReady();
void sensorRead(uint16_t *rawSpectrumBuffer);
int sensorGetIntegrationTime();
int sensorGetAverageCount();

#endif // SENSOR_H