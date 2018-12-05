#include "firmware.h"

#include "stm32l432xx.h"
#include "usbd_cdc_if.h"

#include "protocol.h"
#include "sensor.h"


char buffer[1024];

void setup()
{
    protocolInit();
    sensorInit();
}
void loop()
{
    protocolProcess();
}