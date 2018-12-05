#include "protocol.h"

#include "sensor.h"
#include "usbd_cdc_if.h"

#include <string.h>

#define RX_BUFFER_SIZE 64

int waitForScan;
size_t rxBufferPosition;
uint8_t rxBuffer[RX_BUFFER_SIZE];

int protocolParseCommand(uint8_t *data, size_t *dataSize, ProtocolCommandFrame *command);
void protocolReceiveCallback(uint8_t *data, size_t dataSize);

void protocolInit()
{
    rxBufferPosition = 0;
    waitForScan = 0;
}

void protocolProcess()
{
    if (rxBufferPosition > sizeof(ProtocolCommandFrame) - 1)
    {
        ProtocolCommandFrame command;
        if (protocolParseCommand(rxBuffer, &rxBufferPosition, &command) == 1)
        {
            sensorStartScan(command.integrationTime, command.averageCount);
            waitForScan = 1;
        }
    }
    if (waitForScan == 1 && sensorIsReady())
    {
        ProtocolResponseFrame response;
        sensorRead(response.spectrum);
        response.integrationTime = sensorGetIntegrationTime();
        response.averageCount = sensorGetAverageCount();
        response.start = PROTOCOL_START;
        response.dataStart = PROTOCOL_DATA_START;
        response.end = PROTOCOL_END;
        memset(response.pad, 0, PROTOCOL_PAD_SIZE);

        CDC_Transmit_FS((uint8_t *)&response, sizeof(ProtocolResponseFrame));
        waitForScan = 0;
    }
}

int protocolParseCommand(uint8_t *data, size_t *dataSize, ProtocolCommandFrame *command)
{
    memcpy(command, data, sizeof(ProtocolCommandFrame));
    if (command->start == PROTOCOL_START && command->end == PROTOCOL_END)
    {
        *dataSize = *dataSize - sizeof(ProtocolCommandFrame);
        memcpy(data, &data[sizeof(ProtocolCommandFrame)], *dataSize);
        return 1;
    }
    *dataSize = *dataSize - 1;
    memcpy(data, &data[1], *dataSize);
    return 0;
}

void protocolReceiveCallback(uint8_t *data, size_t dataSize)
{
    int size = dataSize;
    if (size > RX_BUFFER_SIZE - rxBufferPosition)
    {
        size = RX_BUFFER_SIZE - rxBufferPosition;
    }
    memcpy(&rxBuffer[rxBufferPosition], data, size);
    rxBufferPosition += size;
}