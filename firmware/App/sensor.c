#include "sensor.h"
#include "main.h"
#include "stm32l4xx_hal_i2c.h"

#include <string.h>

enum SensorStates
{
    SensorIdle,          // doing nothing
    SensorStart,         // state to start conversion machine
    SensorStartSampling, // state in which ST signal is high, to start a sensortral sample
    SensorIntegrating,   // state in which sensor is integrating ...
    SensorWaiting,       // state after integration is done, but before reading starts
    SensorReading        // state in which sensor is reading
};

volatile int sensorReady;
volatile enum SensorStates sensorState;
volatile int sensorClockSig;
volatile int sensorIntegrationCycleCountdown;
volatile int sensorTriggerCount;
volatile int sensorDataIndex;

int integrationTimeMS;
int averageCount;
int rawSampleBufferPosition;
uint16_t rawSampleBuffer[SENSOR_MAX_AVERAGE_COUNT][SENSOR_PIXEL_COUNT];

extern I2C_HandleTypeDef hi2c1;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;

void setLedPWM(uint32_t dutyCycle)
{
    TIM_OC_InitTypeDef sConfigOC;

    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = dutyCycle;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);

    HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
}

int16_t adcRead()
{
    unsigned char ADSwrite[6];
    uint8_t address = 0b1001000;
    ADSwrite[0] = 0x01;
    // Channel 0
    ADSwrite[1] = 0xC1; // 11000001
    ADSwrite[2] = 0xE3; // 11100011
    HAL_I2C_Master_Transmit(&hi2c1, address << 1, ADSwrite, 3, 100);

    ADSwrite[0] = 0x00;
    HAL_I2C_Master_Transmit(&hi2c1, address << 1, ADSwrite, 1, 100);

    HAL_I2C_Master_Receive(&hi2c1, address << 1, ADSwrite, 2, 100);
    return (ADSwrite[0] << 8 | ADSwrite[1]);
}

void sensorInit()
{
    HAL_GPIO_WritePin(SENSOR_CLK_GPIO_Port, SENSOR_CLK_Pin, 0);
    HAL_GPIO_WritePin(SENSOR_START_GPIO_Port, SENSOR_START_Pin, 0);

    sensorClockSig = 0;
    sensorReady = 0;
    sensorState = SensorIdle;
    sensorDataIndex = 0;

    memset(rawSampleBuffer, 0, sizeof(rawSampleBuffer));
    rawSampleBufferPosition = 0;

    averageCount = SENSOR_MAX_AVERAGE_COUNT;

    HAL_TIM_Base_Start(&htim1);
    HAL_TIM_Base_Start(&htim2);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);
}

void sensorStartScan(int intTimeMS, int avgCount)
{
    if (sensorState == SensorIdle)
    {
        sensorReady = 0;
        integrationTimeMS = intTimeMS;
        averageCount = avgCount < SENSOR_MAX_AVERAGE_COUNT ? avgCount : SENSOR_MAX_AVERAGE_COUNT;

        sensorState = SensorStart;
    }
}

int sensorIsReady()
{
    return sensorReady;
}

void sensorRead(uint16_t *rawSpectrumBuffer)
{
    double averagedSampleBuffer[SENSOR_PIXEL_COUNT];
    memset(averagedSampleBuffer, 0, sizeof(averagedSampleBuffer));
    int i, j;
    for (i = 0; i < averageCount; i++)
    {
        for (j = 0; j < SENSOR_PIXEL_COUNT; j++)
        {
            averagedSampleBuffer[j] += (double)rawSampleBuffer[i][j];
        }
    }
    for (j = 0; j < SENSOR_PIXEL_COUNT; j++)
    {
        rawSpectrumBuffer[j] = (uint16_t)(averagedSampleBuffer[j] / averageCount);
    }

    sensorReady = 0;
}

void handleSensorTrigger()
{
    sensorTriggerCount++; // increment the sensorTriggerCount

    switch (sensorState)
    {
    case SensorStart:
        setLedPWM(2);
        HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_SET);
        sensorReady = 0;
        sensorDataIndex = 0;
        HAL_GPIO_WritePin(SENSOR_START_GPIO_Port, SENSOR_START_Pin, 1);
        sensorIntegrationCycleCountdown = 4 + (integrationTimeMS * 1000 / SENSOR_CLK_PERIOD_US);
        sensorState = SensorStartSampling;
        break;

    case SensorStartSampling:
        sensorIntegrationCycleCountdown--;

        if (sensorIntegrationCycleCountdown == 0)
        {
            HAL_GPIO_WritePin(SENSOR_START_GPIO_Port, SENSOR_START_Pin, 0);
            sensorTriggerCount = 0;
            sensorState = SensorIntegrating;
        }

        break;

    case SensorIntegrating:
        if (sensorTriggerCount >= 48)
        {
            sensorState = SensorWaiting;
        }
        break;

    case SensorWaiting:
        if (sensorTriggerCount >= 87)
        {
            sensorDataIndex = 0;
            sensorState = SensorReading;
        }
        break;
    case SensorReading:
        if (sensorDataIndex < 288)
        {
            rawSampleBuffer[rawSampleBufferPosition][sensorDataIndex] = adcRead();
        }

        sensorDataIndex++;

        if (sensorDataIndex >= 288)
        {
            rawSampleBufferPosition++;
            if (rawSampleBufferPosition > averageCount)
            {
                rawSampleBufferPosition = 0;
                sensorReady = 1;
                sensorState = SensorIdle;
                HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);
                setLedPWM(0);
            }
            else
            {
                sensorState = SensorStart;
            }
        }
        break;

    default:
        break;
    }
}

int sensorGetIntegrationTime()
{
    return integrationTimeMS;
}

int sensorGetAverageCount()
{
    return averageCount;
}

/**
  * @brief EXTI line detection callbacks
  * @param GPIO_Pin: Specifies the pins connected EXTI line
  * @retval None
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == SENSOR_TRIG_Pin)
    {
        // Handle sensortrometer trigger signal
        handleSensorTrigger();
    }
}