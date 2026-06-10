#ifndef SAMPLE_H
#define SAMPLE_H

#include <stdint.h>

#define SAMPLE_VoltageReference (2.907f)
#define SAMPLE_ADCResolution (4096U)

typedef enum {
    Sample_CH1 = 0,
    Sample_CH2,
    Sample_CH3,
    Sample_CH4,
    Sample_CH7,
    Sample_CH8, 
    Sample_CH9,
    Sample_CH12,
    Sample_CH15,
    Sample_ChannelCount
} Sample_ChannelTypeDef;

typedef enum {
    SAMPLE_OK = 0,
    SAMPLE_BUSY,
    SAMPLE_ERROR,
    SAMPLE_INVALID_PARAM,
    SAMPLE_TIMEOUT
} SAMPLE_StatusTypeDef;


SAMPLE_StatusTypeDef Sample_Init(void);
uint16_t Sample_GetRaw(Sample_ChannelTypeDef channel);
float Sample_GetPinVoltage(Sample_ChannelTypeDef channel);
float Sample_GetTheoreticalVoltage(Sample_ChannelTypeDef channel);
float Sample_GetTheoreticalCurrent(Sample_ChannelTypeDef channel);
// float Sample_GetActualVoltage(Sample_ChannelTypeDef channel);


#endif
