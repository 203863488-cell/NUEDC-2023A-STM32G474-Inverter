#include "sample.h"

#include "adc.h"

#define SAMPLE_INJECTED_TIMEOUT_MS 10U

#define SAMPLE_BufferSize Sample_ChannelCount
uint16_t SAMPLE_Buffer[SAMPLE_BufferSize] = {0U};
float VoltageScaleFactor = SAMPLE_VoltageReference / (float)SAMPLE_ADCResolution;

static uint16_t s_vrefintRaw = 0U;

SAMPLE_StatusTypeDef Sample_Init(void){
	if (HAL_ADCEx_InjectedStart(&hadc1) != HAL_OK) {
		return SAMPLE_ERROR;
	}

	if (HAL_ADCEx_InjectedPollForConversion(&hadc1, SAMPLE_INJECTED_TIMEOUT_MS) != HAL_OK) {
		return SAMPLE_TIMEOUT;
	}

    HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
	s_vrefintRaw = (uint16_t)HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_1);
    HAL_ADC_Start_DMA(&hadc1, (uint32_t*)SAMPLE_Buffer, SAMPLE_BufferSize);
	return SAMPLE_OK;
}

uint16_t Sample_GetRaw(Sample_ChannelTypeDef channel) {
    if (channel >= Sample_ChannelCount) {
        return 0U;
    }

    return SAMPLE_Buffer[channel];
}

float Sample_GetPinVoltage(Sample_ChannelTypeDef channel) {
    if (channel >= Sample_ChannelCount) {
        return 0.0f;
    }

    return ((float)SAMPLE_Buffer[channel] * VoltageScaleFactor);
}

float Sample_GetTheoreticalVoltage(Sample_ChannelTypeDef channel){
    if (channel >= Sample_ChannelCount) {
        return 0.0f;
    }
    return 62.6f * (Sample_GetPinVoltage(channel) - 1.65f);
}

float Sample_GetTheoreticalCurrent(Sample_ChannelTypeDef channel){
    if (channel >= Sample_ChannelCount) {
        return 0.0f;
    }
    return 3.049f * (Sample_GetPinVoltage(channel) - 1.65f);
}

