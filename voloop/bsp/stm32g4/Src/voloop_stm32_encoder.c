#include "voloop_stm32_encoder.h"

typedef struct {
	TIM_HandleTypeDef* htim;
	uint8_t isInitialized;
} VOLOOP_STM32_EncoderContextTypeDef;

static VOLOOP_STM32_EncoderContextTypeDef s_encoderCtx = {0};

static uint32_t VOLOOP_STM32_Encoder_EnterCritical(void) {
	uint32_t primask = __get_PRIMASK();
	__disable_irq();
	return primask;
}

static void VOLOOP_STM32_Encoder_ExitCritical(uint32_t primask) {
	if (primask == 0U) {
		__enable_irq();
	}
}

VOLOOP_BSP_StatusTypeDef VOLOOP_STM32_Encoder_Init(TIM_HandleTypeDef* htim) {
	if (htim == NULL) {
		return VOLOOP_BSP_INVALID_PARAM;
	}
	if (VOLOOP_STM32_ENCODER_COUNT_SCALE == 0U) {
		return VOLOOP_BSP_INVALID_PARAM;
	}

	s_encoderCtx.htim = htim;
	if (HAL_TIM_Encoder_Start(s_encoderCtx.htim, TIM_CHANNEL_ALL) != HAL_OK) {
		s_encoderCtx.htim = NULL;
		s_encoderCtx.isInitialized = 0U;
		return VOLOOP_BSP_ERROR;
	}

	__HAL_TIM_SET_COUNTER(s_encoderCtx.htim, 0U);
	s_encoderCtx.isInitialized = 1U;
	return VOLOOP_BSP_OK;
}

VOLOOP_BSP_StatusTypeDef VOLOOP_STM32_Encoder_DeInit(void) {
	if (s_encoderCtx.isInitialized == 0U || s_encoderCtx.htim == NULL) {
		return VOLOOP_BSP_INVALID_STATE;
	}

	if (HAL_TIM_Encoder_Stop(s_encoderCtx.htim, TIM_CHANNEL_ALL) != HAL_OK) {
		return VOLOOP_BSP_ERROR;
	}

	s_encoderCtx.htim = NULL;
	s_encoderCtx.isInitialized = 0U;
	return VOLOOP_BSP_OK;
}

int16_t VOLOOP_STM32_Encoder_GetCount(void) {
	if (s_encoderCtx.isInitialized == 0U || s_encoderCtx.htim == NULL) {
		return 0;
	}

	return (int16_t)__HAL_TIM_GET_COUNTER(s_encoderCtx.htim);
}

VOLOOP_BSP_StatusTypeDef VOLOOP_STM32_Encoder_SetCount(int16_t count) {
	if (s_encoderCtx.isInitialized == 0U || s_encoderCtx.htim == NULL) {
		return VOLOOP_BSP_INVALID_STATE;
	}

	__HAL_TIM_SET_COUNTER(s_encoderCtx.htim, (uint16_t)count);
	return VOLOOP_BSP_OK;
}

VOLOOP_BSP_StatusTypeDef VOLOOP_STM32_Encoder_Clear(void) {
	if (s_encoderCtx.isInitialized == 0U || s_encoderCtx.htim == NULL) {
		return VOLOOP_BSP_INVALID_STATE;
	}

	__HAL_TIM_SET_COUNTER(s_encoderCtx.htim, 0U);
	return VOLOOP_BSP_OK;
}

int16_t VOLOOP_STM32_Encoder_PopCount(void) {
	uint32_t primask;
	int16_t rawCount;
	int16_t delta;
	int16_t remainder;

	if (s_encoderCtx.isInitialized == 0U || s_encoderCtx.htim == NULL) {
		return 0;
	}
	if (VOLOOP_STM32_ENCODER_COUNT_SCALE == 0U) {
		return 0;
	}

	primask = VOLOOP_STM32_Encoder_EnterCritical();
	rawCount = (int16_t)__HAL_TIM_GET_COUNTER(s_encoderCtx.htim);
	delta = (int16_t)(rawCount / (int16_t)VOLOOP_STM32_ENCODER_COUNT_SCALE);
	remainder = (int16_t)(rawCount % (int16_t)VOLOOP_STM32_ENCODER_COUNT_SCALE);
	__HAL_TIM_SET_COUNTER(s_encoderCtx.htim, (uint16_t)remainder);
	VOLOOP_STM32_Encoder_ExitCritical(primask);

	return delta;
}
