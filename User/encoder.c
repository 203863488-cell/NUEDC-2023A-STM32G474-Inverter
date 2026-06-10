#include "encoder.h"

#include "tim.h"

typedef struct {
	TIM_HandleTypeDef* htim;
	uint8_t isInitialized;
} Encoder_ContextTypeDef;

static Encoder_ContextTypeDef s_encoderCtx = {0};

static inline uint32_t Encoder_EnterCritical(void)
{
	uint32_t primask;

	primask = __get_PRIMASK();
	__disable_irq();
	return primask;
}

static inline void Encoder_ExitCritical(uint32_t primask)
{
	if (primask == 0U)
	{
		__enable_irq();
	}
}

void Encoder_Init(void)
{
	if (ENCODER_COUNT_SCALE == 0U)
	{
		return;
	}

	s_encoderCtx.htim = &ENCODER_HANDLE;
	if (HAL_TIM_Encoder_Start(s_encoderCtx.htim, TIM_CHANNEL_ALL) != HAL_OK)
	{
		s_encoderCtx.htim = NULL;
		s_encoderCtx.isInitialized = 0U;
		return;
	}

	__HAL_TIM_SET_COUNTER(s_encoderCtx.htim, 0U);
	s_encoderCtx.isInitialized = 1U;
}

void Encoder_DeInit(void)
{
	if ((s_encoderCtx.isInitialized == 0U) || (s_encoderCtx.htim == NULL))
	{
		return;
	}

	if (HAL_TIM_Encoder_Stop(s_encoderCtx.htim, TIM_CHANNEL_ALL) != HAL_OK)
	{
		return;
	}

	s_encoderCtx.htim = NULL;
	s_encoderCtx.isInitialized = 0U;
}

int16_t Encoder_GetCountRaw(void)
{
	if ((s_encoderCtx.isInitialized == 0U) || (s_encoderCtx.htim == NULL))
	{
		return 0;
	}

	return (int16_t)(__HAL_TIM_GET_COUNTER(s_encoderCtx.htim));
}

int16_t Encoder_GetCount(void)
{
    return (int16_t)(ENCODER_POLARITY * Encoder_GetCountRaw() / (int16_t)ENCODER_COUNT_SCALE);
}

void Encoder_SetCount(int16_t count)
{
	if ((s_encoderCtx.isInitialized == 0U) || (s_encoderCtx.htim == NULL))
	{
		return;
	}

	__HAL_TIM_SET_COUNTER(s_encoderCtx.htim, (uint16_t)count);
}

void Encoder_Clear(void)
{
	if ((s_encoderCtx.isInitialized == 0U) || (s_encoderCtx.htim == NULL))
	{
		return;
	}

	__HAL_TIM_SET_COUNTER(s_encoderCtx.htim, 0U);
}

int16_t Encoder_PopCount(void)
{
	uint32_t primask;
	int16_t rawCount;
	int16_t delta;
	int16_t remainder;

	if ((s_encoderCtx.isInitialized == 0U) || (s_encoderCtx.htim == NULL))
	{
		return 0;
	}
	if (ENCODER_COUNT_SCALE == 0U)
	{
		return 0;
	}

	primask = Encoder_EnterCritical();
	rawCount = (int16_t)__HAL_TIM_GET_COUNTER(s_encoderCtx.htim);
	delta = (int16_t)(rawCount / (int16_t)ENCODER_COUNT_SCALE);
	remainder = (int16_t)(rawCount % (int16_t)ENCODER_COUNT_SCALE);
	__HAL_TIM_SET_COUNTER(s_encoderCtx.htim, (uint16_t)remainder);
	Encoder_ExitCritical(primask);

    delta = (int16_t)(ENCODER_POLARITY * delta);
	return delta;
}
