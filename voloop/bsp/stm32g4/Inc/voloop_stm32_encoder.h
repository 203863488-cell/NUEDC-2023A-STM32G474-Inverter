#ifndef VOLOOP_STM32_ENCODER_H
#define VOLOOP_STM32_ENCODER_H

#include <stdint.h>
#include "../../common/voloop_bsp_def.h"
#include "stm32g4xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

#ifndef VOLOOP_STM32_ENCODER_COUNT_SCALE
#define VOLOOP_STM32_ENCODER_COUNT_SCALE 2U
#endif

VOLOOP_BSP_StatusTypeDef VOLOOP_STM32_Encoder_Init(TIM_HandleTypeDef* htim);
VOLOOP_BSP_StatusTypeDef VOLOOP_STM32_Encoder_DeInit(void);

int16_t VOLOOP_STM32_Encoder_GetCount(void);
VOLOOP_BSP_StatusTypeDef VOLOOP_STM32_Encoder_SetCount(int16_t count);
VOLOOP_BSP_StatusTypeDef VOLOOP_STM32_Encoder_Clear(void);

int16_t VOLOOP_STM32_Encoder_PopCount(void);

#ifdef __cplusplus
}
#endif

#endif /* VOLOOP_STM32_ENCODER_H */
