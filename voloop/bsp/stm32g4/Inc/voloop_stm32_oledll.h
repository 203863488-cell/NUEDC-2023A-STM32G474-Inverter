#ifndef VOLOOP_STM32_OLEDLL_H
#define VOLOOP_STM32_OLEDLL_H

#include <stdint.h>
#include "../../common/voloop_bsp_def.h"
#include "stm32g4xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

#define VOLOOP_STM32_OLEDLL_WIDTH 128U
#define VOLOOP_STM32_OLEDLL_HEIGHT 64U
#define VOLOOP_STM32_OLEDLL_PAGE_COUNT (VOLOOP_STM32_OLEDLL_HEIGHT / 8U)
#define VOLOOP_STM32_OLEDLL_FRAME_BYTES (VOLOOP_STM32_OLEDLL_WIDTH * VOLOOP_STM32_OLEDLL_PAGE_COUNT)

#define VOLOOP_STM32_OLEDLL_ADDR 0x78U

VOLOOP_BSP_StatusTypeDef VOLOOP_STM32_OLEDLL_Init(I2C_HandleTypeDef* hi2c);
VOLOOP_BSP_StatusTypeDef VOLOOP_STM32_OLEDLL_Start(void);
VOLOOP_BSP_StatusTypeDef VOLOOP_STM32_OLEDLL_Stop(void);
VOLOOP_BSP_StatusTypeDef VOLOOP_STM32_OLEDLL_Clear(void);
VOLOOP_BSP_StatusTypeDef VOLOOP_STM32_OLEDLL_Refresh(const uint8_t* buffer);

#ifdef __cplusplus
}
#endif


#endif /* VOLOOP_STM32_OLEDLL_H */
