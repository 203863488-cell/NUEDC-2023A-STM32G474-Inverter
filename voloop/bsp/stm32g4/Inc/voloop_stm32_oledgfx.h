#ifndef VOLOOP_STM32_OLEDGFX_H
#define VOLOOP_STM32_OLEDGFX_H

#include <stdint.h>
#include "../../common/voloop_bsp_def.h"
#include "voloop_stm32_oledll.h"

#ifdef __cplusplus
extern "C" {
#endif

#define VOLOOP_STM32_OLEDGFX_WIDTH VOLOOP_STM32_OLEDLL_WIDTH
#define VOLOOP_STM32_OLEDGFX_HEIGHT VOLOOP_STM32_OLEDLL_HEIGHT
#define VOLOOP_STM32_OLEDGFX_PAGE_COUNT (VOLOOP_STM32_OLEDGFX_HEIGHT / 8U)
#define VOLOOP_STM32_OLEDGFX_FRAME_BYTES (VOLOOP_STM32_OLEDGFX_WIDTH * VOLOOP_STM32_OLEDGFX_PAGE_COUNT)

#define VOLOOP_STM32_OLEDGFX_CHAR_WIDTH 8U
#define VOLOOP_STM32_OLEDGFX_CHAR_HEIGHT 16U

#define VOLOOP_STM32_OLEDGFX_COLOR_OFF 0U
#define VOLOOP_STM32_OLEDGFX_COLOR_ON 1U

#define VOLOOP_STM32_OLEDGFX_DEFAULT_FLOAT_WIDTH 6U
#define VOLOOP_STM32_OLEDGFX_DEFAULT_FLOAT_PRECISION 2U

typedef enum {
	VOLOOP_STM32_OLEDGFX_Clip = 0U,
	VOLOOP_STM32_OLEDGFX_Wrap
} VOLOOP_STM32_OLEDGFX_TextModeTypeDef;

VOLOOP_BSP_StatusTypeDef VOLOOP_STM32_OLEDGFX_Init(I2C_HandleTypeDef* hi2c);
VOLOOP_BSP_StatusTypeDef VOLOOP_STM32_OLEDGFX_Start(void);
VOLOOP_BSP_StatusTypeDef VOLOOP_STM32_OLEDGFX_Stop(void);
VOLOOP_BSP_StatusTypeDef VOLOOP_STM32_OLEDGFX_Clear(void);
VOLOOP_BSP_StatusTypeDef VOLOOP_STM32_OLEDGFX_Refresh(void);

VOLOOP_BSP_StatusTypeDef VOLOOP_STM32_OLEDGFX_DrawPoint(uint8_t x, uint8_t y, uint8_t on);
VOLOOP_BSP_StatusTypeDef VOLOOP_STM32_OLEDGFX_ShowChar(uint8_t x, uint8_t y, char c);
VOLOOP_BSP_StatusTypeDef VOLOOP_STM32_OLEDGFX_ShowString(uint8_t x,
													  uint8_t y,
													  const char* str,
													  VOLOOP_STM32_OLEDGFX_TextModeTypeDef mode);
VOLOOP_BSP_StatusTypeDef VOLOOP_STM32_OLEDGFX_ShowNum(uint8_t x, uint8_t y, uint32_t number, uint8_t length);
VOLOOP_BSP_StatusTypeDef VOLOOP_STM32_OLEDGFX_ShowSignedNum(uint8_t x, uint8_t y, int32_t number, uint8_t length);
VOLOOP_BSP_StatusTypeDef VOLOOP_STM32_OLEDGFX_ShowFloat(uint8_t x,
													uint8_t y,
													float number,
													uint8_t width,
													uint8_t precision);

const uint8_t* VOLOOP_STM32_OLEDGFX_GetFrameBuffer(void);
uint16_t VOLOOP_STM32_OLEDGFX_GetFrameBufferSize(void);

#ifdef __cplusplus
}
#endif

#endif /* VOLOOP_STM32_OLEDGFX_H */
