#ifndef OLEDGFX_H
#define OLEDGFX_H

#include <stdint.h>
#include "oledll.h"

#define OLEDGFX_WIDTH OLEDLL_WIDTH
#define OLEDGFX_HEIGHT OLEDLL_HEIGHT
#define OLEDGFX_FRAME_BYTES (OLEDGFX_WIDTH * OLEDGFX_HEIGHT / 8U)

#define OLEDGFX_CHAR_WIDTH 8U
#define OLEDGFX_CHAR_HEIGHT 16U

#define OLEDGFX_DEFAULT_FLOAT_WIDTH 6U
#define OLEDGFX_DEFAULT_FLOAT_PRECISION 2U

typedef enum {
    OLEDGFX_OK = 0,
    OLEDGFX_BUSY,
    OLEDGFX_ERROR,
    OLEDGFX_INVALID_PARAM
} OLEDGFX_StatusTypeDef;

typedef enum {
	OLEDGFX_Clip = 0U,
	OLEDGFX_Wrap
} OLEDGFX_TextModeTypeDef;

OLEDGFX_StatusTypeDef OLEDGFX_Init(void);
OLEDGFX_StatusTypeDef OLEDGFX_Clear(void);
OLEDGFX_StatusTypeDef OLEDGFX_Submit(void);

static inline OLEDGFX_StatusTypeDef OLEDGFX_Update(void){
    return (OLEDGFX_StatusTypeDef)OLEDLL_Update();
}

#define OLEDGFX_LINE_1 0U
#define OLEDGFX_LINE_2 16U
#define OLEDGFX_LINE_3 32U
#define OLEDGFX_LINE_4 48U

#define OLEDGFX_COL_1 0U
#define OLEDGFX_COL_2 8U
#define OLEDGFX_COL_3 16U
#define OLEDGFX_COL_4 24U
#define OLEDGFX_COL_5 32U
#define OLEDGFX_COL_6 40U
#define OLEDGFX_COL_7 48U
#define OLEDGFX_COL_8 56U
#define OLEDGFX_COL_9 64U
#define OLEDGFX_COL_10 72U
#define OLEDGFX_COL_11 80U
#define OLEDGFX_COL_12 88U
#define OLEDGFX_COL_13 96U
#define OLEDGFX_COL_14 104U
#define OLEDGFX_COL_15 112U
#define OLEDGFX_COL_16 120U

/* The unit of x and y is pixel */
void OLEDGFX_DrawPoint(uint16_t x, uint16_t y, uint8_t on);
void OLEDGFX_ShowChar(uint16_t x, uint16_t y, char c);
void OLEDGFX_ShowString(uint16_t x,
						uint16_t y,
						const char* str,
						OLEDGFX_TextModeTypeDef mode);
void OLEDGFX_ShowNum(uint16_t x, uint16_t y, uint32_t number, uint8_t length);
void OLEDGFX_ShowSignedNum(uint16_t x, uint16_t y, int32_t number, uint8_t length);
void OLEDGFX_ShowHexNum(uint16_t x, uint16_t y, uint32_t number, uint8_t length);
void OLEDGFX_ShowFloat(uint16_t x,
						uint16_t y,
						float number,
						uint8_t width,
						uint8_t precision);

static inline void OLEDGFX_TxCpltCallback(void){
    OLEDLL_TxCpltCallback();
}

static inline void OLEDGFX_ErrorCallback(void){
    OLEDLL_ErrorCallback();
}

#endif
