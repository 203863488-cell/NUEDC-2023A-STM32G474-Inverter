#include "voloop_stm32_oledgfx.h"

#include <string.h>

#include "../../common/voloop_Font.h"

static uint8_t s_oledGram[VOLOOP_STM32_OLEDGFX_PAGE_COUNT][VOLOOP_STM32_OLEDGFX_WIDTH];
static uint8_t s_isInitialized = 0U;

static uint32_t VOLOOP_STM32_OLEDGFX_Pow10(uint8_t n) {
	uint32_t result = 1U;
	while (n > 0U) {
		result *= 10U;
		n--;
	}
	return result;
}

static VOLOOP_BSP_StatusTypeDef VOLOOP_STM32_OLEDGFX_CheckInitialized(void) {
	if (s_isInitialized == 0U) {
		return VOLOOP_BSP_INVALID_STATE;
	}
	return VOLOOP_BSP_OK;
}

VOLOOP_BSP_StatusTypeDef VOLOOP_STM32_OLEDGFX_Init(I2C_HandleTypeDef* hi2c) {
	VOLOOP_BSP_StatusTypeDef status;

	if (hi2c == NULL) {
		return VOLOOP_BSP_INVALID_PARAM;
	}

	status = VOLOOP_STM32_OLEDLL_Init(hi2c);
	if (status != VOLOOP_BSP_OK) {
		return status;
	}

	memset(s_oledGram, 0, sizeof(s_oledGram));
	s_isInitialized = 1U;

	return VOLOOP_BSP_OK;
}

VOLOOP_BSP_StatusTypeDef VOLOOP_STM32_OLEDGFX_Start(void) {
	VOLOOP_BSP_StatusTypeDef status = VOLOOP_STM32_OLEDGFX_CheckInitialized();
	if (status != VOLOOP_BSP_OK) {
		return status;
	}
	return VOLOOP_STM32_OLEDLL_Start();
}

VOLOOP_BSP_StatusTypeDef VOLOOP_STM32_OLEDGFX_Stop(void) {
	VOLOOP_BSP_StatusTypeDef status = VOLOOP_STM32_OLEDGFX_CheckInitialized();
	if (status != VOLOOP_BSP_OK) {
		return status;
	}
	return VOLOOP_STM32_OLEDLL_Stop();
}

VOLOOP_BSP_StatusTypeDef VOLOOP_STM32_OLEDGFX_Clear(void) {
	VOLOOP_BSP_StatusTypeDef status = VOLOOP_STM32_OLEDGFX_CheckInitialized();
	if (status != VOLOOP_BSP_OK) {
		return status;
	}
	memset(s_oledGram, 0, sizeof(s_oledGram));
	return VOLOOP_BSP_OK;
}

VOLOOP_BSP_StatusTypeDef VOLOOP_STM32_OLEDGFX_Refresh(void) {
	VOLOOP_BSP_StatusTypeDef status = VOLOOP_STM32_OLEDGFX_CheckInitialized();
	if (status != VOLOOP_BSP_OK) {
		return status;
	}
	return VOLOOP_STM32_OLEDLL_Refresh((const uint8_t*)s_oledGram);
}

VOLOOP_BSP_StatusTypeDef VOLOOP_STM32_OLEDGFX_DrawPoint(uint8_t x, uint8_t y, uint8_t on) {
	uint8_t page;
	uint8_t bitOffset;
	VOLOOP_BSP_StatusTypeDef status = VOLOOP_STM32_OLEDGFX_CheckInitialized();

	if (status != VOLOOP_BSP_OK) {
		return status;
	}
	if (x >= VOLOOP_STM32_OLEDGFX_WIDTH || y >= VOLOOP_STM32_OLEDGFX_HEIGHT) {
		return VOLOOP_BSP_INVALID_PARAM;
	}
	if (on != VOLOOP_STM32_OLEDGFX_COLOR_OFF && on != VOLOOP_STM32_OLEDGFX_COLOR_ON) {
		return VOLOOP_BSP_INVALID_PARAM;
	}

	page = y / 8U;
	bitOffset = y % 8U;

	if (on == VOLOOP_STM32_OLEDGFX_COLOR_ON) {
		s_oledGram[page][x] |= (uint8_t)(1U << bitOffset);
	} else {
		s_oledGram[page][x] &= (uint8_t)(~(1U << bitOffset));
	}

	return VOLOOP_BSP_OK;
}

VOLOOP_BSP_StatusTypeDef VOLOOP_STM32_OLEDGFX_ShowChar(uint8_t x, uint8_t y, char c) {
	uint8_t i;
	uint8_t j;
	uint8_t fontData;
	uint8_t offset;
	VOLOOP_BSP_StatusTypeDef status = VOLOOP_STM32_OLEDGFX_CheckInitialized();

	if (status != VOLOOP_BSP_OK) {
		return status;
	}
	if (x > (VOLOOP_STM32_OLEDGFX_WIDTH - VOLOOP_STM32_OLEDGFX_CHAR_WIDTH)
		|| y > (VOLOOP_STM32_OLEDGFX_HEIGHT - VOLOOP_STM32_OLEDGFX_CHAR_HEIGHT)) {
		return VOLOOP_BSP_INVALID_PARAM;
	}

	if (c < ' ' || c > '~') {
		c = ' ';
	}
	offset = (uint8_t)(c - ' ');

	for (i = 0U; i < VOLOOP_STM32_OLEDGFX_CHAR_WIDTH; i++) {
		fontData = VOLOOP_F8x16[(uint16_t)offset * 16U + i];
		for (j = 0U; j < 8U; j++) {
			(void)VOLOOP_STM32_OLEDGFX_DrawPoint((uint8_t)(x + i),
												 (uint8_t)(y + j),
												 (fontData & (uint8_t)(1U << j)) ? VOLOOP_STM32_OLEDGFX_COLOR_ON
																				 : VOLOOP_STM32_OLEDGFX_COLOR_OFF);
		}

		fontData = VOLOOP_F8x16[(uint16_t)offset * 16U + i + 8U];
		for (j = 0U; j < 8U; j++) {
			(void)VOLOOP_STM32_OLEDGFX_DrawPoint((uint8_t)(x + i),
												 (uint8_t)(y + j + 8U),
												 (fontData & (uint8_t)(1U << j)) ? VOLOOP_STM32_OLEDGFX_COLOR_ON
																				 : VOLOOP_STM32_OLEDGFX_COLOR_OFF);
		}
	}

	return VOLOOP_BSP_OK;
}

VOLOOP_BSP_StatusTypeDef VOLOOP_STM32_OLEDGFX_ShowString(uint8_t x,
													  uint8_t y,
													  const char* str,
													  VOLOOP_STM32_OLEDGFX_TextModeTypeDef mode) {
	VOLOOP_BSP_StatusTypeDef status = VOLOOP_STM32_OLEDGFX_CheckInitialized();

	if (status != VOLOOP_BSP_OK) {
		return status;
	}
	if (str == NULL) {
		return VOLOOP_BSP_INVALID_PARAM;
	}

	while (*str != '\0') {
		if (x > (VOLOOP_STM32_OLEDGFX_WIDTH - VOLOOP_STM32_OLEDGFX_CHAR_WIDTH)) {
			if (mode == VOLOOP_STM32_OLEDGFX_Wrap) {
				x = 0U;
				y = (uint8_t)(y + VOLOOP_STM32_OLEDGFX_CHAR_HEIGHT);
			} else {
				break;
			}
		}

		if (y > (VOLOOP_STM32_OLEDGFX_HEIGHT - VOLOOP_STM32_OLEDGFX_CHAR_HEIGHT)) {
			break;
		}

		status = VOLOOP_STM32_OLEDGFX_ShowChar(x, y, *str);
		if (status != VOLOOP_BSP_OK) {
			return status;
		}

		x = (uint8_t)(x + VOLOOP_STM32_OLEDGFX_CHAR_WIDTH);
		str++;
	}

	return VOLOOP_BSP_OK;
}

VOLOOP_BSP_StatusTypeDef VOLOOP_STM32_OLEDGFX_ShowNum(uint8_t x, uint8_t y, uint32_t number, uint8_t length) {
	uint8_t t;
	uint8_t digit;
	uint32_t base;
	VOLOOP_BSP_StatusTypeDef status;

	if (length == 0U || length > 10U) {
		return VOLOOP_BSP_INVALID_PARAM;
	}

	for (t = 0U; t < length; t++) {
		base = VOLOOP_STM32_OLEDGFX_Pow10((uint8_t)(length - t - 1U));
		digit = (uint8_t)((number / base) % 10U);
		status = VOLOOP_STM32_OLEDGFX_ShowChar((uint8_t)(x + t * VOLOOP_STM32_OLEDGFX_CHAR_WIDTH),
											   y,
											   (char)('0' + digit));
		if (status != VOLOOP_BSP_OK) {
			return status;
		}
	}

	return VOLOOP_BSP_OK;
}

VOLOOP_BSP_StatusTypeDef VOLOOP_STM32_OLEDGFX_ShowSignedNum(uint8_t x, uint8_t y, int32_t number, uint8_t length) {
	uint32_t absNum;
	VOLOOP_BSP_StatusTypeDef status;

	if (length == 0U || length > 10U) {
		return VOLOOP_BSP_INVALID_PARAM;
	}

	if (number < 0) {
		status = VOLOOP_STM32_OLEDGFX_ShowChar(x, y, '-');
		absNum = (uint32_t)(-number);
	} else {
		status = VOLOOP_STM32_OLEDGFX_ShowChar(x, y, ' ');
		absNum = (uint32_t)number;
	}

	if (status != VOLOOP_BSP_OK) {
		return status;
	}

	return VOLOOP_STM32_OLEDGFX_ShowNum((uint8_t)(x + VOLOOP_STM32_OLEDGFX_CHAR_WIDTH), y, absNum, length);
}

VOLOOP_BSP_StatusTypeDef VOLOOP_STM32_OLEDGFX_ShowFloat(uint8_t x,
													 uint8_t y,
													 float number,
													 uint8_t width,
													 uint8_t precision) {
	char buf[24];
	uint8_t idx = 0U;
	uint8_t i;
	uint32_t intPart;
	uint32_t fracPart;
	float absVal;
	uint32_t scale;

	VOLOOP_BSP_StatusTypeDef status = VOLOOP_STM32_OLEDGFX_CheckInitialized();
	if (status != VOLOOP_BSP_OK) {
		return status;
	}

	if (precision > 6U || width == 0U || width > 20U) {
		return VOLOOP_BSP_INVALID_PARAM;
	}

	absVal = number;
	if (number < 0.0f) {
		buf[idx++] = '-';
		absVal = -number;
	}

	intPart = (uint32_t)absVal;
	scale = VOLOOP_STM32_OLEDGFX_Pow10(precision);
	fracPart = (uint32_t)((absVal - (float)intPart) * (float)scale + 0.5f);
	if (fracPart >= scale) {
		intPart += 1U;
		fracPart = 0U;
	}

	{
		char intTmp[12];
		uint8_t intLen = 0U;
		if (intPart == 0U) {
			intTmp[intLen++] = '0';
		} else {
			while (intPart > 0U && intLen < sizeof(intTmp)) {
				intTmp[intLen++] = (char)('0' + (intPart % 10U));
				intPart /= 10U;
			}
		}
		while (intLen > 0U) {
			buf[idx++] = intTmp[--intLen];
		}
	}

	if (precision > 0U) {
		buf[idx++] = '.';
		for (i = 0U; i < precision; i++) {
			uint32_t div = VOLOOP_STM32_OLEDGFX_Pow10((uint8_t)(precision - i - 1U));
			buf[idx++] = (char)('0' + ((fracPart / div) % 10U));
		}
	}
	buf[idx] = '\0';

	if (idx < width) {
		char padded[24];
		uint8_t pad = (uint8_t)(width - idx);
		uint8_t p = 0U;
		for (i = 0U; i < pad; i++) {
			padded[p++] = ' ';
		}
		for (i = 0U; i <= idx; i++) {
			padded[p++] = buf[i];
		}
		return VOLOOP_STM32_OLEDGFX_ShowString(x, y, padded, VOLOOP_STM32_OLEDGFX_Clip);
	}

	return VOLOOP_STM32_OLEDGFX_ShowString(x, y, buf, VOLOOP_STM32_OLEDGFX_Clip);
}

const uint8_t* VOLOOP_STM32_OLEDGFX_GetFrameBuffer(void) {
	return (const uint8_t*)s_oledGram;
}

uint16_t VOLOOP_STM32_OLEDGFX_GetFrameBufferSize(void) {
	return VOLOOP_STM32_OLEDGFX_FRAME_BYTES;
}
