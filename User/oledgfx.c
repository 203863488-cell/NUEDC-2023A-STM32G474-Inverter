#include "oledgfx.h"
#include "oledFont.h"

#include <stddef.h>
#include <string.h>

#ifndef OLEDGFX_FONT_8X16
#define OLEDGFX_FONT_8X16 OLEDFONT_F8x16
#endif

#define OLEDGFX_ASCII_FIRST          ' '
#define OLEDGFX_ASCII_LAST           '~'
#define OLEDGFX_FONT_BYTES_PER_CHAR  16U
#define OLEDGFX_MAX_UINT32_DIGITS    10U
#define OLEDGFX_MAX_HEX_DIGITS       8U
#define OLEDGFX_MAX_FLOAT_WIDTH      10U
#define OLEDGFX_MAX_FLOAT_PRECISION  6U
#define OLEDGFX_FLOAT_BUFFER_SIZE    20U

static uint8_t OLEDGFX_FrameBuffer[OLEDGFX_FRAME_BYTES];

static OLEDGFX_StatusTypeDef OLEDGFX_FromLLStatus(OLEDLL_StatusTypeDef status);
static uint32_t OLEDGFX_Pow10(uint8_t exponent);
static uint8_t OLEDGFX_FormatUnsigned(char* buffer,
									  uint8_t buffer_size,
									  uint32_t value,
									  uint8_t min_width);
static uint8_t OLEDGFX_FormatHex(char* buffer,
								  uint8_t buffer_size,
								  uint32_t value,
								  uint8_t min_width);

static OLEDGFX_StatusTypeDef OLEDGFX_FromLLStatus(OLEDLL_StatusTypeDef status)
{
	switch (status) {
	case OLEDLL_OK:
		return OLEDGFX_OK;
	case OLEDLL_BUSY:
		return OLEDGFX_BUSY;
	case OLEDLL_INVALID_PARAM:
		return OLEDGFX_INVALID_PARAM;
	case OLEDLL_ERROR:
	default:
		return OLEDGFX_ERROR;
	}
}

static uint32_t OLEDGFX_Pow10(uint8_t exponent)
{
	uint32_t result = 1U;

	while (exponent > 0U) {
		result *= 10U;
		exponent--;
	}

	return result;
}

static uint8_t OLEDGFX_FormatUnsigned(char* buffer,
									  uint8_t buffer_size,
									  uint32_t value,
									  uint8_t min_width)
{
	char reversed[OLEDGFX_MAX_UINT32_DIGITS];
	uint8_t digit_count = 0U;
	uint8_t write_count;
	uint8_t index;

	if ((buffer == NULL) || (buffer_size == 0U)) {
		return 0U;
	}

	if (min_width > OLEDGFX_MAX_UINT32_DIGITS) {
		min_width = OLEDGFX_MAX_UINT32_DIGITS;
	}

	do {
		reversed[digit_count] = (char)('0' + (value % 10U));
		value /= 10U;
		digit_count++;
	} while ((value > 0U) && (digit_count < OLEDGFX_MAX_UINT32_DIGITS));

	write_count = (digit_count > min_width) ? digit_count : min_width;
	if (write_count >= buffer_size) {
		write_count = (uint8_t)(buffer_size - 1U);
	}

	index = 0U;
	while (index < (uint8_t)(write_count - digit_count)) {
		buffer[index] = '0';
		index++;
	}

	while (digit_count > 0U) {
		buffer[index] = reversed[digit_count - 1U];
		digit_count--;
		index++;
	}

	buffer[index] = '\0';

	return index;
}

static uint8_t OLEDGFX_FormatHex(char* buffer,
								  uint8_t buffer_size,
								  uint32_t value,
								  uint8_t min_width)
{
	static const char hex_digits[] = "0123456789ABCDEF";
	char reversed[OLEDGFX_MAX_HEX_DIGITS];
	uint8_t digit_count = 0U;
	uint8_t write_count;
	uint8_t index;

	if ((buffer == NULL) || (buffer_size == 0U)) {
		return 0U;
	}

	if (min_width > OLEDGFX_MAX_HEX_DIGITS) {
		min_width = OLEDGFX_MAX_HEX_DIGITS;
	}

	do {
		reversed[digit_count] = hex_digits[value & 0x0FU];
		value >>= 4U;
		digit_count++;
	} while ((value > 0U) && (digit_count < OLEDGFX_MAX_HEX_DIGITS));

	write_count = (digit_count > min_width) ? digit_count : min_width;
	if (write_count >= buffer_size) {
		write_count = (uint8_t)(buffer_size - 1U);
	}

	index = 0U;
	while (index < (uint8_t)(write_count - digit_count)) {
		buffer[index] = '0';
		index++;
	}

	while (digit_count > 0U) {
		buffer[index] = reversed[digit_count - 1U];
		digit_count--;
		index++;
	}

	buffer[index] = '\0';

	return index;
}

OLEDGFX_StatusTypeDef OLEDGFX_Init(void)
{
	OLEDLL_StatusTypeDef status;

	status = OLEDLL_Init();
	if (status != OLEDLL_OK) {
		return OLEDGFX_FromLLStatus(status);
	}

	(void)memset(OLEDGFX_FrameBuffer, 0, sizeof(OLEDGFX_FrameBuffer));

	return OLEDGFX_OK;
}

OLEDGFX_StatusTypeDef OLEDGFX_Clear(void)
{
	(void)memset(OLEDGFX_FrameBuffer, 0, sizeof(OLEDGFX_FrameBuffer));
	return OLEDGFX_OK;
}

OLEDGFX_StatusTypeDef OLEDGFX_Submit(void)
{
	return OLEDGFX_FromLLStatus(OLEDLL_WriteFrame(OLEDGFX_FrameBuffer));
}

void OLEDGFX_DrawPoint(uint16_t x, uint16_t y, uint8_t on)
{
	uint16_t index;
	uint8_t bit;

	if ((x >= OLEDGFX_WIDTH) || (y >= OLEDGFX_HEIGHT)) {
		return;
	}

	index = (uint16_t)(x + ((y / 8U) * OLEDGFX_WIDTH));
	bit = (uint8_t)(y % 8U);

	if (on != 0U) {
		OLEDGFX_FrameBuffer[index] |= (uint8_t)(1U << bit);
	} else {
		OLEDGFX_FrameBuffer[index] &= (uint8_t)~(1U << bit);
	}
}

void OLEDGFX_ShowChar(uint16_t x, uint16_t y, char c)
{
	const uint8_t* glyph;
	uint8_t column;
	uint8_t row;
	uint8_t column_bits;

	if ((c < OLEDGFX_ASCII_FIRST) || (c > OLEDGFX_ASCII_LAST)) {
		c = OLEDGFX_ASCII_FIRST;
	}

	/* Each 8x16 ASCII glyph occupies 16 consecutive bytes in oledFont.h. */
	glyph = &OLEDGFX_FONT_8X16[(uint16_t)(c - OLEDGFX_ASCII_FIRST) * OLEDGFX_FONT_BYTES_PER_CHAR];

	for (column = 0U; column < OLEDGFX_CHAR_WIDTH; column++) {
		column_bits = glyph[column];
		for (row = 0U; row < 8U; row++) {
			OLEDGFX_DrawPoint((uint16_t)(x + column),
							  (uint16_t)(y + row),
							  (uint8_t)(column_bits & (uint8_t)(1U << row)));
		}

		column_bits = glyph[column + 8U];
		for (row = 0U; row < 8U; row++) {
			OLEDGFX_DrawPoint((uint16_t)(x + column),
							  (uint16_t)(y + row + 8U),
							  (uint8_t)(column_bits & (uint8_t)(1U << row)));
		}
	}
}

void OLEDGFX_ShowString(uint16_t x,
						uint16_t y,
						const char* str,
						OLEDGFX_TextModeTypeDef mode)
{
	uint16_t current_x;
	uint16_t current_y;

	if (str == NULL) {
		return;
	}

	current_x = x;
	current_y = y;

	while (*str != '\0') {
		if (current_x > (OLEDGFX_WIDTH - OLEDGFX_CHAR_WIDTH)) {
			if (mode == OLEDGFX_Wrap) {
				current_x = 0U;
				current_y = (uint16_t)(current_y + OLEDGFX_CHAR_HEIGHT);
			} else {
				break;
			}
		}

		if (current_y > (OLEDGFX_HEIGHT - OLEDGFX_CHAR_HEIGHT)) {
			break;
		}

		OLEDGFX_ShowChar(current_x, current_y, *str);
		current_x = (uint16_t)(current_x + OLEDGFX_CHAR_WIDTH);
		str++;
	}
}

void OLEDGFX_ShowNum(uint16_t x, uint16_t y, uint32_t number, uint8_t length)
{
	char text[OLEDGFX_MAX_UINT32_DIGITS + 1U];

	if (length == 0U) {
		length = 1U;
	}

	if (length > OLEDGFX_MAX_UINT32_DIGITS) {
		length = OLEDGFX_MAX_UINT32_DIGITS;
	}

	(void)OLEDGFX_FormatUnsigned(text, (uint8_t)sizeof(text), number, length);
	OLEDGFX_ShowString(x, y, text, OLEDGFX_Clip);
}

void OLEDGFX_ShowSignedNum(uint16_t x, uint16_t y, int32_t number, uint8_t length)
{
	uint32_t magnitude;

	if (length == 0U) {
		length = 1U;
	}

	if (length > OLEDGFX_MAX_UINT32_DIGITS) {
		length = OLEDGFX_MAX_UINT32_DIGITS;
	}

	if (number < 0) {
		OLEDGFX_ShowChar(x, y, '-');
		magnitude = (uint32_t)(-(number + 1));
		magnitude += 1U;
		OLEDGFX_ShowNum((uint16_t)(x + OLEDGFX_CHAR_WIDTH), y, magnitude, length);
	} else {
		/* Positive numbers are rendered without a leading '+' sign. */
		magnitude = (uint32_t)number;
		OLEDGFX_ShowNum(x, y, magnitude, length);
	}
}

void OLEDGFX_ShowHexNum(uint16_t x, uint16_t y, uint32_t number, uint8_t length)
{
	char text[OLEDGFX_MAX_HEX_DIGITS + 1U];

	if (length == 0U) {
		length = 1U;
	}

	if (length > OLEDGFX_MAX_HEX_DIGITS) {
		length = OLEDGFX_MAX_HEX_DIGITS;
	}

	(void)OLEDGFX_FormatHex(text, (uint8_t)sizeof(text), number, length);
	OLEDGFX_ShowString(x, y, text, OLEDGFX_Clip);
}

void OLEDGFX_ShowFloat(uint16_t x,
					   uint16_t y,
					   float number,
					   uint8_t width,
					   uint8_t precision)
{
	char text[OLEDGFX_FLOAT_BUFFER_SIZE];
	char integer_text[OLEDGFX_MAX_UINT32_DIGITS + 1U];
	uint8_t index = 0U;
	uint8_t integer_length;
	uint8_t digit_index;
	float abs_value;
	uint32_t integer_part;
	uint32_t fractional_part;
	uint32_t scale;

	if (width == 0U) {
		width = 1U;
	}

	if (width > OLEDGFX_MAX_FLOAT_WIDTH) {
		width = OLEDGFX_MAX_FLOAT_WIDTH;
	}

	if (precision > OLEDGFX_MAX_FLOAT_PRECISION) {
		precision = OLEDGFX_MAX_FLOAT_PRECISION;
	}

	abs_value = number;
	if (number < 0.0f) {
		text[index++] = '-';
		abs_value = -number;
	}

	if (abs_value > 4294967295.0f) {
		integer_part = 4294967295U;
		fractional_part = 0U;
		precision = 0U;
	} else {
		integer_part = (uint32_t)abs_value;
		scale = OLEDGFX_Pow10(precision);
		fractional_part = (uint32_t)(((abs_value - (float)integer_part) * (float)scale) + 0.5f);
		if ((precision > 0U) && (fractional_part >= scale)) {
			integer_part += 1U;
			fractional_part = 0U;
		}
	}

	integer_length = OLEDGFX_FormatUnsigned(integer_text,
											(uint8_t)sizeof(integer_text),
											integer_part,
											width);

	if ((uint8_t)(index + integer_length + 1U) >= OLEDGFX_FLOAT_BUFFER_SIZE) {
		return;
	}

	(void)memcpy(&text[index], integer_text, integer_length);
	index = (uint8_t)(index + integer_length);

	if (precision > 0U) {
		text[index++] = '.';
		for (digit_index = 0U; digit_index < precision; digit_index++) {
			uint32_t divisor = OLEDGFX_Pow10((uint8_t)(precision - digit_index - 1U));
			text[index++] = (char)('0' + ((fractional_part / divisor) % 10U));
		}
	}

	text[index] = '\0';
	OLEDGFX_ShowString(x, y, text, OLEDGFX_Clip);
}
