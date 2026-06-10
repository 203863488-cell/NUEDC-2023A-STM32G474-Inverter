#include "voloop_stm32_oledll.h"

#define VOLOOP_STM32_OLEDLL_CTRL_CMD 0x00U
#define VOLOOP_STM32_OLEDLL_CTRL_DATA 0x40U
#define VOLOOP_STM32_OLEDLL_CLEAR_CHUNK 16U
#define VOLOOP_STM32_OLEDLL_TIMEOUT_CMD 10U
#define VOLOOP_STM32_OLEDLL_TIMEOUT_DATA 100U

typedef struct {
	I2C_HandleTypeDef* hi2c;
	uint8_t isInitialized;
} VOLOOP_STM32_OLEDLL_ContextTypeDef;

static VOLOOP_STM32_OLEDLL_ContextTypeDef s_oledCtx = {0};

static VOLOOP_BSP_StatusTypeDef VOLOOP_STM32_OLEDLL_WriteCmd(uint8_t cmd) {
	if (s_oledCtx.hi2c == NULL) {
		return VOLOOP_BSP_INVALID_STATE;
	}

	if (HAL_I2C_Mem_Write(s_oledCtx.hi2c,
						  VOLOOP_STM32_OLEDLL_ADDR,
						  VOLOOP_STM32_OLEDLL_CTRL_CMD,
						  I2C_MEMADD_SIZE_8BIT,
						  &cmd,
						  1U,
						  VOLOOP_STM32_OLEDLL_TIMEOUT_CMD) != HAL_OK) {
		return VOLOOP_BSP_ERROR;
	}

	return VOLOOP_BSP_OK;
}

static VOLOOP_BSP_StatusTypeDef VOLOOP_STM32_OLEDLL_WriteDataStream(const uint8_t* data, uint16_t len) {
	if (s_oledCtx.hi2c == NULL || data == NULL || len == 0U) {
		return VOLOOP_BSP_INVALID_PARAM;
	}

	if (HAL_I2C_Mem_Write(s_oledCtx.hi2c,
						  VOLOOP_STM32_OLEDLL_ADDR,
						  VOLOOP_STM32_OLEDLL_CTRL_DATA,
						  I2C_MEMADD_SIZE_8BIT,
						  (uint8_t*)data,
						  len,
						  VOLOOP_STM32_OLEDLL_TIMEOUT_DATA) != HAL_OK) {
		return VOLOOP_BSP_ERROR;
	}

	return VOLOOP_BSP_OK;
}

static VOLOOP_BSP_StatusTypeDef VOLOOP_STM32_OLEDLL_SetAddrWindow(uint8_t colStart,
											   uint8_t colEnd,
											   uint8_t pageStart,
											   uint8_t pageEnd) {
	VOLOOP_BSP_StatusTypeDef status;

	status = VOLOOP_STM32_OLEDLL_WriteCmd(0x21U);
	if (status != VOLOOP_BSP_OK) return status;
	status = VOLOOP_STM32_OLEDLL_WriteCmd(colStart);
	if (status != VOLOOP_BSP_OK) return status;
	status = VOLOOP_STM32_OLEDLL_WriteCmd(colEnd);
	if (status != VOLOOP_BSP_OK) return status;

	status = VOLOOP_STM32_OLEDLL_WriteCmd(0x22U);
	if (status != VOLOOP_BSP_OK) return status;
	status = VOLOOP_STM32_OLEDLL_WriteCmd(pageStart);
	if (status != VOLOOP_BSP_OK) return status;
	status = VOLOOP_STM32_OLEDLL_WriteCmd(pageEnd);
	if (status != VOLOOP_BSP_OK) return status;

	return VOLOOP_BSP_OK;
}

VOLOOP_BSP_StatusTypeDef VOLOOP_STM32_OLEDLL_Init(I2C_HandleTypeDef* hi2c) {
	VOLOOP_BSP_StatusTypeDef status;

	if (hi2c == NULL) {
		return VOLOOP_BSP_INVALID_PARAM;
	}

	s_oledCtx.hi2c = hi2c;
	s_oledCtx.isInitialized = 1U;

	HAL_Delay(100U);

	status = VOLOOP_STM32_OLEDLL_WriteCmd(0xAEU);
	if (status != VOLOOP_BSP_OK) return status;

	status = VOLOOP_STM32_OLEDLL_WriteCmd(0x20U);
	if (status != VOLOOP_BSP_OK) return status;
	status = VOLOOP_STM32_OLEDLL_WriteCmd(0x00U);
	if (status != VOLOOP_BSP_OK) return status;

	status = VOLOOP_STM32_OLEDLL_WriteCmd(0xB0U);
	if (status != VOLOOP_BSP_OK) return status;
	status = VOLOOP_STM32_OLEDLL_WriteCmd(0xC8U);
	if (status != VOLOOP_BSP_OK) return status;

	status = VOLOOP_STM32_OLEDLL_WriteCmd(0x00U);
	if (status != VOLOOP_BSP_OK) return status;
	status = VOLOOP_STM32_OLEDLL_WriteCmd(0x10U);
	if (status != VOLOOP_BSP_OK) return status;

	status = VOLOOP_STM32_OLEDLL_WriteCmd(0x40U);
	if (status != VOLOOP_BSP_OK) return status;

	status = VOLOOP_STM32_OLEDLL_WriteCmd(0x81U);
	if (status != VOLOOP_BSP_OK) return status;
	status = VOLOOP_STM32_OLEDLL_WriteCmd(0xCFU);
	if (status != VOLOOP_BSP_OK) return status;

	status = VOLOOP_STM32_OLEDLL_WriteCmd(0xA1U);
	if (status != VOLOOP_BSP_OK) return status;

	status = VOLOOP_STM32_OLEDLL_WriteCmd(0xA6U);
	if (status != VOLOOP_BSP_OK) return status;

	status = VOLOOP_STM32_OLEDLL_WriteCmd(0xA8U);
	if (status != VOLOOP_BSP_OK) return status;
	status = VOLOOP_STM32_OLEDLL_WriteCmd(0x3FU);
	if (status != VOLOOP_BSP_OK) return status;

	status = VOLOOP_STM32_OLEDLL_WriteCmd(0xA4U);
	if (status != VOLOOP_BSP_OK) return status;

	status = VOLOOP_STM32_OLEDLL_WriteCmd(0xD3U);
	if (status != VOLOOP_BSP_OK) return status;
	status = VOLOOP_STM32_OLEDLL_WriteCmd(0x00U);
	if (status != VOLOOP_BSP_OK) return status;

	status = VOLOOP_STM32_OLEDLL_WriteCmd(0xD5U);
	if (status != VOLOOP_BSP_OK) return status;
	status = VOLOOP_STM32_OLEDLL_WriteCmd(0x80U);
	if (status != VOLOOP_BSP_OK) return status;

	status = VOLOOP_STM32_OLEDLL_WriteCmd(0xD9U);
	if (status != VOLOOP_BSP_OK) return status;
	status = VOLOOP_STM32_OLEDLL_WriteCmd(0xF1U);
	if (status != VOLOOP_BSP_OK) return status;

	status = VOLOOP_STM32_OLEDLL_WriteCmd(0xDAU);
	if (status != VOLOOP_BSP_OK) return status;
	status = VOLOOP_STM32_OLEDLL_WriteCmd(0x12U);
	if (status != VOLOOP_BSP_OK) return status;

	status = VOLOOP_STM32_OLEDLL_WriteCmd(0xDBU);
	if (status != VOLOOP_BSP_OK) return status;
	status = VOLOOP_STM32_OLEDLL_WriteCmd(0x40U);
	if (status != VOLOOP_BSP_OK) return status;

	status = VOLOOP_STM32_OLEDLL_WriteCmd(0x8DU);
	if (status != VOLOOP_BSP_OK) return status;
	status = VOLOOP_STM32_OLEDLL_WriteCmd(0x14U);
	if (status != VOLOOP_BSP_OK) return status;

	status = VOLOOP_STM32_OLEDLL_Clear();
	if (status != VOLOOP_BSP_OK) return status;

	return VOLOOP_STM32_OLEDLL_Start();
}

VOLOOP_BSP_StatusTypeDef VOLOOP_STM32_OLEDLL_Start(void) {
	if (s_oledCtx.isInitialized == 0U) {
		return VOLOOP_BSP_INVALID_STATE;
	}
	return VOLOOP_STM32_OLEDLL_WriteCmd(0xAFU);
}

VOLOOP_BSP_StatusTypeDef VOLOOP_STM32_OLEDLL_Stop(void) {
	if (s_oledCtx.isInitialized == 0U) {
		return VOLOOP_BSP_INVALID_STATE;
	}
	return VOLOOP_STM32_OLEDLL_WriteCmd(0xAEU);
}

VOLOOP_BSP_StatusTypeDef VOLOOP_STM32_OLEDLL_Clear(void) {
	VOLOOP_BSP_StatusTypeDef status;
	uint8_t zeroBuff[VOLOOP_STM32_OLEDLL_CLEAR_CHUNK] = {0};
	uint16_t loops = (uint16_t)(VOLOOP_STM32_OLEDLL_FRAME_BYTES / VOLOOP_STM32_OLEDLL_CLEAR_CHUNK);
	uint16_t i;

	if (s_oledCtx.isInitialized == 0U) {
		return VOLOOP_BSP_INVALID_STATE;
	}

	status = VOLOOP_STM32_OLEDLL_SetAddrWindow(0U,
											   (uint8_t)(VOLOOP_STM32_OLEDLL_WIDTH - 1U),
											   0U,
											   (uint8_t)(VOLOOP_STM32_OLEDLL_PAGE_COUNT - 1U));
	if (status != VOLOOP_BSP_OK) {
		return status;
	}

	for (i = 0U; i < loops; i++) {
		status = VOLOOP_STM32_OLEDLL_WriteDataStream(zeroBuff, VOLOOP_STM32_OLEDLL_CLEAR_CHUNK);
		if (status != VOLOOP_BSP_OK) {
			return status;
		}
	}

	return VOLOOP_BSP_OK;
}

VOLOOP_BSP_StatusTypeDef VOLOOP_STM32_OLEDLL_Refresh(const uint8_t* buffer) {
	VOLOOP_BSP_StatusTypeDef status;

	if (s_oledCtx.isInitialized == 0U) {
		return VOLOOP_BSP_INVALID_STATE;
	}
	if (buffer == NULL) {
		return VOLOOP_BSP_INVALID_PARAM;
	}

	status = VOLOOP_STM32_OLEDLL_SetAddrWindow(0U,
											   (uint8_t)(VOLOOP_STM32_OLEDLL_WIDTH - 1U),
											   0U,
											   (uint8_t)(VOLOOP_STM32_OLEDLL_PAGE_COUNT - 1U));
	if (status != VOLOOP_BSP_OK) {
		return status;
	}

	return VOLOOP_STM32_OLEDLL_WriteDataStream(buffer, VOLOOP_STM32_OLEDLL_FRAME_BYTES);
}
