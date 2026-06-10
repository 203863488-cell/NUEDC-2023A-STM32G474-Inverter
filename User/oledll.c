#include "oledll.h"

#include <string.h>

#include "i2c.h"

#define OLEDLL_CTRL_CMD   0x00U
#define OLEDLL_CTRL_DATA  0x40U
#define OLEDLL_PAGE_COUNT (OLEDLL_HEIGHT / 8U)

/*
 * draw_buf: current writable back buffer.
 * ready_buf: latest completed frame waiting for transfer.
 * busy_buf: buffer currently owned by the I2C transfer.
 */
static uint8_t fb0[OLEDLL_BUFFER_SIZE];
static uint8_t fb1[OLEDLL_BUFFER_SIZE];
static uint8_t fb2[OLEDLL_BUFFER_SIZE];

static uint8_t* volatile draw_buf = NULL;
static uint8_t* volatile ready_buf = NULL;
static uint8_t* volatile busy_buf = NULL;

static volatile uint8_t oled_initialized = 0U;
static volatile uint8_t oled_error = 0U;

static uint32_t OLEDLL_EnterCritical(void);
static void OLEDLL_ExitCritical(uint32_t primask);
static OLEDLL_StatusTypeDef OLEDLL_WriteCommand(uint8_t cmd);
static OLEDLL_StatusTypeDef OLEDLL_WriteCommandList(const uint8_t* cmds, uint32_t len);
static OLEDLL_StatusTypeDef OLEDLL_SetFullWindow(void);
static OLEDLL_StatusTypeDef OLEDLL_StartTransfer(uint8_t* buffer);
static uint8_t* OLEDLL_FindFreeBuffer(void);
static void OLEDLL_SubmitDrawBuffer(void);

static uint32_t OLEDLL_EnterCritical(void) {
	uint32_t primask;

	primask = __get_PRIMASK();
	__disable_irq();

	return primask;
}

static void OLEDLL_ExitCritical(uint32_t primask) {
	if (primask == 0U) {
		__enable_irq();
	}
}

static OLEDLL_StatusTypeDef OLEDLL_WriteCommand(uint8_t cmd) {
	if (HAL_I2C_Mem_Write(&OLEDLL_HANDLE,
						  OLEDLL_ADDR,
						  OLEDLL_CTRL_CMD,
						  I2C_MEMADD_SIZE_8BIT,
						  &cmd,
						  1U,
						  OLEDLL_TIMEOUT) != HAL_OK) {
		return OLEDLL_ERROR;
	}

	return OLEDLL_OK;
}

static OLEDLL_StatusTypeDef OLEDLL_WriteCommandList(const uint8_t* cmds, uint32_t len) {
	uint32_t index;

	if ((cmds == NULL) || (len == 0U)) {
		return OLEDLL_INVALID_PARAM;
	}

	for (index = 0U; index < len; index++) {
		if (OLEDLL_WriteCommand(cmds[index]) != OLEDLL_OK) {
			return OLEDLL_ERROR;
		}
	}

	return OLEDLL_OK;
}

static OLEDLL_StatusTypeDef OLEDLL_SetFullWindow(void) {
	static const uint8_t window_cmds[] = {
		0x21U,
		0x00U,
		(uint8_t)(OLEDLL_WIDTH - 1U),
		0x22U,
		0x00U,
		(uint8_t)(OLEDLL_PAGE_COUNT - 1U)
	};

	return OLEDLL_WriteCommandList(window_cmds, (uint32_t)sizeof(window_cmds));
}

static OLEDLL_StatusTypeDef OLEDLL_StartTransfer(uint8_t* buffer) {
	if (buffer == NULL) {
		return OLEDLL_INVALID_PARAM;
	}

	if (OLEDLL_SetFullWindow() != OLEDLL_OK) {
		return OLEDLL_ERROR;
	}

	if (HAL_I2C_Mem_Write_DMA(&OLEDLL_HANDLE,
							  OLEDLL_ADDR,
							  OLEDLL_CTRL_DATA,
							  I2C_MEMADD_SIZE_8BIT,
							  buffer,
							  OLEDLL_BUFFER_SIZE) == HAL_OK) {
		return OLEDLL_OK;
	}

	return OLEDLL_ERROR;
}

static uint8_t* OLEDLL_FindFreeBuffer(void) {
	uint8_t* candidate;

	candidate = fb0;
	if ((candidate != draw_buf) && (candidate != ready_buf) && (candidate != busy_buf)) {
		return candidate;
	}

	candidate = fb1;
	if ((candidate != draw_buf) && (candidate != ready_buf) && (candidate != busy_buf)) {
		return candidate;
	}

	candidate = fb2;
	if ((candidate != draw_buf) && (candidate != ready_buf) && (candidate != busy_buf)) {
		return candidate;
	}

	return NULL;
}

static void OLEDLL_SubmitDrawBuffer(void) {
	uint32_t primask;
	uint8_t* old_ready;
	uint8_t* next_draw;

	primask = OLEDLL_EnterCritical();

	if (ready_buf != NULL) {
		old_ready = (uint8_t*)ready_buf;
		ready_buf = draw_buf;
		draw_buf = old_ready;
	} else {
		ready_buf = draw_buf;
		next_draw = OLEDLL_FindFreeBuffer();
		if (next_draw != NULL) {
			draw_buf = next_draw;
		}
	}

	OLEDLL_ExitCritical(primask);
}

OLEDLL_StatusTypeDef OLEDLL_Init(void) {
	static const uint8_t init_cmds[] = {
		0xAEU,
		0xD5U, 0x80U,
		0xA8U, 0x3FU,
		0xD3U, 0x00U,
		0x40U,
		0x8DU, 0x14U,
		0x20U, 0x00U,
		0xA1U,
		0xC8U,
		0xDAU, 0x12U,
		0x81U, 0x7FU,
		0xD9U, 0xF1U,
		0xDBU, 0x40U,
		0xA4U,
		0xA6U,
		0xAFU
	};

	draw_buf = fb0;
	ready_buf = NULL;
	busy_buf = NULL;
	oled_initialized = 0U;
	oled_error = 0U;

	memset(fb0, 0, sizeof(fb0));
	memset(fb1, 0, sizeof(fb1));
	memset(fb2, 0, sizeof(fb2));

	HAL_Delay(100U);

	if (OLEDLL_WriteCommandList(init_cmds, (uint32_t)sizeof(init_cmds)) != OLEDLL_OK) {
		return OLEDLL_ERROR;
	}

	if (OLEDLL_SetFullWindow() != OLEDLL_OK) {
		return OLEDLL_ERROR;
	}

	oled_initialized = 1U;

	return OLEDLL_OK;
}

OLEDLL_StatusTypeDef OLEDLL_Clear(void) {
	if (oled_initialized == 0U) {
		return OLEDLL_ERROR;
	}

	if (draw_buf == NULL) {
		return OLEDLL_ERROR;
	}

	memset((uint8_t*)draw_buf, 0, OLEDLL_BUFFER_SIZE);
	OLEDLL_SubmitDrawBuffer();

	return OLEDLL_OK;
}

OLEDLL_StatusTypeDef OLEDLL_WriteFrame(const uint8_t* buffer) {
	if (oled_initialized == 0U) {
		return OLEDLL_ERROR;
	}

	if (buffer == NULL) {
		return OLEDLL_INVALID_PARAM;
	}

	if (draw_buf == NULL) {
		return OLEDLL_ERROR;
	}

	memcpy((uint8_t*)draw_buf, buffer, OLEDLL_BUFFER_SIZE);
	OLEDLL_SubmitDrawBuffer();

	return OLEDLL_OK;
}

OLEDLL_StatusTypeDef OLEDLL_Update(void) {
	uint32_t primask;
	uint8_t* local_send_buf;

	if (oled_initialized == 0U) {
		return OLEDLL_ERROR;
	}

	if (busy_buf != NULL) {
		return OLEDLL_BUSY;
	}

	if (ready_buf == NULL) {
		return OLEDLL_OK;
	}

	local_send_buf = NULL;

	primask = OLEDLL_EnterCritical();
	if ((busy_buf == NULL) && (ready_buf != NULL)) {
		busy_buf = ready_buf;
		ready_buf = NULL;
		local_send_buf = (uint8_t*)busy_buf;
	}
	OLEDLL_ExitCritical(primask);

	if (local_send_buf == NULL) {
		return OLEDLL_BUSY;
	}

	oled_error = 0U;

	if (OLEDLL_StartTransfer(local_send_buf) != OLEDLL_OK) {
		primask = OLEDLL_EnterCritical();
		busy_buf = NULL;
		oled_error = 1U;
		OLEDLL_ExitCritical(primask);
		return OLEDLL_ERROR;
	}

	return OLEDLL_OK;
}

void OLEDLL_TxCpltCallback(void) {
	uint32_t primask;

	primask = OLEDLL_EnterCritical();
	busy_buf = NULL;
	OLEDLL_ExitCritical(primask);
}

void OLEDLL_ErrorCallback(void) {
	uint32_t primask;

	primask = OLEDLL_EnterCritical();
	busy_buf = NULL;
	oled_error = 1U;
	OLEDLL_ExitCritical(primask);
}




