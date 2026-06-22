#include "oledll.h"

#include <string.h>

#include "i2c.h"

#define OLEDLL_CTRL_CMD   0x00U
#define OLEDLL_CTRL_DATA  0x40U
#define OLEDLL_PAGE_COUNT (OLEDLL_HEIGHT / 8U)

#ifndef OLEDLL_COL_OFFSET
#define OLEDLL_COL_OFFSET 0U
#endif

#ifndef OLEDLL_DATA_TIMEOUT
#define OLEDLL_DATA_TIMEOUT 100U
#endif

#ifndef OLEDLL_ADDR_ALT
#define OLEDLL_ADDR_ALT 0x7AU
#endif

/*
 * draw_buf: current writable back buffer.
 * ready_buf: latest completed frame waiting for transfer.
 * busy_buf: buffer currently owned by the I2C transfer.
 *
 * The panel refresh is intentionally sent one page at a time with blocking I2C.
 * This is slower than a 1024-byte DMA burst, but it is much more tolerant of
 * SSD1306/SH1106-compatible modules and avoids long-frame DMA reload glitches.
 */
static uint8_t fb0[OLEDLL_BUFFER_SIZE];
static uint8_t fb1[OLEDLL_BUFFER_SIZE];
static uint8_t fb2[OLEDLL_BUFFER_SIZE];

static uint8_t* volatile draw_buf = NULL;
static uint8_t* volatile ready_buf = NULL;
static uint8_t* volatile busy_buf = NULL;

static volatile uint8_t oled_initialized = 0U;
static volatile uint8_t oled_error = 0U;
static uint16_t oled_active_addr = OLEDLL_ADDR;

static uint32_t OLEDLL_EnterCritical(void);
static void OLEDLL_ExitCritical(uint32_t primask);
static OLEDLL_StatusTypeDef OLEDLL_SelectAddress(void);
static OLEDLL_StatusTypeDef OLEDLL_WriteCommand(uint8_t cmd);
static OLEDLL_StatusTypeDef OLEDLL_WriteCommandList(const uint8_t* cmds, uint32_t len);
static OLEDLL_StatusTypeDef OLEDLL_WriteDataBlock(const uint8_t* data, uint16_t len);
static OLEDLL_StatusTypeDef OLEDLL_SetPageAddress(uint8_t page);
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

static OLEDLL_StatusTypeDef OLEDLL_SelectAddress(void) {
	if (HAL_I2C_IsDeviceReady(&OLEDLL_HANDLE, OLEDLL_ADDR, 2U, OLEDLL_TIMEOUT) == HAL_OK) {
		oled_active_addr = OLEDLL_ADDR;
		return OLEDLL_OK;
	}

	if (HAL_I2C_IsDeviceReady(&OLEDLL_HANDLE, OLEDLL_ADDR_ALT, 2U, OLEDLL_TIMEOUT) == HAL_OK) {
		oled_active_addr = OLEDLL_ADDR_ALT;
		return OLEDLL_OK;
	}

	oled_active_addr = OLEDLL_ADDR;
	return OLEDLL_ERROR;
}

static OLEDLL_StatusTypeDef OLEDLL_WriteCommand(uint8_t cmd) {
	if (HAL_I2C_Mem_Write(&OLEDLL_HANDLE,
						  oled_active_addr,
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

static OLEDLL_StatusTypeDef OLEDLL_WriteDataBlock(const uint8_t* data, uint16_t len) {
	if ((data == NULL) || (len == 0U)) {
		return OLEDLL_INVALID_PARAM;
	}

	if (HAL_I2C_Mem_Write(&OLEDLL_HANDLE,
						  oled_active_addr,
						  OLEDLL_CTRL_DATA,
						  I2C_MEMADD_SIZE_8BIT,
						  (uint8_t*)data,
						  len,
						  OLEDLL_DATA_TIMEOUT) != HAL_OK) {
		return OLEDLL_ERROR;
	}

	return OLEDLL_OK;
}

static OLEDLL_StatusTypeDef OLEDLL_SetPageAddress(uint8_t page) {
	uint8_t col = (uint8_t)OLEDLL_COL_OFFSET;
	uint8_t page_cmds[] = {
		(uint8_t)(0xB0U | (page & 0x07U)),
		(uint8_t)(0x00U | (col & 0x0FU)),
		(uint8_t)(0x10U | ((col >> 4U) & 0x0FU))
	};

	return OLEDLL_WriteCommandList(page_cmds, (uint32_t)sizeof(page_cmds));
}

static OLEDLL_StatusTypeDef OLEDLL_StartTransfer(uint8_t* buffer) {
	uint8_t page;

	if (buffer == NULL) {
		return OLEDLL_INVALID_PARAM;
	}

	for (page = 0U; page < OLEDLL_PAGE_COUNT; page++) {
		if (OLEDLL_SetPageAddress(page) != OLEDLL_OK) {
			return OLEDLL_ERROR;
		}
		if (OLEDLL_WriteDataBlock(&buffer[(uint16_t)page * OLEDLL_WIDTH],
								  OLEDLL_WIDTH) != OLEDLL_OK) {
			return OLEDLL_ERROR;
		}
	}

	return OLEDLL_OK;
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
		0x2EU,
		0xD5U, 0x80U,
		0xA8U, 0x3FU,
		0xD3U, 0x00U,
		0x40U,
		0x8DU, 0x14U,
		0x20U, 0x02U,
		0xB0U,
		0x00U,
		0x10U,
		0xA1U,
		0xC8U,
		0xDAU, 0x12U,
		0x81U, 0x7FU,
		0xD9U, 0xF1U,
		0xDBU, 0x40U,
		0xA4U,
		0xA6U
	};
	static const uint8_t display_on_cmd = 0xAFU;

	draw_buf = fb0;
	ready_buf = NULL;
	busy_buf = NULL;
	oled_initialized = 0U;
	oled_error = 0U;

	memset(fb0, 0, sizeof(fb0));
	memset(fb1, 0, sizeof(fb1));
	memset(fb2, 0, sizeof(fb2));

	HAL_Delay(100U);

	if (OLEDLL_SelectAddress() != OLEDLL_OK) {
		oled_error = 1U;
		return OLEDLL_ERROR;
	}

	if (OLEDLL_WriteCommandList(init_cmds, (uint32_t)sizeof(init_cmds)) != OLEDLL_OK) {
		return OLEDLL_ERROR;
	}

	if (OLEDLL_StartTransfer(fb0) != OLEDLL_OK) {
		return OLEDLL_ERROR;
	}

	if (OLEDLL_WriteCommand(display_on_cmd) != OLEDLL_OK) {
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

	primask = OLEDLL_EnterCritical();
	busy_buf = NULL;
	OLEDLL_ExitCritical(primask);

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




