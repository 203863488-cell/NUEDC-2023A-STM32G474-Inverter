#ifndef OLEDLL_H
#define OLEDLL_H

#include <stdint.h>

#ifndef OLEDLL_HANDLE
#define OLEDLL_HANDLE hi2c1
#endif

#ifndef OLEDLL_ADDR
#define OLEDLL_ADDR 0x78U
#endif

#ifndef OLEDLL_TIMEOUT
#define OLEDLL_TIMEOUT  10U
#endif

#ifndef OLEDLL_WIDTH
#define OLEDLL_WIDTH    128U
#endif

#ifndef OLEDLL_HEIGHT
#define OLEDLL_HEIGHT    64U
#endif

#ifndef OLEDLL_BUFFER_SIZE
#define OLEDLL_BUFFER_SIZE           ((OLEDLL_WIDTH * OLEDLL_HEIGHT) / 8U)
#endif

typedef enum {
    OLEDLL_OK = 0,
    OLEDLL_BUSY,
    OLEDLL_ERROR,
    OLEDLL_INVALID_PARAM
} OLEDLL_StatusTypeDef;

OLEDLL_StatusTypeDef OLEDLL_Init(void);

OLEDLL_StatusTypeDef OLEDLL_Clear(void);
OLEDLL_StatusTypeDef OLEDLL_WriteFrame(const uint8_t* buffer);
OLEDLL_StatusTypeDef OLEDLL_Update(void);

void OLEDLL_TxCpltCallback(void);
void OLEDLL_ErrorCallback(void);

#endif
