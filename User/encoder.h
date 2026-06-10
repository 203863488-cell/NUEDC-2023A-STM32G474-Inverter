#ifndef ENCODER_H
#define ENCODER_H

#include <stdint.h>

#ifndef ENCODER_HANDLE
#define ENCODER_HANDLE htim3
#endif

#ifndef ENCODER_COUNT_SCALE
#define ENCODER_COUNT_SCALE 4U
#endif

#ifndef ENCODER_POLARITY
#define ENCODER_POLARITY -1
#endif

void Encoder_Init();
void Encoder_DeInit(void);

int16_t Encoder_GetCountRaw(void);
int16_t Encoder_GetCount(void);
void Encoder_SetCount(int16_t count);
void Encoder_Clear(void);

int16_t Encoder_PopCount(void);

#endif