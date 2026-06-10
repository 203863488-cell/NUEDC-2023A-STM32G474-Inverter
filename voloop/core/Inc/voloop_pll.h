// Phase-Locked Loop
#ifndef VOLOOP_PLL_H
#define VOLOOP_PLL_H

#include "voloop_pid.h"
#include "voloop_nco.h"

typedef struct {
    void (*InitFunc)(void);
    void (*DeInitFunc)(void);
    float (*GetInputValue)(void);
    const PID_InitTypeDef* LoopFilterInit;
    const NCO_InitTypeDef* NCOInit;
} PLL_InitTypeDef;

typedef enum {
    PLL_ERROR = 0U,
    PLL_STOPPED,
    PLL_RUNNING
} PLL_StateTypeDef;

typedef enum {
    PLL_UNLOCKED = 0U,
    PLL_LOCKED
} PLL_LockStateTypeDef;

typedef struct PLL_HandleTypeDef PLL_HandleTypeDef;

VOLOOP_StatusTypeDef VOLOOP_PLL_Init(PLL_HandleTypeDef** handleOut, const PLL_InitTypeDef* init);
VOLOOP_StatusTypeDef VOLOOP_PLL_DeInit(PLL_HandleTypeDef** handleOut);
VOLOOP_StatusTypeDef VOLOOP_PLL_Start(PLL_HandleTypeDef* handle);
VOLOOP_StatusTypeDef VOLOOP_PLL_Stop(PLL_HandleTypeDef* handle);
PLL_StateTypeDef VOLOOP_PLL_GetState(PLL_HandleTypeDef* handle);

PLL_LockStateTypeDef VOLOOP_PLL_IsLocked(PLL_HandleTypeDef* handle);
int32_t VOLOOP_PLL_GetPhaseQ31(PLL_HandleTypeDef* handle);  // range: -2^31 to 2^31-1
float VOLOOP_PLL_GetRad(PLL_HandleTypeDef* handle);  // range: [-pi, pi)
float VOLOOP_PLL_GetFrequency(PLL_HandleTypeDef* handle);

VOLOOP_StatusTypeDef VOLOOP_PLL_Sync(PLL_HandleTypeDef* handle);

#endif
