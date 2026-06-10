#ifndef VOLOOP_BUCK_H
#define VOLOOP_BUCK_H

#include "voloop_def.h"
#include "voloop_pid.h"

typedef struct {
    void (*InitFunc)(void);
    void (*DeInitFunc)(void);
    void (*Start)(void);
    void (*Stop)(void);
    void (*SetDuty)(float duty);
    float (*GetInputVoltage)(void);
    float (*GetOutputVoltage)(void);
    float (*GetInductorCurrent)(void);
    PID_InitTypeDef* OutPutVoltagePIDInit;
    PID_InitTypeDef* InductorCurrentPIDInit;
} Buck_InitTypeDef;

typedef enum{
    BUCK_ERROR = 0U,
    BUCK_DISABLED,
    BUCK_CVMODE,
    BUCK_CCMODE
}Buck_StateTypeDef;

typedef enum{
    BUCK_INVALID = 0U,
    BUCK_NOERROR,
    BUCK_OCP,
    BUCK_OVP
}Buck_FaultCodeTypeDef;

typedef struct Buck_HandleTypeDef Buck_HandleTypeDef;

#define BUCK_OVTHRESHOLD 20.0f
#define BUCK_OCTHRESHOLD 2.0f

#define BUCK_MAX_DUTY 0.90f
#define BUCK_MIN_DUTY 0.10f

VOLOOP_StatusTypeDef VOLOOP_Buck_Init(Buck_HandleTypeDef** handleOut, Buck_InitTypeDef* init);
VOLOOP_StatusTypeDef VOLOOP_Buck_DeInit(Buck_HandleTypeDef** handleOut);

VOLOOP_StatusTypeDef VOLOOP_Buck_Start(Buck_HandleTypeDef* handle);
VOLOOP_StatusTypeDef VOLOOP_Buck_Stop(Buck_HandleTypeDef* handle);
Buck_StateTypeDef VOLOOP_Buck_GetState(Buck_HandleTypeDef* handle);
Buck_FaultCodeTypeDef VOLOOP_Buck_GetFaultCode(Buck_HandleTypeDef* handle);
VOLOOP_StatusTypeDef VOLOOP_Buck_ClearFaultCode(Buck_HandleTypeDef* handle);

VOLOOP_StatusTypeDef VOLOOP_Buck_SetValue(Buck_HandleTypeDef* handle, float Voltage, float Current);
float VOLOOP_Buck_GetDuty(Buck_HandleTypeDef* handle);
VOLOOP_StatusTypeDef VOLOOP_Buck_Sync(Buck_HandleTypeDef* handle);


#endif /* VOLOOP_BUCK_H */
