#include "voloop_buck.h"
#include <stdlib.h>

struct Buck_HandleTypeDef {
    Buck_InitTypeDef Init;
    PID_HandleTypeDef* OutPutVoltagePID;
    PID_HandleTypeDef* InductorCurrentPID;
    Buck_StateTypeDef State;
    Buck_FaultCodeTypeDef FaultCode;
    float TargetOutputVoltage;
    float MaxInductorCurrent;
    float Duty;
};

VOLOOP_StatusTypeDef VOLOOP_Buck_Init(Buck_HandleTypeDef** handleOut, Buck_InitTypeDef* init) {
    // Verify input parameters
    if (handleOut == NULL) {
        return VOLOOP_INVALID_PARAM;
    }
    if (*handleOut != NULL || init == NULL 
        || init->InitFunc == NULL
        || init->DeInitFunc == NULL
        || init->Start == NULL
        || init->Stop == NULL
        || init->SetDuty == NULL
        || init->GetOutputVoltage == NULL
        || init->GetInductorCurrent == NULL) {
        return VOLOOP_INVALID_PARAM;
    }

    // Allocate memory for Buck handle
    Buck_HandleTypeDef* handle = malloc(sizeof(Buck_HandleTypeDef));
    if (handle == NULL) {
        return VOLOOP_BAD_ALLOCATE;
    }

    // Load initialization parameters
    handle->Init = *init;
    handle->TargetOutputVoltage = 0.0f;
    handle->MaxInductorCurrent = 0.0f;
    handle->Duty = 0.0f;
    handle->State = BUCK_DISABLED;
    handle->FaultCode = BUCK_NOERROR;
    handle->OutPutVoltagePID = NULL;
    handle->InductorCurrentPID = NULL;
    (*handleOut) = handle;

    // Call user-defined initialization function
    init->InitFunc();

    //PID initialization
    VOLOOP_StatusTypeDef status;
    status = VOLOOP_PID_Init(&(handle->OutPutVoltagePID), init->OutPutVoltagePIDInit);
    if (status != VOLOOP_OK) {
        VOLOOP_Buck_DeInit(handleOut);
        return status;
    }
    status = VOLOOP_PID_Init(&(handle->InductorCurrentPID), init->InductorCurrentPIDInit);
    if (status != VOLOOP_OK) {
        VOLOOP_Buck_DeInit(handleOut);
        return status;
    }

    return VOLOOP_OK;
}


VOLOOP_StatusTypeDef VOLOOP_Buck_DeInit(Buck_HandleTypeDef** handleOut) {
    // Verify input parameter
    if (handleOut == NULL) {
        return VOLOOP_INVALID_PARAM;
    }
	if (*handleOut == NULL){
		return VOLOOP_INVALID_PARAM;
	}

    Buck_HandleTypeDef* handle = *handleOut;

    // Stop the buck converter
    handle->Init.Stop();

    // Call user-defined deinitialization function
    handle->Init.DeInitFunc();

    // Deinitialize PID controllers
    VOLOOP_PID_DeInit(&(handle->OutPutVoltagePID));
    VOLOOP_PID_DeInit(&(handle->InductorCurrentPID));

    // Free Buck handle memory
    free(handle);
    *handleOut = NULL;
    return VOLOOP_OK;
}


VOLOOP_StatusTypeDef VOLOOP_Buck_Start(Buck_HandleTypeDef* handle) {
    if (handle == NULL) {
        return VOLOOP_INVALID_PARAM;
    }

    if (handle->State == BUCK_ERROR
        || handle->State == BUCK_CVMODE
        || handle->State == BUCK_CCMODE) {
        return VOLOOP_INVALID_STATE; 
    }
    
    VOLOOP_PID_Reset(handle->OutPutVoltagePID);
    VOLOOP_PID_Reset(handle->InductorCurrentPID);
    handle->State = BUCK_CVMODE; // Default to CV mode when starting
    handle->Init.Start();
    
    return VOLOOP_OK;
}


VOLOOP_StatusTypeDef VOLOOP_Buck_Stop(Buck_HandleTypeDef* handle) {
    if (handle == NULL) {
        return VOLOOP_INVALID_PARAM;
    }
    handle->Init.Stop();
    handle->State = BUCK_DISABLED;
    return VOLOOP_OK;
}


Buck_StateTypeDef VOLOOP_Buck_GetState(Buck_HandleTypeDef* handle) {
    if (handle == NULL) {
        return BUCK_ERROR;
    }
    return handle->State;
}

Buck_FaultCodeTypeDef VOLOOP_Buck_GetFaultCode(Buck_HandleTypeDef* handle) {
    if (handle == NULL) {
        return BUCK_INVALID;
    }
    return handle->FaultCode;
}


VOLOOP_StatusTypeDef VOLOOP_Buck_ClearFaultCode(Buck_HandleTypeDef* handle) {
    if (handle == NULL) {
        return VOLOOP_INVALID_PARAM;
    }
    if (handle->State != BUCK_ERROR
        && handle->State != BUCK_DISABLED) {
        return VOLOOP_INVALID_STATE;
    }
    handle->FaultCode = BUCK_NOERROR;
    handle->State = BUCK_DISABLED;
    return VOLOOP_OK;
}


VOLOOP_StatusTypeDef VOLOOP_Buck_SetValue(Buck_HandleTypeDef* handle, float Voltage, float Current) {
    if (handle == NULL) {
        return VOLOOP_INVALID_PARAM;
    }
    handle->TargetOutputVoltage = Voltage;
    handle->MaxInductorCurrent = Current;
    return VOLOOP_OK;
}


float VOLOOP_Buck_GetDuty(Buck_HandleTypeDef* handle) {
    if (handle == NULL) {
        return 0.0f;
    }
    return handle->Duty;
}


VOLOOP_StatusTypeDef VOLOOP_Buck_Sync(Buck_HandleTypeDef* handle) {
    // Verify input parameter
    if (handle == NULL) {
        return VOLOOP_INVALID_PARAM;
    }

    if (handle->State == BUCK_ERROR) {
        return VOLOOP_INVALID_STATE; 
    } else if (handle->State == BUCK_DISABLED) {
        return VOLOOP_INVALID_STATE; 
    }

    //Get necessary parameters
    float presentVoltage = handle->Init.GetOutputVoltage();
    float presentCurrent = handle->Init.GetInductorCurrent();

    //Protection
    if (presentCurrent > BUCK_OCTHRESHOLD){
        VOLOOP_Buck_Stop(handle);
        handle->State = BUCK_ERROR;
        handle->FaultCode = BUCK_OCP;
        return VOLOOP_ERROR;
    }
    if  (presentVoltage > BUCK_OVTHRESHOLD){
        VOLOOP_Buck_Stop(handle);
        handle->State = BUCK_ERROR;
        handle->FaultCode = BUCK_OVP;
        return VOLOOP_ERROR;
    }

    //Compute target inductor current based on output voltage error, with anti-windup
    float targetInductorCurrent = VOLOOP_PID_ComputeConditional(handle->OutPutVoltagePID, handle->TargetOutputVoltage, presentVoltage, 0.0f, handle->MaxInductorCurrent);
    
    if (VOLOOP_PID_GetState(handle->OutPutVoltagePID) == PID_UpperSaturated){
        handle->State = BUCK_CCMODE;
    } else {
        handle->State = BUCK_CVMODE;
    }
    
    float duty = VOLOOP_PID_ComputeConditional(handle->InductorCurrentPID, targetInductorCurrent, presentCurrent, BUCK_MIN_DUTY, BUCK_MAX_DUTY);

    // Set duty cycle
    handle->Duty = duty;
    handle->Init.SetDuty(duty);

    return VOLOOP_OK;

}


