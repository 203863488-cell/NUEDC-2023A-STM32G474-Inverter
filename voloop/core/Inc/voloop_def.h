#ifndef VOLOOP_DEF_H
#define VOLOOP_DEF_H

#include <stdint.h>

#define VOLOOP_Pi 3.14159265358979323846f
#define VOLOOP_Pi_Inv (1.0f / VOLOOP_Pi)
#define VOLOOP_TwoPi (2.0f * VOLOOP_Pi)
#define VOLOOP_FourPiSquared (4.0f * VOLOOP_Pi * VOLOOP_Pi)

#include <math.h>
#ifndef VOLOOP_DEF_SIN
#define VOLOOP_DEF_SIN(x) VOLOOP_DEF_SINQ31(x)
#endif

#ifndef VOLOOP_DEF_COS
#define VOLOOP_DEF_COS(x) VOLOOP_DEF_COSQ31(x)
#endif

#ifndef VOLOOP_DEF_PRINTF
#define VOLOOP_DEF_PRINTF(...) ((void)0)
#endif

typedef enum {
    VOLOOP_OK = 0x00U,
    VOLOOP_ERROR,
    VOLOOP_BAD_ALLOCATE,
    VOLOOP_INVALID_PARAM,
    VOLOOP_INVALID_STATE,
    VOLOOP_BUSY,
    VOLOOP_TIMEOUT
} VOLOOP_StatusTypeDef;


float VOLOOP_DEF_ClampFloat(float value, float min, float max);

float VOLOOP_DEF_Q31ToRad(int32_t value);
int32_t VOLOOP_DEF_RadToQ31(float value);

// Q1.31 phase look-up table
float VOLOOP_DEF_SINQ31(int32_t phaseQ31);
float VOLOOP_DEF_COSQ31(int32_t phaseQ31);


#endif /* VOLOOP_DEF_H */
