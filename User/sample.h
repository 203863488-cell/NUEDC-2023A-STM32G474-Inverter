#ifndef SAMPLE_H
#define SAMPLE_H

#include <stdint.h>

#define SAMPLE_ADCResolution (4096U)
#define SAMPLE_CONTROL_CALIBRATION_FRAMES (512U)
#define SAMPLE_CONTROL_SAMPLES_PER_FRAME (2U)

typedef enum {
    SAMPLE_OK = 0,
    SAMPLE_BUSY,
    SAMPLE_ERROR,
    SAMPLE_INVALID_PARAM,
    SAMPLE_TIMEOUT,
    SAMPLE_NOT_CALIBRATED,
    SAMPLE_STALE_DATA,
    SAMPLE_CALIBRATION_ERROR
} SAMPLE_StatusTypeDef;

typedef struct {
    uint16_t voltage_raw;
    float voltage_v;
    uint32_t sequence;
} Sample_ControlFrameTypeDef;

typedef struct {
    float zero_raw;
    float vdda_v;
    float noise_rms_raw;
    uint16_t minimum_raw;
    uint16_t maximum_raw;
    uint16_t accepted_samples;
    uint16_t rejected_samples;
    uint8_t valid;
} Sample_VoltageCalibrationTypeDef;

SAMPLE_StatusTypeDef Sample_Init(void);
SAMPLE_StatusTypeDef Sample_CalibrateZero(
    Sample_VoltageCalibrationTypeDef *result);
SAMPLE_StatusTypeDef Sample_GetControlFrame(Sample_ControlFrameTypeDef *frame);
SAMPLE_StatusTypeDef Sample_GetCalibration(
    Sample_VoltageCalibrationTypeDef *result);
uint16_t Sample_GetRawVoltage(void);
float Sample_GetLatestVoltage(void);
float Sample_GetAnalogReference(void);

#endif
