#include "sample.h"

#include "adc.h"

#include <math.h>
#include <stddef.h>

#define SAMPLE_INJECTED_TIMEOUT_MS 10U
#define SAMPLE_CALIBRATION_SAMPLE_TIMEOUT_MS 2U
#define SAMPLE_VREFINT_SAMPLES 16U
#define SAMPLE_CALIBRATION_TRIM_SAMPLES 32U
#define SAMPLE_VOLTAGE_GAIN 62.6f
#define SAMPLE_ZERO_MIN_RAW ((uint16_t)(SAMPLE_ADCResolution / 5U))
#define SAMPLE_ZERO_MAX_RAW ((uint16_t)(SAMPLE_ADCResolution * 4U / 5U))
#define SAMPLE_ADC_RAIL_GUARD_RAW 16U
#define SAMPLE_CALIBRATION_MAX_RMS_RAW 8.0f
#define SAMPLE_CALIBRATION_MAX_SPAN_RAW 48U

_Static_assert(
    (SAMPLE_CONTROL_CALIBRATION_FRAMES
     % SAMPLE_CONTROL_SAMPLES_PER_FRAME) == 0U,
    "calibration sample count must contain complete DMA frames");

static volatile uint16_t
    s_dmaBuffer[SAMPLE_CONTROL_SAMPLES_PER_FRAME] = {0U};
static uint16_t s_calibrationSamples[SAMPLE_CONTROL_CALIBRATION_FRAMES];
static float s_voltageScaleFactor;
static float s_voltageZeroRaw;
static uint32_t s_controlSequence = 0U;
static Sample_VoltageCalibrationTypeDef s_calibration;
static volatile float s_latestVoltage;

static float Sample_RawDeltaToVoltage(float raw, float zero_raw, float gain)
{
    return (raw - zero_raw) * s_voltageScaleFactor * gain;
}

static void Sample_SortCalibrationSamples(void)
{
    uint16_t i;

    for (i = 1U; i < SAMPLE_CONTROL_CALIBRATION_FRAMES; i++) {
        uint16_t value = s_calibrationSamples[i];
        uint16_t j = i;

        while (j > 0U && s_calibrationSamples[j - 1U] > value) {
            s_calibrationSamples[j] = s_calibrationSamples[j - 1U];
            j--;
        }
        s_calibrationSamples[j] = value;
    }
}

static SAMPLE_StatusTypeDef Sample_MeasureAnalogReference(void)
{
    uint32_t raw_sum = 0U;
    uint16_t i;
    uint16_t average_raw;
    uint32_t analog_reference_mv;

    for (i = 0U; i < SAMPLE_VREFINT_SAMPLES; i++) {
        __HAL_ADC_CLEAR_FLAG(
            &hadc1, ADC_FLAG_JEOC | ADC_FLAG_JEOS);
        if (HAL_ADCEx_InjectedStart(&hadc1) != HAL_OK) {
            return SAMPLE_ERROR;
        }
        if (HAL_ADCEx_InjectedPollForConversion(
                &hadc1, SAMPLE_INJECTED_TIMEOUT_MS) != HAL_OK) {
            return SAMPLE_TIMEOUT;
        }
        raw_sum += (uint16_t)HAL_ADCEx_InjectedGetValue(
            &hadc1, ADC_INJECTED_RANK_1);
    }

    average_raw = (uint16_t)(raw_sum / SAMPLE_VREFINT_SAMPLES);
    if (average_raw == 0U) {
        return SAMPLE_CALIBRATION_ERROR;
    }

    analog_reference_mv = __HAL_ADC_CALC_VREFANALOG_VOLTAGE(
        average_raw, ADC_RESOLUTION_12B);
    if (analog_reference_mv < 2500U || analog_reference_mv > 3600U) {
        return SAMPLE_CALIBRATION_ERROR;
    }

    s_voltageScaleFactor =
        (float)analog_reference_mv * 0.001f
        / (float)SAMPLE_ADCResolution;
    s_calibration.vdda_v =
        s_voltageScaleFactor * (float)SAMPLE_ADCResolution;
    return SAMPLE_OK;
}

SAMPLE_StatusTypeDef Sample_Init(void)
{
    SAMPLE_StatusTypeDef status;

    if (HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED) != HAL_OK) {
        return SAMPLE_ERROR;
    }
    status = Sample_MeasureAnalogReference();
    if (status != SAMPLE_OK) {
        return status;
    }

    s_calibration.valid = 0U;
    s_voltageZeroRaw = 0.0f;
    s_controlSequence = 0U;
    s_latestVoltage = 0.0f;

    HAL_NVIC_DisableIRQ(DMA1_Channel2_IRQn);
    HAL_NVIC_ClearPendingIRQ(DMA1_Channel2_IRQn);
    if (HAL_ADC_Start_DMA(&hadc1,
                          (uint32_t *)(uintptr_t)s_dmaBuffer,
                          SAMPLE_CONTROL_SAMPLES_PER_FRAME) != HAL_OK) {
        return SAMPLE_ERROR;
    }

    /*
     * The control ISR reads the completed frame at the PWM period boundary.
     * DMA transfer interrupts would fire at 100 kHz and add avoidable jitter.
     */
    __HAL_DMA_DISABLE_IT(
        hadc1.DMA_Handle, DMA_IT_HT | DMA_IT_TC | DMA_IT_TE);
    return SAMPLE_OK;
}

SAMPLE_StatusTypeDef Sample_CalibrateZero(
    Sample_VoltageCalibrationTypeDef *result)
{
    const uint32_t transfer_complete_flag =
        __HAL_DMA_GET_TC_FLAG_INDEX(hadc1.DMA_Handle);
    const uint32_t transfer_error_flag =
        __HAL_DMA_GET_TE_FLAG_INDEX(hadc1.DMA_Handle);
    uint64_t sum = 0U;
    float variance_sum = 0.0f;
    uint16_t accepted_samples;
    uint16_t i;

    if (result == NULL) {
        return SAMPLE_INVALID_PARAM;
    }
    *result = (Sample_VoltageCalibrationTypeDef){0};
    s_calibration.valid = 0U;

    if ((hadc1.Instance->CR & ADC_CR_ADSTART) == 0U
        || hadc1.DMA_Handle == NULL) {
        return SAMPLE_ERROR;
    }

    __HAL_DMA_CLEAR_FLAG(
        hadc1.DMA_Handle, transfer_complete_flag | transfer_error_flag);
    for (i = 0U;
         i < SAMPLE_CONTROL_CALIBRATION_FRAMES;
         i = (uint16_t)(
             i + SAMPLE_CONTROL_SAMPLES_PER_FRAME)) {
        uint32_t start_tick = HAL_GetTick();

        while (__HAL_DMA_GET_FLAG(
                   hadc1.DMA_Handle, transfer_complete_flag) == RESET) {
            if (__HAL_DMA_GET_FLAG(
                    hadc1.DMA_Handle, transfer_error_flag) != RESET) {
                __HAL_DMA_CLEAR_FLAG(
                    hadc1.DMA_Handle, transfer_error_flag);
                return SAMPLE_ERROR;
            }
            if ((HAL_GetTick() - start_tick)
                >= SAMPLE_CALIBRATION_SAMPLE_TIMEOUT_MS) {
                return SAMPLE_TIMEOUT;
            }
        }

        __DMB();
        s_calibrationSamples[i] = s_dmaBuffer[0];
        s_calibrationSamples[i + 1U] = s_dmaBuffer[1];
        __HAL_DMA_CLEAR_FLAG(
            hadc1.DMA_Handle, transfer_complete_flag);
    }

    Sample_SortCalibrationSamples();
    accepted_samples = SAMPLE_CONTROL_CALIBRATION_FRAMES
                     - 2U * SAMPLE_CALIBRATION_TRIM_SAMPLES;

    for (i = SAMPLE_CALIBRATION_TRIM_SAMPLES;
         i < SAMPLE_CONTROL_CALIBRATION_FRAMES
             - SAMPLE_CALIBRATION_TRIM_SAMPLES;
         i++) {
        sum += s_calibrationSamples[i];
    }
    s_voltageZeroRaw = (float)sum / (float)accepted_samples;

    for (i = SAMPLE_CALIBRATION_TRIM_SAMPLES;
         i < SAMPLE_CONTROL_CALIBRATION_FRAMES
             - SAMPLE_CALIBRATION_TRIM_SAMPLES;
         i++) {
        float error = (float)s_calibrationSamples[i] - s_voltageZeroRaw;
        variance_sum += error * error;
    }

    s_calibration.zero_raw = s_voltageZeroRaw;
    s_calibration.noise_rms_raw =
        sqrtf(variance_sum / (float)accepted_samples);
    s_calibration.minimum_raw =
        s_calibrationSamples[SAMPLE_CALIBRATION_TRIM_SAMPLES];
    s_calibration.maximum_raw =
        s_calibrationSamples[SAMPLE_CONTROL_CALIBRATION_FRAMES
                             - SAMPLE_CALIBRATION_TRIM_SAMPLES - 1U];
    s_calibration.accepted_samples = accepted_samples;
    s_calibration.rejected_samples =
        2U * SAMPLE_CALIBRATION_TRIM_SAMPLES;

    if (s_voltageZeroRaw < (float)SAMPLE_ZERO_MIN_RAW
        || s_voltageZeroRaw > (float)SAMPLE_ZERO_MAX_RAW
        || s_calibration.noise_rms_raw > SAMPLE_CALIBRATION_MAX_RMS_RAW
        || (uint16_t)(s_calibration.maximum_raw - s_calibration.minimum_raw)
            > SAMPLE_CALIBRATION_MAX_SPAN_RAW) {
        *result = s_calibration;
        return SAMPLE_CALIBRATION_ERROR;
    }

    s_calibration.valid = 1U;
    __HAL_DMA_CLEAR_FLAG(
        hadc1.DMA_Handle, transfer_complete_flag | transfer_error_flag);
    *result = s_calibration;
    return SAMPLE_OK;
}

SAMPLE_StatusTypeDef Sample_GetControlFrame(Sample_ControlFrameTypeDef *frame)
{
    const uint32_t transfer_complete_flag =
        __HAL_DMA_GET_TC_FLAG_INDEX(hadc1.DMA_Handle);
    const uint32_t transfer_error_flag =
        __HAL_DMA_GET_TE_FLAG_INDEX(hadc1.DMA_Handle);
    uint16_t raw_0;
    uint16_t raw_1;
    uint16_t voltage_raw;

    if (frame == NULL) {
        return SAMPLE_INVALID_PARAM;
    }
    if (s_calibration.valid == 0U) {
        return SAMPLE_NOT_CALIBRATED;
    }
    if (__HAL_DMA_GET_FLAG(
            hadc1.DMA_Handle, transfer_error_flag) != RESET) {
        __HAL_DMA_CLEAR_FLAG(
            hadc1.DMA_Handle, transfer_error_flag);
        return SAMPLE_ERROR;
    }
    if (__HAL_DMA_GET_FLAG(
            hadc1.DMA_Handle, transfer_complete_flag) == RESET) {
        return SAMPLE_STALE_DATA;
    }

    /*
     * One DMA frame contains the two 100 kHz samples that belong to the
     * current 50 kHz control interval.  The transfer-complete flag is the
     * ownership boundary: both aligned half-words are stable before it is set.
     */
    __DMB();
    raw_0 = s_dmaBuffer[0];
    raw_1 = s_dmaBuffer[1];
    __HAL_DMA_CLEAR_FLAG(
        hadc1.DMA_Handle, transfer_complete_flag);
    if (raw_0 <= SAMPLE_ADC_RAIL_GUARD_RAW
        || raw_0 >= SAMPLE_ADCResolution
                         - SAMPLE_ADC_RAIL_GUARD_RAW
        || raw_1 <= SAMPLE_ADC_RAIL_GUARD_RAW
        || raw_1 >= SAMPLE_ADCResolution
                            - SAMPLE_ADC_RAIL_GUARD_RAW) {
        return SAMPLE_ERROR;
    }

    voltage_raw = (uint16_t)(
        ((uint32_t)raw_0 + (uint32_t)raw_1 + 1U) / 2U);
    frame->voltage_raw = voltage_raw;
    frame->voltage_v = Sample_RawDeltaToVoltage(
        (float)voltage_raw, s_voltageZeroRaw, SAMPLE_VOLTAGE_GAIN);
    frame->sequence = ++s_controlSequence;
    s_latestVoltage = frame->voltage_v;
    return SAMPLE_OK;
}

SAMPLE_StatusTypeDef Sample_GetCalibration(
    Sample_VoltageCalibrationTypeDef *result)
{
    if (result == NULL) {
        return SAMPLE_INVALID_PARAM;
    }
    *result = s_calibration;
    return s_calibration.valid != 0U ? SAMPLE_OK : SAMPLE_NOT_CALIBRATED;
}

uint16_t Sample_GetRawVoltage(void)
{
    return (uint16_t)(
        ((uint32_t)s_dmaBuffer[0]
         + (uint32_t)s_dmaBuffer[1] + 1U) / 2U);
}

float Sample_GetLatestVoltage(void)
{
    if (s_calibration.valid == 0U) {
        return 0.0f;
    }
    return s_latestVoltage;
}

float Sample_GetAnalogReference(void)
{
    return s_voltageScaleFactor * (float)SAMPLE_ADCResolution;
}
