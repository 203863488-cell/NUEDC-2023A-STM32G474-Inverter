#ifndef INVERTER_CONTROL_H
#define INVERTER_CONTROL_H

#include <stdint.h>

#include "sample.h"

#define INVERTER_CONTROL_PR_BRANCH_COUNT 5U
#define INVERTER_CONTROL_DIAGNOSTIC_HARMONIC_COUNT 4U
#define INVERTER_CONTROL_MAX_LEARNING_HARMONIC 25U

typedef enum {
    INVERTER_CONTROL_OK = 0,
    INVERTER_CONTROL_INVALID_PARAM,
    INVERTER_CONTROL_NOT_INITIALIZED,
    INVERTER_CONTROL_UNSTABLE_CONFIGURATION
} InverterControl_StatusTypeDef;

typedef enum {
    INVERTER_FAULT_NONE = 0,
    INVERTER_FAULT_CONTROL_DEADLINE,
    INVERTER_FAULT_NUMERICAL,
    INVERTER_FAULT_ADC,
    INVERTER_FAULT_ADC_CALIBRATION,
    INVERTER_FAULT_OUTPUT_OVERVOLTAGE,
    INVERTER_FAULT_LEARNING
} InverterControl_FaultTypeDef;

typedef enum {
    INVERTER_LEARNING_DISABLED = 0,
    INVERTER_LEARNING_SETTLING,
    INVERTER_LEARNING_ACTIVE,
    INVERTER_LEARNING_FROZEN
} InverterControl_LearningStateTypeDef;

typedef struct {
    float dc_bus_voltage;
    float inverter_inductance_h;
    float load_inductance_h;
    float filter_capacitance_f;
    float nominal_load_ohm;
    float output_frequency_hz;
    float switching_frequency_hz;
    float control_frequency_hz;
    float design_crossover_hz;
    float requested_phase_margin_deg;
    float minimum_gain_margin_db;
    float feedback_lpf_hz;
    float fundamental_pr_bandwidth_hz;
    float harmonic_pr_bandwidth_hz;
    float voltage_rms_min;
    float voltage_rms_max;
    float voltage_rms_start;
    float voltage_ramp_v_per_s;
    float output_overvoltage_peak_v;
    float deadtime_seconds;
    float deadtime_compensation_gain;
    float zero_cross_soft_angle_deg;
    float deadtime_phase_lead_deg;
    float harmonic_learning_rate;
    float harmonic_learning_total_limit;
    uint16_t pwm_period_counts;
    uint8_t harmonic_learning_enable;
    uint8_t harmonic_learning_max_order;
} InverterControl_InitTypeDef;

typedef struct {
    uint16_t compare_a;
    uint16_t compare_b;
    uint16_t adc_trigger;
    float modulation;
    float reference_rms;
    float feedback_voltage;
    float filtered_error;
    float harmonic_correction;
    float deadtime_correction;
    float learning_correction;
} InverterControl_OutputTypeDef;

typedef struct {
    float harmonic_percent[INVERTER_CONTROL_DIAGNOSTIC_HARMONIC_COUNT];
    float positive_zero_cross_error_percent;
    float negative_zero_cross_error_percent;
    float learning_command_peak;
    float minimum_phase_margin_deg;
    float minimum_gain_margin_db;
    float pr_gain[INVERTER_CONTROL_PR_BRANCH_COUNT];
    InverterControl_LearningStateTypeDef learning_state;
    uint8_t learning_max_order;
} InverterControl_DiagnosticsTypeDef;

InverterControl_StatusTypeDef InverterControl_Init(
    const InverterControl_InitTypeDef *init);
/* Call Reset only while the power-stage outputs are disabled. */
void InverterControl_Reset(void);
void InverterControl_SetTargetRms(float rms);
void InverterControl_Step(const Sample_ControlFrameTypeDef *frame,
                          InverterControl_OutputTypeDef *output);
void InverterControl_ReportExecutionCycles(uint32_t elapsed_cycles,
                                           uint32_t cycle_budget);
void InverterControl_Trip(InverterControl_FaultTypeDef fault);

InverterControl_FaultTypeDef InverterControl_GetFault(void);
float InverterControl_GetTargetRms(void);
float InverterControl_GetReferenceRms(void);
float InverterControl_GetMaxCommandRms(void);
float InverterControl_GetResonantFrequencyHz(void);
float InverterControl_GetEstimatedPhaseMarginDeg(void);
float InverterControl_GetEstimatedGainMarginDb(void);
float InverterControl_GetHarmonicGainScale(void);
uint32_t InverterControl_GetMaxExecutionCycles(void);
uint16_t InverterControl_GetInitialAdcTrigger(void);
InverterControl_StatusTypeDef InverterControl_GetDiagnostics(
    InverterControl_DiagnosticsTypeDef *diagnostics);

#endif
