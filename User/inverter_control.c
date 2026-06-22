#include "inverter_control.h"

#include <float.h>
#include <math.h>
#include <stddef.h>

#if !defined(__VFP_FP__) || defined(__SOFTFP__)
#error "The inverter control loop requires the STM32G474 hardware FPU"
#endif

#define IC_TWO_PI                       6.28318530717958647692f
#define IC_RAD_TO_DEG                   57.2957795130823208768f
#define IC_SQRT_TWO                     1.41421356237309504880f
#define IC_SIN_TABLE_SIZE               256U
#define IC_SIN_FRACTION_SCALE           (1.0f / 256.0f)

#define IC_PR_BRANCH_COUNT              INVERTER_CONTROL_PR_BRANCH_COUNT
#define IC_FUNDAMENTAL_BRANCH           0U
#define IC_HARMONIC_FIRST_BRANCH        1U
#define IC_LEARNING_FIRST_ORDER         3U
#define IC_LEARNING_BRANCH_COUNT        12U

#define IC_MODULATION_LIMIT             0.97f
#define IC_DUTY_MIN                     0.01f
#define IC_DUTY_MAX                     0.99f
#define IC_PHASE_MARGIN_RESERVE_DEG     3.0f
#define IC_GAIN_MARGIN_RESERVE_DB       1.0f
#define IC_MAX_PR_GAIN                  2.0f
#define IC_MODEL_MIN_FREQUENCY_HZ       20.0f
#define IC_MODEL_MAX_FREQUENCY_HZ       1800.0f
#define IC_MODEL_FREQUENCY_STEP_HZ      2.0f
#define IC_GAIN_SEARCH_ITERATIONS       8U
#define IC_FINAL_SCALE_ITERATIONS       10U
#define IC_DEADLINE_TRIP_COUNT          3U

#define IC_ADC_EDGE_GUARD_COUNTS        140U
#define IC_ADC_MAX_SAMPLE_RATIO         0.42f

#define IC_REFERENCE_SETTLED_V          0.02f
#define IC_LEARNING_SETTLE_TIME_S       0.50f
#define IC_LEARNING_REGULARIZATION      0.04f
#define IC_LEARNING_STABILITY_FACTOR    0.25f
#define IC_LEARNING_ACTIVE_LEAK         0.9995f
#define IC_LEARNING_SETTLING_DECAY      0.995f
#define IC_LEARNING_FROZEN_DECAY        0.98f
#define IC_LEARNING_MIN_FUNDAMENTAL_V   1.0f
#define IC_ZERO_CROSS_WINDOW_DEG        4.0f

typedef struct {
    float real;
    float imaginary;
} IC_ComplexTypeDef;

typedef struct {
    float frequency_hz;
    float bandwidth_hz;
    float target_loop_gain;
    float desired_gain;
    float gain;
    float output_limit;
    float b0;
    float b2;
    float a1;
    float a2;
    float state_1;
    float state_2;
} IC_PRBranchTypeDef;

typedef struct {
    uint8_t order;
    uint8_t active;
    float inverse_real;
    float inverse_imaginary;
    float learning_rate;
    float output_limit;
    float command_sine;
    float command_cosine;
    float error_sine_sum;
    float error_cosine_sum;
    float voltage_sine_sum;
    float voltage_cosine_sum;
} IC_LearningBranchTypeDef;

typedef struct {
    InverterControl_InitTypeDef cfg;
    uint8_t initialized;

    uint32_t phase_accumulator;
    uint32_t phase_step;
    uint32_t phase_step_remainder;
    uint32_t phase_remainder_accumulator;
    uint32_t deadtime_phase_lead_step;
    uint16_t samples_per_cycle;
    uint16_t cycle_sample;
    float sin_table[IC_SIN_TABLE_SIZE + 1U];

    volatile float target_rms;
    volatile float reference_rms;
    volatile uint8_t learning_reset_request;

    float proportional_gain;
    IC_PRBranchTypeDef pr[IC_PR_BRANCH_COUNT];
    float harmonic_gain_scale;
    float estimated_phase_margin_deg;
    float estimated_gain_margin_db;

    float feedback_alpha;
    float proportional_error_1;
    float proportional_error_2;

    float zero_cross_sine;
    float deadtime_max_command;
    float harmonic_command_reserve;
    float base_command_limit;
    float resonant_frequency_hz;

    IC_LearningBranchTypeDef learning[IC_LEARNING_BRANCH_COUNT];
    InverterControl_LearningStateTypeDef learning_state;
    uint32_t learning_settled_samples;
    uint32_t learning_settle_required;
    uint8_t learning_cycle_saturated;
    float fundamental_sine_sum;
    float fundamental_cosine_sum;
    float positive_zero_error_sum;
    float negative_zero_error_sum;
    uint16_t positive_zero_error_count;
    uint16_t negative_zero_error_count;

    volatile uint32_t diagnostics_sequence;
    InverterControl_DiagnosticsTypeDef diagnostics;

    volatile InverterControl_FaultTypeDef fault;
    uint8_t deadline_overruns;
    volatile uint32_t max_execution_cycles;
} InverterControl_ContextTypeDef;

static const uint8_t s_prOrders[IC_PR_BRANCH_COUNT] = {1U, 3U, 5U, 7U, 9U};
static const float s_prTargetLoopGains[IC_PR_BRANCH_COUNT] = {
    20.0f, 8.0f, 5.0f, 3.5f, 2.5f
};
static const float s_prOutputLimits[IC_PR_BRANCH_COUNT] = {
    0.22f, 0.030f, 0.022f, 0.017f, 0.013f
};
static const float s_learningOutputLimits[IC_LEARNING_BRANCH_COUNT] = {
    0.025f, 0.018f, 0.014f, 0.011f,
    0.009f, 0.008f, 0.007f, 0.006f,
    0.0055f, 0.0050f, 0.0045f, 0.0040f
};

static InverterControl_ContextTypeDef s_control;

static float IC_Clamp(float value, float minimum, float maximum)
{
    if (value > maximum) {
        return maximum;
    }
    if (value < minimum) {
        return minimum;
    }
    return value;
}

static float IC_Min(float a, float b)
{
    return a < b ? a : b;
}

static float IC_SqrtPositive(float value)
{
    float result;

    if (value <= 0.0f) {
        return 0.0f;
    }
    __asm volatile (
        "vsqrt.f32 %0, %1"
        : "=t" (result)
        : "t" (value));
    return result;
}

/*
 * All frequency-domain design scans stay below 1.8 kHz at a 50 kHz control
 * rate.  Their largest angle is below 0.37 rad, where this eighth-order
 * expansion is more accurate than the plant tolerances and avoids millions of
 * slow libm calls during bridge-disabled startup.
 */
static void IC_FastSinCosSmall(float angle, float *sine, float *cosine)
{
    const float x2 = angle * angle;
    const float x4 = x2 * x2;
    const float x6 = x4 * x2;
    const float x8 = x4 * x4;

    *sine = angle
          * (1.0f - x2 / 6.0f + x4 / 120.0f - x6 / 5040.0f);
    *cosine =
        1.0f - x2 / 2.0f + x4 / 24.0f
        - x6 / 720.0f + x8 / 40320.0f;
}

static IC_ComplexTypeDef IC_ComplexAdd(IC_ComplexTypeDef a,
                                        IC_ComplexTypeDef b)
{
    IC_ComplexTypeDef result = {
        .real = a.real + b.real,
        .imaginary = a.imaginary + b.imaginary,
    };
    return result;
}

static IC_ComplexTypeDef IC_ComplexMultiply(IC_ComplexTypeDef a,
                                             IC_ComplexTypeDef b)
{
    IC_ComplexTypeDef result = {
        .real = a.real * b.real - a.imaginary * b.imaginary,
        .imaginary = a.real * b.imaginary + a.imaginary * b.real,
    };
    return result;
}

static IC_ComplexTypeDef IC_ComplexDivide(IC_ComplexTypeDef numerator,
                                           IC_ComplexTypeDef denominator)
{
    const float magnitude_squared =
        denominator.real * denominator.real
        + denominator.imaginary * denominator.imaginary;
    IC_ComplexTypeDef result = {
        .real = (numerator.real * denominator.real
                 + numerator.imaginary * denominator.imaginary)
              / magnitude_squared,
        .imaginary = (numerator.imaginary * denominator.real
                      - numerator.real * denominator.imaginary)
                   / magnitude_squared,
    };
    return result;
}

static IC_ComplexTypeDef IC_ComplexScale(IC_ComplexTypeDef value,
                                          float scale)
{
    IC_ComplexTypeDef result = {
        .real = value.real * scale,
        .imaginary = value.imaginary * scale,
    };
    return result;
}

static float IC_ComplexMagnitudeSquared(IC_ComplexTypeDef value)
{
    return value.real * value.real
         + value.imaginary * value.imaginary;
}

static float IC_ComplexMagnitude(IC_ComplexTypeDef value)
{
    return IC_SqrtPositive(IC_ComplexMagnitudeSquared(value));
}

static float IC_SineFromPhase(uint32_t phase)
{
    uint32_t index = (phase >> 24) & 0xFFU;
    uint32_t fraction = (phase >> 16) & 0xFFU;

    return s_control.sin_table[index]
         + (float)fraction * IC_SIN_FRACTION_SCALE
         * (s_control.sin_table[index + 1U]
            - s_control.sin_table[index]);
}

static uint16_t IC_ClampCompare(float compare, uint16_t period_counts)
{
    const float minimum = IC_DUTY_MIN * (float)period_counts;
    const float maximum = IC_DUTY_MAX * (float)period_counts;

    compare = IC_Clamp(compare, minimum, maximum);
    return (uint16_t)(compare + 0.5f);
}

static float IC_MaxCommandRms(void)
{
    float maximum = s_control.cfg.dc_bus_voltage
                  * s_control.base_command_limit / IC_SQRT_TWO;
    return IC_Clamp(maximum,
                    s_control.cfg.voltage_rms_min,
                    s_control.cfg.voltage_rms_max);
}

static float IC_ClampTarget(float rms)
{
    return IC_Clamp(rms, s_control.cfg.voltage_rms_min, IC_MaxCommandRms());
}

static void IC_WriteSafeOutput(InverterControl_OutputTypeDef *output)
{
    const uint16_t half = s_control.cfg.pwm_period_counts / 2U;

    output->compare_a = half;
    output->compare_b = half;
    output->adc_trigger = InverterControl_GetInitialAdcTrigger();
    output->modulation = 0.0f;
    output->reference_rms = s_control.reference_rms;
    output->feedback_voltage = 0.0f;
    output->filtered_error = s_control.proportional_error_2;
    output->harmonic_correction = 0.0f;
    output->deadtime_correction = 0.0f;
    output->learning_correction = 0.0f;
}

static uint16_t IC_SelectAdcTrigger(uint16_t compare_a, uint16_t compare_b)
{
    const uint16_t low_edge =
        compare_a < compare_b ? compare_a : compare_b;
    const uint16_t high_edge =
        compare_a < compare_b ? compare_b : compare_a;
    const uint32_t max_sample = (uint32_t)(
        (float)s_control.cfg.pwm_period_counts * IC_ADC_MAX_SAMPLE_RATIO);
    uint32_t best_start = IC_ADC_EDGE_GUARD_COUNTS;
    uint32_t best_end = max_sample;
    uint32_t best_width = 0U;

    if (low_edge > IC_ADC_EDGE_GUARD_COUNTS) {
        uint32_t end =
            (uint32_t)low_edge - IC_ADC_EDGE_GUARD_COUNTS;
        if (end > max_sample) {
            end = max_sample;
        }
        if (end > best_start) {
            best_end = end;
            best_width = end - best_start;
        }
    }

    if ((uint32_t)low_edge + IC_ADC_EDGE_GUARD_COUNTS < max_sample
        && high_edge > IC_ADC_EDGE_GUARD_COUNTS) {
        uint32_t start =
            (uint32_t)low_edge + IC_ADC_EDGE_GUARD_COUNTS;
        uint32_t end =
            (uint32_t)high_edge - IC_ADC_EDGE_GUARD_COUNTS;

        if (end > max_sample) {
            end = max_sample;
        }
        if (end > start && (end - start) > best_width) {
            best_start = start;
            best_end = end;
            best_width = end - start;
        }
    }

    if (best_width == 0U || best_end <= best_start) {
        return InverterControl_GetInitialAdcTrigger();
    }
    return (uint16_t)(best_start + (best_end - best_start) / 2U);
}

static IC_ComplexTypeDef IC_CommandPathResponse(float frequency_hz,
                                                 float load_ohm,
                                                 float l1_scale,
                                                 float l2_scale,
                                                 float c_scale)
{
    const float ts = 1.0f / s_control.cfg.control_frequency_hz;
    const float omega = IC_TWO_PI * frequency_hz;
    const float omega_squared = omega * omega;
    const float l1 = s_control.cfg.inverter_inductance_h * l1_scale;
    const float l2 = s_control.cfg.load_inductance_h * l2_scale;
    const float capacitance =
        s_control.cfg.filter_capacitance_f * c_scale;
    const float zoh_argument = 0.5f * omega * ts;
    float zoh_sine;
    float zoh_cosine;
    float sample_sine;
    float sample_average_gain;
    float delay_sine;
    float delay_cosine;
    const float delay_angle = omega * 1.625f * ts;
    IC_ComplexTypeDef plant_denominator = {
        .real = load_ohm
              - l1 * load_ohm * capacitance * omega_squared,
        .imaginary =
            (l1 + l2) * omega
            - l1 * l2 * capacitance * omega_squared * omega,
    };
    IC_ComplexTypeDef plant = IC_ComplexDivide(
        (IC_ComplexTypeDef){load_ohm, 0.0f}, plant_denominator);
    IC_ComplexTypeDef delay;

    IC_FastSinCosSmall(
        zoh_argument, &zoh_sine, &zoh_cosine);
    IC_FastSinCosSmall(
        0.5f * omega / s_control.cfg.switching_frequency_hz,
        &sample_sine, &sample_average_gain);
    IC_FastSinCosSmall(
        delay_angle, &delay_sine, &delay_cosine);
    (void)zoh_cosine;
    (void)sample_sine;
    delay.real = delay_cosine;
    delay.imaginary = -delay_sine;

    plant = IC_ComplexScale(
        plant,
        s_control.cfg.dc_bus_voltage
        * (fabsf(zoh_argument) > FLT_EPSILON
           ? zoh_sine / zoh_argument
           : 1.0f)
        * sample_average_gain);
    return IC_ComplexMultiply(plant, delay);
}

static IC_ComplexTypeDef IC_ProportionalFilterResponse(float frequency_hz)
{
    const float angle =
        IC_TWO_PI * frequency_hz / s_control.cfg.control_frequency_hz;
    const float one_minus_alpha = 1.0f - s_control.feedback_alpha;
    float sine;
    float cosine;
    IC_ComplexTypeDef denominator = {
        .real = 0.0f,
        .imaginary = 0.0f,
    };
    IC_ComplexTypeDef response;

    IC_FastSinCosSmall(angle, &sine, &cosine);
    denominator.real = 1.0f - one_minus_alpha * cosine;
    denominator.imaginary = one_minus_alpha * sine;
    response = IC_ComplexDivide(
        (IC_ComplexTypeDef){s_control.feedback_alpha, 0.0f},
        denominator);

    return IC_ComplexMultiply(response, response);
}

static void IC_ComputePRCoefficients(IC_PRBranchTypeDef *branch)
{
    const float ts = 1.0f / s_control.cfg.control_frequency_hz;
    const float w0 = IC_TWO_PI * branch->frequency_hz;
    const float wc = IC_TWO_PI * branch->bandwidth_hz;
    const float prewarped = w0 / tanf(0.5f * w0 * ts);
    const float prewarped_squared = prewarped * prewarped;
    const float w0_squared = w0 * w0;
    const float denominator =
        prewarped_squared + 2.0f * wc * prewarped + w0_squared;

    branch->b0 = 2.0f * wc * prewarped / denominator;
    branch->b2 = -branch->b0;
    branch->a1 =
        (-2.0f * prewarped_squared + 2.0f * w0_squared)
        / denominator;
    branch->a2 =
        (prewarped_squared - 2.0f * wc * prewarped + w0_squared)
        / denominator;
}

static IC_ComplexTypeDef IC_PRResponse(float frequency_hz,
                                        const IC_PRBranchTypeDef *branch)
{
    const float angle =
        IC_TWO_PI * frequency_hz / s_control.cfg.control_frequency_hz;
    float sine;
    float cosine;
    IC_ComplexTypeDef z1;
    IC_ComplexTypeDef z2;
    IC_ComplexTypeDef numerator;
    IC_ComplexTypeDef denominator;

    IC_FastSinCosSmall(angle, &sine, &cosine);
    z1.real = cosine;
    z1.imaginary = -sine;
    z2 = IC_ComplexMultiply(z1, z1);
    numerator = IC_ComplexAdd(
        (IC_ComplexTypeDef){branch->b0, 0.0f},
        IC_ComplexScale(z2, branch->b2));
    denominator = IC_ComplexAdd(
        (IC_ComplexTypeDef){1.0f, 0.0f},
        IC_ComplexAdd(
            IC_ComplexScale(z1, branch->a1),
            IC_ComplexScale(z2, branch->a2)));
    return IC_ComplexDivide(numerator, denominator);
}

static IC_ComplexTypeDef IC_ControllerResponse(float frequency_hz)
{
    IC_ComplexTypeDef controller = IC_ComplexScale(
        IC_ProportionalFilterResponse(frequency_hz),
        s_control.proportional_gain);
    uint8_t i;

    for (i = 0U; i < IC_PR_BRANCH_COUNT; i++) {
        controller = IC_ComplexAdd(
            controller,
            IC_ComplexScale(
                IC_PRResponse(frequency_hz, &s_control.pr[i]),
                s_control.pr[i].gain));
    }
    return controller;
}

static IC_ComplexTypeDef IC_LoopResponse(float frequency_hz,
                                          float load_ohm,
                                          float l1_scale,
                                          float l2_scale,
                                          float c_scale)
{
    return IC_ComplexMultiply(
        IC_CommandPathResponse(
            frequency_hz, load_ohm, l1_scale, l2_scale, c_scale),
        IC_ControllerResponse(frequency_hz));
}

static void IC_EvaluateMarginsForPlant(float load_ohm,
                                        float l1_scale,
                                        float l2_scale,
                                        float c_scale,
                                        float *minimum_phase_margin_deg,
                                        float *minimum_gain_margin_db)
{
    const float maximum_frequency = IC_Clamp(
        0.38f * s_control.resonant_frequency_hz,
        IC_MODEL_MIN_FREQUENCY_HZ + IC_MODEL_FREQUENCY_STEP_HZ,
        IC_MODEL_MAX_FREQUENCY_HZ);
    IC_ComplexTypeDef previous = IC_LoopResponse(
        IC_MODEL_MIN_FREQUENCY_HZ,
        load_ohm, l1_scale, l2_scale, c_scale);
    float previous_magnitude = IC_ComplexMagnitude(previous);
    float frequency;

    for (frequency =
             IC_MODEL_MIN_FREQUENCY_HZ + IC_MODEL_FREQUENCY_STEP_HZ;
         frequency <= maximum_frequency;
         frequency += IC_MODEL_FREQUENCY_STEP_HZ) {
        IC_ComplexTypeDef loop = IC_LoopResponse(
            frequency, load_ohm, l1_scale, l2_scale, c_scale);
        float magnitude = IC_ComplexMagnitude(loop);

        if ((previous_magnitude >= 1.0f && magnitude < 1.0f)
            || (previous_magnitude < 1.0f && magnitude >= 1.0f)) {
            float phase = atan2f(loop.imaginary, loop.real) * IC_RAD_TO_DEG;
            float margin;

            if (phase > 0.0f) {
                phase -= 360.0f;
            }
            margin = 180.0f + phase;
            if (margin < *minimum_phase_margin_deg) {
                *minimum_phase_margin_deg = margin;
            }
        }

        if (((previous.imaginary >= 0.0f && loop.imaginary < 0.0f)
             || (previous.imaginary < 0.0f && loop.imaginary >= 0.0f))
            && previous.real < 0.0f
            && loop.real < 0.0f) {
            float crossing_magnitude =
                0.5f * (previous_magnitude + magnitude);
            float gain_margin_db =
                -20.0f * log10f(IC_Clamp(
                    crossing_magnitude, 1.0e-9f, 1.0e9f));

            if (gain_margin_db < *minimum_gain_margin_db) {
                *minimum_gain_margin_db = gain_margin_db;
            }
        }

        previous = loop;
        previous_magnitude = magnitude;
    }
}

static void IC_EvaluateRobustMargins(uint8_t all_component_corners,
                                      float *minimum_phase_margin_deg,
                                      float *minimum_gain_margin_db)
{
    static const float loads[3] = {5.0f, 10.0f, 20.0f};
    uint8_t load_index;

    *minimum_phase_margin_deg = 180.0f;
    *minimum_gain_margin_db = 60.0f;

    for (load_index = 0U; load_index < 3U; load_index++) {
        if (all_component_corners != 0U) {
            uint8_t corner;

            for (corner = 0U; corner < 8U; corner++) {
                const float l1_scale =
                    (corner & 1U) != 0U ? 1.1f : 0.9f;
                const float l2_scale =
                    (corner & 2U) != 0U ? 1.1f : 0.9f;
                const float c_scale =
                    (corner & 4U) != 0U ? 1.1f : 0.9f;

                IC_EvaluateMarginsForPlant(
                    loads[load_index],
                    l1_scale, l2_scale, c_scale,
                    minimum_phase_margin_deg,
                    minimum_gain_margin_db);
            }
        } else {
            IC_EvaluateMarginsForPlant(
                loads[load_index],
                0.9f, 0.9f, 0.9f,
                minimum_phase_margin_deg,
                minimum_gain_margin_db);
            IC_EvaluateMarginsForPlant(
                loads[load_index],
                1.0f, 1.0f, 1.0f,
                minimum_phase_margin_deg,
                minimum_gain_margin_db);
            IC_EvaluateMarginsForPlant(
                loads[load_index],
                1.1f, 1.1f, 1.1f,
                minimum_phase_margin_deg,
                minimum_gain_margin_db);
        }
    }
}

static uint8_t IC_MarginsMeetRequirements(uint8_t all_component_corners)
{
    float phase_margin;
    float gain_margin;

    IC_EvaluateRobustMargins(
        all_component_corners, &phase_margin, &gain_margin);
    return (uint8_t)(
        phase_margin
            >= s_control.cfg.requested_phase_margin_deg
               + IC_PHASE_MARGIN_RESERVE_DEG
        && gain_margin
            >= s_control.cfg.minimum_gain_margin_db
               + IC_GAIN_MARGIN_RESERVE_DB);
}

static float IC_ComputeDesiredPRGain(uint8_t branch_index)
{
    IC_PRBranchTypeDef *branch = &s_control.pr[branch_index];
    IC_ComplexTypeDef plant = IC_CommandPathResponse(
        branch->frequency_hz,
        s_control.cfg.nominal_load_ohm,
        1.0f, 1.0f, 1.0f);
    IC_ComplexTypeDef proportional = IC_ComplexScale(
        IC_ProportionalFilterResponse(branch->frequency_hz),
        s_control.proportional_gain);
    IC_ComplexTypeDef basis = IC_PRResponse(
        branch->frequency_hz, branch);
    const float desired_controller_magnitude =
        branch->target_loop_gain / IC_ComplexMagnitude(plant);
    const float basis_magnitude_squared =
        IC_ComplexMagnitudeSquared(basis);
    const float projection =
        proportional.real * basis.real
        + proportional.imaginary * basis.imaginary;
    const float constant =
        IC_ComplexMagnitudeSquared(proportional)
        - desired_controller_magnitude * desired_controller_magnitude;
    const float discriminant =
        projection * projection
        - basis_magnitude_squared * constant;
    float gain;

    if (basis_magnitude_squared <= 1.0e-12f
        || discriminant <= 0.0f) {
        return 0.0f;
    }

    gain =
        (-projection + IC_SqrtPositive(discriminant))
        / basis_magnitude_squared;
    return IC_Clamp(gain, 0.0f, IC_MAX_PR_GAIN);
}

static InverterControl_StatusTypeDef IC_ComputeController(void)
{
    IC_ComplexTypeDef crossover_path;
    IC_ComplexTypeDef crossover_filter;
    float crossover_magnitude;
    float original_gains[IC_PR_BRANCH_COUNT];
    float minimum_scale = 1.0f;
    float phase_margin;
    float gain_margin;
    uint8_t i;

    crossover_path = IC_CommandPathResponse(
        s_control.cfg.design_crossover_hz,
        s_control.cfg.nominal_load_ohm,
        1.0f, 1.0f, 1.0f);
    crossover_filter =
        IC_ProportionalFilterResponse(
            s_control.cfg.design_crossover_hz);
    crossover_magnitude = IC_ComplexMagnitude(
        IC_ComplexMultiply(crossover_path, crossover_filter));
    if (!isfinite(crossover_magnitude)
        || crossover_magnitude <= 0.0f) {
        return INVERTER_CONTROL_UNSTABLE_CONFIGURATION;
    }
    s_control.proportional_gain = 1.0f / crossover_magnitude;

    for (i = 0U; i < IC_PR_BRANCH_COUNT; i++) {
        IC_PRBranchTypeDef *branch = &s_control.pr[i];

        branch->frequency_hz =
            (float)s_prOrders[i] * s_control.cfg.output_frequency_hz;
        branch->bandwidth_hz =
            i == IC_FUNDAMENTAL_BRANCH
            ? s_control.cfg.fundamental_pr_bandwidth_hz
            : s_control.cfg.harmonic_pr_bandwidth_hz;
        branch->target_loop_gain = s_prTargetLoopGains[i];
        branch->output_limit = s_prOutputLimits[i];
        IC_ComputePRCoefficients(branch);
        branch->desired_gain = IC_ComputeDesiredPRGain(i);
        branch->gain = 0.0f;
    }

    if (IC_MarginsMeetRequirements(0U) == 0U) {
        return INVERTER_CONTROL_UNSTABLE_CONFIGURATION;
    }

    /*
     * Each resonant branch is admitted independently.  Later branches are
     * evaluated with all previously accepted branches present, so one
     * aggressive harmonic cannot consume the margin reserved for the others.
     */
    for (i = 0U; i < IC_PR_BRANCH_COUNT; i++) {
        IC_PRBranchTypeDef *branch = &s_control.pr[i];
        float low = 0.0f;
        float high = branch->desired_gain;
        uint8_t iteration;

        branch->gain = high;
        if (IC_MarginsMeetRequirements(0U) != 0U) {
            low = high;
        } else {
            branch->gain = 0.0f;
            for (iteration = 0U;
                 iteration < IC_GAIN_SEARCH_ITERATIONS;
                 iteration++) {
                const float middle = 0.5f * (low + high);

                branch->gain = middle;
                if (IC_MarginsMeetRequirements(0U) != 0U) {
                    low = middle;
                } else {
                    high = middle;
                }
            }
        }
        branch->gain = low;
    }

    for (i = 0U; i < IC_PR_BRANCH_COUNT; i++) {
        original_gains[i] = s_control.pr[i].gain;
    }

    if (IC_MarginsMeetRequirements(1U) == 0U) {
        float low = 0.0f;
        float high = 1.0f;
        uint8_t iteration;

        for (iteration = 0U;
             iteration < IC_FINAL_SCALE_ITERATIONS;
             iteration++) {
            const float middle = 0.5f * (low + high);

            for (i = 0U; i < IC_PR_BRANCH_COUNT; i++) {
                s_control.pr[i].gain = original_gains[i] * middle;
            }
            if (IC_MarginsMeetRequirements(1U) != 0U) {
                low = middle;
            } else {
                high = middle;
            }
        }
        for (i = 0U; i < IC_PR_BRANCH_COUNT; i++) {
            s_control.pr[i].gain = original_gains[i] * low;
        }
    }

    IC_EvaluateRobustMargins(1U, &phase_margin, &gain_margin);
    if (phase_margin
            < s_control.cfg.requested_phase_margin_deg
              + IC_PHASE_MARGIN_RESERVE_DEG
        || gain_margin
            < s_control.cfg.minimum_gain_margin_db
              + IC_GAIN_MARGIN_RESERVE_DB) {
        return INVERTER_CONTROL_UNSTABLE_CONFIGURATION;
    }

    for (i = IC_HARMONIC_FIRST_BRANCH;
         i < IC_PR_BRANCH_COUNT;
         i++) {
        float scale = s_control.pr[i].desired_gain > 0.0f
            ? s_control.pr[i].gain / s_control.pr[i].desired_gain
            : 1.0f;
        minimum_scale = IC_Min(minimum_scale, scale);
    }

    s_control.harmonic_gain_scale = minimum_scale;
    s_control.estimated_phase_margin_deg = phase_margin;
    s_control.estimated_gain_margin_db = gain_margin;
    return INVERTER_CONTROL_OK;
}

static IC_ComplexTypeDef IC_ClosedLoopCommandResponse(
    float frequency_hz,
    float load_ohm,
    float l1_scale,
    float l2_scale,
    float c_scale)
{
    IC_ComplexTypeDef plant = IC_CommandPathResponse(
        frequency_hz, load_ohm, l1_scale, l2_scale, c_scale);
    IC_ComplexTypeDef loop = IC_ComplexMultiply(
        plant, IC_ControllerResponse(frequency_hz));
    IC_ComplexTypeDef denominator = IC_ComplexAdd(
        (IC_ComplexTypeDef){1.0f, 0.0f}, loop);

    return IC_ComplexDivide(plant, denominator);
}

static void IC_ConfigureLearning(void)
{
    static const float loads[3] = {5.0f, 10.0f, 20.0f};
    uint8_t i;

    for (i = 0U; i < IC_LEARNING_BRANCH_COUNT; i++) {
        IC_LearningBranchTypeDef *branch = &s_control.learning[i];
        const uint8_t order =
            (uint8_t)(IC_LEARNING_FIRST_ORDER + 2U * i);
        const float frequency =
            (float)order * s_control.cfg.output_frequency_hz;
        IC_ComplexTypeDef nominal;
        IC_ComplexTypeDef inverse;
        float nominal_magnitude_squared;
        float maximum_stable_rate = FLT_MAX;
        uint8_t load_index;
        uint8_t corner;

        branch->order = order;
        branch->output_limit = s_learningOutputLimits[i];
        branch->active = (uint8_t)(
            s_control.cfg.harmonic_learning_enable != 0U
            && order <= s_control.cfg.harmonic_learning_max_order);
        if (branch->active == 0U) {
            continue;
        }

        nominal = IC_ClosedLoopCommandResponse(
            frequency,
            s_control.cfg.nominal_load_ohm,
            1.0f, 1.0f, 1.0f);
        nominal_magnitude_squared =
            IC_ComplexMagnitudeSquared(nominal);
        if (!isfinite(nominal_magnitude_squared)
            || nominal_magnitude_squared <= 1.0e-8f) {
            branch->active = 0U;
            continue;
        }

        inverse.real =
            nominal.real
            / (nominal_magnitude_squared
               * (1.0f + IC_LEARNING_REGULARIZATION));
        inverse.imaginary =
            -nominal.imaginary
            / (nominal_magnitude_squared
               * (1.0f + IC_LEARNING_REGULARIZATION));
        branch->inverse_real = inverse.real;
        branch->inverse_imaginary = inverse.imaginary;

        for (load_index = 0U; load_index < 3U; load_index++) {
            for (corner = 0U; corner < 8U; corner++) {
                const float l1_scale =
                    (corner & 1U) != 0U ? 1.1f : 0.9f;
                const float l2_scale =
                    (corner & 2U) != 0U ? 1.1f : 0.9f;
                const float c_scale =
                    (corner & 4U) != 0U ? 1.1f : 0.9f;
                IC_ComplexTypeDef actual =
                    IC_ClosedLoopCommandResponse(
                        frequency,
                        loads[load_index],
                        l1_scale, l2_scale, c_scale);
                IC_ComplexTypeDef normalized =
                    IC_ComplexMultiply(inverse, actual);
                const float normalized_magnitude_squared =
                    IC_ComplexMagnitudeSquared(normalized);
                float stable_rate;

                if (normalized.real <= 0.0f
                    || normalized_magnitude_squared <= 1.0e-12f) {
                    maximum_stable_rate = 0.0f;
                    continue;
                }
                stable_rate =
                    2.0f * normalized.real
                    / normalized_magnitude_squared;
                maximum_stable_rate =
                    IC_Min(maximum_stable_rate, stable_rate);
            }
        }

        if (maximum_stable_rate <= 0.0f
            || !isfinite(maximum_stable_rate)) {
            branch->active = 0U;
            continue;
        }

        branch->learning_rate = IC_Min(
            s_control.cfg.harmonic_learning_rate,
            IC_LEARNING_STABILITY_FACTOR * maximum_stable_rate);
    }

    s_control.learning_state =
        s_control.cfg.harmonic_learning_enable != 0U
        ? INVERTER_LEARNING_SETTLING
        : INVERTER_LEARNING_DISABLED;
}

static void IC_ResetLearning(float retain)
{
    uint8_t i;

    for (i = 0U; i < IC_LEARNING_BRANCH_COUNT; i++) {
        IC_LearningBranchTypeDef *branch = &s_control.learning[i];

        branch->command_sine *= retain;
        branch->command_cosine *= retain;
        branch->error_sine_sum = 0.0f;
        branch->error_cosine_sum = 0.0f;
        branch->voltage_sine_sum = 0.0f;
        branch->voltage_cosine_sum = 0.0f;
    }

    s_control.learning_settled_samples = 0U;
    s_control.learning_cycle_saturated = 0U;
    s_control.fundamental_sine_sum = 0.0f;
    s_control.fundamental_cosine_sum = 0.0f;
    s_control.positive_zero_error_sum = 0.0f;
    s_control.negative_zero_error_sum = 0.0f;
    s_control.positive_zero_error_count = 0U;
    s_control.negative_zero_error_count = 0U;
    s_control.learning_state =
        s_control.cfg.harmonic_learning_enable != 0U
        ? INVERTER_LEARNING_SETTLING
        : INVERTER_LEARNING_DISABLED;
}

static float IC_LearningAccumulate(float raw_error,
                                    float feedback_voltage,
                                    uint32_t phase)
{
    const float fundamental_sine = IC_SineFromPhase(phase);
    const float fundamental_cosine =
        IC_SineFromPhase(phase + 0x40000000UL);
    const uint16_t half_cycle = s_control.samples_per_cycle / 2U;
    const uint16_t zero_window = (uint16_t)(
        (float)s_control.samples_per_cycle
        * IC_ZERO_CROSS_WINDOW_DEG / 360.0f + 0.5f);
    float command = 0.0f;
    uint8_t i;

    s_control.fundamental_sine_sum +=
        feedback_voltage * fundamental_sine;
    s_control.fundamental_cosine_sum +=
        feedback_voltage * fundamental_cosine;

    if (s_control.cycle_sample <= zero_window
        || s_control.cycle_sample
            >= s_control.samples_per_cycle - zero_window) {
        s_control.positive_zero_error_sum += fabsf(raw_error);
        s_control.positive_zero_error_count++;
    }
    if (s_control.cycle_sample >= half_cycle - zero_window
        && s_control.cycle_sample <= half_cycle + zero_window) {
        s_control.negative_zero_error_sum += fabsf(raw_error);
        s_control.negative_zero_error_count++;
    }

    for (i = 0U; i < IC_LEARNING_BRANCH_COUNT; i++) {
        IC_LearningBranchTypeDef *branch = &s_control.learning[i];
        const uint32_t harmonic_phase = phase * (uint32_t)branch->order;
        const float harmonic_sine = IC_SineFromPhase(harmonic_phase);
        const float harmonic_cosine =
            IC_SineFromPhase(harmonic_phase + 0x40000000UL);

        branch->error_sine_sum += raw_error * harmonic_sine;
        branch->error_cosine_sum += raw_error * harmonic_cosine;
        branch->voltage_sine_sum +=
            feedback_voltage * harmonic_sine;
        branch->voltage_cosine_sum +=
            feedback_voltage * harmonic_cosine;

        if (branch->active != 0U) {
            command +=
                branch->command_sine * harmonic_sine
                + branch->command_cosine * harmonic_cosine;
        }
    }

    return IC_Clamp(
        command,
        -s_control.cfg.harmonic_learning_total_limit,
        s_control.cfg.harmonic_learning_total_limit);
}

static void IC_ScaleLearningCommandsToLimit(void)
{
    float sum_amplitude = 0.0f;
    uint8_t i;

    for (i = 0U; i < IC_LEARNING_BRANCH_COUNT; i++) {
        const IC_LearningBranchTypeDef *branch =
            &s_control.learning[i];

        if (branch->active != 0U) {
            sum_amplitude += IC_SqrtPositive(
                branch->command_sine * branch->command_sine
                + branch->command_cosine * branch->command_cosine);
        }
    }

    if (sum_amplitude
            > s_control.cfg.harmonic_learning_total_limit
        && sum_amplitude > 0.0f) {
        const float scale =
            s_control.cfg.harmonic_learning_total_limit
            / sum_amplitude;

        for (i = 0U; i < IC_LEARNING_BRANCH_COUNT; i++) {
            s_control.learning[i].command_sine *= scale;
            s_control.learning[i].command_cosine *= scale;
        }
    }
}

static void IC_UpdateDiagnostics(float fundamental_amplitude)
{
    static const uint8_t diagnostic_indices[
        INVERTER_CONTROL_DIAGNOSTIC_HARMONIC_COUNT] = {
        0U, 1U, 2U, 3U
    };
    const float reference_peak =
        IC_Clamp(
            s_control.reference_rms * IC_SQRT_TWO,
            0.1f, FLT_MAX);
    float learning_peak = 0.0f;
    uint8_t i;

    s_control.diagnostics_sequence++;
    for (i = 0U;
         i < INVERTER_CONTROL_DIAGNOSTIC_HARMONIC_COUNT;
         i++) {
        IC_LearningBranchTypeDef *branch =
            &s_control.learning[diagnostic_indices[i]];
        const float voltage_sine =
            2.0f * branch->voltage_sine_sum
            / (float)s_control.samples_per_cycle;
        const float voltage_cosine =
            2.0f * branch->voltage_cosine_sum
            / (float)s_control.samples_per_cycle;
        const float amplitude =
            IC_SqrtPositive(voltage_sine * voltage_sine
                  + voltage_cosine * voltage_cosine);

        s_control.diagnostics.harmonic_percent[i] =
            fundamental_amplitude > IC_LEARNING_MIN_FUNDAMENTAL_V
            ? 100.0f * amplitude / fundamental_amplitude
            : 0.0f;
    }

    for (i = 0U; i < IC_PR_BRANCH_COUNT; i++) {
        s_control.diagnostics.pr_gain[i] =
            s_control.pr[i].gain;
    }
    for (i = 0U; i < IC_LEARNING_BRANCH_COUNT; i++) {
        IC_LearningBranchTypeDef *branch = &s_control.learning[i];

        if (branch->active != 0U) {
            learning_peak += IC_SqrtPositive(
                branch->command_sine * branch->command_sine
                + branch->command_cosine * branch->command_cosine);
        }
    }

    s_control.diagnostics.positive_zero_cross_error_percent =
        s_control.positive_zero_error_count > 0U
        ? 100.0f
          * s_control.positive_zero_error_sum
          / (float)s_control.positive_zero_error_count
          / reference_peak
        : 0.0f;
    s_control.diagnostics.negative_zero_cross_error_percent =
        s_control.negative_zero_error_count > 0U
        ? 100.0f
          * s_control.negative_zero_error_sum
          / (float)s_control.negative_zero_error_count
          / reference_peak
        : 0.0f;
    s_control.diagnostics.learning_command_peak = learning_peak;
    s_control.diagnostics.minimum_phase_margin_deg =
        s_control.estimated_phase_margin_deg;
    s_control.diagnostics.minimum_gain_margin_db =
        s_control.estimated_gain_margin_db;
    s_control.diagnostics.learning_state = s_control.learning_state;
    s_control.diagnostics.learning_max_order =
        s_control.cfg.harmonic_learning_max_order;
    s_control.diagnostics_sequence++;
}

static void IC_CompleteLearningCycle(void)
{
    const float fundamental_sine =
        2.0f * s_control.fundamental_sine_sum
        / (float)s_control.samples_per_cycle;
    const float fundamental_cosine =
        2.0f * s_control.fundamental_cosine_sum
        / (float)s_control.samples_per_cycle;
    const float fundamental_amplitude =
        IC_SqrtPositive(fundamental_sine * fundamental_sine
              + fundamental_cosine * fundamental_cosine);
    const uint8_t reference_settled = (uint8_t)(
        fabsf(s_control.target_rms - s_control.reference_rms)
            <= IC_REFERENCE_SETTLED_V
        && s_control.reference_rms
            >= s_control.cfg.voltage_rms_min);
    uint8_t learning_allowed = 0U;
    uint8_t i;

    if (s_control.cfg.harmonic_learning_enable != 0U
        && reference_settled != 0U
        && s_control.learning_settled_samples
            >= s_control.learning_settle_required
        && s_control.learning_cycle_saturated == 0U
        && fundamental_amplitude
            >= IC_LEARNING_MIN_FUNDAMENTAL_V) {
        learning_allowed = 1U;
        s_control.learning_state = INVERTER_LEARNING_ACTIVE;
    } else if (s_control.cfg.harmonic_learning_enable == 0U) {
        s_control.learning_state = INVERTER_LEARNING_DISABLED;
    } else if (s_control.learning_cycle_saturated != 0U) {
        s_control.learning_state = INVERTER_LEARNING_FROZEN;
    } else {
        s_control.learning_state = INVERTER_LEARNING_SETTLING;
    }

    for (i = 0U; i < IC_LEARNING_BRANCH_COUNT; i++) {
        IC_LearningBranchTypeDef *branch = &s_control.learning[i];

        if (branch->active != 0U) {
            if (learning_allowed != 0U) {
                const float error_sine =
                    2.0f * branch->error_sine_sum
                    / (float)s_control.samples_per_cycle;
                const float error_cosine =
                    2.0f * branch->error_cosine_sum
                    / (float)s_control.samples_per_cycle;
                const float delta_cosine =
                    branch->learning_rate
                    * (branch->inverse_real * error_cosine
                       + branch->inverse_imaginary * error_sine);
                const float delta_sine =
                    branch->learning_rate
                    * (branch->inverse_real * error_sine
                       - branch->inverse_imaginary * error_cosine);
                float amplitude;

                branch->command_sine =
                    branch->command_sine * IC_LEARNING_ACTIVE_LEAK
                    + delta_sine;
                branch->command_cosine =
                    branch->command_cosine * IC_LEARNING_ACTIVE_LEAK
                    + delta_cosine;
                amplitude = IC_SqrtPositive(
                    branch->command_sine * branch->command_sine
                    + branch->command_cosine * branch->command_cosine);
                if (amplitude > branch->output_limit
                    && amplitude > 0.0f) {
                    const float scale =
                        branch->output_limit / amplitude;
                    branch->command_sine *= scale;
                    branch->command_cosine *= scale;
                }
            } else {
                const float decay =
                    s_control.learning_cycle_saturated != 0U
                    ? IC_LEARNING_FROZEN_DECAY
                    : IC_LEARNING_SETTLING_DECAY;
                branch->command_sine *= decay;
                branch->command_cosine *= decay;
            }

            if (!isfinite(branch->command_sine)
                || !isfinite(branch->command_cosine)) {
                InverterControl_Trip(INVERTER_FAULT_LEARNING);
                return;
            }
        }
    }

    IC_ScaleLearningCommandsToLimit();
    IC_UpdateDiagnostics(fundamental_amplitude);

    for (i = 0U; i < IC_LEARNING_BRANCH_COUNT; i++) {
        s_control.learning[i].error_sine_sum = 0.0f;
        s_control.learning[i].error_cosine_sum = 0.0f;
        s_control.learning[i].voltage_sine_sum = 0.0f;
        s_control.learning[i].voltage_cosine_sum = 0.0f;
    }
    s_control.fundamental_sine_sum = 0.0f;
    s_control.fundamental_cosine_sum = 0.0f;
    s_control.positive_zero_error_sum = 0.0f;
    s_control.negative_zero_error_sum = 0.0f;
    s_control.positive_zero_error_count = 0U;
    s_control.negative_zero_error_count = 0U;
    s_control.learning_cycle_saturated = 0U;
}

InverterControl_StatusTypeDef InverterControl_Init(
    const InverterControl_InitTypeDef *init)
{
    InverterControl_StatusTypeDef status;
    const float samples_per_cycle =
        init != NULL
        ? init->control_frequency_hz / init->output_frequency_hz
        : 0.0f;
    uint16_t i;

    if (init == NULL
        || init->dc_bus_voltage <= 0.0f
        || init->inverter_inductance_h <= 0.0f
        || init->load_inductance_h <= 0.0f
        || init->filter_capacitance_f <= 0.0f
        || init->nominal_load_ohm <= 0.0f
        || init->output_frequency_hz <= 0.0f
        || init->switching_frequency_hz <= 0.0f
        || init->control_frequency_hz <= 0.0f
        || fabsf(init->switching_frequency_hz
                 - 2.0f * init->control_frequency_hz)
            > 0.001f * init->switching_frequency_hz
        || init->design_crossover_hz <= init->output_frequency_hz
        || init->requested_phase_margin_deg < 45.0f
        || init->requested_phase_margin_deg > 85.0f
        || init->minimum_gain_margin_db < 6.0f
        || init->feedback_lpf_hz <= init->design_crossover_hz
        || init->fundamental_pr_bandwidth_hz <= 0.0f
        || init->harmonic_pr_bandwidth_hz <= 0.0f
        || init->pwm_period_counts < 100U
        || init->voltage_rms_min <= 0.0f
        || init->voltage_rms_max < init->voltage_rms_min
        || init->output_overvoltage_peak_v
            <= init->voltage_rms_max * IC_SQRT_TWO
        || init->deadtime_seconds <= 0.0f
        || init->deadtime_compensation_gain <= 0.0f
        || init->zero_cross_soft_angle_deg < 1.0f
        || init->zero_cross_soft_angle_deg > 10.0f
        || fabsf(init->deadtime_phase_lead_deg) > 10.0f
        || init->harmonic_learning_rate <= 0.0f
        || init->harmonic_learning_rate > 0.5f
        || init->harmonic_learning_total_limit <= 0.0f
        || init->harmonic_learning_total_limit > 0.2f
        || init->harmonic_learning_max_order
            > INVERTER_CONTROL_MAX_LEARNING_HARMONIC
        || init->harmonic_learning_max_order < 3U
        || (init->harmonic_learning_max_order & 1U) == 0U
        || fabsf(samples_per_cycle - floorf(samples_per_cycle + 0.5f))
            > 1.0e-4f
        || samples_per_cycle < 100.0f
        || samples_per_cycle > 65535.0f) {
        return INVERTER_CONTROL_INVALID_PARAM;
    }

    s_control = (InverterControl_ContextTypeDef){0};
    s_control.cfg = *init;
    s_control.samples_per_cycle =
        (uint16_t)(samples_per_cycle + 0.5f);
    s_control.phase_step = (uint32_t)(
        0x100000000ULL / (uint64_t)s_control.samples_per_cycle);
    s_control.phase_step_remainder = (uint32_t)(
        0x100000000ULL % (uint64_t)s_control.samples_per_cycle);
    s_control.deadtime_phase_lead_step = (uint32_t)(int32_t)(
        4294967296.0f * init->deadtime_phase_lead_deg / 360.0f);
    s_control.resonant_frequency_hz =
        IC_SqrtPositive(
            (init->inverter_inductance_h + init->load_inductance_h)
              / (init->inverter_inductance_h
                 * init->load_inductance_h
                 * init->filter_capacitance_f))
        / IC_TWO_PI;
    s_control.zero_cross_sine = sinf(
        init->zero_cross_soft_angle_deg / IC_RAD_TO_DEG);
    s_control.deadtime_max_command =
        init->deadtime_seconds
        * init->switching_frequency_hz
        * init->deadtime_compensation_gain;
    s_control.learning_settle_required = (uint32_t)(
        IC_LEARNING_SETTLE_TIME_S * init->control_frequency_hz);

    for (i = IC_HARMONIC_FIRST_BRANCH;
         i < IC_PR_BRANCH_COUNT;
         i++) {
        s_control.harmonic_command_reserve += s_prOutputLimits[i];
    }
    s_control.base_command_limit =
        IC_MODULATION_LIMIT
        - s_control.deadtime_max_command
        - s_control.harmonic_command_reserve
        - init->harmonic_learning_total_limit;
    if (s_control.base_command_limit <= 0.5f) {
        return INVERTER_CONTROL_INVALID_PARAM;
    }

    for (i = 0U; i <= IC_SIN_TABLE_SIZE; i++) {
        s_control.sin_table[i] =
            sinf(IC_TWO_PI * (float)i / (float)IC_SIN_TABLE_SIZE);
    }

    s_control.feedback_alpha =
        1.0f - expf(
            -IC_TWO_PI * init->feedback_lpf_hz
            / init->control_frequency_hz);
    status = IC_ComputeController();
    if (status != INVERTER_CONTROL_OK) {
        return status;
    }
    IC_ConfigureLearning();

    s_control.reference_rms = IC_Clamp(
        init->voltage_rms_start, 0.0f, IC_MaxCommandRms());
    s_control.target_rms = s_control.reference_rms;
    s_control.diagnostics.minimum_phase_margin_deg =
        s_control.estimated_phase_margin_deg;
    s_control.diagnostics.minimum_gain_margin_db =
        s_control.estimated_gain_margin_db;
    s_control.diagnostics.learning_state =
        s_control.learning_state;
    s_control.diagnostics.learning_max_order =
        init->harmonic_learning_max_order;
    for (i = 0U; i < IC_PR_BRANCH_COUNT; i++) {
        s_control.diagnostics.pr_gain[i] =
            s_control.pr[i].gain;
    }
    s_control.initialized = 1U;
    return INVERTER_CONTROL_OK;
}

void InverterControl_Reset(void)
{
    uint8_t i;

    s_control.phase_accumulator = 0U;
    s_control.phase_remainder_accumulator = 0U;
    s_control.cycle_sample = 0U;
    s_control.proportional_error_1 = 0.0f;
    s_control.proportional_error_2 = 0.0f;
    for (i = 0U; i < IC_PR_BRANCH_COUNT; i++) {
        s_control.pr[i].state_1 = 0.0f;
        s_control.pr[i].state_2 = 0.0f;
    }
    s_control.learning_reset_request = 0U;
    IC_ResetLearning(0.0f);
    s_control.deadline_overruns = 0U;
    s_control.max_execution_cycles = 0U;
    s_control.reference_rms = IC_Clamp(
        s_control.cfg.voltage_rms_start, 0.0f, IC_MaxCommandRms());
    s_control.target_rms = s_control.reference_rms;
    s_control.fault = INVERTER_FAULT_NONE;
}

void InverterControl_SetTargetRms(float rms)
{
    if (s_control.initialized != 0U
        && s_control.fault == INVERTER_FAULT_NONE) {
        const float target = IC_ClampTarget(rms);
        const float previous_target = s_control.target_rms;

        s_control.target_rms = target;
        if (fabsf(target - previous_target) > 0.05f) {
            s_control.learning_reset_request = 1U;
        }
    }
}

void InverterControl_Step(const Sample_ControlFrameTypeDef *frame,
                          InverterControl_OutputTypeDef *output)
{
    const float ts = 1.0f / s_control.cfg.control_frequency_hz;
    const float ramp_step = s_control.cfg.voltage_ramp_v_per_s * ts;
    float branch_state_1[IC_PR_BRANCH_COUNT];
    float branch_state_2[IC_PR_BRANCH_COUNT];
    float branch_raw[IC_PR_BRANCH_COUNT];
    float branch_output[IC_PR_BRANCH_COUNT];
    float target;
    float sine;
    float reference_voltage;
    float raw_error;
    float proportional_error;
    float raw_base_command;
    float base_command;
    float harmonic_command = 0.0f;
    float learning_command;
    float deadtime_sine;
    float deadtime_blend;
    float deadtime_command;
    float raw_modulation;
    float modulation;
    uint32_t phase;
    uint8_t cycle_complete = 0U;
    uint8_t global_state_update;
    uint8_t i;

    if (output == NULL) {
        return;
    }
    if (s_control.initialized == 0U || frame == NULL
        || s_control.fault != INVERTER_FAULT_NONE) {
        IC_WriteSafeOutput(output);
        return;
    }
    if (fabsf(frame->voltage_v)
        > s_control.cfg.output_overvoltage_peak_v) {
        InverterControl_Trip(INVERTER_FAULT_OUTPUT_OVERVOLTAGE);
        IC_WriteSafeOutput(output);
        return;
    }
    if (s_control.learning_reset_request != 0U) {
        IC_ResetLearning(0.0f);
        s_control.learning_reset_request = 0U;
    }

    target = IC_ClampTarget(s_control.target_rms);
    if (s_control.reference_rms < target) {
        s_control.reference_rms += ramp_step;
        if (s_control.reference_rms > target) {
            s_control.reference_rms = target;
        }
    } else if (s_control.reference_rms > target) {
        s_control.reference_rms -= ramp_step;
        if (s_control.reference_rms < target) {
            s_control.reference_rms = target;
        }
    }

    if (fabsf(target - s_control.reference_rms)
        <= IC_REFERENCE_SETTLED_V) {
        if (s_control.learning_settled_samples < UINT32_MAX) {
            s_control.learning_settled_samples++;
        }
    } else {
        s_control.learning_settled_samples = 0U;
    }

    phase = s_control.phase_accumulator;
    sine = IC_SineFromPhase(phase);
    reference_voltage =
        s_control.reference_rms * IC_SQRT_TWO * sine;

    raw_error = reference_voltage - frame->voltage_v;
    s_control.proportional_error_1 += s_control.feedback_alpha
        * (raw_error - s_control.proportional_error_1);
    s_control.proportional_error_2 += s_control.feedback_alpha
        * (s_control.proportional_error_1
           - s_control.proportional_error_2);
    proportional_error = s_control.proportional_error_2;

    for (i = 0U; i < IC_PR_BRANCH_COUNT; i++) {
        IC_PRBranchTypeDef *branch = &s_control.pr[i];

        branch_raw[i] =
            branch->gain * branch->b0 * raw_error
            + branch->state_1;
        branch_output[i] = IC_Clamp(
            branch_raw[i],
            -branch->output_limit,
            branch->output_limit);
        branch_state_1[i] =
            -branch->a1 * branch_output[i]
            + branch->state_2;
        branch_state_2[i] =
            branch->gain * branch->b2 * raw_error
            - branch->a2 * branch_output[i];

        if (!isfinite(branch_state_1[i])
            || !isfinite(branch_state_2[i])
            || !isfinite(branch_raw[i])) {
            InverterControl_Trip(INVERTER_FAULT_NUMERICAL);
            IC_WriteSafeOutput(output);
            return;
        }
        if (i >= IC_HARMONIC_FIRST_BRANCH) {
            harmonic_command += branch_output[i];
        }
    }

    learning_command = IC_LearningAccumulate(
        raw_error, frame->voltage_v, phase);

    raw_base_command =
        reference_voltage / s_control.cfg.dc_bus_voltage
        + s_control.proportional_gain * proportional_error
        + branch_output[IC_FUNDAMENTAL_BRANCH];
    base_command = IC_Clamp(
        raw_base_command,
        -s_control.base_command_limit,
        s_control.base_command_limit);

    deadtime_sine = IC_SineFromPhase(
        phase + s_control.deadtime_phase_lead_step);
    deadtime_blend = IC_Clamp(
        deadtime_sine / s_control.zero_cross_sine, -1.0f, 1.0f);
    deadtime_command =
        s_control.deadtime_max_command * deadtime_blend;

    raw_modulation =
        base_command
        + harmonic_command
        + learning_command
        + deadtime_command;
    modulation = IC_Clamp(
        raw_modulation, -IC_MODULATION_LIMIT, IC_MODULATION_LIMIT);
    if (!isfinite(raw_modulation)) {
        InverterControl_Trip(INVERTER_FAULT_NUMERICAL);
        IC_WriteSafeOutput(output);
        return;
    }

    if (raw_modulation != modulation
        || raw_base_command != base_command) {
        s_control.learning_cycle_saturated = 1U;
    }

    global_state_update =
        (uint8_t)((raw_modulation == modulation)
        || (raw_modulation > IC_MODULATION_LIMIT && raw_error < 0.0f)
        || (raw_modulation < -IC_MODULATION_LIMIT && raw_error > 0.0f));

    for (i = 0U; i < IC_PR_BRANCH_COUNT; i++) {
        IC_PRBranchTypeDef *branch = &s_control.pr[i];
        uint8_t branch_state_update =
            (uint8_t)((branch_raw[i] == branch_output[i])
            || (branch_raw[i] > branch->output_limit
                && raw_error < 0.0f)
            || (branch_raw[i] < -branch->output_limit
                && raw_error > 0.0f));

        if (i == IC_FUNDAMENTAL_BRANCH) {
            branch_state_update =
                (uint8_t)(branch_state_update
                && ((raw_base_command == base_command)
                || (raw_base_command > s_control.base_command_limit
                    && raw_error < 0.0f)
                || (raw_base_command < -s_control.base_command_limit
                    && raw_error > 0.0f)));
        }

        if (global_state_update != 0U
            && branch_state_update != 0U) {
            branch->state_1 = IC_Clamp(
                branch_state_1[i], -2.0f, 2.0f);
            branch->state_2 = IC_Clamp(
                branch_state_2[i], -2.0f, 2.0f);
        }
    }

    output->compare_a = IC_ClampCompare(
        (0.5f + 0.5f * modulation)
        * (float)s_control.cfg.pwm_period_counts,
        s_control.cfg.pwm_period_counts);
    output->compare_b = IC_ClampCompare(
        (0.5f - 0.5f * modulation)
        * (float)s_control.cfg.pwm_period_counts,
        s_control.cfg.pwm_period_counts);
    output->adc_trigger =
        IC_SelectAdcTrigger(output->compare_a, output->compare_b);
    output->modulation = modulation;
    output->reference_rms = s_control.reference_rms;
    output->feedback_voltage = frame->voltage_v;
    output->filtered_error = proportional_error;
    output->harmonic_correction = harmonic_command;
    output->deadtime_correction = deadtime_command;
    output->learning_correction = learning_command;

    s_control.phase_accumulator += s_control.phase_step;
    s_control.phase_remainder_accumulator +=
        s_control.phase_step_remainder;
    if (s_control.phase_remainder_accumulator
        >= s_control.samples_per_cycle) {
        s_control.phase_accumulator++;
        s_control.phase_remainder_accumulator -=
            s_control.samples_per_cycle;
    }
    s_control.cycle_sample++;
    if (s_control.cycle_sample >= s_control.samples_per_cycle) {
        s_control.cycle_sample = 0U;
        s_control.phase_accumulator = 0U;
        s_control.phase_remainder_accumulator = 0U;
        cycle_complete = 1U;
    }
    if (cycle_complete != 0U) {
        IC_CompleteLearningCycle();
    }
}

void InverterControl_ReportExecutionCycles(uint32_t elapsed_cycles,
                                           uint32_t cycle_budget)
{
    if (elapsed_cycles > s_control.max_execution_cycles) {
        s_control.max_execution_cycles = elapsed_cycles;
    }

    if (cycle_budget == 0U
        || s_control.fault != INVERTER_FAULT_NONE) {
        return;
    }

    if (elapsed_cycles >= cycle_budget) {
        if (s_control.deadline_overruns < UINT8_MAX) {
            s_control.deadline_overruns++;
        }
        if (s_control.deadline_overruns >= IC_DEADLINE_TRIP_COUNT) {
            InverterControl_Trip(INVERTER_FAULT_CONTROL_DEADLINE);
        }
    } else {
        s_control.deadline_overruns = 0U;
    }
}

void InverterControl_Trip(InverterControl_FaultTypeDef fault)
{
    uint8_t i;

    if (fault == INVERTER_FAULT_NONE
        || s_control.fault != INVERTER_FAULT_NONE) {
        return;
    }
    s_control.fault = fault;
    s_control.target_rms = s_control.cfg.voltage_rms_min;
    s_control.reference_rms = s_control.cfg.voltage_rms_min;
    for (i = 0U; i < IC_PR_BRANCH_COUNT; i++) {
        s_control.pr[i].state_1 = 0.0f;
        s_control.pr[i].state_2 = 0.0f;
    }
    s_control.learning_reset_request = 0U;
    IC_ResetLearning(0.0f);
}

InverterControl_FaultTypeDef InverterControl_GetFault(void)
{
    return s_control.fault;
}

float InverterControl_GetTargetRms(void)
{
    return s_control.target_rms;
}

float InverterControl_GetReferenceRms(void)
{
    return s_control.reference_rms;
}

float InverterControl_GetMaxCommandRms(void)
{
    return s_control.initialized != 0U
        ? IC_MaxCommandRms()
        : 0.0f;
}

float InverterControl_GetResonantFrequencyHz(void)
{
    return s_control.resonant_frequency_hz;
}

float InverterControl_GetEstimatedPhaseMarginDeg(void)
{
    return s_control.estimated_phase_margin_deg;
}

float InverterControl_GetEstimatedGainMarginDb(void)
{
    return s_control.estimated_gain_margin_db;
}

float InverterControl_GetHarmonicGainScale(void)
{
    return s_control.harmonic_gain_scale;
}

uint32_t InverterControl_GetMaxExecutionCycles(void)
{
    return s_control.max_execution_cycles;
}

uint16_t InverterControl_GetInitialAdcTrigger(void)
{
    if (s_control.cfg.pwm_period_counts == 0U) {
        return 425U;
    }
    return (uint16_t)(
        (float)s_control.cfg.pwm_period_counts * 0.25f);
}

InverterControl_StatusTypeDef InverterControl_GetDiagnostics(
    InverterControl_DiagnosticsTypeDef *diagnostics)
{
    uint32_t sequence_before;
    uint32_t sequence_after;
    uint8_t attempt;

    if (diagnostics == NULL) {
        return INVERTER_CONTROL_INVALID_PARAM;
    }
    if (s_control.initialized == 0U) {
        return INVERTER_CONTROL_NOT_INITIALIZED;
    }

    for (attempt = 0U; attempt < 3U; attempt++) {
        sequence_before = s_control.diagnostics_sequence;
        if ((sequence_before & 1U) != 0U) {
            continue;
        }
        *diagnostics = s_control.diagnostics;
        sequence_after = s_control.diagnostics_sequence;
        if (sequence_before == sequence_after
            && (sequence_after & 1U) == 0U) {
            return INVERTER_CONTROL_OK;
        }
    }
    return INVERTER_CONTROL_NOT_INITIALIZED;
}
