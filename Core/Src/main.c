/**
  ******************************************************************************
  * @file    main.c
  * @brief   50 Hz single-phase H-bridge inverter with synchronized PR control.
  ******************************************************************************
  */

#include "main.h"
#include "adc.h"
#include "dma.h"
#include "gpio.h"
#include "hrtim.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"

#include "project_libs.h"

#include <math.h>

/* Power stage and output specification. */
#define VDC_BUS                 48.0f
#define L1_VAL                   0.001f
#define L2_VAL                   0.001f
#define C_VAL                    0.0000022f
#define NOMINAL_LOAD_OHM        10.0f
#define F_OUT                   50.0f
#define F_SW                100000.0f
#define CONTROL_FREQ         50000.0f

#define VREF_RMS_INIT          23.80f
#define VREF_RMS_START           0.0f
#define VREF_RMS_MIN             1.0f
#define VREF_RMS_MAX            30.0f
#define VREF_ENC_STEP            1.0f
#define VREF_RAMP_V_PER_S       40.0f

/*
 * User-facing loop design target.  The controller automatically scales the
 * 3rd/5th/7th/9th PR gains so every modeled unity-gain crossing retains at
 * least this phase margin plus an internal 3-degree design reserve.
 */
#define CONTROL_PHASE_MARGIN_DEG 60.0f
#define CONTROL_GAIN_MARGIN_DB    6.0f
#define CONTROL_CROSSOVER_HZ    100.0f
#define CONTROL_ERROR_LPF_HZ    500.0f
#define FUNDAMENTAL_PR_BW_HZ      2.0f
#define HARMONIC_PR_BW_HZ         5.0f

#define DEADTIME_COMP_GAIN        1.10f
#define DEADTIME_SOFT_ANGLE_DEG    2.0f
#define DEADTIME_PHASE_LEAD_DEG    0.0f
#define HARMONIC_LEARNING_ENABLE    1U
#define HARMONIC_LEARNING_MAX_ORDER 25U
#define HARMONIC_LEARNING_RATE     0.08f
#define HARMONIC_LEARNING_LIMIT    0.075f
#define OUTPUT_OVERVOLTAGE_PEAK_V 45.0f

/* HRTIM uses PER = fHRCK / fPWM, so 170 MHz / 1700 = 100 kHz. */
#define HRTIM_CLOCK_HZ         170000000U
#define HRTIM_PRESCALER         HRTIM_PRESCALERRATIO_DIV1
#define MASTER_PERIOD           1700U
#define PWM_PERIOD              MASTER_PERIOD
#define PWM_COUNTS              PWM_PERIOD
#define HRTIM_IDX_A             HRTIM_TIMERINDEX_TIMER_A
#define HRTIM_IDX_B             HRTIM_TIMERINDEX_TIMER_B
#define HBRIDGE_OUTPUTS \
    (HRTIM_OUTPUT_TA1 | HRTIM_OUTPUT_TA2 \
     | HRTIM_OUTPUT_TB1 | HRTIM_OUTPUT_TB2)
#define CMP_WR(idx, val) \
    (HRTIM1->sTimerxRegs[(idx)].CMP1xR = (uint16_t)(val))
#define ADC_CMP_WR(val) \
    (HRTIM1->sTimerxRegs[HRTIM_IDX_A].CMP3xR = (uint16_t)(val))

/* 30 / 85 MHz = 353 ns dead time. */
#define DT_PRESCALER            HRTIM_TIMDEADTIME_PRESCALERRATIO_DIV2
#define DT_VALUE                30U
#define DT_CLOCK_HZ             85000000.0f

#define CONTROL_CYCLE_BUDGET \
    ((uint32_t)((float)HRTIM_CLOCK_HZ / CONTROL_FREQ))

_Static_assert(PWM_COUNTS == 1700U, "100 kHz HRTIM period must be 1700");
_Static_assert(CONTROL_PHASE_MARGIN_DEG >= 45.0f
               && CONTROL_PHASE_MARGIN_DEG <= 85.0f,
               "phase margin must be between 45 and 85 degrees");
_Static_assert((HARMONIC_LEARNING_MAX_ORDER & 1U) != 0U
               && HARMONIC_LEARNING_MAX_ORDER >= 3U
               && HARMONIC_LEARNING_MAX_ORDER
                    <= INVERTER_CONTROL_MAX_LEARNING_HARMONIC,
               "learning harmonic limit must be an odd order from 3 to 25");

#define RMS_WINDOW_MS           200U
#define RMS_SAMPLE_MS             1U
#define RMS_SAMPLE_CNT          (RMS_WINDOW_MS / RMS_SAMPLE_MS)
#define OLED_REFRESH_MS         100U

static float s_v_sum_sq;
static uint16_t s_rms_count;
static float s_v_rms;
static uint32_t s_adc_last_ms;
static Sample_VoltageCalibrationTypeDef s_adc_calibration;

static uint32_t s_oled_timer;
static volatile uint8_t s_oled_dirty = 1U;
static uint8_t s_oled_ok;

void SystemClock_Config(void);

static HAL_StatusTypeDef Hbridge_WriteOutput(uint32_t timer_index,
                                             uint32_t output_id,
                                             uint32_t set_source,
                                             uint32_t reset_source)
{
    HRTIM_OutputCfgTypeDef output = {0};

    output.Polarity = HRTIM_OUTPUTPOLARITY_HIGH;
    output.SetSource = set_source;
    output.ResetSource = reset_source;
    output.IdleMode = HRTIM_OUTPUTIDLEMODE_NONE;
    output.IdleLevel = HRTIM_OUTPUTIDLELEVEL_INACTIVE;
    output.FaultLevel = HRTIM_OUTPUTFAULTLEVEL_NONE;
    output.ChopperModeEnable = HRTIM_OUTPUTCHOPPERMODE_DISABLED;
    output.BurstModeEntryDelayed = HRTIM_OUTPUTBURSTMODEENTRY_REGULAR;
    return HAL_HRTIM_WaveformOutputConfig(
        &hhrtim1, timer_index, output_id, &output);
}

static HAL_StatusTypeDef Hbridge_WriteDeadTime(uint32_t timer_index)
{
    HRTIM_DeadTimeCfgTypeDef deadtime = {0};

    deadtime.Prescaler = DT_PRESCALER;
    deadtime.RisingValue = DT_VALUE;
    deadtime.FallingValue = DT_VALUE;
    deadtime.RisingSign = HRTIM_TIMDEADTIME_RISINGSIGN_POSITIVE;
    deadtime.FallingSign = HRTIM_TIMDEADTIME_FALLINGSIGN_POSITIVE;
    deadtime.RisingLock = HRTIM_TIMDEADTIME_RISINGLOCK_WRITE;
    deadtime.RisingSignLock = HRTIM_TIMDEADTIME_RISINGSIGNLOCK_WRITE;
    deadtime.FallingLock = HRTIM_TIMDEADTIME_FALLINGLOCK_WRITE;
    deadtime.FallingSignLock = HRTIM_TIMDEADTIME_FALLINGSIGNLOCK_WRITE;
    return HAL_HRTIM_DeadTimeConfig(&hhrtim1, timer_index, &deadtime);
}

static HAL_StatusTypeDef Hbridge_ConfigTimer(uint32_t timer_index,
                                             uint32_t output_1,
                                             uint32_t output_2)
{
    HRTIM_TimeBaseCfgTypeDef timebase = {0};
    HRTIM_TimerCtlTypeDef control = {0};
    HRTIM_TimerCfgTypeDef config = {0};
    HRTIM_CompareCfgTypeDef compare = {0};

    timebase.Period = PWM_PERIOD;
    timebase.RepetitionCounter = 0U;
    timebase.PrescalerRatio = HRTIM_PRESCALER;
    timebase.Mode = HRTIM_MODE_CONTINUOUS;
    if (HAL_HRTIM_TimeBaseConfig(&hhrtim1, timer_index, &timebase) != HAL_OK) {
        return HAL_ERROR;
    }

    control.UpDownMode = HRTIM_TIMERUPDOWNMODE_UP;
    control.GreaterCMP1 = HRTIM_TIMERGTCMP1_EQUAL;
    control.DualChannelDacEnable = HRTIM_TIMER_DCDE_DISABLED;
    if (HAL_HRTIM_WaveformTimerControl(
            &hhrtim1, timer_index, &control) != HAL_OK) {
        return HAL_ERROR;
    }

    config.InterruptRequests = HRTIM_TIM_IT_NONE;
    config.DMARequests = HRTIM_TIM_DMA_NONE;
    config.PreloadEnable = HRTIM_PRELOAD_ENABLED;
    config.RepetitionUpdate = HRTIM_UPDATEONREPETITION_ENABLED;
    config.PushPull = HRTIM_TIMPUSHPULLMODE_DISABLED;
    config.FaultEnable = HRTIM_TIMFAULTENABLE_NONE;
    config.FaultLock = HRTIM_TIMFAULTLOCK_READWRITE;
    config.DeadTimeInsertion = HRTIM_TIMDEADTIMEINSERTION_ENABLED;
    config.DelayedProtectionMode =
        HRTIM_TIMER_A_B_C_DELAYEDPROTECTION_DISABLED;
    config.UpdateTrigger = HRTIM_TIMUPDATETRIGGER_NONE;
    config.ResetTrigger = HRTIM_TIMRESETTRIGGER_MASTER_PER;
    config.ResetUpdate = HRTIM_TIMUPDATEONRESET_DISABLED;
    if (HAL_HRTIM_WaveformTimerConfig(
            &hhrtim1, timer_index, &config) != HAL_OK) {
        return HAL_ERROR;
    }

    compare.CompareValue = PWM_COUNTS / 2U;
    if (HAL_HRTIM_WaveformCompareConfig(
            &hhrtim1, timer_index,
            HRTIM_COMPAREUNIT_1, &compare) != HAL_OK) {
        return HAL_ERROR;
    }

    if (Hbridge_WriteDeadTime(timer_index) != HAL_OK) {
        return HAL_ERROR;
    }
    if (Hbridge_WriteOutput(
            timer_index, output_1,
            HRTIM_OUTPUTSET_TIMPER,
            HRTIM_OUTPUTRESET_TIMCMP1) != HAL_OK) {
        return HAL_ERROR;
    }
    if (Hbridge_WriteOutput(
            timer_index, output_2,
            HRTIM_OUTPUTSET_NONE,
            HRTIM_OUTPUTRESET_NONE) != HAL_OK) {
        return HAL_ERROR;
    }
    return HAL_OK;
}

static HAL_StatusTypeDef Hbridge_ConfigMaster(void)
{
    HRTIM_TimeBaseCfgTypeDef timebase = {0};

    timebase.Period = MASTER_PERIOD;
    timebase.RepetitionCounter = 1U;
    timebase.PrescalerRatio = HRTIM_PRESCALER;
    timebase.Mode = HRTIM_MODE_CONTINUOUS;
    if (HAL_HRTIM_TimeBaseConfig(
            &hhrtim1, HRTIM_TIMERINDEX_MASTER, &timebase) != HAL_OK) {
        return HAL_ERROR;
    }

    return HAL_HRTIM_WaveformCounterStart(
        &hhrtim1, HRTIM_TIMERID_MASTER);
}

static HAL_StatusTypeDef Hbridge_ConfigAdcTrigger(uint16_t trigger_count)
{
    HRTIM_CompareCfgTypeDef compare = {0};
    HRTIM_ADCTriggerCfgTypeDef trigger = {0};

    compare.CompareValue = trigger_count;
    /*
     * Timer A CMP3 shares Timer A's preload/update event with the PWM compare.
     * The ADC trigger therefore moves atomically with the duty command and
     * cannot use a quiet window calculated for a different PWM period.
     */
    if (HAL_HRTIM_WaveformCompareConfig(
            &hhrtim1, HRTIM_IDX_A,
            HRTIM_COMPAREUNIT_3, &compare) != HAL_OK) {
        return HAL_ERROR;
    }

    trigger.UpdateSource = HRTIM_ADCTRIGGERUPDATE_TIMER_A;
    trigger.Trigger = HRTIM_ADCTRIGGEREVENT13_TIMERA_CMP3;
    return HAL_HRTIM_ADCTriggerConfig(
        &hhrtim1, HRTIM_ADCTRIGGER_1, &trigger);
}

static HAL_StatusTypeDef Hbridge_StartTiming(void)
{
    const uint16_t half = PWM_COUNTS / 2U;

    CMP_WR(HRTIM_IDX_A, half);
    CMP_WR(HRTIM_IDX_B, half);

    if (HAL_HRTIM_WaveformCounterStart(
            &hhrtim1, HRTIM_TIMERID_TIMER_A) != HAL_OK
        || HAL_HRTIM_WaveformCounterStart(
            &hhrtim1, HRTIM_TIMERID_TIMER_B) != HAL_OK) {
        return HAL_ERROR;
    }
    return HAL_OK;
}

static HAL_StatusTypeDef Hbridge_EnablePowerStage(void)
{
    if (HAL_HRTIM_WaveformOutputStart(
            &hhrtim1, HBRIDGE_OUTPUTS) != HAL_OK) {
        return HAL_ERROR;
    }
    return HAL_OK;
}

static void Hbridge_StopPowerStage(void)
{
    if (hhrtim1.Instance != HRTIM1) {
        return;
    }

    __HAL_HRTIM_MASTER_DISABLE_IT(&hhrtim1, HRTIM_MASTER_IT_MREP);
    HAL_NVIC_DisableIRQ(HRTIM1_Master_IRQn);
    (void)HAL_HRTIM_WaveformOutputStop(&hhrtim1, HBRIDGE_OUTPUTS);
    (void)HAL_HRTIM_WaveformCounterStop(
        &hhrtim1, HRTIM_TIMERID_TIMER_A);
    (void)HAL_HRTIM_WaveformCounterStop(
        &hhrtim1, HRTIM_TIMERID_TIMER_B);
}

static HAL_StatusTypeDef Hbridge_InitController(void)
{
    InverterControl_InitTypeDef config = {
        .dc_bus_voltage = VDC_BUS,
        .inverter_inductance_h = L1_VAL,
        .load_inductance_h = L2_VAL,
        .filter_capacitance_f = C_VAL,
        .nominal_load_ohm = NOMINAL_LOAD_OHM,
        .output_frequency_hz = F_OUT,
        .switching_frequency_hz = F_SW,
        .control_frequency_hz = CONTROL_FREQ,
        .design_crossover_hz = CONTROL_CROSSOVER_HZ,
        .requested_phase_margin_deg = CONTROL_PHASE_MARGIN_DEG,
        .minimum_gain_margin_db = CONTROL_GAIN_MARGIN_DB,
        .feedback_lpf_hz = CONTROL_ERROR_LPF_HZ,
        .fundamental_pr_bandwidth_hz = FUNDAMENTAL_PR_BW_HZ,
        .harmonic_pr_bandwidth_hz = HARMONIC_PR_BW_HZ,
        .voltage_rms_min = VREF_RMS_MIN,
        .voltage_rms_max = VREF_RMS_MAX,
        .voltage_rms_start = VREF_RMS_START,
        .voltage_ramp_v_per_s = VREF_RAMP_V_PER_S,
        .output_overvoltage_peak_v = OUTPUT_OVERVOLTAGE_PEAK_V,
        .deadtime_seconds = (float)DT_VALUE / DT_CLOCK_HZ,
        .deadtime_compensation_gain = DEADTIME_COMP_GAIN,
        .zero_cross_soft_angle_deg = DEADTIME_SOFT_ANGLE_DEG,
        .deadtime_phase_lead_deg = DEADTIME_PHASE_LEAD_DEG,
        .harmonic_learning_rate = HARMONIC_LEARNING_RATE,
        .harmonic_learning_total_limit = HARMONIC_LEARNING_LIMIT,
        .pwm_period_counts = PWM_COUNTS,
        .harmonic_learning_enable = HARMONIC_LEARNING_ENABLE,
        .harmonic_learning_max_order = HARMONIC_LEARNING_MAX_ORDER,
    };

    if (InverterControl_Init(&config) != INVERTER_CONTROL_OK) {
        return HAL_ERROR;
    }
    InverterControl_SetTargetRms(VREF_RMS_INIT);
    return HAL_OK;
}

static void Encoder_PrepareHardware(void)
{
    GPIO_InitTypeDef config = {0};

    config.Pin = Encoder_A_Pin | Encoder_B_Pin;
    config.Mode = GPIO_MODE_AF_PP;
    config.Pull = GPIO_PULLUP;
    config.Speed = GPIO_SPEED_FREQ_LOW;
    config.Alternate = GPIO_AF2_TIM3;
    HAL_GPIO_Init(Encoder_A_GPIO_Port, &config);

    TIM3->CCMR1 = (TIM3->CCMR1 & ~(TIM_CCMR1_IC1F | TIM_CCMR1_IC2F))
                | (15U << TIM_CCMR1_IC1F_Pos)
                | (15U << TIM_CCMR1_IC2F_Pos);
}

static void Hbridge_EnableControlInterrupt(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0U;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    __HAL_HRTIM_MASTER_CLEAR_FLAG(&hhrtim1, HRTIM_MASTER_FLAG_MREP);
    __HAL_HRTIM_MASTER_ENABLE_IT(&hhrtim1, HRTIM_MASTER_IT_MREP);
    HAL_NVIC_SetPriority(HRTIM1_Master_IRQn, 4U, 0U);
    HAL_NVIC_EnableIRQ(HRTIM1_Master_IRQn);
}

void App_HRTIMControlIRQ(void)
{
    Sample_ControlFrameTypeDef frame = {0};
    InverterControl_OutputTypeDef output;
    uint32_t start_cycles = DWT->CYCCNT;
    uint32_t elapsed_cycles;
    const uint16_t half = PWM_COUNTS / 2U;

    if (Sample_GetControlFrame(&frame) != SAMPLE_OK) {
        InverterControl_Trip(INVERTER_FAULT_ADC);
    }
    InverterControl_Step(&frame, &output);

    CMP_WR(HRTIM_IDX_A, output.compare_a);
    CMP_WR(HRTIM_IDX_B, output.compare_b);
    ADC_CMP_WR(output.adc_trigger);

    elapsed_cycles = DWT->CYCCNT - start_cycles;
    InverterControl_ReportExecutionCycles(
        elapsed_cycles, CONTROL_CYCLE_BUDGET);

    if (InverterControl_GetFault() != INVERTER_FAULT_NONE) {
        HRTIM1->sCommonRegs.ODISR |= HBRIDGE_OUTPUTS;
        CMP_WR(HRTIM_IDX_A, half);
        CMP_WR(HRTIM_IDX_B, half);
        LED_SetLED2State(1U);
        s_oled_dirty = 1U;
    }

    {
        static uint16_t heartbeat_count;
        heartbeat_count++;
        if (heartbeat_count >= 5000U) {
            heartbeat_count = 0U;
            HAL_GPIO_TogglePin(LED3_GPIO_Port, LED3_Pin);
        }
    }
}

static void ADC_RMS_Reset(void)
{
    s_v_sum_sq = 0.0f;
    s_rms_count = 0U;
}

static void ADC_RMS_Task(void)
{
    uint32_t now = HAL_GetTick();

    if ((now - s_adc_last_ms) < RMS_SAMPLE_MS) {
        return;
    }
    s_adc_last_ms = now;

    {
        float voltage = Sample_GetLatestVoltage();
        s_v_sum_sq += voltage * voltage;
        s_rms_count++;
    }

    if (s_rms_count >= RMS_SAMPLE_CNT) {
        s_v_rms = sqrtf(s_v_sum_sq / (float)s_rms_count);
        ADC_RMS_Reset();
        s_oled_dirty = 1U;
    }
}

static void OLED_Redraw(void)
{
    float target = InverterControl_GetTargetRms();
    float maximum = InverterControl_GetMaxCommandRms();
    float ratio;
    uint8_t bar_length;
    uint8_t i;
    char bar[7];

    OLEDGFX_Clear();
    if (InverterControl_GetFault() == INVERTER_FAULT_NONE) {
        OLEDGFX_ShowString(
            OLEDGFX_COL_1, OLEDGFX_LINE_1,
            "H-BRIDGE INV", OLEDGFX_Clip);
    } else {
        OLEDGFX_ShowString(
            OLEDGFX_COL_1, OLEDGFX_LINE_1,
            "FAULT - RESET", OLEDGFX_Clip);
    }

    OLEDGFX_ShowString(
        OLEDGFX_COL_1, OLEDGFX_LINE_2, "Vo=", OLEDGFX_Clip);
    OLEDGFX_ShowFloat(
        OLEDGFX_COL_4, OLEDGFX_LINE_2, s_v_rms, 2U, 1U);
    OLEDGFX_ShowChar(OLEDGFX_COL_9, OLEDGFX_LINE_2, 'V');

    OLEDGFX_ShowString(
        OLEDGFX_COL_1, OLEDGFX_LINE_3, "ADC Z=", OLEDGFX_Clip);
    OLEDGFX_ShowNum(
        OLEDGFX_COL_7, OLEDGFX_LINE_3,
        (uint32_t)(s_adc_calibration.zero_raw + 0.5f), 4U);
    OLEDGFX_ShowString(
        OLEDGFX_COL_12, OLEDGFX_LINE_3,
        s_adc_calibration.valid != 0U ? "OK" : "ERR",
        OLEDGFX_Clip);

    OLEDGFX_ShowString(
        OLEDGFX_COL_1, OLEDGFX_LINE_4, "Set", OLEDGFX_Clip);
    OLEDGFX_ShowFloat(
        OLEDGFX_COL_5, OLEDGFX_LINE_4, target, 2U, 1U);
    OLEDGFX_ShowChar(OLEDGFX_COL_10, OLEDGFX_LINE_4, 'V');

    ratio = maximum > VREF_RMS_MIN
        ? (target - VREF_RMS_MIN) / (maximum - VREF_RMS_MIN)
        : 1.0f;
    if (ratio < 0.0f) {
        ratio = 0.0f;
    } else if (ratio > 1.0f) {
        ratio = 1.0f;
    }

    bar_length = (uint8_t)(ratio * 4.0f + 0.5f);
    if (bar_length > 4U) {
        bar_length = 4U;
    }
    bar[0] = '[';
    bar[5] = ']';
    bar[6] = '\0';
    for (i = 0U; i < 4U; i++) {
        bar[i + 1U] = i < bar_length ? '#' : ' ';
    }
    OLEDGFX_ShowString(
        OLEDGFX_COL_11, OLEDGFX_LINE_4, bar, OLEDGFX_Clip);
    OLEDGFX_Submit();
}

int main(void)
{
    HAL_Init();
    SystemClock_Config();

    MX_GPIO_Init();
    MX_DMA_Init();
    MX_ADC1_Init();
    MX_HRTIM1_Init();
    MX_I2C1_Init();
    MX_TIM3_Init();
    MX_USART1_UART_Init();

    if (Hbridge_ConfigMaster() != HAL_OK
        || Hbridge_ConfigTimer(
            HRTIM_IDX_A, HRTIM_OUTPUT_TA1, HRTIM_OUTPUT_TA2) != HAL_OK
        || Hbridge_ConfigTimer(
            HRTIM_IDX_B, HRTIM_OUTPUT_TB1, HRTIM_OUTPUT_TB2) != HAL_OK
        || Hbridge_InitController() != HAL_OK) {
        Error_Handler();
    }

    Encoder_PrepareHardware();
    Encoder_Init();
    LED_SetLED1State(1U);
    LED_SetLED2State(0U);
    LED_SetLED3State(0U);
    LED_SetLED4State(0U);

    if (Hbridge_ConfigAdcTrigger(
            InverterControl_GetInitialAdcTrigger()) != HAL_OK
        || Hbridge_StartTiming() != HAL_OK
        || Sample_Init() != SAMPLE_OK) {
        LED_SetLED2State(1U);
        Error_Handler();
    }
    HAL_Delay(2U);
    if (Sample_CalibrateZero(&s_adc_calibration) != SAMPLE_OK) {
        InverterControl_Trip(INVERTER_FAULT_ADC_CALIBRATION);
        LED_SetLED2State(1U);
        Error_Handler();
    }

    ADC_RMS_Reset();
    s_adc_last_ms = HAL_GetTick();

    if (OLEDGFX_Init() == OLEDGFX_OK) {
        s_oled_ok = 1U;
        OLED_Redraw();
        if (OLEDGFX_Update() == OLEDGFX_ERROR) {
            s_oled_ok = 0U;
            LED_SetLED4State(1U);
        }
    } else {
        LED_SetLED4State(1U);
    }

    if (Hbridge_EnablePowerStage() != HAL_OK) {
        Error_Handler();
    }
    Hbridge_EnableControlInterrupt();
    s_oled_timer = HAL_GetTick();

    while (1) {
        int16_t encoder_delta = Encoder_PopCount();

        if (encoder_delta != 0) {
            InverterControl_SetTargetRms(
                InverterControl_GetTargetRms()
                + (float)encoder_delta * VREF_ENC_STEP);
            s_oled_dirty = 1U;
        }

        ADC_RMS_Task();

        if (s_oled_ok != 0U) {
            uint32_t now = HAL_GetTick();
            if (s_oled_dirty != 0U
                && (now - s_oled_timer) >= OLED_REFRESH_MS) {
                s_oled_timer = now;
                s_oled_dirty = 0U;
                OLED_Redraw();
            }
            if (OLEDGFX_Update() == OLEDGFX_ERROR) {
                s_oled_ok = 0U;
                LED_SetLED4State(1U);
            }
        }
    }
}

void SystemClock_Config(void)
{
    RCC_OscInitTypeDef oscillator = {0};
    RCC_ClkInitTypeDef clocks = {0};

    HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

    oscillator.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    oscillator.HSEState = RCC_HSE_ON;
    oscillator.PLL.PLLState = RCC_PLL_ON;
    oscillator.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    oscillator.PLL.PLLM = RCC_PLLM_DIV2;
    oscillator.PLL.PLLN = 85;
    oscillator.PLL.PLLP = RCC_PLLP_DIV2;
    oscillator.PLL.PLLQ = RCC_PLLQ_DIV2;
    oscillator.PLL.PLLR = RCC_PLLR_DIV2;
    if (HAL_RCC_OscConfig(&oscillator) != HAL_OK) {
        Error_Handler();
    }

    clocks.ClockType = RCC_CLOCKTYPE_HCLK
                     | RCC_CLOCKTYPE_SYSCLK
                     | RCC_CLOCKTYPE_PCLK1
                     | RCC_CLOCKTYPE_PCLK2;
    clocks.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    clocks.AHBCLKDivider = RCC_SYSCLK_DIV1;
    clocks.APB1CLKDivider = RCC_HCLK_DIV1;
    clocks.APB2CLKDivider = RCC_HCLK_DIV2;
    if (HAL_RCC_ClockConfig(&clocks, FLASH_LATENCY_4) != HAL_OK) {
        Error_Handler();
    }
}

void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c->Instance == I2C1) {
        OLEDGFX_TxCpltCallback();
    }
}

void HAL_I2C_MemTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c->Instance == I2C1) {
        OLEDGFX_TxCpltCallback();
    }
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c->Instance == I2C1) {
        OLEDGFX_ErrorCallback();
    }
}

void Error_Handler(void)
{
    Hbridge_StopPowerStage();
    __disable_irq();
    while (1) {
    }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
    (void)file;
    (void)line;
}
#endif
