#include "PWM.h"

static uint32_t pwm_initialized_timers = 0U;

const uint32_t pwm_hal_timer_ids[PWM_TIMER_COUNT] = {
	HRTIM_TIMERID_TIMER_A,
	HRTIM_TIMERID_TIMER_B,
	HRTIM_TIMERID_TIMER_C,
	HRTIM_TIMERID_TIMER_D,
	HRTIM_TIMERID_TIMER_E,
	HRTIM_TIMERID_TIMER_F,
};

const uint32_t pwm_hal_timer_indexes[PWM_TIMER_COUNT] = {
	HRTIM_TIMERINDEX_TIMER_A,
	HRTIM_TIMERINDEX_TIMER_B,
	HRTIM_TIMERINDEX_TIMER_C,
	HRTIM_TIMERINDEX_TIMER_D,
	HRTIM_TIMERINDEX_TIMER_E,
	HRTIM_TIMERINDEX_TIMER_F,
};

const uint32_t pwm_output1s[PWM_TIMER_COUNT] = {
	HRTIM_OUTPUT_TA1,
	HRTIM_OUTPUT_TB1,
	HRTIM_OUTPUT_TC1,
	HRTIM_OUTPUT_TD1,
	HRTIM_OUTPUT_TE1,
	HRTIM_OUTPUT_TF1,
};

const uint32_t pwm_output2s[PWM_TIMER_COUNT] = {
	HRTIM_OUTPUT_TA2,
	HRTIM_OUTPUT_TB2,
	HRTIM_OUTPUT_TC2,
	HRTIM_OUTPUT_TD2,
	HRTIM_OUTPUT_TE2,
	HRTIM_OUTPUT_TF2,
};

const uint32_t pwm_delayed_protection_modes[PWM_TIMER_COUNT] = {
	HRTIM_TIMER_A_B_C_DELAYEDPROTECTION_DISABLED,
	HRTIM_TIMER_A_B_C_DELAYEDPROTECTION_DISABLED,
	HRTIM_TIMER_A_B_C_DELAYEDPROTECTION_DISABLED,
	HRTIM_TIMER_D_E_DELAYEDPROTECTION_DISABLED,
	HRTIM_TIMER_D_E_DELAYEDPROTECTION_DISABLED,
	HRTIM_TIMER_F_DELAYEDPROTECTION_DISABLED,
};

static HAL_StatusTypeDef PWM_ConfigureTimer(PWM_TimerIdTypeDef timerId)
{
	HRTIM_TimeBaseCfgTypeDef timeBaseCfg = {0};
	HRTIM_TimerCfgTypeDef timerCfg = {0};
	HRTIM_TimerCtlTypeDef timerCtl = {0};
	HRTIM_CompareCfgTypeDef compareCfg = {0};
	HRTIM_DeadTimeCfgTypeDef deadTimeCfg = {0};
	HRTIM_OutputCfgTypeDef outputCfg = {0};
	HAL_StatusTypeDef status;
	uint32_t halTimerId;
	uint32_t timerIndex;

	halTimerId = pwm_hal_timer_ids[timerId];
	timerIndex = pwm_hal_timer_indexes[timerId];

	if ((pwm_initialized_timers & halTimerId) != 0U) {
		return HAL_OK;
	}

	timeBaseCfg.Period = PWM_TIMER_PERIOD;
	timeBaseCfg.RepetitionCounter = 0;
	timeBaseCfg.PrescalerRatio = PWM_TIMER_PRESCALER;
	timeBaseCfg.Mode = HRTIM_MODE_CONTINUOUS;
	status = HAL_HRTIM_TimeBaseConfig(&hhrtim1, timerIndex, &timeBaseCfg);
	if (status != HAL_OK) {
		return status;
	}

	timerCtl.UpDownMode = HRTIM_TIMERUPDOWNMODE_UPDOWN;
	timerCtl.GreaterCMP1 = HRTIM_TIMERGTCMP1_EQUAL;
	timerCtl.DualChannelDacEnable = HRTIM_TIMER_DCDE_DISABLED;
	status = HAL_HRTIM_WaveformTimerControl(&hhrtim1, timerIndex, &timerCtl);
	if (status != HAL_OK) {
		return status;
	}

	status = HAL_HRTIM_RollOverModeConfig(
		&hhrtim1,
		timerIndex,
		HRTIM_TIM_FEROM_BOTH | HRTIM_TIM_BMROM_BOTH |
		HRTIM_TIM_ADROM_BOTH | HRTIM_TIM_OUTROM_BOTH |
		HRTIM_TIM_ROM_BOTH);
	if (status != HAL_OK) {
		return status;
	}

	timerCfg.InterruptRequests = HRTIM_TIM_IT_NONE;
	timerCfg.DMARequests = HRTIM_TIM_DMA_NONE;
	timerCfg.PreloadEnable = HRTIM_PRELOAD_ENABLED;
	timerCfg.RepetitionUpdate = HRTIM_UPDATEONREPETITION_ENABLED;
	timerCfg.PushPull = HRTIM_TIMPUSHPULLMODE_DISABLED;
	timerCfg.FaultEnable = HRTIM_TIMFAULTENABLE_NONE;
	timerCfg.FaultLock = HRTIM_TIMFAULTLOCK_READWRITE;
	timerCfg.DeadTimeInsertion = HRTIM_TIMDEADTIMEINSERTION_ENABLED;
	timerCfg.DelayedProtectionMode = pwm_delayed_protection_modes[timerId];
	timerCfg.UpdateTrigger = HRTIM_TIMUPDATETRIGGER_NONE;
	timerCfg.ResetTrigger = HRTIM_TIMRESETTRIGGER_MASTER_PER;
	timerCfg.ResetUpdate = HRTIM_TIMUPDATEONRESET_DISABLED;
	status = HAL_HRTIM_WaveformTimerConfig(&hhrtim1, timerIndex, &timerCfg);
	if (status != HAL_OK) {
		return status;
	}

	compareCfg.CompareValue = PWM_DEFAULT_CMPARE;
	status = HAL_HRTIM_WaveformCompareConfig(&hhrtim1, timerIndex, HRTIM_COMPAREUNIT_1, &compareCfg);
	if (status != HAL_OK) {
		return status;
	}

	deadTimeCfg.Prescaler = PWM_DEADTIME_PRESCALER;
	deadTimeCfg.RisingValue = PWM_DEADTIME_RISING_VALUE;
	deadTimeCfg.RisingSign = HRTIM_TIMDEADTIME_RISINGSIGN_POSITIVE;
	deadTimeCfg.RisingLock = HRTIM_TIMDEADTIME_RISINGLOCK_WRITE;
	deadTimeCfg.RisingSignLock = HRTIM_TIMDEADTIME_RISINGSIGNLOCK_WRITE;
	deadTimeCfg.FallingValue = PWM_DEADTIME_FALLING_VALUE;
	deadTimeCfg.FallingSign = HRTIM_TIMDEADTIME_FALLINGSIGN_POSITIVE;
	deadTimeCfg.FallingLock = HRTIM_TIMDEADTIME_FALLINGLOCK_WRITE;
	deadTimeCfg.FallingSignLock = HRTIM_TIMDEADTIME_FALLINGSIGNLOCK_WRITE;
	status = HAL_HRTIM_DeadTimeConfig(&hhrtim1, timerIndex, &deadTimeCfg);
	if (status != HAL_OK) {
		return status;
	}

	outputCfg.Polarity = HRTIM_OUTPUTPOLARITY_HIGH;
	outputCfg.SetSource = HRTIM_OUTPUTSET_NONE;
	outputCfg.ResetSource = HRTIM_OUTPUTRESET_TIMCMP1;
	outputCfg.IdleMode = HRTIM_OUTPUTIDLEMODE_NONE;
	outputCfg.IdleLevel = HRTIM_OUTPUTIDLELEVEL_INACTIVE;
	outputCfg.FaultLevel = HRTIM_OUTPUTFAULTLEVEL_NONE;
	outputCfg.ChopperModeEnable = HRTIM_OUTPUTCHOPPERMODE_DISABLED;
	outputCfg.BurstModeEntryDelayed = HRTIM_OUTPUTBURSTMODEENTRY_REGULAR;
	status = HAL_HRTIM_WaveformOutputConfig(&hhrtim1, timerIndex, pwm_output1s[timerId], &outputCfg);
	if (status != HAL_OK) {
		return status;
	}

	outputCfg.SetSource = HRTIM_OUTPUTSET_NONE;
	outputCfg.ResetSource = HRTIM_OUTPUTRESET_NONE;
	status = HAL_HRTIM_WaveformOutputConfig(&hhrtim1, timerIndex, pwm_output2s[timerId], &outputCfg);
	if (status != HAL_OK) {
		return status;
	}

	pwm_initialized_timers |= halTimerId;
	return HAL_OK;
}

HAL_StatusTypeDef PWM_Init(PWM_TimerIdTypeDef Timer){
	HAL_StatusTypeDef status;
	uint32_t halTimerId;

	if (Timer >= PWM_TIMER_COUNT) {
		return HAL_ERROR;
	}

	status = PWM_ConfigureTimer(Timer);
	if (status != HAL_OK) {
		return status;
	}

	halTimerId = pwm_hal_timer_ids[Timer];
	return HAL_HRTIM_WaveformCounterStart(&PWM_TIMER, halTimerId);
}

HAL_StatusTypeDef PWM_DeInit(uint32_t Timer){
	return HAL_HRTIM_WaveformCounterStop(&PWM_TIMER, Timer);
}
