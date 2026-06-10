#ifndef __PWM_H
#define __PWM_H

#include <stdint.h>
#include "main.h"
#include "hrtim.h"

#define PWM_TIMER hhrtim1
#define PWM_MINDUTY 0.05f
#define PWM_MAXDUTY 0.95f
#define PWM_TIMER_PRESCALER HRTIM_PRESCALERRATIO_MUL32
#define PWM_TIMER_PERIOD 27200U
#define PWM_DEFAULT_CMPARE 13600U
#define PWM_DEADTIME_PRESCALER HRTIM_TIMDEADTIME_PRESCALERRATIO_MUL8
#define PWM_DEADTIME_RISING_VALUE 10U
#define PWM_DEADTIME_FALLING_VALUE 10U

/*
	* @param  Timer PWM timer enum
	*                   This parameter can be one of the following values:
	*                   @arg PWM_TIMERA
	*                   @arg PWM_TIMERB
	*                   @arg PWM_TIMERC
	*                   @arg PWM_TIMERD
	*                   @arg PWM_TIMERE
	*                   @arg PWM_TIMERF
	* @note   PWM_Init only configures and starts the requested timer PWM template.
	*         HRTIM common initialization and GPIO alternate function configuration
	*         must still be provided by external board-level code.
*/

typedef enum {
	PWM_TIMERA = 0U,
	PWM_TIMERB = 1U,
	PWM_TIMERC = 2U,
	PWM_TIMERD = 3U,
	PWM_TIMERE = 4U,
	PWM_TIMERF = 5U,
	PWM_TIMER_COUNT
} PWM_TimerIdTypeDef;

extern const uint32_t pwm_hal_timer_ids[PWM_TIMER_COUNT];
extern const uint32_t pwm_hal_timer_indexes[PWM_TIMER_COUNT];
extern const uint32_t pwm_output1s[PWM_TIMER_COUNT];
extern const uint32_t pwm_output2s[PWM_TIMER_COUNT];
extern const uint32_t pwm_delayed_protection_modes[PWM_TIMER_COUNT];

HAL_StatusTypeDef PWM_Init(PWM_TimerIdTypeDef Timer);
HAL_StatusTypeDef PWM_DeInit(uint32_t Timer);

static inline void PWM_Start(PWM_TimerIdTypeDef Timer){
	HAL_HRTIM_WaveformOutputStart(&PWM_TIMER, pwm_output1s[Timer]);
	HAL_HRTIM_WaveformOutputStart(&PWM_TIMER, pwm_output2s[Timer]);
}

static inline void PWM_Stop(PWM_TimerIdTypeDef Timer){
	HAL_HRTIM_WaveformOutputStop(&PWM_TIMER, pwm_output1s[Timer]);
	HAL_HRTIM_WaveformOutputStop(&PWM_TIMER, pwm_output2s[Timer]);
}

static inline void PWM_SetCompareRaw(PWM_TimerIdTypeDef Timer, uint16_t compare){
	HRTIM1->sTimerxRegs[pwm_hal_timer_indexes[Timer]].CMP1xR = compare;
}

static inline void PWM_SetDutyCycle(PWM_TimerIdTypeDef Timer, float dutyCycle){
	if (dutyCycle < PWM_MINDUTY) {
		dutyCycle = PWM_MINDUTY;
	} else if (dutyCycle > PWM_MAXDUTY) {
		dutyCycle = PWM_MAXDUTY;
	}

	PWM_SetCompareRaw(Timer, (uint16_t)(dutyCycle * (__HAL_HRTIM_GETPERIOD(&PWM_TIMER, pwm_hal_timer_indexes[Timer]) + 1)));
}

#endif
