/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : H-Bridge Open-Loop SPWM Inverter — STM32G474VE
  *                   Timer A (PA8/PA9) + Timer B (PA10/PA11) → H 桥
  *                   f_sw = 100.000 kHz, f_out = 50 Hz, 双极性 SPWM
  *                   编码器旋钮实时调节调制指数 → 控制输出交流有效值
  *                   烧录即用, PA8-11 输出 H 桥 PWM。
  ******************************************************************************
  */
/* USER CODE END Header */

#include "main.h"
#include "adc.h"
#include "dma.h"
#include "hrtim.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */
#include "project_libs.h"
#include <math.h>
/* USER CODE END Includes */

/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* USER CODE BEGIN PD */

/* ==========================================================================
 *  Clock — SYSCLK = 170 MHz  (HSI 16M → /4, ×85, /2)
 *  HRTIM  = 170 MHz          (default CPUCLK)
 *  APB1   = 170 MHz          (HCLK / 1)
 * ========================================================================== */

/* ——— HRTIM — 100.000 kHz (exact) ——————————————————————————————
 *  DIV1 → f_cnt = 170 MHz
 *  PERIOD = 849 → f_pwm = 170M/(2×850) = 100 000 Hz
 *  Resolution = 850 steps (9.73 bits)
 *
 *  CRITICAL: OUTPUT1 SetSource MUST be TIMPER (period match SET),
 *  not NONE.  MX_HRTIM1_Init() incorrectly set TA1 SetSource=NONE
 *  which means the output can never assert → stuck LOW.
 *  此处覆盖纠正。
 * ————————————————————————————————————————————————————————————— */
#define HRTIM_PRESCALER    HRTIM_PRESCALERRATIO_DIV1
#define MASTER_PERIOD      849U
#define PWM_PERIOD         MASTER_PERIOD
#define PWM_PERIOD_P1_F    ((float)(MASTER_PERIOD + 1U))

/* ——— Dead-time ————————————————————————————————————————————————
 *  MUL8 → unit = 8 × 5.882ns = 47.06 ns
 *  Value 8 → 376.5 ns (safe for Si MOSFET)
 * ————————————————————————————————————————————————————————————— */
#define DT_PRESCALER       HRTIM_TIMDEADTIME_PRESCALERRATIO_MUL8
#define DT_VALUE           8U

/* ——— Output waveform base ———————————————————————————————————— */
#define INV_FOUT_HZ        50.0f
#define INV_TWOPI          ((float)(2.0 * 3.14159265358979323846))

/* ——— TIM6 — 50 kHz duty-update ISR ———————————————————————————
 *  PSC=169 → 1 MHz counter   ARR=19 → 1M/20 = 50 kHz
 *  1000 pts per 50-Hz cycle
 * ————————————————————————————————————————————————————————————— */
#define TIM6_PSC           169U
#define TIM6_ARR           19U

/* ——— Sine lookup table ————————————————————————————————————————
 *  257 entries, linear interpolation in ISR
 *  Phase step (table space) = 256 × 50 / 50 000 = 0.256
 * ————————————————————————————————————————————————————————————— */
#define SIN_TABLE_SIZE     256U
#define SIN_TABLE_STEP     ((float)((float)SIN_TABLE_SIZE * INV_FOUT_HZ / 50000.0f))

/* ——— Modulation index (encoder-controlled) ————————————————————
 *  旋钮每格变化量 = ENC_STEP
 *  ENCODER_COUNT_SCALE=4, POLARITY=-1 → 每 detent ≈ -1 PopCount
 *  ENC_STEP=0.010 → 90 格覆盖 0.05→0.90
 * ————————————————————————————————————————————————————————————— */
#define ENC_STEP           0.010f
#define M_INIT             0.50f          /* 上电默认 50 %     */
#define M_MIN              0.05f
#define M_MAX              0.90f

/* ——— Duty clamp ——————————————————————————————————————————————— */
#define DUTY_MIN           0.05f
#define DUTY_MAX           0.95f

/* ——— DC bus voltage (for Vrms display on OLED) ———————————————— */
#define VDC_NOMINAL        48.0f          /* 按实际母线电压修改 */

/* ——— CMP1 direct write ———————————————————————————————————————— */
#define HRTIM_IDX_A        HRTIM_TIMERINDEX_TIMER_A
#define HRTIM_IDX_B        HRTIM_TIMERINDEX_TIMER_B
#define CMP_WR(idx, val)   (HRTIM1->sTimerxRegs[(idx)].CMP1xR = (uint16_t)(val))

/* USER CODE END PD */

/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* USER CODE BEGIN PV */

/* Sine table — 257 floats (1028 bytes RAM), computed once at init */
static float sin_tab[SIN_TABLE_SIZE + 1U];

/* Phase in table space [0, SIN_TABLE_SIZE) — ISR write, main-loop read */
static volatile float s_phase_tab = 0.0f;

/* Modulation index — main-loop write, ISR read (atomic float on M4F) */
static volatile float s_mod_index = M_INIT;

/* OLED / encoder state */
static uint32_t s_oled_timer   = 0;
static uint8_t  s_oled_dirty   = 1;      /* force 1st redraw */

/* USER CODE END PV */

void SystemClock_Config(void);

/* USER CODE BEGIN 0 */

/* ==========================================================================
 *  Helper — write output config for one HRTIM output
 * ========================================================================== */
static HAL_StatusTypeDef Hbridge_ConfigOutput(uint32_t timerIdx,
                                              uint32_t outputId,
                                              uint32_t setSource,
                                              uint32_t resetSource)
{
	HRTIM_OutputCfgTypeDef out = {0};

	out.Polarity              = HRTIM_OUTPUTPOLARITY_HIGH;
	out.SetSource             = setSource;
	out.ResetSource           = resetSource;
	out.IdleMode              = HRTIM_OUTPUTIDLEMODE_NONE;
	out.IdleLevel             = HRTIM_OUTPUTIDLELEVEL_INACTIVE;
	out.FaultLevel            = HRTIM_OUTPUTFAULTLEVEL_NONE;
	out.ChopperModeEnable     = HRTIM_OUTPUTCHOPPERMODE_DISABLED;
	out.BurstModeEntryDelayed = HRTIM_OUTPUTBURSTMODEENTRY_REGULAR;

	return HAL_HRTIM_WaveformOutputConfig(&hhrtim1, timerIdx, outputId, &out);
}

/* ==========================================================================
 *  Helper — write deadtime config for one HRTIM timer
 * ========================================================================== */
static HAL_StatusTypeDef Hbridge_ConfigDeadtime(uint32_t timerIdx)
{
	HRTIM_DeadTimeCfgTypeDef dt = {0};

	dt.Prescaler       = DT_PRESCALER;
	dt.RisingValue     = DT_VALUE;
	dt.RisingSign      = HRTIM_TIMDEADTIME_RISINGSIGN_POSITIVE;
	dt.RisingLock      = HRTIM_TIMDEADTIME_RISINGLOCK_WRITE;
	dt.RisingSignLock  = HRTIM_TIMDEADTIME_RISINGSIGNLOCK_WRITE;
	dt.FallingValue    = DT_VALUE;
	dt.FallingSign     = HRTIM_TIMDEADTIME_FALLINGSIGN_POSITIVE;
	dt.FallingLock     = HRTIM_TIMDEADTIME_FALLINGLOCK_WRITE;
	dt.FallingSignLock = HRTIM_TIMDEADTIME_FALLINGSIGNLOCK_WRITE;

	return HAL_HRTIM_DeadTimeConfig(&hhrtim1, timerIdx, &dt);
}

/* ==========================================================================
 *  Hbridge_ConfigTimerB
 *  MX_HRTIM1_Init() 对 Timer B 生成了 UP-only / 无死区 / 无 MASTER
 *  复位的配置, 因此需要全部重写（跟 Timer A 一致）。
 * ========================================================================== */
static HAL_StatusTypeDef Hbridge_ConfigTimerB(void)
{
	HRTIM_TimeBaseCfgTypeDef  tb  = {0};
	HRTIM_TimerCtlTypeDef     ctl = {0};
	HRTIM_TimerCfgTypeDef     cfg = {0};
	HRTIM_CompareCfgTypeDef   cmp = {0};
	HAL_StatusTypeDef         s;

	/* Timebase */
	tb.Period = PWM_PERIOD;  tb.RepetitionCounter = 0;
	tb.PrescalerRatio = HRTIM_PRESCALER;  tb.Mode = HRTIM_MODE_CONTINUOUS;
	s = HAL_HRTIM_TimeBaseConfig(&hhrtim1, HRTIM_IDX_B, &tb);
	if (s != HAL_OK) return s;

	/* Up-down counting */
	ctl.UpDownMode = HRTIM_TIMERUPDOWNMODE_UPDOWN;
	ctl.GreaterCMP1 = HRTIM_TIMERGTCMP1_EQUAL;
	ctl.DualChannelDacEnable = HRTIM_TIMER_DCDE_DISABLED;
	s = HAL_HRTIM_WaveformTimerControl(&hhrtim1, HRTIM_IDX_B, &ctl);
	if (s != HAL_OK) return s;

	/* Roll-over */
	s = HAL_HRTIM_RollOverModeConfig(&hhrtim1, HRTIM_IDX_B,
		HRTIM_TIM_FEROM_BOTH  | HRTIM_TIM_BMROM_BOTH  |
		HRTIM_TIM_ADROM_BOTH  | HRTIM_TIM_OUTROM_BOTH |
		HRTIM_TIM_ROM_BOTH);
	if (s != HAL_OK) return s;

	/* Main config: preload + deadtime + reset from MASTER */
	cfg.InterruptRequests     = HRTIM_TIM_IT_NONE;
	cfg.DMARequests           = HRTIM_TIM_DMA_NONE;
	cfg.PreloadEnable         = HRTIM_PRELOAD_ENABLED;
	cfg.RepetitionUpdate      = HRTIM_UPDATEONREPETITION_ENABLED;
	cfg.PushPull              = HRTIM_TIMPUSHPULLMODE_DISABLED;
	cfg.FaultEnable           = HRTIM_TIMFAULTENABLE_NONE;
	cfg.FaultLock             = HRTIM_TIMFAULTLOCK_READWRITE;
	cfg.DeadTimeInsertion     = HRTIM_TIMDEADTIMEINSERTION_ENABLED;
	cfg.DelayedProtectionMode = HRTIM_TIMER_A_B_C_DELAYEDPROTECTION_DISABLED;
	cfg.UpdateTrigger         = HRTIM_TIMUPDATETRIGGER_NONE;
	cfg.ResetTrigger          = HRTIM_TIMRESETTRIGGER_MASTER_PER;
	cfg.ResetUpdate           = HRTIM_TIMUPDATEONRESET_DISABLED;
	s = HAL_HRTIM_WaveformTimerConfig(&hhrtim1, HRTIM_IDX_B, &cfg);
	if (s != HAL_OK) return s;

	/* CMP1 = 50 % */
	cmp.CompareValue = (PWM_PERIOD + 1U) / 2U;
	s = HAL_HRTIM_WaveformCompareConfig(&hhrtim1, HRTIM_IDX_B,
		HRTIM_COMPAREUNIT_1, &cmp);
	if (s != HAL_OK) return s;

	/* Deadtime */
	s = Hbridge_ConfigDeadtime(HRTIM_IDX_B);
	if (s != HAL_OK) return s;

	/* OUTPUT1 (TB1 @ PA10): SET on period → HIGH; RESET on CMP1 → LOW */
	s = Hbridge_ConfigOutput(HRTIM_IDX_B, HRTIM_OUTPUT_TB1,
		HRTIM_OUTPUTSET_TIMPER, HRTIM_OUTPUTRESET_TIMCMP1);
	if (s != HAL_OK) return s;

	/* OUTPUT2 (TB2 @ PA11): complementary via deadtime (no explicit set/reset) */
	s = Hbridge_ConfigOutput(HRTIM_IDX_B, HRTIM_OUTPUT_TB2,
		HRTIM_OUTPUTSET_NONE, HRTIM_OUTPUTRESET_NONE);
	if (s != HAL_OK) return s;

	return HAL_OK;
}

/* ==========================================================================
 *  Build 257-entry sine table with linear interpolation
 * ========================================================================== */
static void Hbridge_BuildSinTable(void)
{
	uint16_t i;
	for (i = 0U; i <= SIN_TABLE_SIZE; i++) {
		sin_tab[i] = sinf(INV_TWOPI * (float)i / (float)SIN_TABLE_SIZE);
	}
}

/* ==========================================================================
 *  SPWM duty update — ISR @ 50 kHz
 *
 *  双极性调制:
 *    duty_A = 0.5 + (M/2) × sin(ωt)   →  TA1 高侧
 *    duty_B = 0.5 − (M/2) × sin(ωt)   →  TB1 高侧
 *    Vout   = Vdc × (d_A − d_B) = Vdc × M × sin(ωt)
 *
 *  M = s_mod_index (编码器旋钮实时调节)
 * ========================================================================== */
static void Hbridge_UpdateDuty(void)
{
	const float period1 = PWM_PERIOD_P1_F;
	float       half_m;
	uint32_t    idx;
	float       frac, sin_val;
	float       d_a, d_b;
	uint16_t    c_a, c_b;

	/* Read M once for this ISR cycle (atomic float on M4F) */
	half_m = s_mod_index * 0.5f;

	/* Phase accumulator in table space */
	s_phase_tab += SIN_TABLE_STEP;
	if (s_phase_tab >= (float)SIN_TABLE_SIZE) {
		s_phase_tab -= (float)SIN_TABLE_SIZE;
	}

	/* Linear interpolation */
	idx     = (uint32_t)s_phase_tab;
	frac    = s_phase_tab - (float)idx;
	sin_val = sin_tab[idx]
	        + frac * (sin_tab[idx + 1U] - sin_tab[idx]);

	/* Bipolar SPWM */
	d_a = 0.5f + half_m * sin_val;
	d_b = 0.5f - half_m * sin_val;

	if      (d_a > DUTY_MAX) d_a = DUTY_MAX;
	else if (d_a < DUTY_MIN) d_a = DUTY_MIN;
	if      (d_b > DUTY_MAX) d_b = DUTY_MAX;
	else if (d_b < DUTY_MIN) d_b = DUTY_MIN;

	c_a = (uint16_t)(d_a * period1);
	c_b = (uint16_t)(d_b * period1);

	CMP_WR(HRTIM_IDX_A, c_a);
	CMP_WR(HRTIM_IDX_B, c_b);
}

/* ==========================================================================
 *  OLED 刷新 — 绘制 4 行:
 *    L1: 标题
 *    L2: 调制指数 M (动态)
 *    L3: 估算 Vrms (动态)
 *    L4: 条形图 (动态)
 * ========================================================================== */
static void Hbridge_OLED_Redraw(void)
{
	float    v_rms;
	uint8_t  bar_len;
	uint8_t  i;
	char     bar[14];

	OLEDGFX_Clear();

	/* Line 1 */
	OLEDGFX_ShowString(OLEDGFX_COL_1, OLEDGFX_LINE_1,
		"H-BRIDGE INV", OLEDGFX_Clip);

	/* Line 2 — M */
	OLEDGFX_ShowString(OLEDGFX_COL_1, OLEDGFX_LINE_2,
		"M =", OLEDGFX_Clip);
	OLEDGFX_ShowFloat((uint16_t)(OLEDGFX_COL_1 + 4U * OLEDGFX_CHAR_WIDTH),
		OLEDGFX_LINE_2, s_mod_index, 2U, 3U);

	/* Line 3 — estimated Vrms */
	v_rms = VDC_NOMINAL * s_mod_index * 0.7071f;
	OLEDGFX_ShowString(OLEDGFX_COL_1, OLEDGFX_LINE_3,
		"Vrms =", OLEDGFX_Clip);
	OLEDGFX_ShowFloat((uint16_t)(OLEDGFX_COL_1 + 7U * OLEDGFX_CHAR_WIDTH),
		OLEDGFX_LINE_3, v_rms, 2U, 1U);
	OLEDGFX_ShowChar((uint16_t)(OLEDGFX_COL_1 + 12U * OLEDGFX_CHAR_WIDTH),
		OLEDGFX_LINE_3, 'V');

	/* Line 4 — bar */
	{
		float ratio = (s_mod_index - M_MIN) / (M_MAX - M_MIN);
		if (ratio < 0.0f) ratio = 0.0f;
		if (ratio > 1.0f) ratio = 1.0f;
		bar_len = (uint8_t)(ratio * 10.0f + 0.5f);
		if (bar_len > 10U) bar_len = 10U;

		bar[0] = '[';
		for (i = 0U; i < bar_len; i++)   bar[i + 1U] = '#';
		for (i = bar_len; i < 10U; i++)  bar[i + 1U] = ' ';
		bar[11] = ']';
		bar[12] = '\0';

		OLEDGFX_ShowString(OLEDGFX_COL_1, OLEDGFX_LINE_4,
			bar, OLEDGFX_Clip);
	}

	OLEDGFX_Submit();
}

/* USER CODE END 0 */

/* ==========================================================================
 *  main
 * ========================================================================== */
int main(void)
{
	/* ---- CubeMX HAL init ---- */
	HAL_Init();
	SystemClock_Config();

	MX_GPIO_Init();
	MX_DMA_Init();
	MX_ADC1_Init();
	MX_HRTIM1_Init();
	MX_I2C1_Init();
	MX_TIM3_Init();
	MX_USART1_UART_Init();
	MX_TIM6_Init();

	/* USER CODE BEGIN 2 */

	HRTIM_TimeBaseCfgTypeDef tb = {0};

	/* — 1. MASTER → 100 kHz — */
	tb.Period = MASTER_PERIOD;  tb.RepetitionCounter = 0;
	tb.PrescalerRatio = HRTIM_PRESCALER;  tb.Mode = HRTIM_MODE_CONTINUOUS;
	if (HAL_HRTIM_TimeBaseConfig(&hhrtim1, HRTIM_TIMERINDEX_MASTER, &tb) != HAL_OK)
		Error_Handler();

	/* — 2. Start MASTER — */
	if (HAL_HRTIM_WaveformCounterStart(&hhrtim1, HRTIM_TIMERID_MASTER) != HAL_OK)
		Error_Handler();

	/* — 3. Timer A — timebase + deadtime + output fix ————————
	 *  MX init 只设了正确的 UPDOWN / Preload / Reset, 但:
	 *    — Deadtime 用的 DT=10 (需改为 8, 与 Timer B 一致)
	 *    — TA1 SetSource=NONE  → 永远无法输出 HIGH (Bug!)
	 *  这里全部纠正。
	 * ————————————————————————————————————————————————————————— */
	tb.Period = PWM_PERIOD;
	if (HAL_HRTIM_TimeBaseConfig(&hhrtim1, HRTIM_IDX_A, &tb) != HAL_OK)
		Error_Handler();

	if (Hbridge_ConfigDeadtime(HRTIM_IDX_A) != HAL_OK)
		Error_Handler();

	/* TA1 (PA8): SET on period match → HIGH, RESET on CMP1 → LOW */
	if (Hbridge_ConfigOutput(HRTIM_IDX_A, HRTIM_OUTPUT_TA1,
			HRTIM_OUTPUTSET_TIMPER, HRTIM_OUTPUTRESET_TIMCMP1) != HAL_OK)
		Error_Handler();

	/* TA2 (PA9): complementary via deadtime */
	if (Hbridge_ConfigOutput(HRTIM_IDX_A, HRTIM_OUTPUT_TA2,
			HRTIM_OUTPUTSET_NONE, HRTIM_OUTPUTRESET_NONE) != HAL_OK)
		Error_Handler();

	/* — 4. Timer B — full config (includes corrected outputs) —— */
	if (Hbridge_ConfigTimerB() != HAL_OK)
		Error_Handler();

	/* — 5. Start timers + enable all 4 outputs — */
	HAL_HRTIM_WaveformCounterStart(&hhrtim1, HRTIM_TIMERID_TIMER_A);
	HAL_HRTIM_WaveformCounterStart(&hhrtim1, HRTIM_TIMERID_TIMER_B);
	HAL_HRTIM_WaveformOutputStart(&hhrtim1, HRTIM_OUTPUT_TA1);
	HAL_HRTIM_WaveformOutputStart(&hhrtim1, HRTIM_OUTPUT_TA2);
	HAL_HRTIM_WaveformOutputStart(&hhrtim1, HRTIM_OUTPUT_TB1);
	HAL_HRTIM_WaveformOutputStart(&hhrtim1, HRTIM_OUTPUT_TB2);

	/* — 6. Sine table + initial duty — */
	Hbridge_BuildSinTable();
	Hbridge_UpdateDuty();

	/* — 7. Encoder — */
	Encoder_Init();

	/* — 8. LEDs — */
	LED_SetLED1State(1U);
	LED_SetLED2State(0U);

	/* — 9. OLED — */
	if (OLEDGFX_Init() == OLEDGFX_OK) {
		Hbridge_OLED_Redraw();
		OLEDGFX_Update();
	}

	/* — 10. TIM6 @ 50 kHz ISR — */
	TIM6->PSC  = TIM6_PSC;  TIM6->ARR = TIM6_ARR;
	TIM6->EGR  = TIM_EGR_UG;
	TIM6->SR   = 0U;
	HAL_NVIC_SetPriority(TIM6_DAC_IRQn, 5U, 0U);
	HAL_NVIC_EnableIRQ(TIM6_DAC_IRQn);
	TIM6->DIER |= TIM_DIER_UIE;
	TIM6->CR1  |= TIM_CR1_CEN;

	s_oled_timer = HAL_GetTick();

	/* USER CODE END 2 */

	while (1)
	{
	/* USER CODE BEGIN WHILE */
	/* USER CODE END WHILE */

	/* USER CODE BEGIN 3 */

		/* ——— 编码器 ——— */
		{
			int16_t delta = Encoder_PopCount();
			if (delta != 0) {
				float new_m = s_mod_index + (float)delta * ENC_STEP;
				if (new_m > M_MAX)      new_m = M_MAX;
				else if (new_m < M_MIN) new_m = M_MIN;
				s_mod_index = new_m;
				s_oled_dirty = 1;
			}
		}

		/* ——— OLED ——— */
		{
			uint32_t now = HAL_GetTick();
			if (s_oled_dirty && ((now - s_oled_timer) >= 100U)) {
				s_oled_timer = now;
				s_oled_dirty = 0;
				Hbridge_OLED_Redraw();
			}
			OLEDGFX_Update();
		}

	}
	/* USER CODE END 3 */
}

/* ==========================================================================
 *  SystemClock_Config — HSI 16M → PLL 170 MHz (unchanged)
 * ========================================================================== */
void SystemClock_Config(void)
{
	RCC_OscInitTypeDef RCC_OscInitStruct = {0};
	RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

	HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

	RCC_OscInitStruct.OscillatorType      = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState            = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState        = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource       = RCC_PLLSOURCE_HSI;
	RCC_OscInitStruct.PLL.PLLM            = RCC_PLLM_DIV4;
	RCC_OscInitStruct.PLL.PLLN            = 85;
	RCC_OscInitStruct.PLL.PLLP            = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ            = RCC_PLLQ_DIV2;
	RCC_OscInitStruct.PLL.PLLR            = RCC_PLLR_DIV2;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) Error_Handler();

	RCC_ClkInitStruct.ClockType      = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
	                                 | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
		Error_Handler();
}

/* USER CODE BEGIN 4 */

/* ==========================================================================
 *  TIM6 ISR @ 50 kHz
 * ========================================================================== */
void TIM6_DAC_IRQHandler(void)
{
	if ((TIM6->SR & TIM_SR_UIF) != 0U) {
		__HAL_TIM_CLEAR_FLAG(&htim6, TIM_FLAG_UPDATE);
		Hbridge_UpdateDuty();

		{
			static uint16_t cnt;
			cnt++;
			if (cnt >= 5000U) {
				cnt = 0U;
				HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin);
			}
		}
	}
}

/* ==========================================================================
 *  HAL I²C callbacks → OLED
 * ========================================================================== */
void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
	if (hi2c->Instance == I2C1) OLEDGFX_TxCpltCallback();
}
void HAL_I2C_MemTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
	if (hi2c->Instance == I2C1) OLEDGFX_TxCpltCallback();
}
void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
	if (hi2c->Instance == I2C1) OLEDGFX_ErrorCallback();
}

/* USER CODE END 4 */

void Error_Handler(void)
{
	__disable_irq();
	while (1) {}
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line) { (void)file; (void)line; }
#endif
