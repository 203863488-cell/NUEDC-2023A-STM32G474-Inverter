/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c — H-Bridge Open-Loop SPWM Inverter
  * @brief          : Timer A (PA8/PA9) + Timer B (PA10/PA11) → H 桥
  *                   f_sw = 100.000 kHz, f_out = 50 Hz, 双极性 SPWM
  *                   编码器旋钮 → 调制指数 M
  *                   ADC 采样 CH12(电压) CH4(电流) → OLED 显示 RMS
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

/* ————————————————————————————————————————————————————————————————————————
 *  Private defines
 * ———————————————————————————————————————————————————————————————————————— */
/* USER CODE BEGIN PD */

/* ==========  PWM 参数 (不改动) ========================================== */

#define HRTIM_PRESCALER    HRTIM_PRESCALERRATIO_DIV1
#define MASTER_PERIOD      849U
#define PWM_PERIOD         MASTER_PERIOD
#define PWM_PERIOD_P1_F    ((float)(MASTER_PERIOD + 1U))

#define DT_PRESCALER       HRTIM_TIMDEADTIME_PRESCALERRATIO_MUL8
#define DT_VALUE           8U

#define INV_FOUT_HZ        50.0f
#define INV_TWOPI          ((float)(2.0 * 3.14159265358979323846))

#define TIM6_PSC           169U
#define TIM6_ARR           19U

#define SIN_TABLE_SIZE     256U
#define SIN_TABLE_STEP     ((float)((float)SIN_TABLE_SIZE * INV_FOUT_HZ / 50000.0f))

#define ENC_STEP           0.010f
#define M_INIT             0.50f
#define M_MIN              0.05f
#define M_MAX              0.90f

#define DUTY_MIN           0.05f
#define DUTY_MAX           0.95f

#define HRTIM_IDX_A        HRTIM_TIMERINDEX_TIMER_A
#define HRTIM_IDX_B        HRTIM_TIMERINDEX_TIMER_B
#define CMP_WR(idx, val)   (HRTIM1->sTimerxRegs[(idx)].CMP1xR = (uint16_t)(val))

/* ==========  ADC / RMS 参数 (独立于 PWM) ================================ */

#define RMS_WINDOW_MS      200U            /* RMS 计算窗口          */
#define RMS_SAMPLE_MS      1U              /* 采样间隔 1 ms         */
#define RMS_SAMPLE_COUNT   (RMS_WINDOW_MS / RMS_SAMPLE_MS)  /* = 200 */

/* USER CODE END PD */

/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* ————————————————————————————————————————————————————————————————————————
 *  Private variables — 分块管理, PWM 与 ADC 互不耦合
 * ———————————————————————————————————————————————————————————————————————— */
/* USER CODE BEGIN PV */

/* ---- PWM 相关 ---- */
static float       sin_tab[SIN_TABLE_SIZE + 1U];
static volatile float s_phase_tab  = 0.0f;
static volatile float s_mod_index  = M_INIT;

/* ---- ADC / RMS 相关 (完全独立于 PWM) ---- */
static float       s_v_sum_sq        = 0.0f;
static float       s_i_sum_sq        = 0.0f;
static uint16_t    s_rms_count       = 0U;
static float       s_v_rms           = 0.0f;
static float       s_i_rms           = 0.0f;
static uint32_t    s_adc_last_ms     = 0U;

/* ---- OLED 相关 ---- */
static uint32_t    s_oled_timer      = 0U;
static uint8_t     s_oled_dirty      = 1U;

/* USER CODE END PV */

void SystemClock_Config(void);

/* USER CODE BEGIN 0 */

/* ========================================================================
 *  PWM 模块 — 以下函数只操作 HRTIM, 不碰 ADC / OLED
 * ======================================================================== */

static HAL_StatusTypeDef Hbridge_WriteOutput(uint32_t timerIdx,
                                             uint32_t outputId,
                                             uint32_t setSrc,
                                             uint32_t rstSrc)
{
	HRTIM_OutputCfgTypeDef o = {0};
	o.Polarity              = HRTIM_OUTPUTPOLARITY_HIGH;
	o.SetSource             = setSrc;
	o.ResetSource           = rstSrc;
	o.IdleMode              = HRTIM_OUTPUTIDLEMODE_NONE;
	o.IdleLevel             = HRTIM_OUTPUTIDLELEVEL_INACTIVE;
	o.FaultLevel            = HRTIM_OUTPUTFAULTLEVEL_NONE;
	o.ChopperModeEnable     = HRTIM_OUTPUTCHOPPERMODE_DISABLED;
	o.BurstModeEntryDelayed = HRTIM_OUTPUTBURSTMODEENTRY_REGULAR;
	return HAL_HRTIM_WaveformOutputConfig(&hhrtim1, timerIdx, outputId, &o);
}

static HAL_StatusTypeDef Hbridge_WriteDeadtime(uint32_t timerIdx)
{
	HRTIM_DeadTimeCfgTypeDef d = {0};
	d.Prescaler       = DT_PRESCALER;
	d.RisingValue     = DT_VALUE;
	d.RisingSign      = HRTIM_TIMDEADTIME_RISINGSIGN_POSITIVE;
	d.RisingLock      = HRTIM_TIMDEADTIME_RISINGLOCK_WRITE;
	d.RisingSignLock  = HRTIM_TIMDEADTIME_RISINGSIGNLOCK_WRITE;
	d.FallingValue    = DT_VALUE;
	d.FallingSign     = HRTIM_TIMDEADTIME_FALLINGSIGN_POSITIVE;
	d.FallingLock     = HRTIM_TIMDEADTIME_FALLINGLOCK_WRITE;
	d.FallingSignLock = HRTIM_TIMDEADTIME_FALLINGSIGNLOCK_WRITE;
	return HAL_HRTIM_DeadTimeConfig(&hhrtim1, timerIdx, &d);
}

static HAL_StatusTypeDef Hbridge_ConfigTimerB(void)
{
	HRTIM_TimeBaseCfgTypeDef  tb  = {0};
	HRTIM_TimerCtlTypeDef     ctl = {0};
	HRTIM_TimerCfgTypeDef     cfg = {0};
	HRTIM_CompareCfgTypeDef   cmp = {0};
	HAL_StatusTypeDef         s;

	tb.Period = PWM_PERIOD;  tb.RepetitionCounter = 0;
	tb.PrescalerRatio = HRTIM_PRESCALER;  tb.Mode = HRTIM_MODE_CONTINUOUS;
	s = HAL_HRTIM_TimeBaseConfig(&hhrtim1, HRTIM_IDX_B, &tb);
	if (s != HAL_OK) return s;

	ctl.UpDownMode = HRTIM_TIMERUPDOWNMODE_UPDOWN;
	ctl.GreaterCMP1 = HRTIM_TIMERGTCMP1_EQUAL;
	ctl.DualChannelDacEnable = HRTIM_TIMER_DCDE_DISABLED;
	s = HAL_HRTIM_WaveformTimerControl(&hhrtim1, HRTIM_IDX_B, &ctl);
	if (s != HAL_OK) return s;

	s = HAL_HRTIM_RollOverModeConfig(&hhrtim1, HRTIM_IDX_B,
		HRTIM_TIM_FEROM_BOTH  | HRTIM_TIM_BMROM_BOTH  |
		HRTIM_TIM_ADROM_BOTH  | HRTIM_TIM_OUTROM_BOTH |
		HRTIM_TIM_ROM_BOTH);
	if (s != HAL_OK) return s;

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

	cmp.CompareValue = (PWM_PERIOD + 1U) / 2U;
	s = HAL_HRTIM_WaveformCompareConfig(&hhrtim1, HRTIM_IDX_B,
		HRTIM_COMPAREUNIT_1, &cmp);
	if (s != HAL_OK) return s;

	s = Hbridge_WriteDeadtime(HRTIM_IDX_B);
	if (s != HAL_OK) return s;

	/* OUTPUT1: SET on period → HIGH, RESET on CMP1 → LOW */
	s = Hbridge_WriteOutput(HRTIM_IDX_B, HRTIM_OUTPUT_TB1,
		HRTIM_OUTPUTSET_TIMPER, HRTIM_OUTPUTRESET_TIMCMP1);
	if (s != HAL_OK) return s;

	/* OUTPUT2: complementary via deadtime */
	s = Hbridge_WriteOutput(HRTIM_IDX_B, HRTIM_OUTPUT_TB2,
		HRTIM_OUTPUTSET_NONE, HRTIM_OUTPUTRESET_NONE);
	if (s != HAL_OK) return s;

	return HAL_OK;
}

static void Hbridge_BuildSinTable(void)
{
	uint16_t i;
	for (i = 0U; i <= SIN_TABLE_SIZE; i++)
		sin_tab[i] = sinf(INV_TWOPI * (float)i / (float)SIN_TABLE_SIZE);
}

static void Hbridge_UpdateDuty(void)
{
	const float period1 = PWM_PERIOD_P1_F;
	float       half_m  = s_mod_index * 0.5f;
	uint32_t    idx;
	float       frac, sv;
	float       da, db;

	s_phase_tab += SIN_TABLE_STEP;
	if (s_phase_tab >= (float)SIN_TABLE_SIZE)
		s_phase_tab -= (float)SIN_TABLE_SIZE;

	idx = (uint32_t)s_phase_tab;
	frac = s_phase_tab - (float)idx;
	sv = sin_tab[idx] + frac * (sin_tab[idx + 1U] - sin_tab[idx]);

	da = 0.5f + half_m * sv;
	db = 0.5f - half_m * sv;
	if (da > DUTY_MAX) da = DUTY_MAX; else if (da < DUTY_MIN) da = DUTY_MIN;
	if (db > DUTY_MAX) db = DUTY_MAX; else if (db < DUTY_MIN) db = DUTY_MIN;

	CMP_WR(HRTIM_IDX_A, (uint16_t)(da * period1));
	CMP_WR(HRTIM_IDX_B, (uint16_t)(db * period1));
}

/* ========================================================================
 *  ADC / RMS 模块 — 只读 SAMPLE_Buffer, 不碰 HRTIM
 * ======================================================================== */

static void ADC_RMS_Reset(void)
{
	s_v_sum_sq  = 0.0f;
	s_i_sum_sq  = 0.0f;
	s_rms_count = 0U;
}

static void ADC_RMS_Sample(void)
{
	float v_inst, i_inst;

	/* 读取瞬时值 (已减去 1.65V 中点偏置) */
	v_inst = Sample_GetTheoreticalVoltage(Sample_CH12);  /* 输出电压 */
	i_inst = Sample_GetTheoreticalCurrent(Sample_CH4);   /* 输出电流 */

	s_v_sum_sq  += v_inst * v_inst;
	s_i_sum_sq  += i_inst * i_inst;
	s_rms_count += 1U;
}

static uint8_t ADC_RMS_IsReady(void)
{
	return (uint8_t)(s_rms_count >= RMS_SAMPLE_COUNT);
}

static void ADC_RMS_Compute(void)
{
	s_v_rms = sqrtf(s_v_sum_sq / (float)s_rms_count);
	s_i_rms = sqrtf(s_i_sum_sq / (float)s_rms_count);
	ADC_RMS_Reset();
}

static void ADC_RMS_Task(void)
{
	uint32_t now = HAL_GetTick();

	/* 按 1 ms 间隔采样 */
	if ((now - s_adc_last_ms) < RMS_SAMPLE_MS) return;
	s_adc_last_ms = now;

	ADC_RMS_Sample();

	/* 窗口满 → 计算 RMS */
	if (ADC_RMS_IsReady()) {
		ADC_RMS_Compute();
		s_oled_dirty = 1U;
	}
}

/* ========================================================================
 *  OLED 显示模块 — 只读 s_mod_index / s_v_rms / s_i_rms
 * ======================================================================== */

static void OLED_Redraw(void)
{
	float  ratio;
	uint8_t bar_len, i;
	char   bar[14];

	OLEDGFX_Clear();

	/* Line 1 */
	OLEDGFX_ShowString(OLEDGFX_COL_1, OLEDGFX_LINE_1,
		"H-BRIDGE INV", OLEDGFX_Clip);

	/* Line 2 — 实测 Vrms */
	OLEDGFX_ShowString(OLEDGFX_COL_1, OLEDGFX_LINE_2,
		"Vo=", OLEDGFX_Clip);
	OLEDGFX_ShowFloat(OLEDGFX_COL_4, OLEDGFX_LINE_2,
		s_v_rms, 2U, 1U);
	OLEDGFX_ShowChar(OLEDGFX_COL_9, OLEDGFX_LINE_2, 'V');

	/* Line 3 — 实测 Irms */
	OLEDGFX_ShowString(OLEDGFX_COL_1, OLEDGFX_LINE_3,
		"Io=", OLEDGFX_Clip);
	OLEDGFX_ShowFloat(OLEDGFX_COL_4, OLEDGFX_LINE_3,
		s_i_rms, 1U, 2U);
	OLEDGFX_ShowChar(OLEDGFX_COL_9, OLEDGFX_LINE_3, 'A');

	/* Line 4 — M + 条形图 */
	OLEDGFX_ShowString(OLEDGFX_COL_1, OLEDGFX_LINE_4, "M", OLEDGFX_Clip);
	OLEDGFX_ShowFloat((uint16_t)(OLEDGFX_COL_1 + OLEDGFX_CHAR_WIDTH),
		OLEDGFX_LINE_4, s_mod_index, 2U, 2U);

	ratio = (s_mod_index - M_MIN) / (M_MAX - M_MIN);
	if (ratio < 0.0f) ratio = 0.0f;  if (ratio > 1.0f) ratio = 1.0f;
	bar_len = (uint8_t)(ratio * 10.0f + 0.5f);
	if (bar_len > 10U) bar_len = 10U;

	bar[0] = '[';  bar[11] = ']';  bar[12] = '\0';
	for (i = 0U; i < bar_len; i++)  bar[i + 1U] = '#';
	for (i = bar_len; i < 10U; i++) bar[i + 1U] = ' ';

	OLEDGFX_ShowString(OLEDGFX_COL_7, OLEDGFX_LINE_4, bar, OLEDGFX_Clip);

	OLEDGFX_Submit();
}

/* USER CODE END 0 */

/* ========================================================================
 *  main
 * ======================================================================== */
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
	MX_TIM6_Init();

	/* USER CODE BEGIN 2 */

	HRTIM_TimeBaseCfgTypeDef tb = {0};

	/* ================================================================
	 *  PWM 初始化 (不改动已验证的逻辑)
	 * ================================================================ */

	/* 1. MASTER → 100 kHz, 然后启动 */
	tb.Period = MASTER_PERIOD;  tb.RepetitionCounter = 0;
	tb.PrescalerRatio = HRTIM_PRESCALER;  tb.Mode = HRTIM_MODE_CONTINUOUS;
	if (HAL_HRTIM_TimeBaseConfig(&hhrtim1, HRTIM_TIMERINDEX_MASTER, &tb) != HAL_OK)
		Error_Handler();
	if (HAL_HRTIM_WaveformCounterStart(&hhrtim1, HRTIM_TIMERID_MASTER) != HAL_OK)
		Error_Handler();

	/* 2. Timer A — timebase + deadtime + 输出纠正
	 *    CubeMX 给 TA1 的 SetSource=NONE 在实测中无法输出 PWM,
	 *    必须显式设为 TIMPER。TA2 为 NONE+NONE (死区互补自动生成)。 */
	tb.Period = PWM_PERIOD;
	if (HAL_HRTIM_TimeBaseConfig(&hhrtim1, HRTIM_IDX_A, &tb) != HAL_OK)
		Error_Handler();
	if (Hbridge_WriteDeadtime(HRTIM_IDX_A) != HAL_OK)
		Error_Handler();
	if (Hbridge_WriteOutput(HRTIM_IDX_A, HRTIM_OUTPUT_TA1,
			HRTIM_OUTPUTSET_TIMPER, HRTIM_OUTPUTRESET_TIMCMP1) != HAL_OK)
		Error_Handler();
	if (Hbridge_WriteOutput(HRTIM_IDX_A, HRTIM_OUTPUT_TA2,
			HRTIM_OUTPUTSET_NONE, HRTIM_OUTPUTRESET_NONE) != HAL_OK)
		Error_Handler();

	/* 3. Timer B — 完整重配 (CubeMX 留下的状态与 A 不一致) */
	if (Hbridge_ConfigTimerB() != HAL_OK)
		Error_Handler();

	/* 4. 启动从定时器 + 使能全部 4 路输出 */
	HAL_HRTIM_WaveformCounterStart(&hhrtim1, HRTIM_TIMERID_TIMER_A);
	HAL_HRTIM_WaveformCounterStart(&hhrtim1, HRTIM_TIMERID_TIMER_B);
	HAL_HRTIM_WaveformOutputStart(&hhrtim1, HRTIM_OUTPUT_TA1);
	HAL_HRTIM_WaveformOutputStart(&hhrtim1, HRTIM_OUTPUT_TA2);
	HAL_HRTIM_WaveformOutputStart(&hhrtim1, HRTIM_OUTPUT_TB1);
	HAL_HRTIM_WaveformOutputStart(&hhrtim1, HRTIM_OUTPUT_TB2);

	/* 5. 正弦表 + 初始占空比 */
	Hbridge_BuildSinTable();
	Hbridge_UpdateDuty();

	/* 6. 编码器 */
	Encoder_Init();

	/* 7. LED */
	LED_SetLED1State(1U);
	LED_SetLED2State(0U);

	/* ================================================================
	 *  ADC 采样初始化 (在 PWM 启动之后)
	 * ================================================================ */
	if (Sample_Init() != SAMPLE_OK) {
		/* ADC 失败不阻塞 PWM — LED2 快闪作警告 */
		LED_SetLED2State(1U);
	}
	ADC_RMS_Reset();
	s_adc_last_ms = HAL_GetTick();

	/* ================================================================
	 *  OLED
	 * ================================================================ */
	if (OLEDGFX_Init() == OLEDGFX_OK) {
		OLED_Redraw();
		OLEDGFX_Update();
	}

	/* ================================================================
	 *  TIM6 @ 50 kHz (SPWM 更新 ISR)
	 * ================================================================ */
	TIM6->PSC = TIM6_PSC;  TIM6->ARR = TIM6_ARR;
	TIM6->EGR = TIM_EGR_UG;  TIM6->SR = 0U;
	HAL_NVIC_SetPriority(TIM6_DAC_IRQn, 5U, 0U);
	HAL_NVIC_EnableIRQ(TIM6_DAC_IRQn);
	TIM6->DIER |= TIM_DIER_UIE;
	TIM6->CR1  |= TIM_CR1_CEN;

	s_oled_timer = HAL_GetTick();

	/* USER CODE END 2 */

	/* ================================================================
	 *  主循环 — 三件事, 彼此解耦:
	 *    1. 编码器 → 调节 M
	 *    2. ADC 采样 → 计算 RMS (独立任务, 不碰 PWM)
	 *    3. OLED 刷新
	 *    PWM 占空比完全由 TIM6 ISR 独立驱动。
	 * ================================================================ */
	while (1)
	{
	/* USER CODE BEGIN WHILE */
	/* USER CODE END WHILE */

	/* USER CODE BEGIN 3 */

		/* ——— 编码器 ———————————————— */
		{
			int16_t d = Encoder_PopCount();
			if (d != 0) {
				float nm = s_mod_index + (float)d * ENC_STEP;
				if (nm > M_MAX) nm = M_MAX; else if (nm < M_MIN) nm = M_MIN;
				s_mod_index = nm;
				s_oled_dirty = 1U;
			}
		}

		/* ——— ADC / RMS (完全独立, 内部节流 1 ms) ——— */
		ADC_RMS_Task();

		/* ——— OLED ————————————————— */
		{
			uint32_t now = HAL_GetTick();
			if (s_oled_dirty && ((now - s_oled_timer) >= 100U)) {
				s_oled_timer = now;
				s_oled_dirty = 0U;
				OLED_Redraw();
			}
			OLEDGFX_Update();
		}

	}
	/* USER CODE END 3 */
}

/* ========================================================================
 *  SystemClock_Config
 * ======================================================================== */
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

/* ========================================================================
 *  TIM6 ISR — SPWM only, 不碰 ADC
 * ======================================================================== */
void TIM6_DAC_IRQHandler(void)
{
	if ((TIM6->SR & TIM_SR_UIF) != 0U) {
		__HAL_TIM_CLEAR_FLAG(&htim6, TIM_FLAG_UPDATE);
		Hbridge_UpdateDuty();

		{ static uint16_t c; c++; if (c >= 5000U) { c = 0U;
		  HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin); } }
	}
}

void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c)
{ if (hi2c->Instance == I2C1) OLEDGFX_TxCpltCallback(); }
void HAL_I2C_MemTxCpltCallback(I2C_HandleTypeDef *hi2c)
{ if (hi2c->Instance == I2C1) OLEDGFX_TxCpltCallback(); }
void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{ if (hi2c->Instance == I2C1) OLEDGFX_ErrorCallback(); }

/* USER CODE END 4 */

void Error_Handler(void) { __disable_irq(); while (1) {} }
#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line) { (void)file; (void)line; }
#endif
