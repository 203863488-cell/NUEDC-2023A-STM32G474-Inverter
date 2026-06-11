/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c — H-Bridge PR-Closed-Loop Inverter
  * @brief          : Timer A (PA8/PA9) + Timer B (PA10/PA11) → H 桥
  *                   f_sw = 100.000 kHz, f_out = 50 Hz
  *                   PR 谐振闭环控制, LCL 滤波
  *                   用户只需修改下面 "用户参数" 区域的 #define,
  *                   代码自动计算 Kp / Kr / ωc / LPF / 相位裕度。
  *                   只修改此文件 — 不触碰任何驱动库源码。
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

/* ==========================================================================
 *  ╔══════════════════════════════════════════════════════════════════╗
 *  ║              用 户 参 数  (只改这里!)                           ║
 *  ╚══════════════════════════════════════════════════════════════════╝
 * ========================================================================== */
/* USER CODE BEGIN PD */

/* ——— 电源 ——————————————————————————————————————————————————— */
#define VDC_BUS            48.0f    /* 直流母线电压 (V)              */

/* ——— LCL 滤波器 —————————————————————————————————————————————— */
#define L1_VAL             0.001f   /* 逆变器侧电感 (H)              */
#define L2_VAL             0.001f   /* 负载侧电感 (H)                */
#define C_VAL              0.000001f /* 滤波电容 (F)                 */

/* ——— 输出规格 ———————————————————————————————————————————————— */
#define VREF_RMS_INIT       1.0f    /* 上电默认输出 RMS 电压 (V) — 首次上电从1V开始, 调试后改为24V */
#define F_OUT              50.0f    /* 输出基波频率 (Hz)             */
#define I_MAX_RMS           7.0f    /* 输出过流保护阈值 (A RMS)      */
#define VREF_RMS_MIN        1.0f    /* 旋钮可调下限 (V)              */
#define VREF_RMS_MAX       30.0f    /* 旋钮可调上限 (V)              */
#define VREF_ENC_STEP       0.5f    /* 旋钮每格电压变化量 (V)        */

/* ——— 控制性能目标 ————————————————————————————————————————————
 *  f_c_target : 期望开环交叉频率。
 *      越大 → 响应越快, 但过近 LCL 谐振峰(f_res) → 振荡。
 *      经验: f_c_target < min(f_res/4, f_sw/20)。
 *  pm_target  : 期望相位裕度 (°), 推荐 ≥ 45°。
 *  f_bw_res   : PR 谐振控制器带宽 (Hz)。
 *      窄带(1~3 Hz) → 精确跟踪, 对频率漂移敏感。
 *      宽带(5~10 Hz) → 鲁棒性强, 跟踪精度略降。
 *  err_pct    : 期望稳态跟踪误差 (%)。
 *      用于计算 Kr。 值越小 → Kr 越大 → 50 Hz 增益越高。
 *  f_lpf_fb   : 反馈 ADC 低通截止频率 (Hz)。
 *      用于衰减 LCL 谐振峰, 防止开环增益在 f_res 处 > 0 dB。
 *      设为 f_res / 3 ~ f_res / 5 效果较好。
 * ————————————————————————————————————————————————————————————— */
#define F_C_TARGET         200.0f   /* 目标交叉频率 (Hz) — 极保守配置 */
#define PM_TARGET           75.0f   /* 目标相位裕度 (°) — 极大裕度   */
#define F_BW_RES             5.0f   /* PR 谐振带宽 (Hz) — 宽带鲁棒   */
#define ERR_TRACK_PCT        3.0f   /* 期望跟踪误差 (%) — 放宽要求   */
#define F_LPF_FB           600.0f   /* 反馈低通截止 (Hz) — 深度滤波  */

/* ——— 反馈极性 ————————————————————————————————————————————————
 *  +1 : 输出电压正半周 → ADC 读数为正 (相对 1.65V 中点)。
 *  -1 : 翻转。 若闭环发散/振荡 → 试改此值。
 * ————————————————————————————————————————————————————————————— */
#define PHASE_POLARITY        1

/* --- 多谐振控制器 (消除 3/5/7 次谐波) --------------------
 *  HARM_COUNT : 谐波数量 — 设为0可完全禁用以提高稳定性
 *  F_BW_HARM  : 谐波谐振器带宽 (比基波宽, 增强鲁棒性)
 *  HARM_KR_FACTOR : 谐波 Kr = Kp * 此系数
 * ------------------------------------------------------- */
#define HARM_COUNT             3     /* 3/5/7次谐波补偿 */
#define HARM_FREQ_1         150.0f
#define HARM_FREQ_2         250.0f
#define HARM_FREQ_3         350.0f
#define HARM_FREQ_4           0.0f
#define HARM_FREQ_5           0.0f
#define HARM_FREQ_6           0.0f
#define F_BW_HARM           15.0f   /* 带宽加宽 */
#define HARM_KR_FACTOR       5.0f   /* 增益降低 */

/* --- ADC 同步触发: 1=在 PWM 中心点触发(低噪声), 0=周期起点 --- */
#define ADC_SYNC_SAMPLE        1

/* ==========================================================================
 *  以下为自动推导参数 — 无需手动修改
 * ========================================================================== */

/* ——— PWM 时基 (已验证) —————————————————————————————————————— */
#define HRTIM_PRESCALER     HRTIM_PRESCALERRATIO_DIV1
#define MASTER_PERIOD       1699U
#define PWM_PERIOD           MASTER_PERIOD
#define PWM_PERIOD_F         ((float)(MASTER_PERIOD + 1U))  /* 1700.0 */
#define F_SW                100000.0f

#define DT_PRESCALER        HRTIM_TIMDEADTIME_PRESCALERRATIO_DIV2
#define DT_VALUE            34U  /* 死区 ≈ 34 / (170MHz/2) = 400ns */

/* ——— ISR ——————————————————————————————————————————————————— */
#define TIM6_PSC             169U
#define TIM6_ARR              19U
#define ISR_FREQ           50000.0f
#define ISR_TS              (1.0f / ISR_FREQ)

/* ——— 正弦查表 ——————————————————————————————————————————————— */
#define SIN_TABLE_SIZE       256U
#define SIN_INV_256          (1.0f / 256.0f)
#define INV_TWOPI            ((float)(2.0 * 3.14159265358979323846))
#define INV_SQRT2            ((float)(1.4142135623730951))

/* ——— 硬限幅 ————————————————————————————————————————————————— */
#define DUTY_MIN             0.05f
#define DUTY_MAX             0.95f
#define D_MOD_MAX            0.90f

/* ——— 寄存器快捷写 ——————————————————————————————————————————— */
#define HRTIM_IDX_A          HRTIM_TIMERINDEX_TIMER_A
#define HRTIM_IDX_B          HRTIM_TIMERINDEX_TIMER_B
#define CMP_WR(idx, val)     (HRTIM1->sTimerxRegs[(idx)].CMP1xR = (uint16_t)(val))

/* ——— ADC RMS 窗口 ——————————————————————————————————————————— */
#define RMS_WINDOW_MS         200U
#define RMS_SAMPLE_MS           1U
#define RMS_SAMPLE_CNT       (RMS_WINDOW_MS / RMS_SAMPLE_MS)

/* USER CODE END PD */
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* ==========================================================================
 *  运行时变量
 * ========================================================================== */
/* USER CODE BEGIN PV */

/* 正弦表 (init 一次写入, ISR 只读) */
static float sin_tab[SIN_TABLE_SIZE + 1U];

/* 相位累加器 (uint32_t 无漂移) — ISR 写, init 写 step */
static volatile uint32_t s_phase_acc = 0U;
static uint32_t s_phase_step;       /* 每次 ISR 步进, 在 ComputeParams 中计算 */

/* 目标电压 — 主循环写, ISR 读 */
static volatile float s_vref_rms = VREF_RMS_INIT;

/* ---- 自动计算出的控制系数 (init 写入, ISR 只读) ---- */
static float s_w0;              /* ω₀ = 2π·F_OUT                */
static float s_w0_sq;           /* ω₀²                          */
static float s_wc_res;          /* ωc_res = 2π·F_BW_RES         */
static float s_two_wc_res;      /* 2·ωc_res                     */
static float s_Kp;              /* 比例增益 (mod/V)              */
static float s_Kr;              /* 谐振增益 (mod/V)              */
static float s_lpf_alpha;       /* 反馈低通系数                  */
static volatile float s_vref_peak = 0.0f;  /* ISR 读, 主循环写 */

/* ---- 多谐振控制器状态 (ISR 独占, 各 HARM_COUNT 组) ---- */
#if HARM_COUNT > 0
static float s_harm_w0[HARM_COUNT];
static float s_harm_w0_sq[HARM_COUNT];
static float s_harm_wc;
static float s_harm_two_wc;
static float s_harm_Kr;
static float s_harm_x1[HARM_COUNT];
static float s_harm_x2[HARM_COUNT];
#endif

/* ---- PR 控制器状态 (ISR 独占) ---- */
static float s_pr_x1     = 0.0f;
static float s_pr_x2     = 0.0f;

/* 反馈 LPF 状态 */
static float s_vfb_filt  = 0.0f;

/* ---- ADC / RMS ---- */
static float    s_v_sum_sq    = 0.0f;
static float    s_i_sum_sq    = 0.0f;
static uint16_t s_rms_cnt     = 0U;
static          float s_v_rms = 0.0f;
static          float s_i_rms = 0.0f;
static uint32_t s_adc_last_ms = 0U;

/* ---- OLED ---- */
static uint32_t s_oled_timer  = 0U;
static uint8_t  s_oled_dirty  = 1U;

/* ---- 自动计算诊断值 (init 写入, 只读) ---- */
static float s_f_res_est;       /* 估算 LCL 谐振频率 (Hz)       */
static float s_pm_est;          /* 估算相位裕度 (°)              */
static float s_ol_gain_50Hz;    /* 50 Hz 开环增益 (dB)           */

/* USER CODE END PV */

void SystemClock_Config(void);

/* ==========================================================================
 *  PWM 模块 — 不改动已验证配置
 * ========================================================================== */
/* USER CODE BEGIN 0 */

static HAL_StatusTypeDef Hbridge_WriteOut(uint32_t tIdx, uint32_t oId,
                                          uint32_t set, uint32_t rst)
{
	HRTIM_OutputCfgTypeDef o = {0};
	o.Polarity = HRTIM_OUTPUTPOLARITY_HIGH;
	o.SetSource = set;  o.ResetSource = rst;
	o.IdleMode = HRTIM_OUTPUTIDLEMODE_NONE;
	o.IdleLevel = HRTIM_OUTPUTIDLELEVEL_INACTIVE;
	o.FaultLevel = HRTIM_OUTPUTFAULTLEVEL_NONE;
	o.ChopperModeEnable = HRTIM_OUTPUTCHOPPERMODE_DISABLED;
	o.BurstModeEntryDelayed = HRTIM_OUTPUTBURSTMODEENTRY_REGULAR;
	return HAL_HRTIM_WaveformOutputConfig(&hhrtim1, tIdx, oId, &o);
}

static HAL_StatusTypeDef Hbridge_WriteDT(uint32_t tIdx)
{
	HRTIM_DeadTimeCfgTypeDef d = {0};
	d.Prescaler = DT_PRESCALER;
	d.RisingValue = DT_VALUE;  d.FallingValue = DT_VALUE;
	d.RisingSign  = HRTIM_TIMDEADTIME_RISINGSIGN_POSITIVE;
	d.FallingSign = HRTIM_TIMDEADTIME_FALLINGSIGN_POSITIVE;
	d.RisingLock  = HRTIM_TIMDEADTIME_RISINGLOCK_WRITE;
	d.RisingSignLock  = HRTIM_TIMDEADTIME_RISINGSIGNLOCK_WRITE;
	d.FallingLock     = HRTIM_TIMDEADTIME_FALLINGLOCK_WRITE;
	d.FallingSignLock = HRTIM_TIMDEADTIME_FALLINGSIGNLOCK_WRITE;
	return HAL_HRTIM_DeadTimeConfig(&hhrtim1, tIdx, &d);
}

static HAL_StatusTypeDef Hbridge_ConfigTimerB(void)
{
	HRTIM_TimeBaseCfgTypeDef tb  = {0};
	HRTIM_TimerCtlTypeDef    ctl = {0};
	HRTIM_TimerCfgTypeDef    cfg = {0};
	HRTIM_CompareCfgTypeDef  cmp = {0};
	HAL_StatusTypeDef        s;

	tb.Period = PWM_PERIOD;  tb.RepetitionCounter = 0;
	tb.PrescalerRatio = HRTIM_PRESCALER;  tb.Mode = HRTIM_MODE_CONTINUOUS;
	s = HAL_HRTIM_TimeBaseConfig(&hhrtim1, HRTIM_IDX_B, &tb);
	if (s != HAL_OK) return s;

	ctl.UpDownMode = HRTIM_TIMERUPDOWNMODE_UP;
	ctl.GreaterCMP1 = HRTIM_TIMERGTCMP1_EQUAL;
	ctl.DualChannelDacEnable = HRTIM_TIMER_DCDE_DISABLED;
	s = HAL_HRTIM_WaveformTimerControl(&hhrtim1, HRTIM_IDX_B, &ctl);
	if (s != HAL_OK) return s;

	cfg.InterruptRequests = HRTIM_TIM_IT_NONE;
	cfg.DMARequests       = HRTIM_TIM_DMA_NONE;
	cfg.PreloadEnable     = HRTIM_PRELOAD_ENABLED;
	cfg.RepetitionUpdate  = HRTIM_UPDATEONREPETITION_ENABLED;
	cfg.PushPull          = HRTIM_TIMPUSHPULLMODE_DISABLED;
	cfg.FaultEnable       = HRTIM_TIMFAULTENABLE_NONE;
	cfg.FaultLock         = HRTIM_TIMFAULTLOCK_READWRITE;
	cfg.DeadTimeInsertion = HRTIM_TIMDEADTIMEINSERTION_ENABLED;
	cfg.DelayedProtectionMode = HRTIM_TIMER_A_B_C_DELAYEDPROTECTION_DISABLED;
	cfg.UpdateTrigger     = HRTIM_TIMUPDATETRIGGER_NONE;
	cfg.ResetTrigger      = HRTIM_TIMRESETTRIGGER_NONE;
	cfg.ResetUpdate       = HRTIM_TIMUPDATEONRESET_DISABLED;
	s = HAL_HRTIM_WaveformTimerConfig(&hhrtim1, HRTIM_IDX_B, &cfg);
	if (s != HAL_OK) return s;

	cmp.CompareValue = (PWM_PERIOD + 1U) / 2U;
	s = HAL_HRTIM_WaveformCompareConfig(&hhrtim1, HRTIM_IDX_B,
		HRTIM_COMPAREUNIT_1, &cmp);
	if (s != HAL_OK) return s;

	s = Hbridge_WriteDT(HRTIM_IDX_B);  if (s != HAL_OK) return s;
	s = Hbridge_WriteOut(HRTIM_IDX_B, HRTIM_OUTPUT_TB1,
		HRTIM_OUTPUTSET_TIMPER, HRTIM_OUTPUTRESET_TIMCMP1);
	if (s != HAL_OK) return s;
	s = Hbridge_WriteOut(HRTIM_IDX_B, HRTIM_OUTPUT_TB2,
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

/* ========================================================================
 *  Hbridge_ComputeParams — 根据用户 #define 自动推导所有控制系数
 *
 *  推导过程:
 *
 *  1) LCL 谐振频率
 *     f_res = 1/(2π)·√((L1+L2)/(L1·L2·C))
 *     例: 1mH+1mH+1µF → f_res ≈ 7120 Hz
 *
 *  2) 等效负载 (最恶劣 — 满载时 R 最小)
 *     R_min = VREF_RMS / I_MAX_RMS
 *     例: 24V / 2A = 12 Ω
 *
 *  3) 比例增益 Kp
 *     在交叉频率 f_c 处, 开环增益 = 1:
 *       |Kp * Vdc * G_filter(jωc)| = 1
 *     G_filter(s) ≈ R/(R+s·L_total)    (对 f << f_res)
 *     Kp = sqrt(R_min² + (ωc·L_total)²) / (Vdc·R_min)
 *
 *  4) 谐振增益 Kr
 *     在 50 Hz 处需要 Kp+Kr 的开环增益达到:
 *       G_OL_50Hz ≈ 100 / ERR_TRACK_PCT
 *     (1% 跟踪误差 → 需要 40 dB 开环增益)
 *     Kr = G_OL_50Hz / (Vdc·|G_filter_50Hz|) - Kp
 *
 *  5) 数字控制延迟估算
 *     T_delay ≈ 1.5 × Ts  (1 ISR 采样 + 0.5 PWM 周期)
 *     在 f_c 处的相位延迟 = 360° × f_c × T_delay
 *
 *  6) PR 控制器在 f_c 处的相位贡献
 *     H_PR(jωc) ≈ Kp + j·Kr·2ωc_res/ωc   (ωc >> ω₀)
 *     ∠H_PR ≈ atan(Kr·2ωc_res / (Kp·ωc))
 *
 *  7) 相位裕度估算
 *     PM ≈ 180° + ∠H_PR + ∠G_filter + ∠delay
 *     ∠G_filter = -atan(ωc·L_total / R_min)
 *     ∠delay    = -360°·f_c·1.5·Ts
 *
 *  8) 反馈 LPF 系数
 *     α = 2π·f_lpf·Ts    (一阶 IIR)
 *
 *  9) PR 谐振带宽 (非理想谐振控制器)
 *     ωc_res = 2π·f_bw_res
 *
 *  10) 稳定性校验
 *      若 f_c_target > f_res/4: 交叉频率过近谐振峰, 建议降低
 *      若 PM_est < PM_target: 建议降低 f_c 或增大 f_bw_res
 *      LPF 在 f_res 处的衰减 = f_lpf/f_res (一阶)
 *      谐振峰处开环增益 = Kp·Vdc·Q·(f_lpf/f_res)
 *      若 > 1 → 不稳定, 需降低 f_lpf 或 f_c
 * ======================================================================== */
static void Hbridge_ComputeParams(void)
{
	const float l_total   = L1_VAL + L2_VAL;
	const float r_min     = VREF_RMS_MAX / I_MAX_RMS;
	const float ts        = ISR_TS;

	/* —— 基础量 —— */
	const float w0    = INV_TWOPI * F_OUT;
	const float w0_sq = w0 * w0;

	/* —— 1. LCL 谐振频率 —— */
	const float f_res = (1.0f / INV_TWOPI)
	                  * sqrtf((L1_VAL + L2_VAL) / (L1_VAL * L2_VAL * C_VAL));

	/* —— 2. 交叉频率自动钳位 (≤ f_res/4 确保远离谐振峰) —— */
	float f_c = F_C_TARGET;
	if (f_c > f_res * 0.25f) f_c = f_res * 0.25f;
	const float w_c = INV_TWOPI * f_c;

	/* —— 3. Kp: 在 f_c 处 |Kp·Vdc·G_filter| = 1 —— */
	const float g_filt_c = r_min / sqrtf(r_min*r_min + (w_c*l_total)*(w_c*l_total));
	s_Kp = 1.0f / (VDC_BUS * g_filt_c);
	if (s_Kp > 1.0f) s_Kp = 1.0f;

	/* —— 4. 50 Hz 滤波器增益 —— */
	const float g_50 = r_min / sqrtf(r_min*r_min + (w0*l_total)*(w0*l_total));

	/* —— 5. Kr: 满足跟踪误差
	 *    50 Hz 处开环增益需 ≥ 100/ERR_TRACK_PCT  (例: 1% → 40 dB)
	 *    (Kp+Kr)·Vdc·g_50 ≥ 100/ERR_TRACK_PCT  —— */
	const float need_gain_50 = 100.0f / ERR_TRACK_PCT;
	const float need_pr_gain = need_gain_50 / (VDC_BUS * g_50);
	s_Kr = need_pr_gain - s_Kp;
	if (s_Kr < s_Kp * 5.0f) s_Kr = s_Kp * 5.0f;
	if (s_Kr > s_Kp * 200.0f) s_Kr = s_Kp * 200.0f;

	/* —— 6. PR 谐振带宽 (须在相位裕度计算之前) —— */
	s_wc_res     = INV_TWOPI * F_BW_RES;
	s_two_wc_res = 2.0f * s_wc_res;

	/* —— 7. 相位裕度估算
	 *   H_PR(jωc) = Kp + Kr·2ωc_res·jωc / (ω₀²-ωc² + 2ωc_res·jωc)
	 *   分解实部/虚部 → 加滤波器滞后 + 数字延迟 (1.5 Ts)
	 *   PM = 180° + total_phase  —— */
	{
		const float dr   = w0_sq - w_c*w_c;
		const float di   = s_two_wc_res * w_c;
		const float num  = s_Kr * s_two_wc_res * w_c;
		const float msq  = dr*dr + di*di + 1e-30f;
		const float pr_re = s_Kp + num * di / msq;
		const float pr_im = num * dr / msq;
		const float pr_phi    = atan2f(pr_im, pr_re);
		const float filt_phi  = -atanf(w_c * l_total / r_min);
		const float delay_phi = -INV_TWOPI * f_c * 1.5f * ts;
		const float total_phi = pr_phi + filt_phi + delay_phi;
		s_pm_est = 180.0f + total_phi * 180.0f / (float)3.14159265358979323846;
	}

	/* —— 8. 反馈 LPF 系数 —— */
	s_lpf_alpha = INV_TWOPI * F_LPF_FB * ts;
	if (s_lpf_alpha > 1.0f) s_lpf_alpha = 1.0f;

	/* —— 8b. 相位累加器步进 (uint32_t 无漂移) —— */
		s_phase_step = (uint32_t)(4294967296.0f * F_OUT / ISR_FREQ);

	/* —— 9. 保存诊断值 —— */
	s_w0           = w0;
	s_w0_sq        = w0_sq;
	s_f_res_est    = f_res;
	s_ol_gain_50Hz = 20.0f * log10f((s_Kp + s_Kr) * VDC_BUS * g_50);

	/* —— 10. 谐波谐振器参数 —— */
#if HARM_COUNT > 0
	s_harm_wc     = INV_TWOPI * F_BW_HARM;
	s_harm_two_wc = 2.0f * s_harm_wc;
	s_harm_Kr = s_Kp * HARM_KR_FACTOR;
	if (s_harm_Kr > s_Kp * 50.0f) s_harm_Kr = s_Kp * 50.0f;

	{
		const float all_harm_freq[6] = {HARM_FREQ_1, HARM_FREQ_2, HARM_FREQ_3, HARM_FREQ_4, HARM_FREQ_5, HARM_FREQ_6};
		for (uint8_t h = 0U; h < HARM_COUNT; h++) {
			s_harm_w0[h]    = INV_TWOPI * all_harm_freq[h];
			s_harm_w0_sq[h] = s_harm_w0[h] * s_harm_w0[h];
		}
	}
#endif
}

/* ========================================================================
 *  Hbridge_ControlStep — PR 闭环控制 (TIM6 ISR @ 50 kHz)
 *
 *  信号链:
 *    ADC → LPF → [×PHASE_POLARITY] → 误差 e = Vref·sin - Vfb
 *    e → PR 控制器 (Kp + 谐振部分) → u_pr
 *    d_mod = clamp(Vref·sin/Vdc + u_pr, ±0.9)
 *    duty_A = 0.5 + 0.5·d_mod,  duty_B = 0.5 - 0.5·d_mod
 *    CMP = duty × 1700 → 写 HRTIM 寄存器 (Preload=ON, 周期边界更新)
 *
 *  PR 谐振器离散实现 (forward Euler, Ts = 20 µs):
 *    ẋ₁ = x₂
 *    ẋ₂ = -ω₀²·x₁ - 2ωc_res·x₂ + e
 *    y  = Kr·2ωc_res·x₂
 *    u  = Kp·e + y
 *
 *  抗饱和: d_mod 限幅后若被钳位, 丢弃此次状态更新 (冻结积分器)
 * ======================================================================== */
static void Hbridge_ControlStep(void)
{
	const float ts    = ISR_TS;
	const float scale = PWM_PERIOD_F;

	float vfb_raw, vfb_sign, vref_inst, err;
	float x1_old, x2_old, x1_new, x2_new, y_res, u_pr;
	float d_ff, d_mod;
	uint16_t cmp_a, cmp_b;

	/* ---- 读反馈 ---- */
	vfb_raw = Sample_GetTheoreticalVoltage(Sample_CH12);
	s_vfb_filt += s_lpf_alpha * (vfb_raw - s_vfb_filt);
	vfb_sign = (float)PHASE_POLARITY * s_vfb_filt;

	/* ---- 正弦参考 (uint32_t无漂移) ---- */
	s_phase_acc += s_phase_step;
	{
		uint32_t idx  = (s_phase_acc >> 24) & 0xFFU;
		uint32_t frac = (s_phase_acc >> 16) & 0xFFU;
		float    frc  = (float)frac * SIN_INV_256;
		float    sv   = sin_tab[idx]
		             + frc * (sin_tab[idx + 1U] - sin_tab[idx]);
		vref_inst = s_vref_peak * sv;
	}

	/* ---- 误差 ---- */
	err = vref_inst - vfb_sign;

	/* ---- PR 谐振器: 基波 + 谐波 (先暂存所有旧状态) ---- */
	x1_old = s_pr_x1;
	x2_old = s_pr_x2;

	{
		float    y_harm = 0.0f;
#if HARM_COUNT > 0
		float    hx1_old[HARM_COUNT];
		float    hx2_old[HARM_COUNT];
		float    hx1_new[HARM_COUNT];
		float    hx2_new[HARM_COUNT];
		for (uint8_t h = 0U; h < HARM_COUNT; h++) {
			hx1_old[h] = s_harm_x1[h];
			hx2_old[h] = s_harm_x2[h];
		}
#endif

		/* —— 基波谐振器 —— */
		x1_new = x1_old + ts * x2_old;
		x2_new = x2_old + ts * (-s_w0_sq * x1_old
		                        - s_two_wc_res * x2_old
		                        + err);
		y_res  = s_Kr * s_two_wc_res * x2_new;

		/* —— 谐波谐振器 (3rd=150Hz, 5th=250Hz, 7th=350Hz) —— */
#if HARM_COUNT > 0
		for (uint8_t h = 0U; h < HARM_COUNT; h++) {
			hx1_new[h] = hx1_old[h] + ts * hx2_old[h];
			hx2_new[h] = hx2_old[h] + ts * (-s_harm_w0_sq[h] * hx1_old[h]
			                                - s_harm_two_wc   * hx2_old[h]
			                                + err);
			y_harm += s_harm_Kr * s_harm_two_wc * hx2_new[h];
		}
#endif

		u_pr = s_Kp * err + y_res + y_harm;

			/* ---- 前馈 + 总调制 + 限幅 ---- */
			d_ff = vref_inst / VDC_BUS;
			{
				float raw    = d_ff + u_pr;
				uint8_t sat  = 0U;

				if      (raw >  D_MOD_MAX) { d_mod =  D_MOD_MAX; sat = 1U; }
				else if (raw < -D_MOD_MAX) { d_mod = -D_MOD_MAX; sat = 1U; }
				else                       { d_mod = raw; }

				/* 抗饱和: d_mod被钳位 → 冻结所有谐振状态 */
				if (sat == 0U) {
					s_pr_x1 = x1_new;  s_pr_x2 = x2_new;
#if HARM_COUNT > 0
					for (uint8_t h = 0U; h < HARM_COUNT; h++) {
						s_harm_x1[h] = hx1_new[h];
						s_harm_x2[h] = hx2_new[h];
					}
#endif
					{
						float abs_x;
						abs_x = s_pr_x1; if (abs_x < 0.0f) abs_x = -abs_x;
						if (abs_x > 100.0f) s_pr_x1 = (s_pr_x1 > 0.0f ? 100.0f : -100.0f);
						abs_x = s_pr_x2; if (abs_x < 0.0f) abs_x = -abs_x;
						if (abs_x > 10000.0f) s_pr_x2 = (s_pr_x2 > 0.0f ? 10000.0f : -10000.0f);
#if HARM_COUNT > 0
						for (uint8_t h = 0U; h < HARM_COUNT; h++) {
							abs_x = s_harm_x1[h]; if (abs_x < 0.0f) abs_x = -abs_x;
							if (abs_x > 100.0f) s_harm_x1[h] = (s_harm_x1[h] > 0.0f ? 100.0f : -100.0f);
							abs_x = s_harm_x2[h]; if (abs_x < 0.0f) abs_x = -abs_x;
							if (abs_x > 10000.0f) s_harm_x2[h] = (s_harm_x2[h] > 0.0f ? 10000.0f : -10000.0f);
						}
#endif
					}
				}
			}
		}

	/* ---- Bipolar SPWM ---- */
	{
		float da = 0.5f + 0.5f * d_mod;
		float db = 0.5f - 0.5f * d_mod;
		if (da > DUTY_MAX) da = DUTY_MAX; else if (da < DUTY_MIN) da = DUTY_MIN;
		if (db > DUTY_MAX) db = DUTY_MAX; else if (db < DUTY_MIN) db = DUTY_MIN;
		cmp_a = (uint16_t)(da * scale);
		cmp_b = (uint16_t)(db * scale);
	}

	CMP_WR(HRTIM_IDX_A, cmp_a);
	CMP_WR(HRTIM_IDX_B, cmp_b);
}

/* ========================================================================
 *  ADC / RMS / OLED — 仅用于显示, 与闭环控制解耦
 * ======================================================================== */
static void ADC_RMS_Reset(void)
{ s_v_sum_sq = 0.0f;  s_i_sum_sq = 0.0f;  s_rms_cnt = 0U; }
static void ADC_RMS_Sample(void)
{ float vi=Sample_GetTheoreticalVoltage(Sample_CH12);
  float ii=Sample_GetTheoreticalCurrent(Sample_CH4);
  s_v_sum_sq += vi*vi;  s_i_sum_sq += ii*ii;  s_rms_cnt++; }
static uint8_t ADC_RMS_Ready(void){ return (uint8_t)(s_rms_cnt >= RMS_SAMPLE_CNT); }
static void ADC_RMS_Compute(void)
{ s_v_rms = sqrtf(s_v_sum_sq/(float)s_rms_cnt);
  s_i_rms = sqrtf(s_i_sum_sq/(float)s_rms_cnt);  ADC_RMS_Reset(); }
static void ADC_RMS_Task(void)
{ uint32_t t=HAL_GetTick(); if((t-s_adc_last_ms)<RMS_SAMPLE_MS)return;
  s_adc_last_ms=t; ADC_RMS_Sample(); if(ADC_RMS_Ready()){ADC_RMS_Compute();s_oled_dirty=1U;} }

static void OLED_Redraw(void)
{
	float r; uint8_t bl,i; char bar[7];
	OLEDGFX_Clear();

	OLEDGFX_ShowString(OLEDGFX_COL_1, OLEDGFX_LINE_1,"H-BRIDGE INV",OLEDGFX_Clip);

	OLEDGFX_ShowString(OLEDGFX_COL_1, OLEDGFX_LINE_2,"Vo=",OLEDGFX_Clip);
	OLEDGFX_ShowFloat(OLEDGFX_COL_4,  OLEDGFX_LINE_2,s_v_rms,2U,1U);
	OLEDGFX_ShowChar(OLEDGFX_COL_9,   OLEDGFX_LINE_2,'V');

	OLEDGFX_ShowString(OLEDGFX_COL_1, OLEDGFX_LINE_3,"Io=",OLEDGFX_Clip);
	OLEDGFX_ShowFloat(OLEDGFX_COL_4,  OLEDGFX_LINE_3,s_i_rms,1U,2U);
	OLEDGFX_ShowChar(OLEDGFX_COL_9,   OLEDGFX_LINE_3,'A');

	OLEDGFX_ShowString(OLEDGFX_COL_1, OLEDGFX_LINE_4,"Set",OLEDGFX_Clip);
	OLEDGFX_ShowFloat(OLEDGFX_COL_5,  OLEDGFX_LINE_4,s_vref_rms,2U,1U);
	OLEDGFX_ShowChar(OLEDGFX_COL_10,  OLEDGFX_LINE_4,'V');

	r =(s_vref_rms - VREF_RMS_MIN)/(VREF_RMS_MAX - VREF_RMS_MIN);
	if(r<0.0f)r=0.0f; if(r>1.0f)r=1.0f; bl=(uint8_t)(r*4.0f+0.5f); if(bl>4U)bl=4U;
	bar[0]='['; bar[5]=']'; bar[6]='\0';
	for(i=0U;i<bl;i++)bar[i+1U]='#'; for(i=bl;i<4U;i++)bar[i+1U]=' ';
	OLEDGFX_ShowString(OLEDGFX_COL_12,OLEDGFX_LINE_4,bar,OLEDGFX_Clip);

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
	MX_GPIO_Init(); MX_DMA_Init(); MX_ADC1_Init(); MX_HRTIM1_Init();
	MX_I2C1_Init(); MX_TIM3_Init(); MX_USART1_UART_Init(); MX_TIM6_Init();

	/* USER CODE BEGIN 2 */

	/* DMA IRQ 优先级降到控制 ISR 以下, 避免抢占导致抖动 */
	HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 6, 0);

	HRTIM_TimeBaseCfgTypeDef tb = {0};

	/* 1. MASTER */
	tb.Period = MASTER_PERIOD;  tb.RepetitionCounter = 0;
	tb.PrescalerRatio = HRTIM_PRESCALER;  tb.Mode = HRTIM_MODE_CONTINUOUS;
	if (HAL_HRTIM_TimeBaseConfig(&hhrtim1,HRTIM_TIMERINDEX_MASTER,&tb)!=HAL_OK)
		Error_Handler();
	if (HAL_HRTIM_WaveformCounterStart(&hhrtim1,HRTIM_TIMERID_MASTER)!=HAL_OK)
		Error_Handler();

	/* 2. Timer A */
	{
		HRTIM_TimerCtlTypeDef ctlA = {0};
		HRTIM_TimerCfgTypeDef cfgA = {0};

		tb.Period = PWM_PERIOD;
		if (HAL_HRTIM_TimeBaseConfig(&hhrtim1,HRTIM_IDX_A,&tb)!=HAL_OK) Error_Handler();

		ctlA.UpDownMode = HRTIM_TIMERUPDOWNMODE_UP;
		ctlA.GreaterCMP1 = HRTIM_TIMERGTCMP1_EQUAL;
		ctlA.DualChannelDacEnable = HRTIM_TIMER_DCDE_DISABLED;
		if (HAL_HRTIM_WaveformTimerControl(&hhrtim1,HRTIM_IDX_A,&ctlA)!=HAL_OK) Error_Handler();


		cfgA.InterruptRequests = HRTIM_TIM_IT_NONE;
		cfgA.DMARequests       = HRTIM_TIM_DMA_NONE;
		cfgA.PreloadEnable     = HRTIM_PRELOAD_ENABLED;
		cfgA.RepetitionUpdate  = HRTIM_UPDATEONREPETITION_ENABLED;
		cfgA.PushPull          = HRTIM_TIMPUSHPULLMODE_DISABLED;
		cfgA.FaultEnable       = HRTIM_TIMFAULTENABLE_NONE;
		cfgA.FaultLock         = HRTIM_TIMFAULTLOCK_READWRITE;
		cfgA.DeadTimeInsertion = HRTIM_TIMDEADTIMEINSERTION_ENABLED;
		cfgA.DelayedProtectionMode = HRTIM_TIMER_A_B_C_DELAYEDPROTECTION_DISABLED;
		cfgA.UpdateTrigger     = HRTIM_TIMUPDATETRIGGER_NONE;
		cfgA.ResetTrigger      = HRTIM_TIMRESETTRIGGER_NONE;
		cfgA.ResetUpdate       = HRTIM_TIMUPDATEONRESET_DISABLED;
		if (HAL_HRTIM_WaveformTimerConfig(&hhrtim1,HRTIM_IDX_A,&cfgA)!=HAL_OK) Error_Handler();
		{
			HRTIM_CompareCfgTypeDef cmpA = {0};
			cmpA.CompareValue = (PWM_PERIOD + 1U) / 2U;
			if (HAL_HRTIM_WaveformCompareConfig(&hhrtim1,HRTIM_IDX_A,
				HRTIM_COMPAREUNIT_1,&cmpA)!=HAL_OK) Error_Handler();
		}

		if (Hbridge_WriteDT(HRTIM_IDX_A)!=HAL_OK) Error_Handler();
		if (Hbridge_WriteOut(HRTIM_IDX_A,HRTIM_OUTPUT_TA1,
				HRTIM_OUTPUTSET_TIMPER,HRTIM_OUTPUTRESET_TIMCMP1)!=HAL_OK) Error_Handler();
		if (Hbridge_WriteOut(HRTIM_IDX_A,HRTIM_OUTPUT_TA2,
				HRTIM_OUTPUTSET_NONE,HRTIM_OUTPUTRESET_NONE)!=HAL_OK) Error_Handler();
	}

	/* 3. Timer B */
	if (Hbridge_ConfigTimerB()!=HAL_OK) Error_Handler();

	/* 4. 启动 */
	HAL_HRTIM_WaveformCounterStart(&hhrtim1,HRTIM_TIMERID_TIMER_A);
	HAL_HRTIM_WaveformCounterStart(&hhrtim1,HRTIM_TIMERID_TIMER_B);
	HAL_HRTIM_WaveformOutputStart(&hhrtim1,HRTIM_OUTPUT_TA1);
	HAL_HRTIM_WaveformOutputStart(&hhrtim1,HRTIM_OUTPUT_TA2);
	HAL_HRTIM_WaveformOutputStart(&hhrtim1,HRTIM_OUTPUT_TB1);
	HAL_HRTIM_WaveformOutputStart(&hhrtim1,HRTIM_OUTPUT_TB2);

	/* 5. 正弦表 */
	Hbridge_BuildSinTable();

	/* 6. 自动计算控制参数 */
	Hbridge_ComputeParams();

	/* 6b. 初始化 s_vref_peak (ISR 只读) */
	s_vref_peak = s_vref_rms * INV_SQRT2;

	/* 7. 编码器 */
	Encoder_Init();

	/* 8. LED */
	LED_SetLED1State(1U); LED_SetLED2State(0U);

	/* 9. ADC — 启动采样 */
	if (Sample_Init()!=SAMPLE_OK) { LED_SetLED2State(1U); }
	ADC_RMS_Reset();  s_adc_last_ms = HAL_GetTick();

	/* 9b. 同步 ADC 触发到 PWM 中心点 (低开关噪声采样) */
#if ADC_SYNC_SAMPLE
	{
		HRTIM_CompareCfgTypeDef mcmp = {0};
		mcmp.CompareValue = MASTER_PERIOD / 2U;   /* 1699/2 = 849 */
		HAL_HRTIM_WaveformCompareConfig(&hhrtim1, HRTIM_TIMERINDEX_MASTER,
			HRTIM_COMPAREUNIT_1, &mcmp);

		HRTIM_ADCTriggerCfgTypeDef at = {0};
		at.UpdateSource = HRTIM_ADCTRIGGERUPDATE_MASTER;
		at.Trigger = HRTIM_ADCTRIGGEREVENT13_MASTER_CMP1;
		HAL_HRTIM_ADCTriggerConfig(&hhrtim1, HRTIM_ADCTRIGGER_1, &at);
	}
#endif

	/* 10. OLED — 上电显示诊断信息 */
	if (OLEDGFX_Init()==OLEDGFX_OK) {
		OLED_Redraw();
		OLEDGFX_Update();
	}

	/* 11. TIM6 ISR */
	TIM6->PSC=TIM6_PSC; TIM6->ARR=TIM6_ARR; TIM6->EGR=TIM_EGR_UG; TIM6->SR=0U;
	HAL_NVIC_SetPriority(TIM6_DAC_IRQn,5U,0U); HAL_NVIC_EnableIRQ(TIM6_DAC_IRQn);
	TIM6->DIER|=TIM_DIER_UIE;  TIM6->CR1|=TIM_CR1_CEN;
	s_oled_timer=HAL_GetTick();

	/* USER CODE END 2 */

	while (1) {
	/* USER CODE BEGIN WHILE */
	/* USER CODE END WHILE */
	/* USER CODE BEGIN 3 */

		/* 编码器 */
		{ int16_t d=Encoder_PopCount();
		  if(d){ float nv=s_vref_rms+(float)d*VREF_ENC_STEP;
		         if(nv>VREF_RMS_MAX)nv=VREF_RMS_MAX; else if(nv<VREF_RMS_MIN)nv=VREF_RMS_MIN;
		         s_vref_rms=nv; s_vref_peak = nv * INV_SQRT2; s_oled_dirty=1U; } }

		/* ADC RMS */
		ADC_RMS_Task();

		/* OLED */
		{ uint32_t t=HAL_GetTick();
		  if(s_oled_dirty&&((t-s_oled_timer)>=100U)){s_oled_timer=t;s_oled_dirty=0U;OLED_Redraw();}
		  OLEDGFX_Update(); }

		/* 过流保护: Irms>I_MAX → 强制 Vref=1V */
		if(s_i_rms>I_MAX_RMS){ s_vref_rms=VREF_RMS_MIN; s_vref_peak=VREF_RMS_MIN*INV_SQRT2; s_oled_dirty=1U; }
	}
	/* USER CODE END 3 */
}

void SystemClock_Config(void)
{
	RCC_OscInitTypeDef RCC_OscInitStruct = {0};
	RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
	HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);
	RCC_OscInitStruct.OscillatorType=RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState=RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue=RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState=RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource=RCC_PLLSOURCE_HSI;
	RCC_OscInitStruct.PLL.PLLM=RCC_PLLM_DIV4; RCC_OscInitStruct.PLL.PLLN=85;
	RCC_OscInitStruct.PLL.PLLP=RCC_PLLP_DIV2; RCC_OscInitStruct.PLL.PLLQ=RCC_PLLQ_DIV2;
	RCC_OscInitStruct.PLL.PLLR=RCC_PLLR_DIV2;
	if(HAL_RCC_OscConfig(&RCC_OscInitStruct)!=HAL_OK)Error_Handler();
	RCC_ClkInitStruct.ClockType=RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
	                           |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource=RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider=RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider=RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB2CLKDivider=RCC_HCLK_DIV2;
	if(HAL_RCC_ClockConfig(&RCC_ClkInitStruct,FLASH_LATENCY_4)!=HAL_OK)Error_Handler();
}

/* USER CODE BEGIN 4 */
void TIM6_DAC_IRQHandler(void)
{
	if((TIM6->SR&TIM_SR_UIF)!=0U){
		__HAL_TIM_CLEAR_FLAG(&htim6,TIM_FLAG_UPDATE);
		Hbridge_ControlStep();
		{ static uint16_t c; c++; if(c>=5000U){c=0U;
		  HAL_GPIO_TogglePin(LED2_GPIO_Port,LED2_Pin);} }
	}
}
void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef*hi2c)
{ if(hi2c->Instance==I2C1)OLEDGFX_TxCpltCallback(); }
void HAL_I2C_MemTxCpltCallback(I2C_HandleTypeDef*hi2c)
{ if(hi2c->Instance==I2C1)OLEDGFX_TxCpltCallback(); }
void HAL_I2C_ErrorCallback(I2C_HandleTypeDef*hi2c)
{ if(hi2c->Instance==I2C1)OLEDGFX_ErrorCallback(); }
/* USER CODE END 4 */

void Error_Handler(void){__disable_irq();while(1){}}
#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t*f,uint32_t l){(void)f;(void)l;}
#endif
