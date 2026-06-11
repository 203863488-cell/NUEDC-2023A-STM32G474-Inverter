/* USER CODE BEGIN Header */
/**
  * @file    main.c
  * @brief   H桥 PR谐振闭环逆变器
  *
  * 硬件:  STM32G474VE @170MHz, HRTIM, LCL滤波
  * 拓扑:  单相全桥, 双极性SPWM
  * 开关:  100kHz 边缘对齐 (up-only, Period=1699)
  * 输出:  50Hz 正弦交流, 1~30V RMS 可调
  * 控制:  基波PR谐振 + 3/5/7次谐波谐振器 @50kHz ISR
  * 反馈:  ADC1 DMA 9通道扫描, CH12=电压 CH4=电流
  * 显示:  OLED 128x64 I2C, 编码器旋钮调压
  *
  * 用户只需修改下面 #define 区域, 代码自动计算 Kp/Kr/带宽/相位裕度.
  * 只修改此文件 — 不触碰任何驱动库源码.
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

/* ---------- 功率级参数 ---------- */
#define VDC_BUS           30.0f    /* 直流母线电压 (V)                          */
#define L1_VAL             0.001f   /* 逆变器侧电感 (H)                          */
#define L2_VAL             0.001f   /* 负载侧电感 (H)                            */
#define C_VAL              0.000002f /* 滤波电容 (F), f_res = 1/(2π)*√((L1+L2)/(L1*L2*C)) ≈ 7120Hz */

/* ---------- 输出规格 ---------- */
#define VREF_RMS_INIT       20.0f    /* 上电默认输出 RMS (V), 调试后改为目标值     */
#define F_OUT              50.0f    /* 输出基波频率 (Hz)                          */
#define I_MAX_RMS           10.0f    /* 过流保护阈值 (A RMS), 触发后强制 Vref=1V   */
#define VREF_RMS_MIN        20.0f    /* 旋钮可调下限 (V)                           */
#define VREF_RMS_MAX       30.0f    /* 旋钮可调上限 (V)                           */
#define VREF_ENC_STEP       1.0f    /* 旋钮每格电压变化量 (V)                     */

/* ---------- 控制性能目标 ----------
 *  f_c_target : 期望开环交叉频率 (Hz). 越大→响应越快, 但过近 f_res 会振荡.
 *               经验: f_c_target < min(f_res/4, f_sw/20) ≈ min(1780, 5000)
 *  pm_target  : 期望相位裕度 (°), ≥45° 推荐.
 *  f_bw_res   : 基波PR谐振带宽 (Hz). 窄带(1~3Hz)→精确跟踪, 宽带(5~10Hz)→鲁棒.
 *  err_pct    : 期望稳态跟踪误差 (%). 值越小→Kr越大→50Hz处开环增益越高.
 *  f_lpf_fb   : 反馈ADC低通截止 (Hz). 用于衰减LCL谐振峰, 建议 f_res/3~f_res/5.
 * ---------- */
#define F_C_TARGET         200.0f    /* 目标交叉频率 (Hz), 极保守, 调试后可提高    */
#define PM_TARGET           75.0f    /* 目标相位裕度 (°)                           */
#define F_BW_RES             5.0f    /* 基波PR谐振带宽 (Hz)                        */
#define ERR_TRACK_PCT        3.0f    /* 期望跟踪误差 (%), 3%→开环增益≈33倍         */
#define F_LPF_FB           600.0f    /* 反馈LPF截止 (Hz), 深度滤除LCL谐振峰        */

/* ---------- 反馈极性 ----------
 * +1: ADC读数正→电压正, -1: 翻转. 若闭环发散/振荡→改此值.
 * ---------- */
#define PHASE_POLARITY        1

/* ---------- 多谐振控制器 (消除 3/5/7 次谐波) ----------
 * HARM_COUNT=0 可完全禁用谐波补偿以提高稳定性.
 * 每个谐振器结构与基波PR相同, 独立带宽和增益.
 * ---------- */
#define HARM_COUNT             3     /* 谐波数量                                    */
#define HARM_FREQ_1         150.0f   /* 3次谐波 (Hz)                                */
#define HARM_FREQ_2         250.0f   /* 5次谐波 (Hz)                                */
#define HARM_FREQ_3         350.0f   /* 7次谐波 (Hz)                                */
#define HARM_FREQ_4           0.0f   /* 备用, 0=禁用                                 */
#define HARM_FREQ_5           0.0f   /* 备用, 0=禁用                                 */
#define HARM_FREQ_6           0.0f   /* 备用, 0=禁用                                 */
#define F_BW_HARM           15.0f    /* 谐波谐振带宽 (Hz), 比基波宽以增强鲁棒性      */
#define HARM_KR_FACTOR       5.0f    /* 谐波 Kr = 基波 Kp × 此系数, 降低避免过调     */

/* ---------- ADC 触发: 1=在PWM中点触发, 0=周期起点(PERIOD事件, 推荐) ---------- */
#define ADC_SYNC_SAMPLE        0

/* ==========================================================================
 *  以下为自动推导参数 — 由 Hbridge_ComputeParams() 使用, 无需手动修改
 * ========================================================================== */

/* ---------- PWM 时基 ----------
 * f_HRTIM = 170MHz, DIV1 prescaler.  f_sw = 170MHz / (1699+1) = 100kHz.
 * 上行计数模式 (up-only), 边缘对齐PWM.  Preload=ON, 周期边界更新CMP.
 * ---------- */
#define HRTIM_PRESCALER     HRTIM_PRESCALERRATIO_DIV1
#define MASTER_PERIOD       1699U
#define PWM_PERIOD           MASTER_PERIOD
#define PWM_PERIOD_F         ((float)(MASTER_PERIOD + 1U))    /* 1700.0, CMP = duty × 1700  */
#define F_SW                100000.0f

/* 死区: DIV2 → f_DTG = 170/2 = 85MHz, DT = 34/85MHz ≈ 400ns */
#define DT_PRESCALER        HRTIM_TIMDEADTIME_PRESCALERRATIO_DIV2
#define DT_VALUE            34U

/* ---------- 控制 ISR (TIM6)  ----------
 * TIM6 挂在 APB1 @170MHz (HCLK/1).
 * ISR频率 = 170MHz / ((PSC+1)*(ARR+1)) = 170MHz / (170*20) = 50kHz, Ts=20µs.
 * ---------- */
#define TIM6_PSC             169U
#define TIM6_ARR              19U
#define ISR_FREQ           50000.0f
#define ISR_TS              (1.0f / ISR_FREQ)                 /* 20µs              */

/* ---------- 正弦查表 ----------
 * 256点表 + 1个额外点 (sin(2π)=0) 用于线性插值 wrap-around.
 * 相位累加器: uint32_t, 0→2³²映射到0→2π, 上溢自动wrap, 零长期漂移.
 * 表索引: bits[31:24], 插值小数: bits[23:16].
 * ---------- */
#define SIN_TABLE_SIZE       256U
#define SIN_INV_256          (1.0f / 256.0f)
#define INV_TWOPI            ((float)(2.0 * 3.14159265358979323846))   /* = 2π, 注意命名 */
#define INV_SQRT2            ((float)(1.4142135623730951))             /* = √2, RMS↔峰值  */

/* ---------- 硬限幅 ----------
 * 调制比 d_mod ∈ [-0.9, 0.9], 对应占空比 duty ∈ [0.05, 0.95].
 * 两级限幅: d_mod 先钳位, duty 再钳位 (冗余安全).
 * ---------- */
#define DUTY_MIN             0.05f
#define DUTY_MAX             0.95f
#define D_MOD_MAX            0.90f

/* ---------- 寄存器快捷写 ----------
 * CMP_WR 直接写 HRTIM CMP1xR 寄存器. Preload=ON 时写入preload,
 * 在计数器归零时自动转移到active寄存器.
 * ---------- */
#define HRTIM_IDX_A          HRTIM_TIMERINDEX_TIMER_A
#define HRTIM_IDX_B          HRTIM_TIMERINDEX_TIMER_B
#define CMP_WR(idx, val)     (HRTIM1->sTimerxRegs[(idx)].CMP1xR = (uint16_t)(val))

/* ---------- ADC RMS 显示窗口 ----------
 * 每1ms采1个样, 200个样=200ms窗口=10个50Hz周期, 覆盖整数倍周期避免纹波.
 * ---------- */
#define RMS_WINDOW_MS         200U
#define RMS_SAMPLE_MS           1U
#define RMS_SAMPLE_CNT       (RMS_WINDOW_MS / RMS_SAMPLE_MS)

/* USER CODE END PD */
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* ==========================================================================
 *  运行时变量 — static持久化, 部分用volatile标记多上下文访问
 * ========================================================================== */
/* USER CODE BEGIN PV */

/* ---- 正弦表: init一次写入, ISR只读 ---- */
static float sin_tab[SIN_TABLE_SIZE + 1U];                   /* 257点, 最后点=sin(2π)=0 */

/* ---- 相位累加器: uint32_t无漂移, ISR写step ---- */
static volatile uint32_t s_phase_acc = 0U;                  /* 当前相位, 0→2³²映射0→2π  */
static uint32_t s_phase_step;                                /* 每ISR步进值, ComputeParams中计算 */

/* ---- 目标电压: 主循环写, ISR通过s_vref_peak间接读 ---- */
static volatile float s_vref_rms = VREF_RMS_INIT;

/* ---- 控制系数: ComputeParams写入, ISR只读 ---- */
static float s_w0;              /* ω₀ = 2π·F_OUT (rad/s)                          */
static float s_w0_sq;           /* ω₀²                                              */
static float s_wc_res;          /* ωc_res = 2π·F_BW_RES, PR谐振带宽 (rad/s)       */
static float s_two_wc_res;      /* 2·ωc_res, 减少ISR中乘法                        */

/* ---- 基波PR Tustin离散化系数 (Ts=20µs) ---- */
static float s_pr_a11, s_pr_a12, s_pr_a21, s_pr_a22, s_pr_b1;
static float s_Kp;              /* 比例增益 (mod/V), 即每V误差产生的调制量         */
static float s_Kr;              /* 谐振增益 (mod/V), 50Hz处Kp+Kr实现目标跟踪精度   */
static float s_lpf_alpha;       /* 反馈LPF系数, α=2π·f_lpf·Ts, ≤1.0              */
static volatile float s_vref_peak = 0.0f;  /* Vref_rms×√2, ISR读主循环写          */

/* ---- 多谐振控制器状态: ISR独占 ---- */
#if HARM_COUNT > 0
static float s_harm_w0[HARM_COUNT];      /* 各谐波谐振频率 (rad/s)                 */
static float s_harm_w0_sq[HARM_COUNT];   /* ω₀²                                   */
static float s_harm_wc;                  /* 谐波谐振带宽 (rad/s)                    */
static float s_harm_two_wc;              /* 2·ωc_harm                              */
static float s_harm_Kr;                  /* 谐波谐振增益 (mod/V)                    */
static float s_harm_x1[HARM_COUNT];      /* 谐波状态x1                              */
static float s_harm_x2[HARM_COUNT];      /* 谐波状态x2                              */

/* ---- 谐波PR Tustin离散化系数 ---- */
static float s_harm_a11[HARM_COUNT];     /* Tustin系数 a11, 各谐波独立               */
static float s_harm_a12[HARM_COUNT];     /* Tustin系数 a12 (=b2), 各谐波独立          */
static float s_harm_a21[HARM_COUNT];     /* Tustin系数 a21, 各谐波独立               */
static float s_harm_a22[HARM_COUNT];     /* Tustin系数 a22, 各谐波独立               */
static float s_harm_b1[HARM_COUNT];      /* Tustin系数 b1=a12*Ts/2, 各谐波独立       */
#endif

/* ---- 基波PR控制器状态: ISR独占 ---- */
static float s_pr_x1     = 0.0f;         /* 基波状态x1                              */
static float s_pr_x2     = 0.0f;         /* 基波状态x2                              */

/* ---- 反馈LPF状态: ISR独占 ---- */
static float s_vfb_filt  = 0.0f;         /* 滤波后反馈电压                          */

/* ---- ADC RMS 计算: 主循环独占 ---- */
static float    s_v_sum_sq    = 0.0f;    /* Σ(V²) 累加器                            */
static float    s_i_sum_sq    = 0.0f;    /* Σ(I²) 累加器                            */
static uint16_t s_rms_cnt     = 0U;      /* 累加计数器                              */
static          float s_v_rms = 0.0f;    /* 输出电压RMS (V), OLED显示                */
static          float s_i_rms = 0.0f;    /* 输出电流RMS (A), OLED显示+过流保护       */
static uint32_t s_adc_last_ms = 0U;      /* 上次采样时刻 (ms)                        */

/* ---- OLED: 主循环独占 ---- */
static uint32_t s_oled_timer  = 0U;      /* 上次刷新时刻 (ms)                        */
static uint8_t  s_oled_dirty  = 1U;      /* 1=需要刷新                               */

/* ---- 诊断值: ComputeParams写入, 仅用于参考 ---- */
static float s_f_res_est;                /* 估算LCL谐振频率 (Hz)                     */
static float s_pm_est;                   /* 估算相位裕度 (°)                         */
static float s_ol_gain_50Hz;             /* 50Hz开环增益 (dB)                        */

/* USER CODE END PV */

void SystemClock_Config(void);

/* ==========================================================================
 *  HRTIM 辅助函数 — 封装 HAL 调用, 减少 main() 中重复代码
 * ========================================================================== */
/* USER CODE BEGIN 0 */

/*
 * Hbridge_WriteOut — 配置单个 HRTIM 输出的 Set/Reset 源.
 * 输出极性HIGH, 无burst/chopper, 空闲态inactive.
 * TA1/TB1: Set=TIMPER (计数器归零时置高), Reset=TIMCMP1 (比较匹配时置低)
 * TA2/TB2: Set=NONE, Reset=NONE → 由死区硬件自动生成互补波形
 */
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

/*
 * Hbridge_WriteDT — 配置死区.
 * 双沿均正延迟 → 两路输出都在上升沿延迟, 死区出现在两路之间.
 * Lock位设WRITE → 允许后续软件修改.
 */
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

/*
 * Hbridge_ConfigTimerB — Timer B 完整配置.
 * 设置上行计数模式, Preload=ON, 死区使能, ResetTrigger=NONE (自由运行).
 * 初始 CMP=(1699+1)/2=850 → 50%占空比, 上电0V差分输出.
 */
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
	cfg.PreloadEnable     = HRTIM_PRELOAD_ENABLED;         /* CMP在周期边界更新, 零毛刺 */
	cfg.RepetitionUpdate  = HRTIM_UPDATEONREPETITION_ENABLED;
	cfg.PushPull          = HRTIM_TIMPUSHPULLMODE_DISABLED;
	cfg.FaultEnable       = HRTIM_TIMFAULTENABLE_NONE;
	cfg.FaultLock         = HRTIM_TIMFAULTLOCK_READWRITE;
	cfg.DeadTimeInsertion = HRTIM_TIMDEADTIMEINSERTION_ENABLED;  /* 硬件自动生成TA2互补 */
	cfg.DelayedProtectionMode = HRTIM_TIMER_A_B_C_DELAYEDPROTECTION_DISABLED;
	cfg.UpdateTrigger     = HRTIM_TIMUPDATETRIGGER_NONE;
	cfg.ResetTrigger      = HRTIM_TIMRESETTRIGGER_NONE;          /* 自由运行, 不与Master同步 */
	cfg.ResetUpdate       = HRTIM_TIMUPDATEONRESET_DISABLED;
	s = HAL_HRTIM_WaveformTimerConfig(&hhrtim1, HRTIM_IDX_B, &cfg);
	if (s != HAL_OK) return s;

	cmp.CompareValue = (PWM_PERIOD + 1U) / 2U;   /* 850 → 50%占空比 */
	s = HAL_HRTIM_WaveformCompareConfig(&hhrtim1, HRTIM_IDX_B,
		HRTIM_COMPAREUNIT_1, &cmp);
	if (s != HAL_OK) return s;

	s = Hbridge_WriteDT(HRTIM_IDX_B);  if (s != HAL_OK) return s;
	s = Hbridge_WriteOut(HRTIM_IDX_B, HRTIM_OUTPUT_TB1,
		HRTIM_OUTPUTSET_TIMPER, HRTIM_OUTPUTRESET_TIMCMP1);
	if (s != HAL_OK) return s;
	s = Hbridge_WriteOut(HRTIM_IDX_B, HRTIM_OUTPUT_TB2,
		HRTIM_OUTPUTSET_NONE, HRTIM_OUTPUTRESET_NONE); /* 死区硬件自动生成互补 */
	if (s != HAL_OK) return s;

	return HAL_OK;
}

/*
 * Hbridge_BuildSinTable — 生成257点正弦查表.
 * sin_tab[i] = sin(2π·i/256), i=0..256.
 * i=256 → sin(2π)=0, 用于线性插值 wrap-around.
 */
static void Hbridge_BuildSinTable(void)
{
	uint16_t i;
	for (i = 0U; i <= SIN_TABLE_SIZE; i++)
		sin_tab[i] = sinf(INV_TWOPI * (float)i / (float)SIN_TABLE_SIZE);
}

/*
 * Hbridge_ComputeParams — 根据用户 #define 自动计算所有控制参数.
 *
 * 推导过程 (参考, 不参与运行时计算):
 *
 *   f_res = 1/(2π)·√((L1+L2)/(L1·L2·C))
 *         例: 1mH+1mH+1µF → ~7120 Hz
 *
 *   R_min = VREF_RMS_MAX / I_MAX_RMS     (最恶劣工况: 最大电压=最小负载电阻)
 *         例: 30V/7A ≈ 4.3Ω
 *
 *   被控对象 (简化L模型, f<<f_res时适用):
 *     G_plant(s) = Vdc * R / (R + s·L_total)     [V/mod]
 *
 *   Kp: 在交叉频率f_c处 |Kp·G_plant(jωc)| = 1
 *     g_filt_c = R/√(R²+ωc²·L²)
 *     Kp = 1 / (Vdc · g_filt_c)
 *
 *   Kr: 在50Hz处 (Kp+Kr)·G_plant(jω₀) ≥ 100/ERR_TRACK_PCT
 *     g_50 = R/√(R²+ω₀²·L²)
 *     (Kp+Kr)·Vdc·g_50 ≥ 100/ERR_TRACK_PCT
 *     Kr ∈ [Kp×5, Kp×200]   (上下限钳位)
 *
 *   相位裕度 (在f_c处):
 *     H_PR(jωc) = Kp + Kr·2ωc_res·jωc / (ω₀²-ωc² + 2ωc_res·jωc)
 *     G_filter滞后 = -atan(ωc·L/R)
 *     数字延迟 = -2π·f_c·1.5·Ts  (1.5拍 = 30µs)
 *     PM = 180° + ∠H_PR + ∠G_filter + ∠delay
 *
 *   LPF系数: α = 2π·f_lpf·Ts  (一阶IIR: y[n]=y[n-1]+α·(x[n]-y[n-1]))
 *            α ≤ 1.0 (无条件稳定)
 *
 *   相位累加器步进: step = 2³² × F_OUT / ISR_FREQ
 *     = 4294967296 × 50 / 50000 = 4294967  (每ISR约0.256个表索引)
 *
 *   谐波谐振器: 每个独立计算ω₀和ω₀², 共享统一的ωc_harm和Kr_harm
 */
static void Hbridge_ComputeParams(void)
{
	const float l_total   = L1_VAL + L2_VAL;                      /* 总电感 0.002H      */
	const float r_min     = VREF_RMS_MAX / I_MAX_RMS;              /* 最恶劣负载 ~4.3Ω   */
	const float ts        = ISR_TS;                                /* 采样周期 20µs      */

	/* ---- 基波频率 ---- */
	const float w0    = INV_TWOPI * F_OUT;                         /* 2π×50 = 314.16     */
	const float w0_sq = w0 * w0;

	/* ---- LCL谐振频率 ---- */
	const float f_res = (1.0f / INV_TWOPI)
	                  * sqrtf((L1_VAL + L2_VAL) / (L1_VAL * L2_VAL * C_VAL));

	/* ---- 交叉频率, 钳位到 f_res/4 以下 ---- */
	float f_c = F_C_TARGET;
	if (f_c > f_res * 0.25f) f_c = f_res * 0.25f;                 /* f_c ≤ ~1780Hz       */
	const float w_c = INV_TWOPI * f_c;

	/* ---- Kp: 在f_c处开环增益=1 ---- */
	const float g_filt_c = r_min / sqrtf(r_min*r_min + (w_c*l_total)*(w_c*l_total));
	s_Kp = 1.0f / (VDC_BUS * g_filt_c);
	if (s_Kp > 1.0f) s_Kp = 1.0f;                                  /* 防VDC_BUS过小导致Kp过大 */

	/* ---- Kr: 满足50Hz跟踪误差 ---- */
	const float g_50 = r_min / sqrtf(r_min*r_min + (w0*l_total)*(w0*l_total));
	const float need_gain_50 = 100.0f / ERR_TRACK_PCT;             /* 需要开环增益 ~33×    */
	const float need_pr_gain = need_gain_50 / (VDC_BUS * g_50);
	s_Kr = need_pr_gain - s_Kp;
	if (s_Kr < s_Kp * 5.0f) s_Kr = s_Kp * 5.0f;                   /* 下界: Kr≥Kp×5       */
	if (s_Kr > s_Kp * 200.0f) s_Kr = s_Kp * 200.0f;               /* 上界: 防ERR过小炸Kr  */

	/* ---- 基波PR谐振带宽 ---- */
	s_wc_res     = INV_TWOPI * F_BW_RES;                            /* 2π×5 = 31.4 rad/s   */
	s_two_wc_res = 2.0f * s_wc_res;

	/* ---- 基波Tustin离散化系数 (替换前向欧拉, 消除高频频率畸变) ---- */
	{
		const float h2 = ts * 0.5f;
		const float det = 1.0f + h2 * s_two_wc_res + h2 * h2 * s_w0_sq;
		s_pr_a11 = (1.0f + h2 * s_two_wc_res - h2 * h2 * s_w0_sq) / det;
		s_pr_a12 = ts / det;
		s_pr_a21 = (-ts * s_w0_sq) / det;
		s_pr_a22 = (1.0f - h2 * s_two_wc_res - h2 * h2 * s_w0_sq) / det;
		s_pr_b1  = s_pr_a12 * h2;
	}

	/* ---- 相位裕度估算 (在f_c处) ---- */
	{
		/* H_PR(jωc) = Kp + Kr·2ωc_res·jωc / (ω₀²-ωc² + 2ωc_res·jωc)   */
		const float dr   = w0_sq - w_c*w_c;
		const float di   = s_two_wc_res * w_c;
		const float num  = s_Kr * s_two_wc_res * w_c;
		const float msq  = dr*dr + di*di + 1e-30f;                 /* +ε防除零           */
		const float pr_re = s_Kp + num * di / msq;
		const float pr_im = num * dr / msq;
		const float pr_phi    = atan2f(pr_im, pr_re);              /* PR相位贡献          */
		const float filt_phi  = -atanf(w_c * l_total / r_min);     /* 滤波器相位滞后      */
		const float delay_phi = -INV_TWOPI * f_c * 1.5f * ts;     /* 数字延迟 ~30µs      */
		const float total_phi = pr_phi + filt_phi + delay_phi;
		s_pm_est = 180.0f + total_phi * 180.0f / (float)3.14159265358979323846;
	}

	/* ---- 反馈LPF系数: 一阶IIR, y+=α×(x-y) ---- */
	s_lpf_alpha = INV_TWOPI * F_LPF_FB * ts;                       /* 2π×600×20µs ≈ 0.075 */
	if (s_lpf_alpha > 1.0f) s_lpf_alpha = 1.0f;                   /* α>1→不稳定, 钳位    */

	/* ---- 相位累加器步进: 2³²×50/50000 ≈ 4294967 ---- */
	s_phase_step = (uint32_t)(4294967296.0f * F_OUT / ISR_FREQ);

	/* ---- 保存诊断值 ---- */
	s_w0           = w0;
	s_w0_sq        = w0_sq;
	s_f_res_est    = f_res;
	s_ol_gain_50Hz = 20.0f * log10f((s_Kp + s_Kr) * VDC_BUS * g_50);

	/* ---- 谐波谐振器参数 ---- */
#if HARM_COUNT > 0
	s_harm_wc     = INV_TWOPI * F_BW_HARM;                         /* 谐波带宽 2π×15     */
	s_harm_two_wc = 2.0f * s_harm_wc;
	s_harm_Kr = s_Kp * HARM_KR_FACTOR;
	if (s_harm_Kr > s_Kp * 50.0f) s_harm_Kr = s_Kp * 50.0f;       /* 谐波增益上限        */

	{
		/* 6个预定义谐波频率, for循环取前HARM_COUNT个 */
		const float all_harm_freq[6] = {
			HARM_FREQ_1, HARM_FREQ_2, HARM_FREQ_3,
			HARM_FREQ_4, HARM_FREQ_5, HARM_FREQ_6
		};
		for (uint8_t h = 0U; h < HARM_COUNT; h++) {
			s_harm_w0[h]    = INV_TWOPI * all_harm_freq[h];
			s_harm_w0_sq[h] = s_harm_w0[h] * s_harm_w0[h];
		}
	}


	/* 各谐波Tustin离散化系数 (独立计算, 替代前向欧拉) */
	{
		const float h2 = ts * 0.5f;
		for (uint8_t h = 0U; h < HARM_COUNT; h++) {
			const float det = 1.0f + h2 * s_harm_two_wc + h2 * h2 * s_harm_w0_sq[h];
			s_harm_a11[h] = (1.0f + h2 * s_harm_two_wc - h2 * h2 * s_harm_w0_sq[h]) / det;
			s_harm_a12[h] = ts / det;
			s_harm_a21[h] = (-ts * s_harm_w0_sq[h]) / det;
			s_harm_a22[h] = (1.0f - h2 * s_harm_two_wc - h2 * h2 * s_harm_w0_sq[h]) / det;
			s_harm_b1[h]  = s_harm_a12[h] * h2;
		}
	}
#endif
}

/*
 * Hbridge_ControlStep — 核心闭环控制, 由 TIM6 ISR 每 20µs 调用一次.
 *
 * 完整信号链 (按执行顺序):
 *
 *   ① 读ADC反馈
 *     vfb_raw = ADC_CH12 → 实际电压 (62.6倍分压, 1.65V中点)
 *     vfb_filt += α × (vfb_raw - vfb_filt)    ← 一阶IIR LPF, 截止600Hz
 *     vfb_sign = PHASE_POLARITY × vfb_filt     ← 极性翻转
 *
 *   ② 生成正弦参考
 *     phase_acc += phase_step                   ← uint32_t累加, 自wrap
 *     idx = phase_acc[31:24]                    ← 0~255 表索引
 *     frc = phase_acc[23:16] / 256              ← 插值小数
 *     sv = sin_tab[idx] + frc×(sin_tab[idx+1] - sin_tab[idx])  ← 线性插值
 *     vref_inst = s_vref_peak × sv              ← 瞬时参考电压
 *
 *   ③ 误差
 *     err = vref_inst - vfb_sign                ← 正→输出偏低需增大duty
 *
 *   ④ PR 控制器 (基波 + 谐波)
 *     基波 谐振器:
 *       ẋ₁ = x₂,  ẋ₂ = -ω₀²·x₁ - 2ωc_res·x₂ + err    (Tustin双线性变换)
 *       y_res = Kr × 2ωc_res × x₂_new
 *     谐波 谐振器 (×HARM_COUNT):
 *       同结构, 各用独立ω₀但共享ωc_harm和Kr_harm
 *       y_harm = Σ Kr_harm × 2ωc_harm × x₂_harm_i
 *     u_pr = Kp × err + y_res + y_harm            ← 总PR控制器输出 (mod)
 *
 *   ⑤ 前馈 + 限幅 + 抗饱和
 *     d_ff = vref_inst / VDC_BUS                  ← 理想前馈调制比
 *     raw  = d_ff + u_pr                          ← 总调制比
 *     d_mod = clamp(raw, ±0.9)                     ← 调制比硬限幅
 *     if d_mod被钳位 → 所有谐振状态冻结            ← 条件积分抗饱和
 *
 *   ⑥ 双极性 SPWM → CMP 寄存器
 *     da = 0.5 + 0.5 × d_mod,  clamp [0.05, 0.95]
 *     db = 0.5 - 0.5 × d_mod,  clamp [0.05, 0.95]
 *     cmp_a = da × 1700, cmp_b = db × 1700        ← Period=1699, scale=1700
 *     CMP_WR → preload寄存器 → 计数器归零时生效
 *
 *   ⑦ 状态钳位 (防数值漂移, 正常运行时不会触发)
 *     |x1| ≤ 100, |x2| ≤ 10000
 */
static void Hbridge_ControlStep(void)
{
	const float scale = PWM_PERIOD_F;         /* 1700.0, duty→CMP换算            */

	float vfb_raw, vfb_sign, vref_inst, err;
	float x1_old, x2_old, x1_new, x2_new, y_res, u_pr;
	float d_ff, d_mod;
	uint16_t cmp_a, cmp_b;

	/* ① 读反馈 → LPF → 极性 */
	vfb_raw = Sample_GetTheoreticalVoltage(Sample_CH12);
	s_vfb_filt += s_lpf_alpha * (vfb_raw - s_vfb_filt);
	vfb_sign = (float)PHASE_POLARITY * s_vfb_filt;

	/* ② 正弦参考: uint32_t相位累加 → 查表+线性插值 */
	s_phase_acc += s_phase_step;
	{
		uint32_t idx  = (s_phase_acc >> 24) & 0xFFU;
		uint32_t frac = (s_phase_acc >> 16) & 0xFFU;
		float    frc  = (float)frac * SIN_INV_256;
		float    sv   = sin_tab[idx]
		             + frc * (sin_tab[idx + 1U] - sin_tab[idx]);
		vref_inst = s_vref_peak * sv;
	}

	/* ③ 误差 */
	err = vref_inst - vfb_sign;

	/* ④ PR谐振器: 基波 + 谐波 */
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

			/* 基波谐振器: Tustin双线性变换, 状态空间 */
			{
				float b2_err = s_pr_a12 * err;
				x1_new = s_pr_a11 * x1_old + s_pr_a12 * x2_old + s_pr_b1 * err;
				x2_new = s_pr_a21 * x1_old + s_pr_a22 * x2_old + b2_err;
			}
			y_res  = s_Kr * s_two_wc_res * x2_new;

			/* 谐波谐振器: Tustin双线性变换, 各谐波独立 */
#if HARM_COUNT > 0
			for (uint8_t h = 0U; h < HARM_COUNT; h++) {
				float b2_err = s_harm_a12[h] * err;
				hx1_new[h] = s_harm_a11[h] * hx1_old[h] + s_harm_a12[h] * hx2_old[h] + s_harm_b1[h] * err;
				hx2_new[h] = s_harm_a21[h] * hx1_old[h] + s_harm_a22[h] * hx2_old[h] + b2_err;
				y_harm += s_harm_Kr * s_harm_two_wc * hx2_new[h];
			}
#endif


		u_pr = s_Kp * err + y_res + y_harm;

		/* ⑤ 前馈 + 调制限幅 + 抗饱和 */
		d_ff = vref_inst / VDC_BUS;
		{
			float raw    = d_ff + u_pr;
			uint8_t sat  = 0U;

			if      (raw >  D_MOD_MAX) { d_mod =  D_MOD_MAX; sat = 1U; }
			else if (raw < -D_MOD_MAX) { d_mod = -D_MOD_MAX; sat = 1U; }
			else                       { d_mod = raw; }

			/* 抗饱和: d_mod被钳位 → 谐振状态冻结不更新 */
			if (sat == 0U) {
				s_pr_x1 = x1_new;  s_pr_x2 = x2_new;
#if HARM_COUNT > 0
				for (uint8_t h = 0U; h < HARM_COUNT; h++) {
					s_harm_x1[h] = hx1_new[h];
					s_harm_x2[h] = hx2_new[h];
				}
#endif
				/* 状态幅值钳位: 防数值漂移, 阈值远大于正常值 */
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

	/* ⑥ 双极性SPWM → CMP */
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

/* ==========================================================================
 *  ADC RMS / OLED 显示 — 与闭环控制解耦, 仅在主循环运行
 * ========================================================================== */

/* RMS 累加器重置 */
static void ADC_RMS_Reset(void)
{ s_v_sum_sq = 0.0f;  s_i_sum_sq = 0.0f;  s_rms_cnt = 0U; }

/* 单次ADC采样并累加平方值 */
static void ADC_RMS_Sample(void)
{ float vi=Sample_GetTheoreticalVoltage(Sample_CH12);
  float ii=Sample_GetTheoreticalCurrent(Sample_CH4);
  s_v_sum_sq += vi*vi;  s_i_sum_sq += ii*ii;  s_rms_cnt++; }

/* 是否攒够200个样 (200ms窗口) */
static uint8_t ADC_RMS_Ready(void){ return (uint8_t)(s_rms_cnt >= RMS_SAMPLE_CNT); }

/* 计算RMS值并重置累加器 */
static void ADC_RMS_Compute(void)
{ s_v_rms = sqrtf(s_v_sum_sq/(float)s_rms_cnt);
  s_i_rms = sqrtf(s_i_sum_sq/(float)s_rms_cnt);  ADC_RMS_Reset(); }

/* 主循环调用: 每1ms采1次, 攒够200次后计算RMS并标记OLED刷新 */
static void ADC_RMS_Task(void)
{ uint32_t t=HAL_GetTick(); if((t-s_adc_last_ms)<RMS_SAMPLE_MS)return;
  s_adc_last_ms=t; ADC_RMS_Sample(); if(ADC_RMS_Ready()){ADC_RMS_Compute();s_oled_dirty=1U;} }

/* OLED 四行显示: 标题 / Vo=xx.xV / Io=x.xxA / Set xx.xV [####] */
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

/* ==========================================================================
 *  main — 系统初始化 + 主循环
 *
 *  初始化顺序 (每一步依赖前一步):
 *    1. HAL / 时钟 / 外设 (CubeMX生成)
 *    2. DMA IRQ 降优先级
 *    3. HRTIM Master 配置+启动
 *    4. HRTIM TimerA 完整配置 (上行计数, Preload, 死区, 输出)
 *    5. HRTIM TimerB 完整配置 (同TimerA)
 *    6. 启动所有定时器和输出
 *    7. 正弦表生成
 *    8. 控制参数自动计算
 *    9. 编码器 / LED
 *   10. ADC采样启动 (校准→注入→DMA)
 *   11. ADC触发 (PERIOD事件, CubeMX默认)
 *   12. OLED初始化+上电画面
 *   13. TIM6 ISR启动 (最后一步, 启动后闭环即开始运行)
 *
 *  主循环:
 *    - 编码器旋钮 → 调节 Vref_rms
 *    - ADC RMS 计算 (200ms滑动窗口)
 *    - OLED 刷新 (100ms间隔)
 *    - 过流保护 (Irms > I_MAX → Vref强制=1V)
 * ========================================================================== */
int main(void)
{
	HAL_Init();
	SystemClock_Config();
	MX_GPIO_Init(); MX_DMA_Init(); MX_ADC1_Init(); MX_HRTIM1_Init();
	MX_I2C1_Init(); MX_TIM3_Init(); MX_USART1_UART_Init(); MX_TIM6_Init();

	/* USER CODE BEGIN 2 */

	/* DMA IRQ 降到 prio=6, 低于控制 ISR 的 prio=5, 防止抢占抖动 */
	HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 6, 0);

	HRTIM_TimeBaseCfgTypeDef tb = {0};

	/* ---- 1. Master 定时器 ---- */
	tb.Period = MASTER_PERIOD;  tb.RepetitionCounter = 0;
	tb.PrescalerRatio = HRTIM_PRESCALER;  tb.Mode = HRTIM_MODE_CONTINUOUS;
	if (HAL_HRTIM_TimeBaseConfig(&hhrtim1,HRTIM_TIMERINDEX_MASTER,&tb)!=HAL_OK)
		Error_Handler();
	if (HAL_HRTIM_WaveformCounterStart(&hhrtim1,HRTIM_TIMERID_MASTER)!=HAL_OK)
		Error_Handler();

	/* ---- 2. Timer A (左桥臂: PA8/PA9) ---- */
	{
		HRTIM_TimerCtlTypeDef ctlA = {0};
		HRTIM_TimerCfgTypeDef cfgA = {0};

		/* 时基: Period=1699, DIV1, 上行计数 */
		tb.Period = PWM_PERIOD;
		if (HAL_HRTIM_TimeBaseConfig(&hhrtim1,HRTIM_IDX_A,&tb)!=HAL_OK) Error_Handler();

		/* 控制: 上行模式, CMP1相等匹配 */
		ctlA.UpDownMode = HRTIM_TIMERUPDOWNMODE_UP;
		ctlA.GreaterCMP1 = HRTIM_TIMERGTCMP1_EQUAL;
		ctlA.DualChannelDacEnable = HRTIM_TIMER_DCDE_DISABLED;
		if (HAL_HRTIM_WaveformTimerControl(&hhrtim1,HRTIM_IDX_A,&ctlA)!=HAL_OK) Error_Handler();

		/* Timer配置: Preload=ON, 死区=ON, 自由运行 */
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

		/* CMP初始值 = 50%占空比 → 0V差分 */
		{
			HRTIM_CompareCfgTypeDef cmpA = {0};
			cmpA.CompareValue = (PWM_PERIOD + 1U) / 2U;
			if (HAL_HRTIM_WaveformCompareConfig(&hhrtim1,HRTIM_IDX_A,
				HRTIM_COMPAREUNIT_1,&cmpA)!=HAL_OK) Error_Handler();
		}

		/* 死区 + 输出配置 */
		if (Hbridge_WriteDT(HRTIM_IDX_A)!=HAL_OK) Error_Handler();
		if (Hbridge_WriteOut(HRTIM_IDX_A,HRTIM_OUTPUT_TA1,
				HRTIM_OUTPUTSET_TIMPER,HRTIM_OUTPUTRESET_TIMCMP1)!=HAL_OK) Error_Handler();
		if (Hbridge_WriteOut(HRTIM_IDX_A,HRTIM_OUTPUT_TA2,
				HRTIM_OUTPUTSET_NONE,HRTIM_OUTPUTRESET_NONE)!=HAL_OK) Error_Handler();
	}

	/* ---- 3. Timer B (右桥臂: PA10/PA11) ---- */
	if (Hbridge_ConfigTimerB()!=HAL_OK) Error_Handler();

	/* ---- 4. 启动定时器和所有输出 ---- */
	HAL_HRTIM_WaveformCounterStart(&hhrtim1,HRTIM_TIMERID_TIMER_A);
	HAL_HRTIM_WaveformCounterStart(&hhrtim1,HRTIM_TIMERID_TIMER_B);
	HAL_HRTIM_WaveformOutputStart(&hhrtim1,HRTIM_OUTPUT_TA1);
	HAL_HRTIM_WaveformOutputStart(&hhrtim1,HRTIM_OUTPUT_TA2);
	HAL_HRTIM_WaveformOutputStart(&hhrtim1,HRTIM_OUTPUT_TB1);
	HAL_HRTIM_WaveformOutputStart(&hhrtim1,HRTIM_OUTPUT_TB2);

	/* ---- 5. 正弦查表 ---- */
	Hbridge_BuildSinTable();

	/* ---- 6. 自动计算所有控制参数 ---- */
	Hbridge_ComputeParams();

	/* ---- 7. 目标电压初始化 (在ISR启动前) ---- */
	s_vref_peak = s_vref_rms * INV_SQRT2;

	/* ---- 8. 编码器 + LED ---- */
	Encoder_Init();
	LED_SetLED1State(1U); LED_SetLED2State(0U);

	/* ---- 9. ADC 采样启动 (校准→注入→DMA) ---- */
	if (Sample_Init()!=SAMPLE_OK) { LED_SetLED2State(1U); }
	ADC_RMS_Reset();  s_adc_last_ms = HAL_GetTick();

	/* ---- 10. ADC触发 (CubeMX MASTER_PERIOD, 周期起点采样, 远离开关噪声) ---- */
/* 10b. [备用] 设为ADC_SYNC_SAMPLE=1时将触发改为MASTER_CMP1中点, 默认0使用MASTER_PERIOD */
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

	/* ---- 11. OLED 初始化 + 阻塞清屏 (绕过DMA验证I2C硬件) ---- */
	if (OLEDGFX_Init()==OLEDGFX_OK) {
		/* 阻塞I2C直接写全零帧到SSD1306 GDDRAM, 逐个命令发送 (与oledll.c一致) */
		{
			static uint8_t clr[1024];  /* BSS段零初始化 */
			uint8_t i;
			/* 设列地址 0-127: cmd 0x21 + data 0x00,0x7F */
			uint8_t c1=0x21U; HAL_I2C_Mem_Write(&hi2c1,0x78U,0x00U,I2C_MEMADD_SIZE_8BIT,&c1,1,100U);
			uint8_t d1[]={0x00U,0x7FU}; HAL_I2C_Mem_Write(&hi2c1,0x78U,0x40U,I2C_MEMADD_SIZE_8BIT,d1,2,100U);
			/* 设页地址 0-7: cmd 0x22 + data 0x00,0x07 */
			uint8_t c2=0x22U; HAL_I2C_Mem_Write(&hi2c1,0x78U,0x00U,I2C_MEMADD_SIZE_8BIT,&c2,1,100U);
			uint8_t d2[]={0x00U,0x07U}; HAL_I2C_Mem_Write(&hi2c1,0x78U,0x40U,I2C_MEMADD_SIZE_8BIT,d2,2,100U);
			/* 发送1024字节全零数据 (D/C#=1数据模式, 覆盖全部GDDRAM) */
			HAL_I2C_Mem_Write(&hi2c1,0x78U,0x40U,I2C_MEMADD_SIZE_8BIT,clr,sizeof(clr),200U);
		}
		/* GFX层同步初始化, 供主循环后续刷新使用 */
		OLEDGFX_Clear();
		OLEDGFX_Submit();
		OLEDGFX_Update();
	}
	/* ---- 12. TIM6 ISR 启动 (最后一步, 启动后闭环即刻运行) ---- */
	TIM6->PSC=TIM6_PSC; TIM6->ARR=TIM6_ARR; TIM6->EGR=TIM_EGR_UG; TIM6->SR=0U;
	HAL_NVIC_SetPriority(TIM6_DAC_IRQn,5U,0U); HAL_NVIC_EnableIRQ(TIM6_DAC_IRQn);
	TIM6->DIER|=TIM_DIER_UIE;  TIM6->CR1|=TIM_CR1_CEN;
	s_oled_timer=HAL_GetTick();

	/* USER CODE END 2 */

	while (1) {
	/* USER CODE BEGIN WHILE */
	/* USER CODE END WHILE */
	/* USER CODE BEGIN 3 */

		/* 编码器旋钮: 调节目标电压, 钳位到 [VREF_RMS_MIN, VREF_RMS_MAX] */
		{ int16_t d=Encoder_PopCount();
		  if(d){ float nv=s_vref_rms+(float)d*VREF_ENC_STEP;
		         if(nv>VREF_RMS_MAX)nv=VREF_RMS_MAX; else if(nv<VREF_RMS_MIN)nv=VREF_RMS_MIN;
		         s_vref_rms=nv; s_vref_peak = nv * INV_SQRT2; s_oled_dirty=1U; } }

		/* ADC RMS 计算: 每1ms采样, 200ms窗口 */
		ADC_RMS_Task();

		/* OLED 刷新: 每100ms更新一次, 其余时间仅驱动I2C传输 */
		{ uint32_t t=HAL_GetTick();
		  if(s_oled_dirty&&((t-s_oled_timer)>=100U)){s_oled_timer=t;s_oled_dirty=0U;OLED_Redraw();}
		  OLEDGFX_Update(); }

		/* 过流保护: Irms > I_MAX → 强制 Vref 降到最低, 手动恢复 */
		if(s_i_rms>I_MAX_RMS){ s_vref_rms=VREF_RMS_MIN; s_vref_peak=VREF_RMS_MIN*INV_SQRT2; s_oled_dirty=1U; }
	}
	/* USER CODE END 3 */
}

/*
 * SystemClock_Config — HSI→PLL→170MHz
 * HSI=16MHz, PLLM=/4→4MHz, PLLN=×85→340MHz, PLLR=/2→170MHz SYSCLK
 * HCLK=170MHz, PCLK1=170MHz, PCLK2=85MHz, FLASH=4WS
 */
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

/*
 * TIM6_DAC_IRQHandler — 控制 ISR, 50kHz.
 * 清除中断标志 → 执行闭环控制 → 每5000次翻转LED2 (10Hz心跳).
 */
void TIM6_DAC_IRQHandler(void)
{
	if((TIM6->SR&TIM_SR_UIF)!=0U){
		__HAL_TIM_CLEAR_FLAG(&htim6,TIM_FLAG_UPDATE);
		Hbridge_ControlStep();
		{ static uint16_t c; c++; if(c>=5000U){c=0U;
		  HAL_GPIO_TogglePin(LED2_GPIO_Port,LED2_Pin);} }
	}
}

/* I2C 回调: 转发给 OLED 驱动层 */
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
