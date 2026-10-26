# 电赛第一次模拟题

## 项目概述

- **主控芯片**：STM32G474VE（Cortex-M4F，170 MHz）
- **开发框架**：PlatformIO + STM32Cube HAL
- **用途**：数字电源控制（半桥 / Buck / LLC 等拓扑）
- **关键外设**：HRTIM（高精度 PWM）、ADC1（DMA 多通道采样）、I2C OLED、编码器

---

## 文件结构

```
DIANSAI_1/
├── platformio.ini          # PlatformIO 项目配置（include路径、源文件过滤器）
├── README.md               # 本文件
├── include/
│   └── project_libs.h      # 聚合头文件，一行引入所有自定义库
├── Core/
│   ├── Inc/                # CubeMX 生成的头文件（main.h, adc.h, hrtim.h …）
│   └── Src/                # CubeMX 生成的源文件 + main.c
├── User/                   # 自定义外设驱动库
│   ├── PWM.h / PWM.c           # HRTIM 半桥 PWM 驱动
│   ├── encoder.h / encoder.c   # 旋转编码器驱动
│   ├── oledll.h / oledll.c     # OLED 底层 I2C 驱动（三缓冲DMA）
│   ├── oledgfx.h / oledgfx.c   # OLED 图形层（字符/数字/浮点绘制）
│   ├── oledFont.h              # 8×16 ASCII 字库
│   ├── buttonAndLED.h          # 按键读取 / LED 控制（static inline）
│   └── sample.h / sample.c     # ADC DMA 多通道采样
├── voloop/                 # 控制环路算法库
│   ├── core/
│   │   ├── Inc/                # voloop.h, voloop_pid.h, voloop_buck.h …
│   │   └── Src/                # PID / Buck / NCO / PLL 实现
│   └── bsp/
│       ├── common/             # 公共定义 + 字体
│       └── stm32g4/            # STM32G4 BSP 适配层
├── Drivers/                # STM32G4 HAL 驱动库（CMSIS + HAL）
└── boards/                 # PlatformIO 板级定义
```

---

## 如何在 main.c 中调用库函数

### 前提：只需引入一个头文件

```c
#include "project_libs.h"  // 聚合了所有自定义库 + voloop
```

main.c 已经默认包含了它。**你不需要修改任何库文件的源码**，只需要在 main.c 中写应用代码。

### 最小编译单元

`platformio.ini` 中的 `build_src_filter` 已经配置好：
- `Core/Src/*.c` — HAL 初始化和 main
- `User/*.c` — 所有自定义驱动
- `voloop/core/Src/*.c` + `voloop/bsp/stm32g4/Src/*.c` — 控制环路

HAL 驱动库由框架自动编译，无需手动配置。

---

## 各库 API 速查与操作逻辑

### 1. PWM（HRTIM 半桥驱动）— `PWM.h`

**文件**：`User/PWM.h` + `User/PWM.c`

**功能**：封装 STM32G4 HRTIM1 的 6 个定时器（A-F），每个输出两路互补 PWM（TA1/TA2 ~ TF1/TF2），支持死区插入。

**硬件引脚**：

| Timer | 输出1 (高侧) | 引脚 | 输出2 (低侧) | 引脚 |
|-------|-------------|------|-------------|------|
| PWM_TIMERA | HRTIM_CHA1 | PA8 | HRTIM_CHA2 | PA9 |
| PWM_TIMERB | HRTIM_CHB1 | PA10 | HRTIM_CHB2 | PA11 |
| PWM_TIMERC | HRTIM_CHC1 | PB12 | HRTIM_CHC2 | PB13 |
| PWM_TIMERD | HRTIM_CHD1 | PB14 | HRTIM_CHD2 | PB15 |
| PWM_TIMERE | HRTIM_CHE1 | PC8 | HRTIM_CHE2 | PC9 |
| PWM_TIMERF | HRTIM_CHF1 | PC6 | HRTIM_CHF2 | PC7 |

**API**：

```c
HAL_StatusTypeDef PWM_Init(PWM_TimerIdTypeDef Timer);          // 初始化并启动定时器
void PWM_Start(PWM_TimerIdTypeDef Timer);                      // 开启双路输出
void PWM_Stop(PWM_TimerIdTypeDef Timer);                       // 关闭双路输出
void PWM_SetDutyCycle(PWM_TimerIdTypeDef Timer, float duty);   // 设置占空比 (5%~95%)
void PWM_SetCompareRaw(PWM_TimerIdTypeDef Timer, uint16_t c);  // 直接写比较值
```

**重要**：所有 PWM 定时器的复位源都是 MASTER 定时器的周期事件 (`HRTIM_TIMRESETTRIGGER_MASTER_PER`)。因此在使用任何 `PWM_Init()` 之前，**必须先手动启动 MASTER 定时器**：

```c
// 1. 先启动 MASTER（只需一次）
HAL_HRTIM_WaveformCounterStart(&hhrtim1, HRTIM_TIMERID_MASTER);

// 2. 再初始化所需的从定时器
PWM_Init(PWM_TIMERA);
PWM_Init(PWM_TIMERB);
// ...
```

**操作逻辑**：
1. PWM_Init 配置时基（上下计数、预装载）、比较器、死区（10+10 cycles）、输出极性
2. 死区插入模式自动从 OUTPUT1 生成互补的 OUTPUT2
3. 占空比计算：`CMP1 = duty × (PERIOD + 1)`，硬件自动钳位到 [5%, 95%]
4. 预装载开启 → 运行时修改占空比无毛刺
5. 初始化后各定时器默认处于 STOP 状态，需调用 `PWM_Start()` 输出

---

### 2. Sample（ADC DMA 多通道采样）— `sample.h`

**文件**：`User/sample.h` + `User/sample.c`

**功能**：使用 ADC1 + DMA 连续采集 9 路模拟信号，并提供电压/电流换算。

**ADC 通道映射**：

| 枚举 | ADC 通道 | 物理引脚 |
|------|---------|---------|
| Sample_CH1 | ADC1_IN1 | PA0 |
| Sample_CH2 | ADC1_IN2 | PA1 |
| Sample_CH3 | ADC1_IN3 | PA2 |
| Sample_CH4 | ADC1_IN4 | PA3 |
| Sample_CH7 | ADC1_IN7 | PC1 |
| Sample_CH8 | ADC1_IN8 | PC2 |
| Sample_CH9 | ADC1_IN9 | PC3 |
| Sample_CH12 | ADC1_IN12 | PB1 |
| Sample_CH15 | ADC1_IN15 | PB0 |

**API**：

```c
SAMPLE_StatusTypeDef Sample_Init(void);                              // 启动 ADC + DMA
uint16_t Sample_GetRaw(Sample_ChannelTypeDef channel);               // 读取原始 ADC 值 (0-4095)
float Sample_GetPinVoltage(Sample_ChannelTypeDef channel);           // 引脚电压 (V)
float Sample_GetTheoreticalVoltage(Sample_ChannelTypeDef channel);   // 换算电压 (V)  — 系数 62.6
float Sample_GetTheoreticalCurrent(Sample_ChannelTypeDef channel);   // 换算电流 (A)  — 系数 3.049
```

**操作逻辑**：
1. ADC 由 HRTIM 硬件触发，DMA 循环写入 `SAMPLE_Buffer[9]`
2. `Sample_GetPinVoltage` = raw × (2.907V / 4096)
3. `Sample_GetTheoreticalVoltage` = 62.6 × (Vpin - 1.65V) — 用于分压电阻采样
4. `Sample_GetTheoreticalCurrent` = 3.049 × (Vpin - 1.65V) — 用于电流传感器
5. 每个通道对应哪个物理量（电压/电流）由外部电路决定，选择调用对应的换算函数即可

---

### 3. OLED（显示屏）— `oledll.h` + `oledgfx.h`

**文件**：`User/oledll.h/c`（底层 I2C）、`User/oledgfx.h/c`（图形层）、`User/oledFont.h`（字库）

**功能**：基于 SSD1306 的 128×64 OLED 驱动，支持三缓冲 DMA 异步刷新。

**oledll（底层）API**：

```c
OLEDLL_StatusTypeDef OLEDLL_Init(void);           // 初始化 OLED，发送配置命令序列
OLEDLL_StatusTypeDef OLEDLL_Clear(void);          // 清空当前绘制缓冲
OLEDLL_StatusTypeDef OLEDLL_WriteFrame(const uint8_t* buffer);  // 提交一帧到发送队列
OLEDLL_StatusTypeDef OLEDLL_Update(void);         // 将就绪帧通过 I2C DMA 发出
void OLEDLL_TxCpltCallback(void);                 // DMA 发送完成回调
void OLEDLL_ErrorCallback(void);                  // DMA 错误回调
```

**oledgfx（图形层）API**：

```c
OLEDGFX_StatusTypeDef OLEDGFX_Init(void);         // 初始化图形层（调 oledll_init + 清帧缓冲）
OLEDGFX_StatusTypeDef OLEDGFX_Clear(void);        // 清空帧缓冲
OLEDGFX_StatusTypeDef OLEDGFX_Submit(void);       // 将帧缓冲提交到 OLED 发送队列
OLEDGFX_StatusTypeDef OLEDGFX_Update(void);       // 触发一次 DMA 发送（需循环调用）

// 绘制函数（操作帧缓冲，调用 Submit 后才会发出）
void OLEDGFX_DrawPoint(uint16_t x, uint16_t y, uint8_t on);
void OLEDGFX_ShowChar(uint16_t x, uint16_t y, char c);
void OLEDGFX_ShowString(uint16_t x, uint16_t y, const char* str, OLEDGFX_TextModeTypeDef mode);
void OLEDGFX_ShowNum(uint16_t x, uint16_t y, uint32_t number, uint8_t length);
void OLEDGFX_ShowSignedNum(uint16_t x, uint16_t y, int32_t number, uint8_t length);
void OLEDGFX_ShowHexNum(uint16_t x, uint16_t y, uint32_t number, uint8_t length);
void OLEDGFX_ShowFloat(uint16_t x, uint16_t y, float number, uint8_t width, uint8_t precision);
```

**预定义坐标宏**：
```c
OLEDGFX_LINE_1 (0)  / OLEDGFX_LINE_2 (16) / OLEDGFX_LINE_3 (32) / OLEDGFX_LINE_4 (48)
OLEDGFX_COL_1 (0) ~ OLEDGFX_COL_16 (120)   // 步长 8 像素 = 1 字符宽
```

**典型使用流程**：

```c
OLEDGFX_Clear();                                      // 1. 清帧缓冲
OLEDGFX_ShowString(0, 0, "Vout:", OLEDGFX_Clip);      // 2. 绘制内容
OLEDGFX_ShowFloat(48, 0, voltage, 5, 2);              //
OLEDGFX_Submit();                                     // 3. 提交帧缓冲 → 就绪队列
OLEDGFX_Update();                                     // 4. 启动 DMA 发送（需循环调用）
```

**操作逻辑（三缓冲机制）**：
1. 3 个帧缓冲（fb0, fb1, fb2）分为 3 种角色：draw（绘图）、ready（就绪待发）、busy（DMA 占用）
2. `Submit()`：将 draw_buf 标记为 ready，分配新的 draw_buf
3. `Update()`：如果 I2C 空闲，将 ready_buf 标记为 busy，启动 DMA 发送
4. I2C DMA 完成后 `TxCpltCallback` 释放 busy_buf
5. **必须周期性调用 `OLEDGFX_Update()`**，它在 `while(1)` 中轮询即可

---

### 4. Encoder（旋转编码器）— `encoder.h`

**文件**：`User/encoder.h` + `User/encoder.c`

**功能**：基于 TIM3 编码器模式的旋转编码器读取，支持计数读取清零。

**API**：

```c
void Encoder_Init(void);              // 启动 TIM3 编码器模式
void Encoder_DeInit(void);            // 停止编码器
int16_t Encoder_GetCountRaw(void);    // 读取硬件原始计数值
int16_t Encoder_GetCount(void);       // 读取带极性和分频的计数值
void Encoder_SetCount(int16_t count); // 设置计数初值
void Encoder_Clear(void);             // 清零计数器
int16_t Encoder_PopCount(void);       // 原子读取增量并清零（临界区保护）
```

**配置宏**（在 `encoder.h` 中修改）：

```c
#define ENCODER_HANDLE      htim3          // 使用的 TIM 句柄
#define ENCODER_COUNT_SCALE 4U             // 分频系数（4 倍频）
#define ENCODER_POLARITY    -1             // 极性反转
```

**操作逻辑**：
- TIM3 配置为编码器模式，对 A/B 相脉冲计数
- `Encoder_GetCount()` = polarity × raw_count / scale
- `Encoder_PopCount()` 在临界区内读取计数并清零余数，适合周期性读取增量

---

### 5. Button & LED — `buttonAndLED.h`

**文件**：`User/buttonAndLED.h`（纯头文件，全部 static inline）

**按键 API**：

```c
uint8_t Button_GetButton1State(void);         // 返回 1=按下, 0=释放（含消抖逻辑为硬件RC）
uint8_t Button_GetButton2State(void);
uint8_t Button_GetButton3State(void);
uint8_t Button_GetButton4State(void);
uint8_t Button_GetEncoderButtonState(void);   // 编码器自带按键
```

**按键引脚**：Button1=PD2, Button2=PD1, Button3=PD0, Button4=PC12, EncoderBtn=PE4

**LED API**：

```c
void LED_SetLED1State(uint8_t state);   // 1=亮, 0=灭（低电平驱动）
void LED_SetLED2State(uint8_t state);
void LED_SetLED3State(uint8_t state);
void LED_SetLED4State(uint8_t state);
```

**LED 引脚**：LED1=PD6, LED2=PD5, LED3=PD4, LED4=PD3（低电平点亮）

---

### 6. voloop — 控制环路算法库

**文件**：`voloop/core/` + `voloop/bsp/`

#### 6.1 PID 控制器 — `voloop_pid.h`

**四种初始化模式**：
- `PID_Discrete` — 直接指定 Kp/Ki/Kd 离散系数
- `PID_Continue` — 连续域参数，内部自动离散化（根据 triggerFrequency）
- `PID_OneZero` — 一零点补偿器（gain + zero + triggerFrequency）
- `PID_TwoZero` — 二零点补偿器（gain + zero1 + zero2 + triggerFrequency）

**三种计算模式**：
- `VOLOOP_PID_Compute()` — 无抗饱和，纯线性
- `VOLOOP_PID_ComputeConditional()` — 条件抗饱和（冻结积分）
- `VOLOOP_PID_ComputeBackCalculation()` — 反算抗饱和（最平滑）

**API**：

```c
VOLOOP_PID_Init(&handle, &init);
VOLOOP_PID_Reset(handle);          // 清零积分和历史误差
float duty = VOLOOP_PID_ComputeConditional(handle, setpoint, measurement, min, max);
```

#### 6.2 Buck 控制器 — `voloop_buck.h`

**功能**：双环 Buck 变换器控制（外环电压 PID → 内环电流 PID → 占空比），含 OVP/OCP 保护。

**状态机**：BUCK_DISABLED → BUCK_CVMODE ⇄ BUCK_CCMODE → BUCK_ERROR

**API**：

```c
VOLOOP_Buck_Init(&hBuck, &init);             // 初始化，绑定 PWM/采样回调
VOLOOP_Buck_SetValue(hBuck, 12.0f, 2.0f);    // 设置目标 12V / 2A
VOLOOP_Buck_Start(hBuck);                    // 启动（进入 CV 模式）
VOLOOP_Buck_Sync(hBuck);                     // 每个控制周期调用一次
VOLOOP_Buck_Stop(hBuck);                     // 停止
```

**回调结构体 Buck_InitTypeDef**：

```c
typedef struct {
    void (*InitFunc)(void);                        // 用户提供的 PWM+ADC 初始化
    void (*DeInitFunc)(void);
    void (*Start)(void);                           // 开启 PWM 输出
    void (*Stop)(void);                            // 关闭 PWM 输出
    void (*SetDuty)(float duty);                   // 设置占空比
    float (*GetInputVoltage)(void);               // 读取输入电压
    float (*GetOutputVoltage)(void);              // 读取输出电压
    float (*GetInductorCurrent)(void);            // 读取电感电流
    PID_InitTypeDef* OutPutVoltagePIDInit;         // 电压环 PID 参数
    PID_InitTypeDef* InductorCurrentPIDInit;       // 电流环 PID 参数
} Buck_InitTypeDef;
```

**保护阈值**：OVP = 20V, OCP = 2A（可在 `voloop_buck.h` 中修改 `BUCK_OVTHRESHOLD` / `BUCK_OCTHRESHOLD`）

#### 6.3 NCO（数控振荡器）— `voloop_nco.h`

生成正弦/余弦信号，支持 Q1.31 定点相位查表。

#### 6.4 PLL（锁相环）— `voloop_pll.h`

基于 PID + NCO 的软件锁相环，用于电网同步 / 频率跟踪。

---

## 硬件引脚总览

### HRTIM PWM 输出
| Timer | CH1 引脚 | CH2 引脚 |
|-------|---------|---------|
| A | PA8 | PA9 |
| B | PA10 | PA11 |
| C | PB12 | PB13 |
| D | PB14 | PB15 |
| E | PC8 | PC9 |
| F | PC6 | PC7 |

### ADC 采样输入
| CH1 (PA0) | CH2 (PA1) | CH3 (PA2) | CH4 (PA3) | CH7 (PC1) | CH8 (PC2) | CH9 (PC3) | CH12 (PB1) | CH15 (PB0) |

### 人机交互
| 功能 | 引脚 | 说明 |
|------|------|------|
| LED1 | PD6 | 低电平亮 |
| LED2 | PD5 | 低电平亮 |
| LED3 | PD4 | 低电平亮 |
| LED4 | PD3 | 低电平亮 |
| Board LED | PC0 | 低电平亮 |
| Button1 | PD2 | 按下为低 |
| Button2 | PD1 | 按下为低 |
| Button3 | PD0 | 按下为低 |
| Button4 | PC12 | 按下为低 |
| Encoder A | PE2 | TIM3 CH1 |
| Encoder B | PE3 | TIM3 CH2 |
| Encoder Btn | PE4 | 按下为低 |

### 通信
| 外设 | 引脚 | 说明 |
|------|------|------|
| I2C1 SCL | PA15 | OLED 时钟 |
| I2C1 SDA | PB9 | OLED 数据 |
| USART1 TX | PB6 | 调试串口 |
| USART1 RX | PB7 | 调试串口 |

---

## 编译与烧录

```bash
# 编译
pio run

# 烧录（ST-Link）
pio run --target upload

# 串口监视
pio device monitor
```

---

## 扩展指南

1. **新增自定义驱动**：放在 `User/` 目录，头文件加入 `include/project_libs.h`，源文件由 `build_src_filter` 自动编译
2. **修改 CubeMX 配置**：重新生成代码到 `Core/`，注意保护 `USER CODE` 区域
3. **修改 sample 换算系数**：编辑 `sample.h` 中的 `SAMPLE_VoltageReference`，编辑 `sample.c` 中的 `Sample_GetTheoreticalVoltage/Current` 函数
4. **修改 PWM 频率/死区**：编辑 `PWM.h` 中的 `PWM_TIMER_PERIOD`、`PWM_TIMER_PRESCALER`、`PWM_DEADTIME_*`
