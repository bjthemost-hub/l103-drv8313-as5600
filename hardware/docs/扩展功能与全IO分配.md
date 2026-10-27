# CH32L103K8U6 全功能扩展：IO 分配与硬件接线
> 配套：`USB2CAN_接线图.svg`（USB/CAN 骨架）、`硬件连接与原理图设计.md`（核心板）、
> `../02_固件/固件与软件设计说明书.md` §8（固件 ext_io 模块）、`../04_工程/…/User/ext_io.*`。
> 本文档把 QFN32 的 **32+1 个焊盘全部排满**，是“整板”的唯一权威 IO 表。

## 0 一句话结论
- **能带 2 个舵机**：TIM4_CH1(PB6) + TIM4_CH2(PB7) 硬件 PWM，50Hz、0.5~2.5ms，
  不占 CPU、不影响 USB/CAN 中断，是最省事的方案。
- UART1(PA9/PA10)=Debug；UART2(PA2/PA3)=预留；4 路 ADC 全齐；
  WS2812=PB5（需注意 5V 电平与关中断时长）；状态 LED=PA15、错误 LED=PB4。
- 剩下 PA6/PA7/PB0/PB1/PA8/PB15/PB3/PC14/PC15/PD0/PD1 全部引到扩展排针。

## 1 QFN32 焊盘号（已按 CH32L103DS0 引脚表逐脚核对）
| 焊盘 | 引脚 | 引脚/主要复用功能 | 类型 |
|------|------|--------------|------|
| 0(EP) | VSS | 地（散热焊盘必须接地） | P |
| 1 | VDD | 3.3V | P |
| 2 | PC14 | OSC32_IN / 低速GPIO | I/O |
| 3 | PC15 | OSC32_OUT / 低速GPIO | I/O |
| 4 | PD0 | OSC_IN（HSI 方案可作 GPIO） | I/O |
| 5 | PD1 | OSC_OUT（同上） | I/O |
| 6 | PA0 | WKUP / ADC_IN0 | I/O/A |
| 7 | PA1 | ADC_IN1 | I/O/A |
| 8 | PA2 | USART2_TX / ADC_IN2 | I/O/A |
| 9 | PA3 | USART2_RX / ADC_IN3 | I/O/A |
| 10 | PA4 | ADC_IN4 | I/O/A |
| 11 | PA5 | ADC_IN5 / SPI1_SCK | I/O/A |
| 12 | PA6 | ADC_IN6 / SPI1_MISO | I/O/A |
| 13 | PA7 | ADC_IN7 / SPI1_MOSI | I/O/A |
| 14 | PB0 | ADC_IN8 | I/O/A |
| 15 | PB1 | ADC_IN9 | I/O/A |
| 16 | PB2 / BOOT1 | BOOT1（Boot 选择） | I |
| 17 | PB15 | TIM1_CH3N / GPIO | I/O |
| 18 | PA8 | TIM1_CH1 / GPIO | I/O |
| 19 | PA9 | USART1_TX | I/O |
| 20 | PA10 | USART1_RX | I/O |
| 21 | PA11 | USBDM（USB D−） | I/O |
| 22 | PA12 | USBDP（USB D+） | I/O |
| 23 | PA13 | SWDIO | I/O |
| 24 | PA14 | SWCLK | I/O |
| 25 | PA15 | GPIO | I/O |
| 26 | PB3 | GPIO / SPI1_SCK(重映射) | I/O |
| 27 | PB4 | GPIO / TIM3_CH1(重映射) | I/O |
| 28 | PB5 | GPIO / I2C1_SMBA | I/O |
| 29 | PB6 | TIM4_CH1 / I2C1_SCL | I/O |
| 30 | PB7 | TIM4_CH2 / I2C1_SDA | I/O |
| 31 | PB9 / BOOT0 | BOOT0（与 PB9 内部合封） | I/O* |
| 32 | PB8 | CAN1_RX | I/O |

> I/O*：PB9 与 BOOT0 合封，见 DS 注6 —— 需外接 4.7kΩ 下拉，且**不耐 5V**。

## 2 整板最终 IO 分配（权威表）
| 焊盘 | 引脚 | 用途 | 外部接线/说明 |
|------|------|------|----------------|
| 0 | VSS(EP) | 系统地 | Exposed Pad 大面积接地 |
| 1 | VDD | 3.3V | LDO 输出 |
| 21 | PA11 | USBDM | USB D−（22Ω 可选） |
| 22 | PA12 | USBDP | USB D+（22Ω 可选） |
| 23 | PA13 | SWDIO | WCH-Link |
| 24 | PA14 | SWCLK | WCH-Link |
| 32 | PB8 | CAN1_RX | 收发器 RXD |
| 31 | PB9/BOOT0 | CAN1_TX | 收发器 TXD + **4.7kΩ 下拉**；收发器必须 3.3V |
| 16 | PB2/BOOT1 | BOOT1 | 10kΩ 下拉；可兼作“输入型”备用 IO（接按键需上拉到 3.3V 而非强驱） |
| 19 | PA9 | USART1_TX | **Debug 口**：4P 排针 3V3/TX/RX/GND，115200 8N1 |
| 20 | PA10 | USART1_RX | 同上（可不接） |
| 8 | PA2 | USART2_TX | **UART2 预留**：2P/排针引出（固件默认不初始化） |
| 9 | PA3 | USART2_RX | 同上 |
| 6 | PA0 | ADC0 | VR1：10kΩ 电位器分压 0..3.3V |
| 7 | PA1 | ADC1 | VR2：同上 |
| 10 | PA4 | ADC4 | VR3：同上 |
| 11 | PA5 | ADC5 | 电池电压：分压电阻采样（见 §3.3） |
| 25 | PA15 | 状态 LED | 心跳 0.5s 翻转（固件已配），串 470Ω~1kΩ → VDD |
| 27 | PB4 | 错误 LED | 出错/丢帧 20ms 脉冲（可选），接法同上 |
| 28 | PB5 | WS2812 DATA | 电平转换后接灯带 DIN（见 §3.4） |
| 29 | PB6 | TIM4_CH1 | SERVO1 信号（3P 排针：V+/GND/S） |
| 30 | PB7 | TIM4_CH2 | SERVO2 信号（3P 排针：V+/GND/S） |
| 12/13 | PA6/PA7 | 扩展头 ADC6/ADC7 | 可再接 2 路可调电阻或传感器（SPI1 也可在此） |
| 14/15 | PB0/PB1 | 扩展头 ADC8/ADC9 | 同上（OPA 输入也可复用） |
| 18 | PA8 | 扩展头 | TIM1_CH1：可做第 3 路 PWM/WS2812 硬件方案/频率计输入 |
| 17 | PB15 | 扩展头 | TIM1_CH3N/GPIO |
| 26 | PB3 | 扩展头 | GPIO（输入优先） |
| 2/3 | PC14/15 | 扩展头 | ⚠ 低速：仅 2MHz、驱动≤3mA（DS 注），只接慢速信号/小 LED |
| 4/5 | PD0/PD1 | 扩展头 | 本方案 HSI 96MHz 不用外部晶振，可作 GPIO；若日后加晶振则此二脚被占 |

> “用满”含义：除固定功能外，所有 GPIO 都有明确去处（功能或排针），
> 板上不出现“空置焊盘”。排针建议做成 2×10 双排：`3V3 GND PA6 PA7 PB0 PB1 PA8 PB15 PB3 PC14 PC15 PD0 PD1 …`

## 3 各扩展接口电路

### 3.1 Debug / UART1 + 预留 UART2
```
J_DBG(4P):  [1]=3V3  [2]=PA9/TX(19)  [3]=PA10/RX(20)  [4]=GND
J_U2(4P):   [1]=3V3  [2]=PA2/TX(8)   [3]=PA3/RX(9)    [4]=GND
```
- UART1 固件 115200 8N1，开机打印系统时钟、运行统计（0.5s 一行）。
- UART2 默认关闭（GPIO 复位态）。要启用：自己初始化 USART2（PA2=TX/PA3=RX，
  库函数 `USART_Init` + GPIO AF），然后实现收发处理。
- ⚠ 不要直接调用 EVT 例程 `UART2_CfgInit()`：它会把 **PA15 配成“测试 IO”**，
  覆盖状态 LED 心跳（本工程没调用它）。两个串口共用 `Uart` 结构与 USB CDC 段缓冲，
  注意不要与桥链路抢上行带宽（建议 UART2 仅作调试/透传备用）。
- 若想省 RX：Debug 只留 TX 也行（不接不影响）。

### 3.2 三路 10kΩ 可调电阻（VR1..VR3）
```
3.3V ──┬── VR1(10k) ──┬── GND         3.3V ──┬── VR2(10k) ──┬── GND
       │               │                      │              │
      (1)  (滑臂→PA0) (3)                  (1) (滑臂→PA1) (3)
```
- 电位器两端接 3.3V 与 GND，滑臂接 ADC：`V = ADC/4095 × 3.3V`。
- PCB 上滑臂到 MCU 走线短，加 100nF 到地可滤波；不建议电位器直接长线进 ADC。
- 代码 `EXT_ADC_GetRaw(0..2)` / `EXT_ADC_GetVolt(0..2)`。

### 3.3 电池电压采样（ADC5/PA5）
```
电池+ ── R1(100k,1%) ──┬─── PA5(pad11)    电池−  ── GND（与 CAN GND 单点共地）
                      │
              R2(10k,1%) ── GND
                      可选：节点处 1kΩ 串阻 + 100nF 到地 + 3.3V TVS/稳压管保护
```
- 分压比 11:1：`Vbat = Vpin × 11`。
- 量程/分辨率（12bit）：满量程 36V；29.4V(7S Li 满电) → 2.67V，1 LSB ≈ 8.9mV@电池端。
- ⚠ 电池负极与板上 GND 必须“单点共地”（通过 CAN 线 GND 或电源端子连一次），
  否则分压参考点悬空、读数漂移甚至损坏。
- 换电池规格就换 R1/R2，并同步改 `ext_io.h` 的 `EXT_VBAT_DIVIDER`。
  例如 12V 系统可 R1=33k、R2=10k（4.3:1，满量程 14.2V）。
- 代码：`EXT_VBat_GetVolt()`。

### 3.4 WS2812（PB5/pad28）
- 数据口 = **PB5**（推挽 50MHz）。空闲必须低。
- **5V 灯带必须加电平转换**：MCU 3.3V 高电平对 5V 灯珠 VIH≈0.7×5V=3.5V 不达标，
  加一级 `74HCT1G125`（VCC=5V，输入 3.3V 兼容）或 NPN/2N7002 反相驱动。
  用 3.3V 供电的 WS2812 灯珠/单颗 WS2812B-3.3 可直连。
- 供电：5V 灯带电源不要取自 USB VBUS（500mA 限制），用独立 5V 电源，GND 共地。
- 时序/中断注意：固件位带发送会**关中断**，n 颗灯珠 ≈ n×31µs + 80µs。
  8 颗≈0.33ms / 16 颗≈0.58ms。1M CAN 帧间隔 ~130µs，灯效刷新期间恰好来帧会
  有极小概率丢失硬件 FIFO 溢出——常规灯效（低频刷新、≤16 颗）可用；
  对时序敏感的场合把灯珠数量控制到 ≤8，或把数据脚改到 PA8(TIM1_CH1) 用
  “定时器+DMA 逐 bit 刷 CCR”方案（参考实现见备注，接口已在排针预留）。
- 代码：`EXT_WS2812_SendGRB(grb, n)`；**首次上板需用示波器标定 `ext_io.c`
  顶部 W_T0H/W_T1H 等 4 个常量**（按 96MHz 粗估，余量充足）。

### 3.5 两个舵机（PB6=SV1 / PB7=SV2）
```
SV1: 3P 排针  [1]=5V(舵机电源)  [2]=GND  [3]=PB6(29)  TIM4_CH1
SV2: 3P 排针  [1]=5V(舵机电源)  [2]=GND  [3]=PB7(30)  TIM4_CH2
```
- 硬件 PWM：TIM4 50Hz（20ms），脉宽 0.5~2.5ms（1000~2000us≈常见 0~180°），
  1µs 分辨率。上电默认 1500us 中位，防止舵机猛跳。
- **供电严禁取自 USB 5V**：两个 9g 舵机堵转可达 1A+，用独立 5V/6V BEC 或稳压电源，
  排针处并 100~470µF 电解电容；舵机电流回路只在舵机电源处与系统 GND 相连。
- 舵机信号是 3.3V 逻辑，普通接收机舵机可直接驱动；个别“5V 逻辑”老舵机若高电平
  阈值偏高，可在信号线上加 74HCT1G125（VCC=5V）升压，成本 1 元。
- 舵机 PWM 与 USB/CAN 完全独立（TIM4 硬件产生），不影响 CAN 1M 实时性。
- 代码：`EXT_Servo_SetUs(0, us)` / `EXT_Servo_SetUs(1, us)`。

### 3.6 LED
- PA15 状态 LED：0.5s 翻转=系统正常（上电即有）。
- PB4 错误 LED：CAN TX 失败/丢帧/坏帧 20ms 脉冲。
- 均：LED 串 470Ω~1kΩ 接 VDD（高有效）。

## 4 BOOT0/PB9 复用为 CAN TX 的注意事项（回应“能行吗”）
**能行**，WCH 官方在 CH32L103K8U6 上就是这样定义的（DS 表注6）：
- pad31 = BOOT0 与 PB9 在**芯片内部短接合封**，没有独立 BOOT0 引脚；
- 必须外接 **4.7kΩ 下拉**，保证上电复位期间 BOOT0=0、从用户 Flash 启动；
- 因为合封引脚按 BOOT0 工艺处理，**PB9 不支持 5V 输入** —— CAN 收发器必须选
  3.3V 型（SIT1051T/3、TJA1051T/3、SN65HVD230），TXD 为输入方向由 PB9 驱动输出，
  平时为隐性高/显性低推挽，不会干扰 BOOT 采样（下拉只影响静态电平，不碍 1M 通信）。
- 固件已直接写 `AFIO_PCFR1[14:13]=10b` 把 CAN1 映射到 PB8/PB9，无需再动 BOOT。

## 5 软件集成（固件 ext_io 模块）
文件：`../04_工程/CH32L103_USB2CAN_DM/User/ext_io.h` / `ext_io.c`（已并入 Makefile）。
- `EXT_IO_Init()`：ADC1(PA0/1/4/5)、TIM4 舵机 PWM(PB6/PB7)、PB5=WS2812；
- `EXT_IO_Poll()`：主循环每 ~10ms 采 4 路 ADC（每路约 2µs 短阻塞，不影响链路）；
- `EXT_IO_Tick100us()`：由 TIM2 中断调用做调度（已在 `ch32l103_it.c` 挂好）；
- 读值：`EXT_ADC_GetRaw/GetVolt`、`EXT_VBat_GetVolt`；
- 舵机：`EXT_Servo_SetUs`；灯带：`EXT_WS2812_SendGRB`（默认不在主循环主动调用）。
- 若完全不需要扩展功能，删除 `main.c` 中 `EXT_IO_Init/Poll`、`it.c` 的
  `EXT_IO_Tick100us`、Makefile 中 `ext_io.c` 即可回到最小桥版本。
- 调试打印/USB-CAN 统计仍然走 UART1(PA9/PA10)，见固件说明书 §8。

## 6 扩展 BOM（增量）
| 位号 | 规格 | 数量 | 用途 |
|------|------|------|------|
| VR1..VR3 | 10kΩ 电位器/可调电阻 | 3 | 手调输入 |
| R1/R2 | 100kΩ+10kΩ 1% | 1+1 | 电池分压（按电池改） |
| J_DBG | 4P 2.54 排针 | 1 | UART1 |
| J_U2 | 4P 2.54 排针 | 1 | UART2 预留 |
| SV1/SV2 | 3P 2.54 排针 | 2 | 舵机（V/G/S） |
| U4 | 74HCT1G125 | 1(选) | WS2812 5V 电平转换 |
| D3 | WS2812B-3.3 或 5V 灯带+U4 | 1(选) | 指示/灯带 |
| C | 100nF | 4 | ADC 滤波×4 |
| C | 100~470µF | 1(选) | 舵机电源去耦 |