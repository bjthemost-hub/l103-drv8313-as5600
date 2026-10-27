#ifndef __CSENSE_H
#define __CSENSE_H

#include "ch32l103.h"

/* =============================================================================
 * INA240 相电流反馈采样 (ADC1)
 * -----------------------------------------------------------------------------
 * 芯片 : INA240A2PW, 增益 50 V/V, 采样电阻 R19/R20 = 10 mR  ->  0.5 V/A
 *   U10 -> 串在 W 相 (R20, OUT3)  -> U3.7  PA1 = ADC_Channel_1   MOT_A_V  (TP18)
 *   U8  -> 串在 V 相 (R19, OUT2)  -> U3.10 PA4 = ADC_Channel_4   MOT_B_V  (TP3)
 * 接法 : REF1=GND / REF2=+3V3 == 手册 8.4.2.2 推荐的 midscale 双向接法
 *        -> 零电流输出 = VS/2 = 1.65V (设计值)
 * 极性 : IN+ 在 DRV 侧、IN- 在电机侧 (手册: IN+ 接 shunt 的 supply side)
 *        -> 电流从 DRV 流向电机时, 输出高于 1.65V
 * 零点 : 不再硬用 1.65V。CS_CalZero() 在 DRV 还没使能时实测一次, 之后 CS_Amp()
 *        都减这个实测零点。上电自动做, 'U' 可以手动重做。
 * 说明 : "相电流 1Hz 采样读不出幅值" 是欠采样 (电频率 7~35Hz)。要做堵转保护,
 *        必须提高采样率: CS_Task() 在主循环里按 >=500us 的间隔连续采,
 *        并维护"窗口峰值", 由它去比 2A 阈值。
 * ========================================================================== */

#define CS_NCH             2
#define CS_VDDA            3.3f      /* ADC 参考 = 3V3 (设计值) */
#define CS_FULLSCALE       4095.0f   /* 12-bit */
#define CS_GAIN_V_PER_A    0.5f      /* 50 V/V * 10 mR (设计值) */
#define CS_VREF_NOM        1.65f     /* VS/2 (设计值, 仅作未标定时的兜底) */
#define CS_TASK_MIN_US     500u      /* 两次 ADC 采样之间的最小间隔 */

void     CS_Init(void);
uint16_t CS_Raw(uint8_t idx);        /* 0 = MOT_A_V(PA1,W相)  1 = MOT_B_V(PA4,V相) */
float    CS_Volt(uint8_t idx);       /* 引脚绝对电压 */
float    CS_Amp(uint8_t idx);        /* 正 = 电流由 DRV 流向电机 (已减实测零点) */

void     CS_CalZero(void);           /* 电机必须停转、DRV 未使能时做 */
float    CS_ZeroVolt(uint8_t idx);

void     CS_Task(void);              /* 主循环调用: 限速采样 + 维护窗口峰值 */
float    CS_PeakAmp(void);           /* 带回符号的峰值 (取 |i| 最大的那相) */
float    CS_PeakAbs(void);
uint32_t CS_OverWins(void);          /* 连续超阈值的窗口数 */
void     CS_ResetWins(void);
float    CS_PhaseAmp(uint8_t ph);    /* 0=U 1=V 2=W, U 由 iV/iW 反推 */

/* ---- 母线电压 VM ----
 * 本版硬件没接分压 (见 board.h), VM_SENSE_ENABLE=0 时:
 *   CS_VM_Available() = 0, CS_VM_Volt() 返回配置的 voltage_power_supply。
 * 装了分压再打开宏, 就是真采样。 */
int      CS_VM_Available(void);      /* 1 = 真的在读 ADC */
uint16_t CS_VM_Raw(void);
float    CS_VM_Volt(void);

#endif
