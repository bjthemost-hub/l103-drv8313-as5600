#ifndef __DRV8313_H
#define __DRV8313_H

#include "ch32l103.h"

/* DRV8313 真值表 (SLVSBA5D Table 1):
 *   INx = X, ENx = 0 -> OUTx = 高阻 Z
 *   INx = 0, ENx = 1 -> OUTx = 低
 *   INx = 1, ENx = 1 -> OUTx = 高
 *  => EN 常高, 在 IN 上打 PWM; 桥内自带死区, MCU 不需要互补 PWM.
 */

void     DRV8313_Init(void);                        /* TIM3 三相 PWM + EN + nFAULT */
void     DRV8313_Enable(void);                      /* EN_A/B/C = 1 */
void     DRV8313_Disable(void);                     /* EN_A/B/C = 0 (输出高阻) */
int      DRV8313_IsEnabled(void);                   /* 直接读 EN_A/B/C 引脚输出锁存 */
void     DRV8313_SetDuty(float ta, float tb, float tc);  /* 0.0 ~ 1.0, 已限幅 */
uint8_t  DRV8313_Faulted(void);                     /* 1 = nFAULT 被拉低 */
void     DRV8313_GetDuty(float *ta, float *tb, float *tc);  /* 最近一次占空比 */

/* 1MHz 自由运行计数器 (TIM4, 32bit) 用于 FOC 的 Ts 计算 */
void     Micros_Init(void);
uint32_t Micros(void);

#endif
