#include "drv8313.h"
#include "board.h"

/* =============================================================================
 * DRV8313 硬件层 (CH32L103)
 *   PWM : TIM3 CH1/CH2/CH3 -> PA6 / PA7 / PB0, 中心对齐
 *   EN  : PB3 / PB4 / PB5
 *   FAULT: PC14 输入上拉
 * 参考: loop222 @郑州 的 SimpleFOC C 移植 (4_closeloop_M1/Hardware/timer.c)
 * ========================================================================== */

void DRV8313_Init(void)
{
    GPIO_InitTypeDef        gpio;
    TIM_TimeBaseInitTypeDef tb;
    TIM_OCInitTypeDef       oc;

    RCC_PB2PeriphClockCmd(RCC_PB2Periph_GPIOA | RCC_PB2Periph_GPIOB |
                          RCC_PB2Periph_GPIOC | RCC_PB2Periph_AFIO, ENABLE);
    RCC_PB1PeriphClockCmd(RCC_PB1Periph_TIM3, ENABLE);

    /* ---- PWM 输出: PA6(TIM3_CH1) / PA7(TIM3_CH2) / PB0(TIM3_CH3), 复用推挽 ---- */
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    gpio.GPIO_Mode  = GPIO_Mode_AF_PP;

    gpio.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
    GPIO_Init(GPIOA, &gpio);

    gpio.GPIO_Pin = GPIO_Pin_0;
    GPIO_Init(GPIOB, &gpio);

    /* ---- EN_A/B/C: 推挽输出, 先拉低 (高阻) ---- */
    gpio.GPIO_Mode = GPIO_Mode_Out_PP;
    gpio.GPIO_Pin  = GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5;
    GPIO_Init(GPIOB, &gpio);
    GPIO_ResetBits(GPIOB, GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5);

    /* ---- nFAULT: 输入上拉 ---- */
    gpio.GPIO_Mode = GPIO_Mode_IPU;
    gpio.GPIO_Pin  = GPIO_Pin_14;
    GPIO_Init(GPIOC, &gpio);

    /* ---- TIM3: 中心对齐, 20kHz (96MHz / (2*2400)) ---- */
    TIM_DeInit(TIM3);
    tb.TIM_Prescaler         = 0;
    tb.TIM_Period            = (uint16_t)DRV_PWM_ARR();
    tb.TIM_CounterMode       = TIM_CounterMode_CenterAligned1;
    tb.TIM_ClockDivision     = TIM_CKD_DIV1;
    tb.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM3, &tb);

    oc.TIM_OCMode      = TIM_OCMode_PWM1;
    oc.TIM_OutputState = TIM_OutputState_Enable;
    oc.TIM_OCPolarity  = TIM_OCPolarity_High;
    oc.TIM_Pulse       = 0;
    oc.TIM_OutputNState = TIM_OutputNState_Disable;
    oc.TIM_OCNPolarity  = TIM_OCNPolarity_High;
    oc.TIM_OCIdleState  = TIM_OCIdleState_Reset;
    oc.TIM_OCNIdleState = TIM_OCNIdleState_Reset;

    TIM_OC1Init(TIM3, &oc);
    TIM_OC2Init(TIM3, &oc);
    TIM_OC3Init(TIM3, &oc);

    TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);
    TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);
    TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Enable);
    TIM_ARRPreloadConfig(TIM3, ENABLE);

    TIM_Cmd(TIM3, ENABLE);
}

void DRV8313_Enable(void)
{
    GPIO_SetBits(GPIOB, GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5);
}

void DRV8313_Disable(void)
{
    TIM_SetCompare1(TIM3, 0);
    TIM_SetCompare2(TIM3, 0);
    TIM_SetCompare3(TIM3, 0);
    GPIO_ResetBits(GPIOB, GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5);
}

int DRV8313_IsEnabled(void)
{
    return (int)((GPIO_ReadOutputDataBit(GPIOB, GPIO_Pin_3) &&
                  GPIO_ReadOutputDataBit(GPIOB, GPIO_Pin_4) &&
                  GPIO_ReadOutputDataBit(GPIOB, GPIO_Pin_5)) ? 1 : 0);
}

static float last_duty[3];

void DRV8313_SetDuty(float ta, float tb, float tc)
{
    uint32_t arr = (uint32_t)DRV_PWM_ARR();

    if(ta < 0.0f) ta = 0.0f; else if(ta > 1.0f) ta = 1.0f;
    if(tb < 0.0f) tb = 0.0f; else if(tb > 1.0f) tb = 1.0f;
    if(tc < 0.0f) tc = 0.0f; else if(tc > 1.0f) tc = 1.0f;

    last_duty[0] = ta; last_duty[1] = tb; last_duty[2] = tc;

    TIM_SetCompare1(TIM3, (uint32_t)(ta * (float)arr));
    TIM_SetCompare2(TIM3, (uint32_t)(tb * (float)arr));
    TIM_SetCompare3(TIM3, (uint32_t)(tc * (float)arr));
}

void DRV8313_GetDuty(float *ta, float *tb, float *tc)
{
    *ta = last_duty[0]; *tb = last_duty[1]; *tc = last_duty[2];
}

uint8_t DRV8313_Faulted(void)
{
    return (GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_14) == Bit_RESET) ? 1 : 0;
}

/* ---- TIM4: 32 位自由运行计数器, 1MHz, 供 FOC 计算 Ts ---- */
void Micros_Init(void)
{
    TIM_TimeBaseInitTypeDef tb;

    RCC_PB1PeriphClockCmd(RCC_PB1Periph_TIM4, ENABLE);

    TIM_DeInit(TIM4);
    tb.TIM_Prescaler         = (uint16_t)(SystemCoreClock / 1000000u - 1u);
    tb.TIM_Period            = 0xFFFFFFFFu;
    tb.TIM_CounterMode       = TIM_CounterMode_Up;
    tb.TIM_ClockDivision     = TIM_CKD_DIV1;
    tb.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM4, &tb);

    TIM_Cmd(TIM4, ENABLE);
}

uint32_t Micros(void)
{
    return TIM4->CNT_TIM4;
}
