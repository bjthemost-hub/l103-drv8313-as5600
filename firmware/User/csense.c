#include "csense.h"
#include "wave.h"
#include "board.h"
#include "drv8313.h"
#include "foc_motor.h"
#include <math.h>

/* PA1 -> ADC_Channel_1 (W相/U10), PA4 -> ADC_Channel_4 (V相/U8) */
static const uint8_t cs_ch[CS_NCH] = { ADC_Channel_1, ADC_Channel_4 };

static float    cs_zero_v[CS_NCH];
static uint8_t  cs_zero_ok;

static uint32_t cs_last_us;
static uint32_t cs_win_start_us;

static float    cs_live_abs, cs_live_signed;   /* 正在累计的窗口 */
static float    cs_win_abs,  cs_win_signed;    /* 上一个已结算的窗口 */
static uint32_t cs_over_wins;

void CS_Init(void)
{
    ADC_InitTypeDef  adc  = {0};
    GPIO_InitTypeDef gpio = {0};
    uint8_t i;

    cs_zero_v[0] = CS_VREF_NOM;
    cs_zero_v[1] = CS_VREF_NOM;
    cs_zero_ok   = 0;

    cs_live_abs = cs_live_signed = 0.0f;
    cs_win_abs  = cs_win_signed  = 0.0f;
    cs_over_wins = 0;
    cs_last_us = cs_win_start_us = 0u;
    for(i = 0; i < CS_NCH; i++) (void)i;

    RCC_PB2PeriphClockCmd(RCC_PB2Periph_GPIOA | RCC_PB2Periph_ADC1, ENABLE);
    RCC_ADCCLKConfig(RCC_PCLK2_Div8);

    gpio.GPIO_Pin   = GPIO_Pin_1 | GPIO_Pin_4;
    gpio.GPIO_Mode  = GPIO_Mode_AIN;
    gpio.GPIO_Speed = GPIO_Speed_10MHz;
    GPIO_Init(GPIOA, &gpio);

    ADC_DeInit(ADC1);
    (void)Get_CalibrationValue(ADC1);

    adc.ADC_Mode               = ADC_Mode_Independent;
    adc.ADC_ScanConvMode       = DISABLE;
    adc.ADC_ContinuousConvMode = DISABLE;
    adc.ADC_ExternalTrigConv   = ADC_ExternalTrigConv_None;
    adc.ADC_DataAlign          = ADC_DataAlign_Right;
    adc.ADC_NbrOfChannel       = 1;
    adc.ADC_OutputBuffer       = ADC_OutputBuffer_Disable;
    adc.ADC_Pga                = ADC_Pga_1;
    ADC_Init(ADC1, &adc);

    ADC_Cmd(ADC1, ENABLE);

    ADC_ResetCalibration(ADC1);
    while(ADC_GetResetCalibrationStatus(ADC1) != RESET);
    ADC_StartCalibration(ADC1);
    while(ADC_GetCalibrationStatus(ADC1) != RESET);
}

/* 单次软件触发的转换; 带超时保护, 免得 ADC 没起来把主循环卡死 */
uint16_t CS_Raw(uint8_t idx)
{
    uint32_t guard = 200000u;

    if(idx >= CS_NCH) return 0u;

    ADC_ClearFlag(ADC1, ADC_FLAG_EOC);
    ADC_RegularChannelConfig(ADC1, cs_ch[idx], 1, ADC_SampleTime_CyclesMode7);
    ADC_SoftwareStartConvCmd(ADC1, ENABLE);

    while(ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == RESET)
    {
        if(--guard == 0u) return 0u;
    }

    return ADC_GetConversionValue(ADC1);
}

float CS_Volt(uint8_t idx)
{
    return (float)CS_Raw(idx) * CS_VDDA / CS_FULLSCALE;
}

float CS_Amp(uint8_t idx)
{
    if(idx >= CS_NCH) return 0.0f;
    return (CS_Volt(idx) - cs_zero_v[idx]) / CS_GAIN_V_PER_A;
}

float CS_ZeroVolt(uint8_t idx)
{
    return (idx < CS_NCH) ? cs_zero_v[idx] : 0.0f;
}

/* 零点标定: 平均 64 次。必须在 DRV8313 没使能(相电流为 0)的时候调用,
 * 否则标定出来的是"当时的电流", 后面所有判断都会偏。 */
void CS_CalZero(void)
{
    uint32_t i, n = 64u;
    float acc0 = 0.0f, acc1 = 0.0f;

    for(i = 0; i < n; i++)
    {
        acc0 += CS_Volt(0);
        acc1 += CS_Volt(1);
    }

    cs_zero_v[0] = acc0 / (float)n;
    cs_zero_v[1] = acc1 / (float)n;
    cs_zero_ok   = 1;
}

float CS_PhaseAmp(uint8_t ph)
{
    float iv, iw;

    iv = CS_Amp(1);
    iw = CS_Amp(0);

    if(ph == 0u) return -(iv + iw);      /* iU + iV + iW = 0 */
    if(ph == 1u) return iv;
    return iw;
}

float CS_PeakAmp(void)
{
    return (cs_live_abs > cs_win_abs) ? cs_live_signed : cs_win_signed;
}

float CS_PeakAbs(void)
{
    float a = cs_live_abs;
    return (cs_win_abs > a) ? cs_win_abs : a;
}

uint32_t CS_OverWins(void) { return cs_over_wins; }

void CS_ResetWins(void)
{
    cs_over_wins  = 0;
    cs_live_abs   = cs_live_signed = 0.0f;
    cs_win_abs    = cs_win_signed  = 0.0f;
    cs_win_start_us = Micros();
}

/* ---- 母线电压 VM (见 board.h: 本版硬件没接分压) ---- */
int CS_VM_Available(void)
{
#if VM_SENSE_ENABLE
    return 1;
#else
    return 0;
#endif
}

uint16_t CS_VM_Raw(void)
{
#if VM_SENSE_ENABLE
    uint32_t guard = 200000u;

    ADC_ClearFlag(ADC1, ADC_FLAG_EOC);
    ADC_RegularChannelConfig(ADC1, VM_ADC_CHANNEL, 1, ADC_SampleTime_CyclesMode7);
    ADC_SoftwareStartConvCmd(ADC1, ENABLE);
    while(ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == RESET)
    {
        if(--guard == 0u) return 0u;
    }
    return ADC_GetConversionValue(ADC1);
#else
    return 0u;
#endif
}

float CS_VM_Volt(void)
{
#if VM_SENSE_ENABLE
    return (float)CS_VM_Raw() * CS_VDDA / CS_FULLSCALE * VM_DIVIDER_RATIO;
#else
    /* 硬件没有采样: 只能回配置值(不是实测值), 免得上层把它当电压看 */
    return voltage_power_supply;
#endif
}

void CS_Task(void)
{
    uint32_t now = Micros();
    float    iv, iw, iu, best, cand;

    if(cs_zero_ok == 0u) return;

    if((uint32_t)(now - cs_last_us) < CS_TASK_MIN_US) return;
    cs_last_us = now;

    if(cs_win_start_us == 0u) cs_win_start_us = now;

    iw = CS_Amp(0);                      /* PA1 = W 相 (U10) */
    iv = CS_Amp(1);                      /* PA4 = V 相 (U8)  */
    iu = -(iv + iw);                     /* 三相无中线 */

    /* 取三相里 |i| 最大的那一相, 连符号一起记住 */
    best = fabsf(iu); cand = iu;
    if(fabsf(iv) > best) { best = fabsf(iv); cand = iv; }
    if(fabsf(iw) > best) { best = fabsf(iw); cand = iw; }

    if(best > cs_live_abs)
    {
        cs_live_abs    = best;
        cs_live_signed = cand;
    }

    if((uint32_t)(now - cs_win_start_us) >= (uint32_t)MOTOR_STALL_WIN_MS * 1000u)
    {
        cs_win_start_us = now;
        cs_win_abs      = cs_live_abs;
        cs_win_signed   = cs_live_signed;
        cs_live_abs     = 0.0f;
        cs_live_signed  = 0.0f;

        if(cs_win_abs > motor_stall_amp)
        {
            if(cs_over_wins < 1000u) cs_over_wins++;
        }
        else
        {
            cs_over_wins = 0;
        }
    }

    /* 波形流: 非阻塞塞帧, 关着时立刻返回 (见 wave.c/h) */
    Wave_Sample(iu, iv, iw);
}
