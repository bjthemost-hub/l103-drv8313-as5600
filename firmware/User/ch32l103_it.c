#include "ch32l103_it.h"
#include "board.h"

/* =============================================================================
 * 中断服务程序
 * 台架第 1 步不需要任何中断; 这里只留异常处理, 后面加 CAN/USB 再补.
 * ========================================================================== */

void NMI_Handler(void)        __attribute__((interrupt("WCH-Interrupt-fast")));
void HardFault_Handler(void)  __attribute__((interrupt("WCH-Interrupt-fast")));

void NMI_Handler(void)
{
    while(1) { }
}

/* 硬故障: 不再复位重启 (会造成 USB 枚举失败 / 串口乱码的假象),
 * 改成 PC15 (LED_ERR) 快闪, 用眼睛就能判断. */
void HardFault_Handler(void)
{
    GPIO_InitTypeDef gpio;

    RCC_PB2PeriphClockCmd(RCC_PB2Periph_GPIOC, ENABLE);
    gpio.GPIO_Pin   = GPIO_Pin_15;
    gpio.GPIO_Mode  = GPIO_Mode_Out_PP;
    gpio.GPIO_Speed = GPIO_Speed_10MHz;
    GPIO_Init(GPIOC, &gpio);

    while(1)
    {
        GPIO_SetBits(GPIOC, GPIO_Pin_15);
        {
            volatile uint32_t i;
            for(i = 0; i < 200000u; i++) { }
        }
        GPIO_ResetBits(GPIOC, GPIO_Pin_15);
        {
            volatile uint32_t i;
            for(i = 0; i < 200000u; i++) { }
        }
    }
}
