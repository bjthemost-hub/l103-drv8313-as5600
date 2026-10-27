#include "debug.h"
#include "ch32l103.h"

/* =============================================================================
 * 「只测 I2C」最小固件 + 单键自测  (CH32L103)
 *
 *   开:  系统时钟 + UART1(PA9 打印 / PA10 收键) + I2C1(PB6=SCL / PB7=SDA)
 *   不碰: TIM3(PWM) / TIM4 / EN_A/B/C / nFAULT / ADC / CAN / USB / PC15 / LED
 *
 *   UART1 (COM15, 115200 8N1) 发单个字符切换:
 *     a -> 方波模式: PB6/PB7 切成普通「开漏 GPIO」, 1Hz 方波, 两脚反相
 *          万用表都能量。用来判断 MCU 的脚到底有没有驱动出来。
 *     b -> 回连续 I2C 模式 (上电默认)
 *     c -> 慢速 I2C: 每 500ms 只发一次事务, 慢示波器/万用表也能抓
 *     s -> 打一行状态(只读寄存器, 不动引脚)
 *   其它字符忽略; 回车换行忽略。
 * ========================================================================== */

#define AS5600_ADDR     0x36
#define I2C_TO          200000u     /* 超时判据: 循环次数 (设计值, 非实测) */

#define MODE_I2C   0u
#define MODE_SQW   1u
#define MODE_SLOW  2u
#define MODE_READ  3u

static uint8_t g_mode      = MODE_I2C;
static uint8_t sqw_phase   = 0u;
static uint8_t g_step      = 0u;   /* \u8bfb\u5bc4\u5b58\u5668\u5361\u5728\u7b2c\u51e0\u6b65 (0x40 = \u5931\u8d25) */
static uint32_t g_i2c_speed = 400000u;   /* \u5bf9\u9f50\u5e95\u672c loop222 \u7684 400kHz */

/* ---------------------------------------------------------------- I2C 部分 */

static void i2c_clear_err(void)
{
    I2C_ClearFlag(I2C1, I2C_FLAG_AF);
    I2C_ClearFlag(I2C1, I2C_FLAG_BERR);
    I2C_ClearFlag(I2C1, I2C_FLAG_ARLO);
    I2C_ClearFlag(I2C1, I2C_FLAG_OVR);
}

static void i2c_init(void)
{
    GPIO_InitTypeDef gpio;
    I2C_InitTypeDef  i2c;

    RCC_PB2PeriphClockCmd(RCC_PB2Periph_GPIOA | RCC_PB2Periph_GPIOB | RCC_PB2Periph_AFIO, ENABLE);
    RCC_PB1PeriphClockCmd(RCC_PB1Periph_I2C1, ENABLE);

    gpio.GPIO_Pin   = GPIO_Pin_6 | GPIO_Pin_7;
    gpio.GPIO_Mode  = GPIO_Mode_AF_OD;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &gpio);

    I2C_DeInit(I2C1);
    i2c.I2C_ClockSpeed          = g_i2c_speed;  /* 对齐底本: 400kHz */
    i2c.I2C_Mode                = I2C_Mode_I2C;
    i2c.I2C_DutyCycle           = I2C_DutyCycle_2;
    i2c.I2C_OwnAddress1         = 0x00;
    i2c.I2C_Ack                 = I2C_Ack_Enable;
    i2c.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
    I2C_Init(I2C1, &i2c);
    I2C_Cmd(I2C1, ENABLE);
    i2c_clear_err();
}

static void i2c_set_speed(uint32_t spd)
{
    g_i2c_speed = spd;
    i2c_init();
}

/* 临时把两线切成输入, 读空闲电平, 再交还 AF 开漏 */
static void i2c_idle_levels(uint8_t *scl, uint8_t *sda)
{
    GPIO_InitTypeDef gpio;

    I2C_Cmd(I2C1, DISABLE);
    gpio.GPIO_Pin   = GPIO_Pin_6 | GPIO_Pin_7;
    gpio.GPIO_Mode  = GPIO_Mode_IN_FLOATING;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &gpio);
    Delay_Us(50);
    *scl = GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_6);
    *sda = GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_7);

    gpio.GPIO_Mode = GPIO_Mode_AF_OD;
    GPIO_Init(GPIOB, &gpio);
    I2C_Cmd(I2C1, ENABLE);
}

/* STOP + 等总线空: 后面要连续发, 不能把下一次 START 发在 STOP 没完成的时候 */
static void i2c_stop_wait(void)
{
    uint32_t t = 50000u;

    I2C_GenerateSTOP(I2C1, ENABLE);
    while((I2C_GetFlagStatus(I2C1, I2C_FLAG_BUSY) != RESET) && --t) { }
}

/* 只做 START + 地址: 0 = 有 ACK, 1 = NACK, -1 = START 都没发出去 */
static int i2c_ping(uint8_t addr7)
{
    uint32_t t;

    i2c_clear_err();

    I2C_GenerateSTART(I2C1, ENABLE);
    t = I2C_TO;
    while(!I2C_GetFlagStatus(I2C1, I2C_FLAG_SB) && --t) { }
    if(t == 0u) { i2c_stop_wait(); return -1; }

    I2C_Send7bitAddress(I2C1, (uint8_t)(addr7 << 1), I2C_Direction_Transmitter);
    t = I2C_TO;
    while(--t)
    {
        if(I2C_GetFlagStatus(I2C1, I2C_FLAG_ADDR) != RESET)
        {
            (void)I2C1->STAR2;                       /* 清 ADDR */
            i2c_stop_wait();
            return 0;
        }
        if(I2C_GetFlagStatus(I2C1, I2C_FLAG_AF) != RESET)
        {
            I2C_ClearFlag(I2C1, I2C_FLAG_AF);
            i2c_stop_wait();
            return 1;
        }
    }
    i2c_stop_wait();
    return -1;
}

/* reg -> buf[0..len-1]: 0 \u6210\u529f, -1 \u5931\u8d25; \u5931\u8d25\u65f6 g_step \u6307\u793a\u5361\u5728\u54ea\u4e00\u6b65 */
static int i2c_read_regs(uint8_t reg, uint8_t *buf, uint8_t len)
{
    uint32_t t;
    uint8_t  i;

    i2c_clear_err();

    g_step = 1u;                                     /* 1: \u53d1 START(\u5199) */
    I2C_GenerateSTART(I2C1, ENABLE);
    t = I2C_TO; while(!I2C_GetFlagStatus(I2C1, I2C_FLAG_SB) && --t) { }
    if(t == 0u) goto err;

    g_step = 2u;                                     /* 2: \u53d1 0x6C, \u7b49 ACK */
    I2C_Send7bitAddress(I2C1, (uint8_t)(AS5600_ADDR << 1), I2C_Direction_Transmitter);
    t = I2C_TO; while(!I2C_GetFlagStatus(I2C1, I2C_FLAG_ADDR) && --t) { }
    if(t == 0u) goto err;
    (void)I2C1->STAR2;

    g_step = 3u;                                     /* 3: \u53d1\u5bc4\u5b58\u5668\u53f7 */
    t = I2C_TO; while(!I2C_GetFlagStatus(I2C1, I2C_FLAG_TXE) && --t) { }
    if(t == 0u) goto err;
    I2C_SendData(I2C1, reg);

    g_step = 4u;                                     /* 4: \u7b49 BTF */
    t = I2C_TO; while(!I2C_GetFlagStatus(I2C1, I2C_FLAG_BTF) && --t) { }
    if(t == 0u) goto err;

    g_step = 5u;                                     /* 5: \u91cd\u590d START */
    I2C_GenerateSTART(I2C1, ENABLE);
    t = I2C_TO; while(!I2C_GetFlagStatus(I2C1, I2C_FLAG_SB) && --t) { }
    if(t == 0u) goto err;

    g_step = 6u;                                     /* 6: \u53d1 0x6D, \u7b49 ACK */
    I2C_Send7bitAddress(I2C1, (uint8_t)((AS5600_ADDR << 1) | 1u), I2C_Direction_Receiver);
    t = I2C_TO; while(!I2C_GetFlagStatus(I2C1, I2C_FLAG_ADDR) && --t) { }
    if(t == 0u) goto err;
    (void)I2C1->STAR2;

    g_step = 7u;                                     /* 7: \u6536\u6570\u636e */
    for(i = 0; i < len; i++)
    {
        if(i == (uint8_t)(len - 1u)) I2C_AcknowledgeConfig(I2C1, DISABLE);
        t = I2C_TO; while(!I2C_GetFlagStatus(I2C1, I2C_FLAG_RXNE) && --t) { }
        if(t == 0u) goto err;
        buf[i] = I2C_ReceiveData(I2C1);
    }

    I2C_GenerateSTOP(I2C1, ENABLE);
    I2C_AcknowledgeConfig(I2C1, ENABLE);
    g_step = 0u;
    return 0;

err:
    I2C_GenerateSTOP(I2C1, ENABLE);
    I2C_AcknowledgeConfig(I2C1, ENABLE);
    i2c_clear_err();
    g_step |= 0x40u;
    return -1;
}

/* 把几个速率都试一遍, 看从机在哪个速率下会应答 */
static void i2c_speed_probe(void)
{
    static const uint32_t spd[4] = { 100000u, 400000u, 1000000u, 50000u };
    uint8_t i;
    int r;

    printf("SPEED PROBE (AS5600 \u624b\u518c: \u6700\u5927 1MHz, \u4e0d\u652f\u6301\u65f6\u949f\u62c9\u4f38):\r\n");
    for(i = 0u; i < 4u; i++)
    {
        i2c_set_speed(spd[i]);
        Delay_Ms(2);
        r = i2c_ping(AS5600_ADDR);
        printf("  %7lu Hz -> %s\r\n", (unsigned long)spd[i],
               (r == 0) ? "ACK !!!" : ((r == 1) ? "NACK" : "START_FAIL"));
    }
    i2c_set_speed(400000u);
    Delay_Ms(2);
}

/* ------------------------------------------------- 引脚在 I2C / GPIO 间切换 */

static void pins_to_i2c(void)
{
    i2c_init();
}

/* \u5207\u6210\u666e\u901a\u5f00\u6f0f GPIO: \u53ea\u5f80\u4e0b\u62c9, \u4e0d\u4f1a\u548c\u4ece\u673a\u5bf9\u6253(\u4e0a\u62c9\u7531 R3/R4 4.7k \u63d0\u4f9b) */
static void pins_to_gpio(void)
{
    GPIO_InitTypeDef gpio;

    I2C_Cmd(I2C1, DISABLE);
    gpio.GPIO_Pin   = GPIO_Pin_6 | GPIO_Pin_7;
    gpio.GPIO_Mode  = GPIO_Mode_Out_OD;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &gpio);
    GPIO_SetBits(GPIOB, GPIO_Pin_6 | GPIO_Pin_7);       /* 先都释放 */
}

/* ------------------------------------------------------- 状态灯(PA15 LED_OK) */

static uint8_t led_on = 0u;

static void led_init(void)
{
    GPIO_InitTypeDef gpio;

    RCC_PB2PeriphClockCmd(RCC_PB2Periph_GPIOA, ENABLE);
    gpio.GPIO_Pin   = GPIO_Pin_15;          /* LED_OK */
    gpio.GPIO_Mode  = GPIO_Mode_Out_PP;
    gpio.GPIO_Speed = GPIO_Speed_10MHz;
    GPIO_Init(GPIOA, &gpio);
    GPIO_SetBits(GPIOA, GPIO_Pin_15);
}

static void led_toggle(void)
{
    led_on = (uint8_t)(led_on ? 0u : 1u);
    if(led_on) { GPIO_SetBits(GPIOA, GPIO_Pin_15); }
    else       { GPIO_ResetBits(GPIOA, GPIO_Pin_15); }
}

/* 总线卡住(从机拉住 SDA 等)时: 9 个时钟把从机宕位毕, 再补一个 STOP, 然后重初始化硬件 I2C */
static void bus_recover(void)
{
    GPIO_InitTypeDef gpio;
    uint8_t i;

    I2C_Cmd(I2C1, DISABLE);
    I2C_DeInit(I2C1);

    gpio.GPIO_Pin   = GPIO_Pin_6 | GPIO_Pin_7;
    gpio.GPIO_Mode  = GPIO_Mode_Out_OD;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &gpio);
    GPIO_SetBits(GPIOB, GPIO_Pin_6 | GPIO_Pin_7);   /* 两线都释放 */

    for(i = 0u; i < 9u; i++)
    {
        GPIO_ResetBits(GPIOB, GPIO_Pin_6);
        Delay_Us(5);
        GPIO_SetBits(GPIOB, GPIO_Pin_6);
        Delay_Us(5);
    }

    GPIO_ResetBits(GPIOB, GPIO_Pin_7);              /* STOP: SCL 高时 SDA 从低抬高 */
    Delay_Us(5);
    GPIO_SetBits(GPIOB, GPIO_Pin_6);
    Delay_Us(5);
    GPIO_SetBits(GPIOB, GPIO_Pin_7);
    Delay_Us(5);

    i2c_init();                                     /* 交还硬件 I2C */
}

/* ------------------------------------------------------------- UART1 收键 */

static void uart_rx_init(void)
{
    GPIO_InitTypeDef gpio;

    RCC_PB2PeriphClockCmd(RCC_PB2Periph_GPIOA, ENABLE);
    gpio.GPIO_Pin   = GPIO_Pin_10;
    gpio.GPIO_Mode  = GPIO_Mode_IN_FLOATING;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &gpio);

    USART1->CTLR1 |= (uint16_t)0x0004;      /* RE: 打开接收 */
}

static int uart_getc(void)
{
    if((USART1->STATR & 0x0020u) != 0u)     /* RXNE */
        return (int)(USART1->DATAR & 0x00FFu);
    return -1;
}

/* 只读寄存器, 不动引脚, 任何模式下都能安全调用 */
static void print_state(void)
{
    uint8_t pe, cfg6, cfg7, clk;

    pe   = (uint8_t)(I2C1->CTLR1 & 0x01u);
    cfg6 = (uint8_t)((GPIOB->CFGLR >> 24) & 0x0Fu);
    cfg7 = (uint8_t)((GPIOB->CFGLR >> 28) & 0x0Fu);
    clk  = ((RCC->PB1PCENR & RCC_PB1Periph_I2C1) != 0u) ? 1u : 0u;

    printf("STATE mode=%u PE=%u clk=%u STAR1=0x%04X STAR2=0x%04X CFGLR PB6=%Xh PB7=%Xh INDR PB6=%u PB7=%u\r\n",
           (unsigned)g_mode, (unsigned)pe, (unsigned)clk,
           (unsigned)I2C1->STAR1, (unsigned)I2C1->STAR2,
           (unsigned)cfg6, (unsigned)cfg7,
           (unsigned)((GPIOB->INDR >> 6) & 1u), (unsigned)((GPIOB->INDR >> 7) & 1u));
}

/* 处理一个按键; 返回 1 = 按到键了 */
static uint8_t key_step(void)
{
    int c = uart_getc();

    if(c < 0) return 0;
    if(c == '\r' || c == '\n' || c == ' ') return 0;

    if(c == 'a')
    {
        g_mode = MODE_SQW;
        pins_to_gpio();
        printf("\r\n[SWQ] PB6/PB7 = \u5f00\u6f0f GPIO, 1Hz \u65b9\u6ce2(\u4e24\u811a\u53cd\u76f8). PB6 \u5148\u9ad8 500ms\r\n");
    }
    else if(c == 'b')
    {
        g_mode = MODE_I2C;
        pins_to_i2c();
        printf("\r\n[I2C] \u8fde\u7eed\u53d1 0x6C\r\n");
    }
    else if(c == 'c')
    {
        g_mode = MODE_SLOW;
        pins_to_i2c();
        printf("\r\n[SLOW] \u6bcf 500ms \u4e00\u6b21\u4e8b\u52a1\r\n");
    }
    else if(c == 'r')
    {
        g_mode = MODE_READ;
        pins_to_i2c();
        printf("\r\n[READ] \u6bcf 100ms \u8bfb\u4e00\u6b21 AS5600 (STATUS/RAW/AGC)\r\n");
    }
    else if(c == 's')
    {
        print_state();
    }
    else
    {
        printf("\r\nkeys: a=sqw b=i2c c=slow s=state\r\n");
    }
    return 1;
}

/* ------------------------------------------------------------------ main */

int main(void)
{
    uint8_t  addr, i, n, found[16];
    uint8_t  scl, sda;
    uint32_t k, tx, ack, stfail, burst = 0;
    int      r;

    SystemCoreClockUpdate();
    Delay_Init();
    USART_Printf_Init(115200);
    uart_rx_init();
    led_init();

    printf("\r\n=== I2C ONLY TEST + PIN SELF-TEST (build %s %s) ===\r\n", __DATE__, __TIME__);
    printf("SystemClk:%lu  (\u53ea\u5f00 UART1 + I2C1, \u5176\u4ed6\u5916\u8bbe\u4e00\u5f8b\u4e0d\u521d\u59cb\u5316)\r\n",
           (unsigned long)SystemCoreClock);

    pins_to_i2c();

    i2c_idle_levels(&scl, &sda);
    printf("IDLE lvl SCL=%u SDA=%u   (1=\u4e24\u7ebf\u88ab\u4e0a\u62c9\u62c9\u9ad8)\r\n",
           (unsigned)scl, (unsigned)sda);
    print_state();

    i2c_speed_probe();

    /* \u5f00\u673a\u5148\u626b\u4e00\u904d\u5168\u5730\u5740 (\u53ea\u505a\u4e00\u6b21) */
    n = 0;
    for(addr = 0x08u; addr <= 0x77u; addr++)
        if(i2c_ping(addr) == 0) { if(n < 16u) found[n] = addr; n++; }
    printf("SCAN 0x08..0x77 -> %u dev:", (unsigned)n);
    for(i = 0; i < n && i < 16u; i++) printf(" 0x%02X", (unsigned)found[i]);
    printf("\r\n");

    if(i2c_ping(AS5600_ADDR) == 0)
    {
        uint8_t  st, agc, b[2];
        uint16_t raw;

        printf("0x36: ACK\r\n");
        if(i2c_read_regs(0x0B, &st, 1) == 0)
            printf("  STATUS=0x%02X  MD=%u ML=%u MH=%u\r\n",
                   (unsigned)st, (unsigned)((st >> 5) & 1u),
                   (unsigned)((st >> 4) & 1u), (unsigned)((st >> 3) & 1u));
        if(i2c_read_regs(0x0C, b, 2) == 0)
        {
            raw = (uint16_t)(((uint16_t)(b[0] & 0x0Fu) << 8) | b[1]);
            printf("  RAW=%u  (=%u.%01u deg)\r\n", (unsigned)raw,
                   (unsigned)((uint32_t)raw * 360u / 4096u),
                   (unsigned)(((uint32_t)raw * 3600u / 4096u) % 10u));
        }
        if(i2c_read_regs(0x1A, &agc, 1) == 0)
            printf("  AGC=%u\r\n", (unsigned)agc);
    }
    else
    {
        printf("0x36: NACK (\u5f00\u673a\u65f6)\r\n");
    }

    printf("\r\n\u6309\u952e: a=\u65b9\u6ce2 b=I2C c=\u6162I2C r=\u8bfb5600 s=\u72b6\u6001\r\n");
    printf("CONT \u6a21\u5f0f: \u8fde\u7eed\u53d1 0x6C, \u4e0d\u7b49 ACK; \u6bcf 1000 \u6b21\u4e00\u884c\r\n\r\n");

    for(;;)
    {
        if(g_mode != MODE_I2C) key_step();

        if(g_mode == MODE_SQW)
        {
            if((sqw_phase & 1u) == 0u)
            {
                GPIO_SetBits(GPIOB, GPIO_Pin_6);
                GPIO_ResetBits(GPIOB, GPIO_Pin_7);
            }
            else
            {
                GPIO_ResetBits(GPIOB, GPIO_Pin_6);
                GPIO_SetBits(GPIOB, GPIO_Pin_7);
            }
            for(k = 0u; k < 20u; k++)           /* 500ms \u5206\u6210 20 \u5c0f\u6b65, \u4fbf\u4e8e\u53ca\u65f6\u6536\u952e */
            {
                if(key_step()) break;
                Delay_Ms(25);
            }
            sqw_phase++;
        }
        else if(g_mode == MODE_READ)
        {
            uint8_t  st, agc, b[2];
            uint16_t raw;

            if(i2c_read_regs(0x0C, b, 2) == 0)
            {
                raw = (uint16_t)(((uint16_t)(b[0] & 0x0Fu) << 8) | b[1]);
                printf("ANG raw=%4u  %3u.%01u deg   ", (unsigned)raw,
                       (unsigned)((uint32_t)raw * 360u / 4096u),
                       (unsigned)(((uint32_t)raw * 3600u / 4096u) % 10u));
            }
            else
            {
                printf("ANG FAIL step=%u   ", (unsigned)(g_step & 0x3Fu));
            }
            if(i2c_read_regs(0x0B, &st, 1) == 0)
                printf("STATUS=0x%02X MD=%u ML=%u MH=%u ", (unsigned)st,
                       (unsigned)((st >> 5) & 1u), (unsigned)((st >> 4) & 1u),
                       (unsigned)((st >> 3) & 1u));
            if(i2c_read_regs(0x1A, &agc, 1) == 0)
                printf("AGC=%u", (unsigned)agc);
            printf("\r\n");

            for(k = 0u; k < 10u; k++) { if(key_step()) break; Delay_Ms(10); }
        }
        else if(g_mode == MODE_SLOW)
        {
            r = i2c_ping(AS5600_ADDR);
            printf("SLOW tx: %s\r\n",
                   (r == 0) ? "ACK" : ((r == 1) ? "NACK" : "START_FAIL"));
            for(k = 0u; k < 20u; k++)
            {
                if(key_step()) break;
                Delay_Ms(25);
            }
        }
        else
        {
            ack    = 0u;
            stfail = 0u;
            tx     = 0u;
            for(k = 0u; k < 1000u; k++)
            {
                if(key_step()) break;
                r = i2c_ping(AS5600_ADDR);
                tx++;
                if(r == 0)     { ack++; }
                else if(r < 0) { stfail++; }
                if(stfail >= 4u) break;   /* 连着 4 次发不出 START = 总线卡住, 别等完 1000 次 */
            }
            if(g_mode == MODE_I2C)
            {
                burst++;
                led_toggle();
                printf("#%lu tx=%lu ACK=%lu STFAIL=%lu BUSY=%u SCL=%u SDA=%u STAR1=0x%04X\r\n",
                       (unsigned long)burst, (unsigned long)tx,
                       (unsigned long)ack, (unsigned long)stfail,
                       (unsigned)((I2C1->STAR2 >> 1) & 1u),
                       (unsigned)((GPIOB->INDR >> 6) & 1u),
                       (unsigned)((GPIOB->INDR >> 7) & 1u),
                       (unsigned)I2C1->STAR1);
                if(stfail != 0u)
                {
                    printf("!! START not ACKed -> 9 clock pulses to free the bus\r\n");
                    bus_recover();
                }
                Delay_Ms(50);
            }
        }
    }
}
