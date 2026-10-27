#include "foc_sensor.h"
#include "foc_utils.h"
#include "drv8313.h"
#include "board.h"
#include "debug.h"

/* =============================================================================
 * AS5600 磁编码器 (U5) - 硬件 I2C1 (PB6=SCL / PB7=SDA)
 *   寄存器 (手册 v1-06 第18页寄存器表):
 *     0x0B STATUS   : bit5=MD 检测到磁铁, bit4=ML 磁场太弱, bit3=MH 太强
 *     0x0C/0x0D RAW ANGLE (12bit)
 *     0x1A AGC      : 自动增益 (3.3V 供电时范围 0~128)
 *     0x1B/0x1C MAGNITUDE (12bit)
 *   累计圈数 / 速度算法来源: loop222 @郑州 SimpleFOC C 移植 MagneticSensor.c
 *
 *   ★ 必读 (2026-09-19 定案, 别删) ★
 *   CH32L103K8U6 的 PB6/PB7 复用名是 CC1/CC2, 复位后**内置的 Type-C Rd 下拉是接通的**
 *   (USBPD->PORT_CC1 / PORT_CC2 的 bit1 = CC_PD, 实测复位值 0x0003)。
 *   不关掉它, 它就和板上 R3/R4 的 4.7k 上拉分压, 总线空闲只有约 1.7V,
 *   低于 AS5600 的 V_IH = 0.7 x VDD = 2.31V -> 从机一律 NACK。
 *   i2c_hw_init() 开头已把它关掉。依据: 数据手册 V2.1 第2页注1 + 表3-24-2 (Rd 4.08/5.1/6.12k),
 *   WCH 官方 EVT PWR/Sleep_Mode/main.c 也是同样的清法。
 * ========================================================================== */

long  sensor_cpr;
float full_rotation_offset;
long  angle_data_prev;
float angle_prev;

static uint32_t velocity_calc_timestamp;
static uint32_t sensor_err_cnt;

static uint8_t  i2c_fail_step;
static uint16_t i2c_recov_cnt;
static uint16_t i2c_star1_at_fail;
static uint16_t i2c_star2_at_fail;

#define I2C_TIMEOUT_LOOPS   2000u

static void i2c_note_fail(uint8_t step)
{
    i2c_fail_step     = step;
    i2c_star1_at_fail = (uint16_t)I2C1->STAR1;
    i2c_star2_at_fail = (uint16_t)I2C1->STAR2;
}

static int i2c_wait_flag(uint32_t flag, uint8_t step)
{
    uint32_t t = I2C_TIMEOUT_LOOPS;

    while(t--)
    {
        if(I2C_GetFlagStatus(I2C1, flag) != RESET) return 0;
    }
    i2c_note_fail(step);
    return -1;
}

static void i2c_bus_recover(void)
{
    GPIO_InitTypeDef gpio;
    uint8_t i;

    I2C_Cmd(I2C1, DISABLE);

    gpio.GPIO_Pin   = GPIO_Pin_6;
    gpio.GPIO_Mode  = GPIO_Mode_Out_OD;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &gpio);

    gpio.GPIO_Pin  = GPIO_Pin_7;
    gpio.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOB, &gpio);

    for(i = 0; i < 9; i++)
    {
        GPIO_ResetBits(GPIOB, GPIO_Pin_6);
        Delay_Us(5);
        GPIO_SetBits(GPIOB, GPIO_Pin_6);
        Delay_Us(5);
    }
    gpio.GPIO_Pin  = GPIO_Pin_7;
    gpio.GPIO_Mode = GPIO_Mode_Out_OD;
    GPIO_Init(GPIOB, &gpio);
    GPIO_ResetBits(GPIOB, GPIO_Pin_7);
    Delay_Us(5);
    GPIO_SetBits(GPIOB, GPIO_Pin_6);
    Delay_Us(5);
    GPIO_SetBits(GPIOB, GPIO_Pin_7);
    Delay_Us(5);
}

/* 硬件 I2C1 复位 + 重初始化 (PB6=SCL / PB7=SDA, AF 开漏)
 * 注意: 这里不读寄存器, 免得和自动恢复互相递归 */
static void i2c_hw_init(void)
{
    GPIO_InitTypeDef gpio;
    I2C_InitTypeDef  i2c;

    /* 第 0 步: 断开 PB6/PB7 内置的 Type-C Rd 下拉 (复位默认是接通的!)
     * 不关的话: 与 4.7k 上拉分压 -> 总线约 1.7V < AS5600 的 V_IH(2.31V) -> 必定 NACK。
     * 依据: 数据手册 V2.1 第2页注1 (K8U6 内置可控 Rd, 约5.1k) / 表3-24-2 (Rd 4.08~6.12k);
     *       WCH 官方 EVT PWR/Sleep_Mode/main.c 同样是 USBPD->PORT_CC1 &= ~(0x01<<1)。 */
    RCC->HBPCENR |= RCC_USBPDEN;                /* USBPD 外设时钟 (bit17) */
    USBPD->PORT_CC1 &= (uint16_t)~(1u << 1);    /* 断开 CC1 -> PB6/SCL 的内置 Rd */
    USBPD->PORT_CC2 &= (uint16_t)~(1u << 1);    /* 断开 CC2 -> PB7/SDA 的内置 Rd */

    RCC_PB2PeriphClockCmd(RCC_PB2Periph_GPIOB | RCC_PB2Periph_AFIO, ENABLE);
    RCC_PB1PeriphClockCmd(RCC_PB1Periph_I2C1, ENABLE);

    gpio.GPIO_Pin   = GPIO_Pin_6 | GPIO_Pin_7;
    gpio.GPIO_Mode  = GPIO_Mode_AF_OD;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &gpio);

    I2C_DeInit(I2C1);
    i2c.I2C_ClockSpeed          = 400000;
    i2c.I2C_Mode                = I2C_Mode_I2C;
    i2c.I2C_DutyCycle           = I2C_DutyCycle_2;
    i2c.I2C_OwnAddress1         = 0x00;
    i2c.I2C_Ack                 = I2C_Ack_Enable;
    i2c.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
    I2C_Init(I2C1, &i2c);
    I2C_Cmd(I2C1, ENABLE);
}

/* 清掉 I2C 的错误标志 (AF 位不清, 下一次 START 就可能发不出来 -> step=1, 两根线全平) */
static void i2c_clear_err(void)
{
    I2C_ClearFlag(I2C1, I2C_FLAG_AF);
    I2C_ClearFlag(I2C1, I2C_FLAG_BERR);
    I2C_ClearFlag(I2C1, I2C_FLAG_ARLO);
    I2C_ClearFlag(I2C1, I2C_FLAG_OVR);
}

/* ---- 连续失败 -> 总线恢复 策略 -------------------------------------------
 * 2026-09-19 实测现象: 板子跑几分钟后 AS5600 变成 **永久 NACK**
 *   (step=2, STAR1=0x0000, SCL/SDA 空闲都是高), 只有复位能救回来。
 * 这种"从机自己卡在半途 / 外设自己卡住"的状态, 9 个时钟 + 外设复位重来
 * 是唯一手段, 所以判定标准不能只看 BERR/ARLO —— 持续失败也照样救。
 * 代价: 真没片子(或真坏了)时会周期性白刷 9 个时钟, 用退避(thr 逐次翻倍)压住。
 * ----------------------------------------------------------------------- */
#define I2C_AUTO_RECOVER_N    16u       /* 连续 16 次失败(≈连续 10ms 以上) 起救 */
#define I2C_RECOVER_MAX_SHIFT 10u       /* 退避上限: thr 最大 16<<10 = 16384 */
static uint16_t i2c_fail_run;
static uint16_t i2c_recov_thr = I2C_AUTO_RECOVER_N;
static uint8_t  i2c_recov_shift;
static uint32_t i2c_snap_us;

static uint8_t i2c_bus_broken(void)
{
    if((uint16_t)I2C1->STAR1 & 0x0300u) return 1;               /* BERR | ARLO */
    if(i2c_fail_step == 1u || i2c_fail_step == 5u) return 1;    /* START 根本没发出来 */
    return 0;
}

/* 第一次失败时抓一份寄存器现场 (限速 1s 一次, 免得间歇性失败刷屏) */
static void i2c_snapshot(const char *why)
{
    uint32_t now = Micros();

    if((uint32_t)(now - i2c_snap_us) < 1000000u) return;
    i2c_snap_us = now;

    printf("I2C1: %s step=%u broken=%u STAR1=0x%04X STAR2=0x%04X CTLR1=0x%04X CTLR2=0x%04X CKCFGR=0x%04X err=%lu recov=%u\r\n",
           why, (unsigned)i2c_fail_step, (unsigned)i2c_bus_broken(),
           (unsigned)I2C1->STAR1, (unsigned)I2C1->STAR2,
           (unsigned)I2C1->CTLR1, (unsigned)I2C1->CTLR2, (unsigned)I2C1->CKCFGR,
           (unsigned long)sensor_err_cnt, (unsigned)i2c_recov_cnt);
}

void Sensor_Init(void)
{
    uint8_t  b[2];

    i2c_hw_init();
    i2c_clear_err();
    i2c_fail_run = 0;

    sensor_cpr              = AS5600_CPR;
    sensor_err_cnt          = 0;
    i2c_recov_thr           = I2C_AUTO_RECOVER_N;
    i2c_recov_shift         = 0;
    i2c_snap_us             = 0;
    full_rotation_offset    = 0.0f;
    velocity_calc_timestamp = Micros();
    i2c_fail_step           = 0;

    if(Sensor_ReadRegs(AS5600_REG_RAW_ANGLE_H, b, 2) == 0)
        angle_data_prev = (long)(((uint16_t)(b[0] & 0x0F) << 8) | b[1]);
    else
        angle_data_prev = 0;
}

/* 只做 START + 地址, 看从机 ACK 不 ACK */
int Sensor_Ping(uint8_t addr7)
{
    i2c_fail_step = 0;
    i2c_clear_err();
    I2C_AcknowledgeConfig(I2C1, ENABLE);
    I2C_GenerateSTOP(I2C1, ENABLE);

    I2C_GenerateSTART(I2C1, ENABLE);
    if(i2c_wait_flag(I2C_FLAG_SB, 1) != 0) { I2C_GenerateSTOP(I2C1, ENABLE); return -1; }

    I2C_Send7bitAddress(I2C1, (uint8_t)(addr7 << 1), I2C_Direction_Transmitter);
    if(i2c_wait_flag(I2C_FLAG_ADDR, 2) != 0) { I2C_GenerateSTOP(I2C1, ENABLE); return -1; }

    (void)I2C1->STAR2;
    I2C_GenerateSTOP(I2C1, ENABLE);
    return 0;
}

/* 通用寄存器读: reg -> buf[0..len-1] */
int Sensor_ReadRegs(uint8_t reg, uint8_t *buf, uint8_t len)
{
    uint8_t i;

    i2c_fail_step = 0;
    i2c_clear_err();

    I2C_AcknowledgeConfig(I2C1, ENABLE);

    I2C_GenerateSTART(I2C1, ENABLE);
    if(i2c_wait_flag(I2C_FLAG_SB, 1) != 0) goto err;
    I2C_Send7bitAddress(I2C1, AS5600_ADDR << 1, I2C_Direction_Transmitter);
    if(i2c_wait_flag(I2C_FLAG_ADDR, 2) != 0) goto err;
    (void)I2C1->STAR2;
    if(i2c_wait_flag(I2C_FLAG_TXE, 3) != 0) goto err;
    I2C_SendData(I2C1, reg);
    if(i2c_wait_flag(I2C_FLAG_BTF, 4) != 0) goto err;

    I2C_GenerateSTART(I2C1, ENABLE);
    if(i2c_wait_flag(I2C_FLAG_SB, 5) != 0) goto err;
    I2C_Send7bitAddress(I2C1, (AS5600_ADDR << 1) | 1, I2C_Direction_Receiver);
    if(i2c_wait_flag(I2C_FLAG_ADDR, 6) != 0) goto err;
    (void)I2C1->STAR2;

    for(i = 0; i < len; i++)
    {
        if(i == (uint8_t)(len - 1)) I2C_AcknowledgeConfig(I2C1, DISABLE);   /* 最后一个字节回 NACK */
        if(i2c_wait_flag(I2C_FLAG_RXNE, (uint8_t)(7 + i)) != 0) goto err;
        buf[i] = I2C_ReceiveData(I2C1);
    }

    I2C_GenerateSTOP(I2C1, ENABLE);
    I2C_AcknowledgeConfig(I2C1, ENABLE);
    i2c_fail_run    = 0;
    i2c_recov_shift = 0;
    i2c_recov_thr   = I2C_AUTO_RECOVER_N;
    return 0;

err:
    I2C_GenerateSTOP(I2C1, ENABLE);
    I2C_AcknowledgeConfig(I2C1, ENABLE);
    i2c_clear_err();
    sensor_err_cnt++;

    if(++i2c_fail_run == 1u) i2c_snapshot("FAIL first");

    if(i2c_fail_run >= i2c_recov_thr)
    {
        i2c_fail_run = 0;
        i2c_bus_recover();                            /* 9 个时钟把卡住的从机顶出来 */
        i2c_hw_init();                                /* 再把 I2C1 外设复位重来 */
        i2c_clear_err();
        i2c_recov_cnt++;
        if(i2c_recov_shift < I2C_RECOVER_MAX_SHIFT) i2c_recov_shift++;
        i2c_recov_thr = (uint16_t)(I2C_AUTO_RECOVER_N << i2c_recov_shift);
        printf("I2C1: bus recover #%u (next thr=%u)\r\n",
               (unsigned)i2c_recov_cnt, (unsigned)i2c_recov_thr);
    }
    return -1;
}

int Sensor_ReadRaw(uint16_t *raw)
{
    uint8_t b[2];

    *raw = 0;
    if(Sensor_ReadRegs(AS5600_REG_RAW_ANGLE_H, b, 2) != 0) return -1;
    *raw = ((uint16_t)(b[0] & 0x0F) << 8) | b[1];
    return 0;
}

int Sensor_ReadStatus(uint8_t *st)
{
    return Sensor_ReadRegs(0x0B, st, 1);
}

int Sensor_ReadAGC(uint8_t *agc)
{
    return Sensor_ReadRegs(0x1A, agc, 1);
}

int Sensor_ReadMagnitude(uint16_t *mag)
{
    uint8_t b[2];

    if(Sensor_ReadRegs(0x1B, b, 2) != 0) return -1;
    *mag = ((uint16_t)(b[0] & 0x0F) << 8) | b[1];
    return 0;
}

void Sensor_GetDiag(uint8_t *step, uint16_t *star1, uint16_t *star2)
{
    *step  = i2c_fail_step;
    *star1 = i2c_star1_at_fail;
    *star2 = i2c_star2_at_fail;
}

uint16_t Sensor_RecoverCount(void)
{
    return i2c_recov_cnt;
}

/* 抓一份硬件状态快照, 用来判断"START 发不出来"是哪种原因 */
void Sensor_GetHwState(uint8_t *pe, uint16_t *star1, uint8_t *cfg6, uint8_t *cfg7, uint8_t *clk)
{
    *pe    = (uint8_t)(I2C1->CTLR1 & 0x01u);
    *star1 = (uint16_t)I2C1->STAR1;
    *cfg6  = (uint8_t)((GPIOB->CFGLR >> 24) & 0x0Fu);       /* PB6 nibble */
    *cfg7  = (uint8_t)((GPIOB->CFGLR >> 28) & 0x0Fu);       /* PB7 nibble */
    *clk   = ((RCC->PB1PCENR & RCC_PB1Periph_I2C1) != 0u) ? 1u : 0u;
}

void Sensor_BusLevels(uint8_t *scl, uint8_t *sda)
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

void Sensor_BusRecover(void)
{
    i2c_bus_recover();
    Sensor_Init();
}

/* 手动强制一次总线恢复 (控制台 'R'), 用于现场确认"卡住的从机能不能被顶出来" */
void Sensor_ForceRecover(void)
{
    i2c_bus_recover();
    i2c_hw_init();
    i2c_clear_err();
    i2c_fail_run    = 0;
    i2c_recov_shift = 0;
    i2c_recov_thr   = I2C_AUTO_RECOVER_N;
    i2c_recov_cnt++;
}

float Sensor_ReadSingle(void)
{
    uint16_t raw;

    if(Sensor_ReadRaw(&raw) != 0) return -1.0f;
    return (float)raw / (float)sensor_cpr * _2PI;
}

float Sensor_GetAngle(void)
{
    uint16_t raw;
    float angle_data, d_angle;

    if(Sensor_ReadRaw(&raw) != 0)
    {
        return full_rotation_offset + ((float)angle_data_prev / (float)sensor_cpr) * _2PI;
    }

    angle_data = (float)raw;
    d_angle    = angle_data - (float)angle_data_prev;

    if(fabsf(d_angle) > (0.8f * (float)sensor_cpr))
    {
        full_rotation_offset += (d_angle > 0.0f) ? -_2PI : _2PI;
    }
    angle_data_prev = (long)angle_data;

    return full_rotation_offset + (angle_data / (float)sensor_cpr) * _2PI;
}

/* 测速: **固定窗口**, 不是每次调用都算一次。
 * 原来每次调用都算: 主循环 ~0.57ms 一次, 30rpm 时 0.57ms 只走 1.17 个编码器计数,
 * 量化误差 +-1 count 就等价于 +-2.7 rad/s (实测见到 vel=-20.13 rad/s 的假峰)。
 * 现在攒够 VEL_WINDOW_US 再更新一次, 中间直接返回上一次的值。 */
#define VEL_WINDOW_US   5000u
static float vel_last;

float Sensor_GetVelocity(void)
{
    uint32_t now_us, d;
    float Ts, angle_c, vel;

    now_us = Micros();
    d      = now_us - velocity_calc_timestamp;

    if(d < VEL_WINDOW_US)
    {
        return vel_last;                       /* 窗口没到, 沿用上次结果 */
    }
    velocity_calc_timestamp = now_us;

    Ts = (float)d * 1e-6f;
    if(Ts <= 0.0f || Ts > 0.5f) Ts = 1e-3f;

    angle_c = Sensor_GetAngle();
    vel     = (angle_c - angle_prev) / Ts;

    angle_prev = angle_c;
    vel_last   = vel;
    return vel;
}

uint32_t Sensor_ErrorCount(void)
{
    return sensor_err_cnt;
}
