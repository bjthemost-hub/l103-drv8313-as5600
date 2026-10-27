#include "cmd.h"
#include "debug.h"
#include "board.h"
#include "drv8313.h"
#include "csense.h"
#include "foc_utils.h"
#include "foc_sensor.h"
#include "foc_motor.h"
#include "can_bus.h"
#include "motion_pv.h"
#include "pos_log.h"
#include "faults.h"
#include "wave.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdarg.h>

/* =============================================================================
 * 命令解析 (UART1 / USB CDC 共用)
 *
 *   H or ?      帮助
 *   S           打印一次状态;  S0/S1 = 关/开周期状态;  ST = 踢一次 TICK 统计
 *   M<n>        模式: 0=力矩(电压) 1=速度闭环 2=角度闭环 3=速度开环 4=角度开环
 *   V<rpm>      速度目标 (机械转速, rpm)
 *   P<deg>      位置目标 (度, 绝对, 可多圈)
 *   C<turns>    相对转动 (圈, 可正负)
 *   L<volts>    相电压限幅 (0 ~ Vbus/sqrt3; Vbus 是参数, 默认 24V -> 13.86V)
 *   E / D       使能 / 失能 DRV8313
 *   A           重新做编码器对齐 (需要电机 + 磁铁 + AS5600)
 *   Z           把当前机械角清零
 *   #           看电机参数 (极对数 / 堵转阈值 / 母线电压)
 *   #<pp>,<A>,<V>  设电机参数并存 Flash (see Motion_SetMotorCfg; 没给的那项 = 不改)
 * ========================================================================== */

#define RX_RING_SIZE    192
#define LINE_MAX        128      /* 2026-09-29: 64 -> 128, 让一条动作脚本 4 段能写一行 */

static volatile char     rx_ring[RX_RING_SIZE];
static volatile uint16_t rx_head;
static volatile uint16_t rx_tail;

static char    line[LINE_MAX];
static uint8_t line_len;
static uint8_t auto_status;
static uint8_t aligned;

static void (*out_hook)(const char *);

void Main_RequestTick(void);        /* main.c: 踢一次 TICK 统计 (见 'ST') */

float cmd_target;
static uint8_t sensor_poll = 1;

int  Cmd_IsAligned(void)     { return (int)aligned; }
void Cmd_MarkAligned(int ok) { aligned = ok ? 1u : 0u; }

void Cmd_SetOutHook(void (*fn)(const char *buf))
{
    out_hook = fn;
}

int Cmd_AutoStatus(void)
{
    return (int)auto_status;
}

/* ---- 输出: 有 hook 走 hook (CDC), 否则走 UART1 printf ---- */
static void cprintf(const char *fmt, ...)
{
    char    buf[384];
    va_list ap;

    va_start(ap, fmt);
    mini_vsnprintf(buf, sizeof(buf), fmt, ap);   /* 不用 newlib 的 vsnprintf: 它会拉进整个 stdio */
    va_end(ap);

    if(out_hook) out_hook(buf);
    else         printf("%s", buf);
}

/* ---- USART1 接收 (PA10), 与 printf 的 PA9 共用一个外设 ---- */
void Cmd_Init(void)
{
    GPIO_InitTypeDef  gpio;
    USART_InitTypeDef usart;
    NVIC_InitTypeDef  nvic;

    RCC_PB2PeriphClockCmd(RCC_PB2Periph_USART1 | RCC_PB2Periph_GPIOA, ENABLE);

    /* PA10 = USART1_RX, 输入上拉 */
    gpio.GPIO_Pin   = GPIO_Pin_10;
    gpio.GPIO_Mode  = GPIO_Mode_IPU;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &gpio);

    /* 重新初始化: 打开 Rx (debug.c 里只开了 Tx) */
    usart.USART_BaudRate            = 115200;
    usart.USART_WordLength          = USART_WordLength_8b;
    usart.USART_StopBits            = USART_StopBits_1;
    usart.USART_Parity              = USART_Parity_No;
    usart.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    usart.USART_Mode                = USART_Mode_Tx | USART_Mode_Rx;
    USART_Init(USART1, &usart);
    USART_Cmd(USART1, ENABLE);

    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);

    nvic.NVIC_IRQChannel                   = USART1_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 2;
    nvic.NVIC_IRQChannelSubPriority        = 0;
    nvic.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init(&nvic);

    rx_head = rx_tail = 0;
    line_len = 0;
    auto_status = 0;
    aligned     = 0;
    cmd_target  = 0.0f;

    cprintf("CMD: ready (type H for help)\r\n");
}

void USART1_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));

void USART1_IRQHandler(void)
{
    if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
    {
        char    ch = (char)(USART_ReceiveData(USART1) & 0xFF);
        uint16_t next = (uint16_t)((rx_head + 1u) % RX_RING_SIZE);

        if(next != rx_tail)          /* 满了就丢 */
        {
            rx_ring[rx_head] = ch;
            rx_head = next;
        }
        USART_ClearITPendingBit(USART1, USART_IT_RXNE);
    }

    /* 发送: 每次塞一个字节; 塞空了就把 TXEIE 关掉, 不做空转中断 */
    if(USART_GetITStatus(USART1, USART_IT_TXE) != RESET)
    {
        if(UsartTx_Drain1() == 0) USART_ITConfig(USART1, USART_IT_TXE, DISABLE);
    }
}

void Cmd_Poll(void)
{
    while(rx_tail != rx_head)
    {
        char ch = rx_ring[rx_tail];
        rx_tail = (uint16_t)((rx_tail + 1u) % RX_RING_SIZE);

        if(ch == '\r' || ch == '\n')
        {
            if(line_len > 0)
            {
                line[line_len] = '\0';
                Cmd_Exec(line);
                line_len = 0;
            }
        }
        else if(line_len < (LINE_MAX - 1))
        {
            line[line_len++] = ch;
        }
    }
}

static const char *mode_name(MotionControlType m)
{
    switch(m)
    {
        case Type_torque:           return "torque(V)";
        case Type_velocity:         return "velocity-closed";
        case Type_angle:            return "angle-closed";
        case Type_velocity_openloop:return "velocity-openloop";
        case Type_angle_openloop:   return "angle-openloop";
        default:                    return "?";
    }
}

/* 一次把 "MCU 到底在不在出 PWM" 摊开: TIM3 实际寄存器 + 引脚实测电平
 * CCR 不变 -> 固件没在指挥;  CCR 在变但外部无波形 -> 问题在 MCU 之后 */
void Cmd_DrvDiag(void)
{
    float da, db, dc;

    DRV8313_GetDuty(&da, &db, &dc);

    cprintf("DRV: ARR=%u CNT=%u CCR1=%u CCR2=%u CCR3=%u\r\n",
            (unsigned)TIM3->ATRLR, (unsigned)TIM3->CNT,
            (unsigned)TIM3->CH1CVR, (unsigned)TIM3->CH2CVR, (unsigned)TIM3->CH3CVR);
    cprintf("DRV: duty=%s/%s/%s EN1=%d EN2=%d EN3=%d nFAULT=%d\r\n",
            FmtF(da, 3), FmtF(db, 3), FmtF(dc, 3),
            (int)GPIO_ReadOutputDataBit(GPIOB, GPIO_Pin_3),
            (int)GPIO_ReadOutputDataBit(GPIOB, GPIO_Pin_4),
            (int)GPIO_ReadOutputDataBit(GPIOB, GPIO_Pin_5),
            (int)GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_14));
    /* 引脚复用状态: 定时器在跑 != 引脚还挂在 TIM3 上 (Bh = AF_PP 输出) */
    cprintf("DRV: mux PA6=%Xh PA7=%Xh PB0=%Xh  (Bh=AF_PP, 4h=floating in)\r\n",
            (unsigned)((GPIOA->CFGLR >> 24) & 0x0Fu),
            (unsigned)((GPIOA->CFGLR >> 28) & 0x0Fu),
            (unsigned)((GPIOB->CFGLR >>  0) & 0x0Fu));
}

void Cmd_Status(void)
{
    cprintf("ST mode=%s target=%s shaft=%s rad vel=%s rad/s elec=%s q=%sV en=%d aligned=%d stalled=%d\r\n",
            mode_name(controller),
            FmtF(cmd_target, 3),
            FmtF(shaft_angle, 3),
            FmtF(shaft_velocity, 3),
            FmtF(_normalizeAngle(_electricalAngle(shaft_angle, pole_pairs)), 3),
            FmtF(voltage.q, 2),
            Motion_IsEnabled(),
            (int)aligned, Motion_IsStalled());
}

/* 1Hz 打印 INA240 相电流反馈 (U10 = W 相 / U8 = V 相)
 * 电流已减掉 CS_CalZero() 实测的零点, 不再硬用 1.65V。
 * peak 是 CS_Task 维护的窗口峰值 (跨整个电周期), 堵转判断看它。 */
void Cmd_CurrentSense(void)
{
    uint16_t ra = CS_Raw(0);
    uint16_t rb = CS_Raw(1);
    float    va = (float)ra * CS_VDDA / CS_FULLSCALE;
    float    vb = (float)rb * CS_VDDA / CS_FULLSCALE;

    cprintf("CS: W(PA1) raw=%4u %sV %sA | V(PA4) raw=%4u %sV %sA | U=%sA peak=%sA zeroW=%sV\r\n",
            (unsigned)ra, FmtF(va, 3), FmtFS(CS_Amp(0), 3),
            (unsigned)rb, FmtF(vb, 3), FmtFS(CS_Amp(1), 3),
            FmtFS(CS_PhaseAmp(0), 3), FmtFS(CS_PeakAmp(), 3), FmtF(CS_ZeroVolt(0), 3));
}

static void print_help(void)
{
    cprintf("H/?  help   S status   ST one-shot TICK stats   S0/S1 periodic dump: S0=idle only(default) S1=always\r\n");
    cprintf("SW  phase-current wave: SW0 off / SW1 on / SWD<div> decimate / SWB<baud> set UART1 baud\r\n");
    cprintf("M<n> 0=torque 1=vel-closed 2=angle-closed 3=vel-openloop 4=angle-openloop\r\n");
    cprintf("V<rpm> velocity   P<deg> position   C<turns> relative\r\n");
    cprintf("T<volts> torque   L<volts> vlimit  E enable  D disable\r\n");
    cprintf("A align   Z zero angle   Y sensor diag   Q drv/TIM3   R force i2c recover\r\n");
    cprintf("X0/X1 sensor poll off/on\r\n");
    cprintf("-- CAN / position-velocity (PV) --\r\n");
    cprintf("K can status   K0/K1 upstream(debug) off/on [default OFF=passive]   KR counters reset   J send test frame\r\n");
    cprintf("G<turns>@<rpm> go to absolute position (turns), speed limit (rpm)\r\n");
    cprintf("B stop+hold    B1 stop+release(coast)    O clear stall fault\r\n");
    cprintf("W motion/poslog status   U recalibrate current zero (motor stopped!)\r\n");
    cprintf("N[<rpm>] go home(mech zero)   F set mech zero here   I fault status\r\n");
    cprintf("CAN: 0x100 PV / 0x101 speed / 0x110 en/disable / 0x111 home / 0x112 set zero\r\n");
    cprintf("CAN: 0x113 query / 0x114 fault-report cfg -> TX 0x180/0x181/0x182/0x183, event 0x1A0\r\n");
    cprintf("CAN: 0x115 set angle-loop vmax (b0..b1 int16 x0.01rad/s) -> TX 0x181\r\n");
    cprintf("CAN: 0x116 set motor cfg (b0 pole_pairs, b1..2 stall x0.01A, b4..5 vbus x0.01V, b3 bit0=1) -> TX 0x184\r\n");
    cprintf("@<V>[,<s>] DC hold (measure Ibus / phase R, e.g. @6 or @6,3) max 5s\r\n");
    cprintf("# show motor cfg   #<pp>,<A>[,<V>] set pole_pairs/stall/vbus (saved to Flash; re-run A after changing pole_pairs)\r\n");
    cprintf("-- onboard action script (2026-09-29) --\r\n");
    cprintf("*{wait_us,+-rpm,deg,dwell_ms}.. queue a segment (deg POSITIVE, dir = rpm sign; rel. to script start)\r\n");
    cprintf("*! run once   *!<n> n passes   *0 clear   *? show    CAN 0x120 add seg / 0x121 ctl\r\n");
}

int Cmd_SensorPoll(void)
{
    return (int)sensor_poll;
}

/* 读 STATUS / RAW / AGC / MAGNITUDE 并打印一行诊断 */
void Cmd_SensorDiag(void)
{
    uint8_t  st = 0, agc = 0;
    uint16_t raw = 0, mag = 0;
    int      ok_raw, ok_st, ok_agc, ok_mag;

    {
        uint8_t  pe, cfg6, cfg7, clk, lvl_scl, lvl_sda;
        uint16_t s1;

        Sensor_BusLevels(&lvl_scl, &lvl_sda);
        Sensor_GetHwState(&pe, &s1, &cfg6, &cfg7, &clk);
        cprintf("I2C1: lvl SCL=%u SDA=%u | PE=%u clk=%u STAR1=0x%04X | PB6=%Xh PB7=%Xh (F=AF_OD) recov=%u\r\n",
                (unsigned)lvl_scl, (unsigned)lvl_sda, (unsigned)pe, (unsigned)clk,
                (unsigned)s1, (unsigned)cfg6, (unsigned)cfg7,
                (unsigned)Sensor_RecoverCount());
    }

    ok_raw = Sensor_ReadRaw(&raw);
    ok_st  = Sensor_ReadStatus(&st);
    ok_agc = Sensor_ReadAGC(&agc);
    ok_mag = Sensor_ReadMagnitude(&mag);

    if(ok_raw || ok_st || ok_agc || ok_mag)
    {
        uint8_t  step;
        uint16_t s1, s2;

        Sensor_GetDiag(&step, &s1, &s2);
        cprintf("AS5600: FAIL raw=%d st=%d agc=%d mag=%d step=%u STAR1=0x%04X STAR2=0x%04X err=%lu\r\n",
                ok_raw, ok_st, ok_agc, ok_mag, (unsigned)step, (unsigned)s1, (unsigned)s2,
                (unsigned long)Sensor_ErrorCount());
        return;
    }

    cprintf("AS5600: raw=%4u ang=%6sdeg MD=%u ML=%u MH=%u AGC=%3u MAG=%4u err=%lu\r\n",
            (unsigned)raw, FmtF((float)raw * 360.0f / 4096.0f, 1),
            (unsigned)((st >> 5) & 1u), (unsigned)((st >> 4) & 1u), (unsigned)((st >> 3) & 1u),
            (unsigned)agc, (unsigned)mag, (unsigned long)Sensor_ErrorCount());

    /* 只读诊断: 把 ANGLE(0x0E, 过 ZPOS/MPOS 的角度) 与 CONF(0x07) 也读出来。
     * RAW(0x0C) 与 ANGLE(0x0E) 应当同步变化; CONF 里能看到低功耗/滤波/输出级配置。
     * 用途: 2026-09-28 现场出现 RAW 恒为 0 时, 靠这两个寄存器区分
     *       "芯片内部角输出停了" vs "配置被改成低功耗/非磁角度输出"。 */
    {
        uint8_t ab[2], cb[2];

        if(Sensor_ReadRegs(0x0E, ab, 2) == 0 && Sensor_ReadRegs(0x07, cb, 2) == 0)
            cprintf("AS5600: ANGLE(0x0E)=%4u CONF(0x07)=0x%04X\r\n",
                    (unsigned)(((uint16_t)(ab[0] & 0x0F) << 8) | ab[1]),
                    (unsigned)(((uint16_t)cb[0] << 8) | cb[1]));
    }
}

/* =============================================================================
 * '@' 实验用: 直流保持 (给一个固定矢量, 让相电流变成纯直流)
 * -----------------------------------------------------------------------------
 * 用法: @<volts>[,<秒>]      例: @6      @6,3      (上限 = Vbus/sqrt(3))
 *
 * 干什么: 先断 EN, 挂起中断里那个 FOC 环路, 自己沿转子 d 轴(不产生转矩,
 *         转子不动)拍一个固定电压矢量, 然后每 200ms 打印一次三相电流。
 *         因为矢量在空间里是静止的, 相电流是纯直流, cos(phi)=1, 于是:
 *
 *             母线电流  I_bus = 1.5 * (V_phase / Vdc) * I_amp
 *             相电阻    R_phase = V_phase / I_amp       (Rll = 2*R_phase)
 *
 *         其中 I_amp 由三相电流合成: I_amp = sqrt( 2/3 * (iu^2+iv^2+iw^2) ),
 *         这样跟矢量具体停在哪个角度无关 (否则单相最大值会差 0~15%)。
 *
 * 为什么要它: 母线上串个电流表, 跟这里打的 Ibus 对一下, 就能判定
 *         INA240 -> ADC 这一路的增益到底对不对。
 *
 * 安全: 直流下相电流就是 V/R, 会一直发热 (6V 大约 10W, 13.8V 大约 55W),
 *       所以秒数夹在 0.2~5s, 跑完自动断 DRV。
 * ========================================================================== */
static void Cmd_DcHold(float volts, float secs)
{
    float    uout, ang, iu, iv, iw, iamp, rph, t;
    uint32_t t0, tnext;
    float    acc_i = 0.0f, acc_b = 0.0f;
    int      nacc = 0, n = 0;

    if(volts <= 0.05f)
    {
        cprintf("DCHOLD: usage @<volts>[,<sec>]  e.g. @6 or @6,3  (limit %sV)\r\n",
                FmtF(voltage_power_supply / _SQRT3, 2));
        return;
    }
    if(volts > voltage_power_supply / _SQRT3) volts = voltage_power_supply / _SQRT3;
    if(secs  < 0.2f) secs = 0.2f;
    if(secs  > 5.0f) secs = 5.0f;
    uout = volts / voltage_power_supply;

    Motion_Stop(1);                    /* 松轴 + 断 EN, PV 归 IDLE (ram 里 cmd_target 也理顺) */
    DRV8313_Disable();
    Delay_Ms(30);
    Motor_DriveSuspend(1);             /* 中断别再写 PWM, 由这里自己写 */
    CS_CalZero();                      /* EN 已断 -> 顺便把电流零点重标一次 */

    ang = ElectricalAngle();           /* 沿转子 d 轴: 转矩≈0, 转子不会动 */
    SetPhaseVoltage(0.0f, volts, ang);
    DRV8313_Enable();

    cprintf("DCHOLD: start %sV (Uout=%s, %ss)  read the ammeter now and compare with Ibus\r\n",
            FmtF(volts, 2), FmtF(uout, 3), FmtF(secs, 2));

    t0    = Micros();
    tnext = t0;
    while(1)
    {
        t = (float)(Micros() - t0) * 1e-6f;
        if(t >= secs) break;

        SetPhaseVoltage(0.0f, volts, ang);       /* 矢量保持不动 */

        if((int32_t)(Micros() - tnext) >= 0)
        {
            tnext = Micros() + 200000u;

            n++;

            iv   = CS_PhaseAmp(1);
            iw   = CS_PhaseAmp(2);
            iu   = CS_PhaseAmp(0);
            iamp = SqrtF((iu * iu + iv * iv + iw * iw) * 0.6666667f);
            rph  = (iamp > 0.03f) ? (volts / iamp) : 0.0f;

            /* 注意: FmtF 是 6 槽环形缓冲 (FMT_RING=6), 一行里最多 6 个, 超了会互相覆盖 */
            cprintf("DCHOLD #%d U=%sA V=%sA W=%sA Iamp=%sA Rll=%s Ibus=%sA\r\n",
                    n, FmtF(iu, 3), FmtF(iv, 3), FmtF(iw, 3),
                    FmtF(iamp, 3), FmtF(rph * 2.0f, 2), FmtF(1.5f * uout * iamp, 3));

            if(t > 0.4f)
            {
                acc_i += iamp;
                acc_b += 1.5f * uout * iamp;
                nacc++;
            }
        }
    }

    DRV8313_Disable();
    SetPhaseVoltage(0.0f, 0.0f, 0.0f);
    Motor_DriveSuspend(0);

    if(nacc > 0)
    {
        float ai = acc_i / (float)nacc;
        float ab = acc_b / (float)nacc;

        cprintf("DCHOLD: end  avg Iamp=%sA  Rll=%s ohm  Ibus=%sA  (%d points, first 0.4s dropped)  DRV off\r\n",
                FmtF(ai, 3), FmtF((ai > 0.03f) ? (volts * 2.0f / ai) : 0.0f, 2),
                FmtF(ab, 3), nacc);
    }
    else
    {
        cprintf("DCHOLD: end (too short, no steady-state point)  DRV off\r\n");
    }
}

/* ── 轻量十进制解析 (取代 atof / strtod) ────────────────────────────────
 * 为什么自己写: atof() 会连锁拉进 strtod -> mprec(多精度) -> gdtoa -> 双精度软浮点
 * (__adddf3/__subdf3/__muldf3/__divdf3) -> malloc/calloc/free/locale/strlen,
 * 实测在 build/L103_DRV8313_FOC.map 里占 18.7 KB Flash(占 63 KB 的 29.6%)。
 * 命令行只需要 "11" / "2.5" / "-90" / "1e-3" 这类简单十进制, 故自写。
 * 换掉之后实测省 10440 B (98.05% -> 81.86%)。
 *
 * 与标准库 strtod 一致的地方: 允许前后空白与正负号; 遇到第一个非法字符就停
 * (不报错, 返回已经解析到的部分; 一个数字都解不出则返回 0, 与 atof 同)。
 * 有意不支持: inf / nan / 十六进制浮点 "0x1p3" / 千分位 —— 命令行用不到。
 *
 * 精度口径 (float32 本来就只有约 7 位有效数字):
 *   整数部分  最多累积到 1e8, 再大就不累积了 (命令行用不到这个量级)
 *   小数部分  前导 0 不占精度位, 其后最多累积 9 位有效数字, 一次浮点除法出结果;
 *             超出 9 位的尾数直接丢掉 (相对误差 < 1e-7, 已在 PC 上比对过)
 * 例: "#11,2.5,24" -> 11, 逗号后 2.5, 再后 24
 */
float Cmd_ParseFloat(const char *s)
{
    const char *p = s;
    int neg = 0, any = 0, fd = 0, fskip = 0, exp10 = 0, k;
    unsigned long ip = 0, fp = 0, scal = 1;
    float v;

    while(*p == ' ' || *p == '\t') p++;
    if(*p == '+' || *p == '-') { neg = (*p == '-'); p++; }

    while(*p >= '0' && *p <= '9')
    {
        if(ip < 100000000UL) ip = ip * 10UL + (unsigned long)(*p - '0');
        p++; any = 1;
    }
    if(*p == '.')
    {
        p++;
        while(*p == '0') { p++; fskip++; any = 1; }        /* 前导 0 不占精度位 */
        while(*p >= '0' && *p <= '9')
        {
            if(fd < 9) { fp = fp * 10UL + (unsigned long)(*p - '0'); fd++; }
            p++; any = 1;
        }
    }
    if(any && (*p == 'e' || *p == 'E'))
    {
        const char *q = p + 1;
        int eneg = 0, ev = 0, eany = 0;
        if(*q == '+' || *q == '-') { eneg = (*q == '-'); q++; }
        while(*q >= '0' && *q <= '9') { if(ev < 100) ev = ev * 10 + (*q - '0'); q++; eany = 1; }
        if(eany) { exp10 += eneg ? -ev : ev; p = q; }
    }
    (void)p;
    if(!any) return 0.0f;

    /* 小数: 有效数字除以 10^fd, 再按前导 0 的位数缩回去。只动小数项, 不能碰整数项 */
    for(k = 0; k < fd; k++) scal *= 10UL;
    v = (float)fp / (float)scal;
    for(k = 0; k < fskip; k++) v *= 0.1f;
    v += (float)ip;

    while(exp10 > 0) { v *= 10.0f; exp10--; }
    while(exp10 < 0) { v *= 0.1f;  exp10++; }
    return neg ? -v : v;
}

void Cmd_Exec(const char *cmd)
{
    char  c;
    float val;

    while(*cmd == ' ' || *cmd == '\t') cmd++;
    if(*cmd == '\0') return;

    c = *cmd;
    if(c >= 'a' && c <= 'z') c = (char)(c - 'a' + 'A');

    val = (float)Cmd_ParseFloat(cmd + 1);

    switch(c)
    {
        case '@':
        {
            /* 直流保持: 量母线电流 / 量相电阻 (细节见函数头) */
            const char *cm = strchr(cmd, ',');
            Cmd_DcHold(val, cm ? (float)Cmd_ParseFloat(cm + 1) : 2.0f);
            break;
        }

        case 'H':
        case '?':
            print_help();
            break;

        case 'S':
            if(val != 0.0f) { auto_status = 1; cprintf("STATUS: periodic dump ALWAYS (will cost ~0.7ms/s phase error while running)\r\n"); }
            /* 'ST': 不回显 —— 回显本身也会进 loopmax, 会污染这次测量 */
            else if(cmd[1] == 'T' || cmd[1] == 't') { Main_RequestTick(); }
            else if(cmd[1] == '0') { auto_status = 0; cprintf("STATUS: periodic dump IDLE-ONLY (default; silent while motor enabled)\r\n"); }
            else if(cmd[1] == 'W' || cmd[1] == 'w')
            {
                /* 相电流波形流: SW / SW0 / SW1 / SWD<div> / SWB<baud>  (见 wave.c) */
                char k = cmd[2];

                if(k >= 'a' && k <= 'z') k = (char)(k - 'a' + 'A');

                if(k == '\0')       Wave_PrintInfo();
                else if(k == '0')  { Wave_SetOn(0); cprintf("WAVE: OFF\r\n"); }
                else if(k == '1')  { Wave_SetOn(1); Wave_PrintInfo(); }
                else if(k == 'D')
                {
                    uint16_t d = (uint16_t)Cmd_ParseFloat(cmd + 3);

                    if(d < 1u) { cprintf("ERR: SWD<div> 1..200\r\n"); break; }
                    Wave_SetDiv(d);
                    Wave_PrintInfo();
                }
                else if(k == 'B')
                {
                    uint32_t nb = (uint32_t)Cmd_ParseFloat(cmd + 3);

                    /* 回显必须在改之前发出去: 改完就是新波特率, 上位机收到的会是乱码 */
                    if(nb < 9600u || nb > 3000000u) { cprintf("ERR: SWB<baud> 9600..3000000\r\n"); break; }
                    cprintf("UART1: baud -> %lu  (上位机请立刻改成同样的值; 波形流已自动关)\r\n",
                            (unsigned long)nb);
                    (void)Wave_SetBaud(nb);
                }
                else cprintf("ERR: SW / SW0 / SW1 / SWD<div> / SWB<baud>\r\n");
                break;
            }
            else Cmd_Status();
            break;

        case 'M':
        {
            int m = (int)val;
            if(m < 0 || m > 4) { cprintf("ERR: mode 0..4\r\n"); break; }

            if(m == Type_velocity || m == Type_angle)
            {
                if(!aligned) { cprintf("ERR: not aligned, run 'A' first\r\n"); break; }
            }

            controller = (MotionControlType)m;
            Motion_AbortPV();                       /* 先丢掉上一条 PV 轨迹/状态 */
            if(m == Type_angle) Motion_SyncAngleTarget();   /* 进位置环: 原地保持, 不要跳 */
            else                cmd_target = 0.0f;          /* 力矩 0V / 速度 0 */
            cprintf("MODE: %s\r\n", mode_name(controller));
            break;
        }

        case 'T':
            /* 力矩模式: 目标就是电压 (V) */
            cmd_target = _constrain(val, -voltage_limit, voltage_limit);
            cprintf("TARGET: %s V (torque)\r\n", FmtF(cmd_target, 3));
            break;

        case 'V':
            /* rpm (机械) -> rad/s */
            if(controller == Type_torque) { cprintf("ERR: torque mode, use T<volts>\r\n"); break; }
            cmd_target = val * _2PI / 60.0f;
            cprintf("TARGET: %s rpm -> %s rad/s\r\n", FmtF(val, 3), FmtF(cmd_target, 3));
            break;

        case 'P':
            /* 度 (绝对, 可多圈) -> rad */
            cmd_target = val * _PI / 180.0f;
            cprintf("TARGET: %s deg -> %s rad\r\n", FmtF(val, 2), FmtF(cmd_target, 3));
            break;

        case 'C':
            /* 相对圈数 */
            cmd_target += val * _2PI;
            cprintf("TARGET: %s turns -> %s rad (relative)\r\n", FmtFS(val, 4), FmtF(cmd_target, 3));
            break;

        case 'L':
            if(val > 0.0f)
            {
                voltage_limit = _constrain(val, 0.0f, voltage_power_supply / _SQRT3);
                cprintf("VLIMIT: %s V\r\n", FmtF(voltage_limit, 2));
            }
            break;

        case 'E':
            /* 合 EN 并进角度环保持当前位置 (没对齐就没法保持, 直接拒) */
            if(Motion_SetEnabled(1) != 0) cprintf("ERR: align first before enable ('A')\r\n");
            break;

        case 'D':
            Motion_SetEnabled(0);
            break;

        case 'X':
            if(cmd[1] == '0') { sensor_poll = 0; cprintf("SENSOR poll: OFF\r\n"); }
            else              { sensor_poll = 1; cprintf("SENSOR poll: ON (%d ms)\r\n", 5); }
            break;

        case 'Y':
            Cmd_SensorDiag();
            break;

        case 'R':
            Sensor_ForceRecover();
            cprintf("I2C1: force recover done (recov=%u)\r\n", (unsigned)Sensor_RecoverCount());
            break;

        case 'Q':
            Cmd_DrvDiag();
            break;

        case 'A':
            cprintf("ALIGN: start (motor must be free to move)\r\n");
            Motion_Seq_Abort();                 /* 对齐会硬拽转子: 脚本必须先停 */
            Motor_DriveSuspend(1);              /* 对齐自己写 PWM, 先让中断/主循环停手 */
            aligned = Motor_InitFOC() ? 1 : 0;
            Motor_DriveSuspend(0);
            if(aligned) Motion_SyncAngleTarget();   /* 合了 EN: 角度目标先跟住当前轴, 免得满压冲 */
            cprintf("ALIGN: %s\r\n", aligned ? "OK" : "FAILED");
            break;

        case 'K':
            if(cmd[1] == '0')
            {
                CAN_Bus_SetDebug(0);
                CAN_Bus_SetFeedback(0);
                cprintf("CAN: upstream OFF (passive, default) - no 0x180, no fault push\r\n");
            }
            else if(cmd[1] == '1')
            {
                CAN_Bus_SetDebug(1);
                CAN_Bus_SetFeedback(1);
                cprintf("CAN: upstream ON (debug) - 10ms 0x180 + fault 0x1A0\r\n");
            }
            else if(cmd[1] == 'R' || cmd[1] == 'r') { CAN_Bus_ResetCounters(); cprintf("CAN: counters reset\r\n"); }
            else CAN_Bus_PrintStatus();
            break;

        case 'J':
            CAN_Bus_SendTest();
            break;

        case 'W':
            Motion_PrintStatus();
            break;

        case 'N':
            /* 回机械原点 (0x111); 参数按 rpm 给, 跟 'V' 一致 */
            (void)Motion_GotoHome(val * _2PI / 60.0f);
            break;

        case 'F':
            /* 把当前位置设为机械 0 点 (0x112) */
            Motion_SetHomeHere();
            break;

        case '#':
        {
            /* 电机参数 (极对数 / 堵转阈值 / 母线电压), 存 Flash。
             *   '#'              看
             *   '#11'            只设极对数
             *   '#11,2.5'        设极对数 + 堵转阈值
             *   '#0,2.5'         只设堵转阈值 (pp 给 0 = 不改)
             *   '#11,2.5,24'     三个一起设
             *   '#0,0,36'        只设母线电压 (台子从 24V 电源换成 36V 时用)
             * 解析跟 '@6,3' 一个路子: 逗号分隔的十进制数, 没给的项当 0 (= 不改)。 */
            const char *cm  = strchr(cmd, ',');
            const char *cm2 = cm ? strchr(cm + 1, ',') : 0;

            if(cmd[1] == '\0')
            {
                Motion_PrintMotorCfg();
                break;
            }

            (void)Motion_SetMotorCfg((int)val,
                                     cm  ? (float)Cmd_ParseFloat(cm + 1)  : 0.0f,
                                     cm2 ? (float)Cmd_ParseFloat(cm2 + 1) : 0.0f);
            break;
        }

        case 'I':
            /* 故障现场 (0x183/0x1A0 那一套) */
            Fault_Print();
            break;

        case 'O':
            Motion_ClearFault();
            break;

        case 'B':
            Motion_Stop((val != 0.0f) ? 1u : 0u);
            break;

        case 'U':
            DRV8313_Disable();
            Delay_Ms(30);
            Motor_DriveSuspend(1);
            Motion_CalCurrentZero();
            Motor_DriveSuspend(0);
            cprintf("CS: re-zero  W=%sV V=%sV (motor must have been stopped)\r\n",
                    FmtF(CS_ZeroVolt(0), 4), FmtF(CS_ZeroVolt(1), 4));
            break;

        case 'G':
        {
            float turns = val;
            float rpm   = 0.0f;
            const char *at = strchr(cmd, '@');

            if(at) rpm = (float)Cmd_ParseFloat(at + 1);
            if(rpm == 0.0f) rpm = 30.0f;
            Motion_RunPV(turns * _2PI, rpm * _2PI / 60.0f);
            break;
        }

        case '*':
        {
            /* 板载动作脚本 (2026-09-29): '*{等待us,+-rpm,角deg,停留ms}{...}'
             *   '*<脚本>' 追加 | '*!' 跑一遍 | '*!<n>' 跑 n 遍 | '*0' 清空 | '*?' 打印 */
            const char *a1 = cmd + 1;

            while(*a1 == ' ' || *a1 == '\t') a1++;

            if(*a1 == '!')
            {
                long n = 0;
                const char *q = a1 + 1;

                while(*q >= '0' && *q <= '9') { n = n * 10 + (*q - '0'); q++; }
                (void)Motion_Seq_Start((int)(n > 0 ? n : 1));
            }
            else if(*a1 == '0')       Motion_Seq_Clear();
            else if(*a1 == '?')       Motion_Seq_Print();
            else if(*a1 == '\0')     cprintf("ERR: '*' needs {..} / ! / 0 / ? (H for help)\r\n");
            else                      (void)Motion_Seq_Append(a1);
            break;
        }

        case 'Z':
            full_rotation_offset = 0.0f;
            full_rotation_offset = 0.0f;              /* 传感器累计角归零 */
            sensor_offset = (float)sensor_direction * Sensor_GetAngle();
            shaft_angle   = 0.0f;
            cmd_target    = 0.0f;
            Motion_SaveNow();
            cprintf("ZERO: shaft angle = 0 (saved)\r\n");
            break;

        default:
            cprintf("ERR: unknown cmd '%c' (H for help)\r\n", c);
            break;
    }
}
