#include "can_bus.h"
#include "board.h"
#include "debug.h"
#include "cmd.h"
#include "foc_motor.h"
#include "foc_sensor.h"
#include "csense.h"
#include "motion_pv.h"
#include "drv8313.h"
#include "faults.h"
#include <string.h>

/* =============================================================================
 * CAN1 (SN65HVD230) 收发 + 报文协议 + 10ms 反馈
 * -----------------------------------------------------------------------------
 * 引脚/速率定案的依据见 board.h 里 CAN_PRESCALER 那一段注释。
 * 中断向量名 USB_LP_CAN1_RX0_IRQn 与 USB 共用(见 ch32l103.h 枚举),
 * 本工程不用 USB, 所以只判 CAN 的 IT 位即可。
 * ========================================================================== */

#define CAN_RXQ_LEN     16

typedef struct {
    uint16_t id;
    uint8_t  dlc;
    uint8_t  data[8];
} CanRxItem_t;

static volatile CanRxItem_t rxq[CAN_RXQ_LEN];
static volatile uint8_t  rxq_head;
static volatile uint8_t  rxq_tail;

static volatile uint32_t n_rx, n_overrun;
static volatile uint16_t last_id;
static volatile uint8_t  last_dlc;
static uint32_t n_tx, n_txfail;
static uint32_t next_fb_us;
static uint8_t  fb_on = 1;    /* 周期反馈帧的子开关 ('K0'/'K1') */
/* 主动上报总闸 (2026-09-29 新增)。
 * 0 = **被动设备**: 除非上位机来问, 一条都不主动发。
 *   管住: 10ms 周期反馈 0x180 + 故障事件 0x1A0 + 周期故障位图 0x183。
 *   不管: "被问就答" 的 0x181/0x182/0x183/0x184 (0x113 询问 / 各条命令的 ACK)。
 * 理由: 一块 CAN 上挂十几个设备时谁都不许自己刷屏; 而且这里刷屏的代价不是"多占点带宽",
 *   是**把控制环拖死** (发帧忙等会堵主循环, 实测闭环 FOC 从 ~2000 次/秒 掉到 3~7 次/秒)。
 * 默认 0; **不存 Flash**, 每次上电都是被动的; 串口 'K1' 打开, 'K0' 关掉。 */
static uint8_t  dbg_on = 0;
static uint8_t  ready;

static int32_t rd_i32(const uint8_t *p)
{
    return (int32_t)((uint32_t)p[0] | ((uint32_t)p[1] << 8) |
                     ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24));
}

static int16_t rd_i16(const uint8_t *p)
{
    return (int16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

void CAN_Bus_Init(void)
{
    GPIO_InitTypeDef gpio = {0};
    CAN_InitTypeDef  can  = {0};
    NVIC_InitTypeDef nvic = {0};

    rxq_head = rxq_tail = 0;
    n_rx = n_overrun = 0; n_tx = n_txfail = 0;
    last_id = 0xFFFFu; last_dlc = 0;

    fb_on  = 1;          /* 子开关默认可用 */
    dbg_on = 0;          /* 但总闸默认关: 上电即被动, 不主动发任何帧 */

    RCC_PB2PeriphClockCmd(RCC_PB2Periph_AFIO | RCC_PB2Periph_GPIOB, ENABLE);
    RCC_PB1PeriphClockCmd(RCC_PB1Periph_CAN1, ENABLE);

    /* CAN_RM[1:0] = 10b -> CAN1_RX = PB8, CAN1_TX = PB9
     * 依据: 数据手册 V2.1 表2-1-1 (PB8 重映射功能列写 "CAN1_RX_2", PB9 写 "CAN1_TX_2"),
     *       注10 "重映射功能下划线后的数值表示 AFIO 寄存器中相对应位的配置值"
     *       -> CAN_RM[1:0] = 10b。
     * 这里直接写寄存器, 不用 GPIO_Remap1_CAN1 宏: 那个宏的编码跟位域位置有关,
     * 直接写 CAN_RM 的 13/14 位不会歧义。 */
    AFIO->PCFR1 &= ~(0x3u << 13);
    AFIO->PCFR1 |=  (0x2u << 13);

    /* PB9 = CAN1_TX: 复用推挽; PB8 = CAN1_RX: 输入上拉 */
    gpio.GPIO_Pin   = GPIO_Pin_9;
    gpio.GPIO_Mode  = GPIO_Mode_AF_PP;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &gpio);

    gpio.GPIO_Pin   = GPIO_Pin_8;
    gpio.GPIO_Mode  = GPIO_Mode_IPU;
    GPIO_Init(GPIOB, &gpio);

    CAN_DeInit(CAN1);

    /* 不做软件滤波: 默认全通, 标准帧/扩展帧都收进来, 在软件里按 ID 分派。
       (参考同板子跑通过的 CH32L103_USB2CAN_DM/User/can_bsp.c, 它也没配滤波) */
    can.CAN_TTCM   = DISABLE;
    can.CAN_ABOM   = ENABLE;            /* 自动 bus-off 恢复 */
    can.CAN_AWUM   = DISABLE;
    can.CAN_NART   = DISABLE;           /* 仲裁失败自动重传 */
    can.CAN_RFLM   = DISABLE;
    can.CAN_TXFP   = ENABLE;            /* 按请求顺序发 */
    can.CAN_Mode   = CAN_Mode_Normal;
    can.CAN_SJW    = CAN_SJW_1tq;
    can.CAN_BS1    = CAN_BS1_TQ;
    can.CAN_BS2    = CAN_BS2_TQ;
    can.CAN_Prescaler = CAN_PRESCALER;
    /* 注意: 这个库的返回值是 CAN_InitStatus_Success = 1 (ch32l103_can.h),
       不是"0 表示成功"。 */
    ready = (CAN_Init(CAN1, &can) == CAN_InitStatus_Success) ? 1 : 0;

    CAN_ITConfig(CAN1, CAN_IT_FMP0, ENABLE);

    nvic.NVIC_IRQChannel                   = USB_LP_CAN1_RX0_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 3;
    nvic.NVIC_IRQChannelSubPriority        = 0;
    nvic.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init(&nvic);

    next_fb_us = Micros() + CAN_FB_PERIOD_US;
}

int CAN_Bus_Ready(void) { return (int)ready; }

int CAN_Bus_SendStd(uint16_t id, const uint8_t *data, uint8_t dlc)
{
    CanTxMsg tx = {0};
    uint8_t  mbox;
    uint32_t t0;

    if(dlc > 8u) dlc = 8u;

    tx.StdId = id;
    tx.IDE   = CAN_Id_Standard;
    tx.RTR   = CAN_RTR_Data;
    tx.DLC   = dlc;
    if(dlc) memcpy(tx.Data, data, dlc);

    /* 有界等待: 按**时间**算, 不按圈数算 (理由见 board.h 的 CAN_TX_TIMEOUT_US) */
    t0 = Micros();

    for(;;)
    {
        mbox = CAN_Transmit(CAN1, &tx);
        if(mbox != CAN_NO_MB) break;
        if((uint32_t)(Micros() - t0) > CAN_TX_TIMEOUT_US) { n_txfail++; return 1; }
    }

    while(CAN_TransmitStatus(CAN1, mbox) == CAN_TxStatus_Pending)
    {
        if((uint32_t)(Micros() - t0) > CAN_TX_TIMEOUT_US)
        {
            CAN_CancelTransmit(CAN1, mbox);   /* 别空转、别堵主循环: 宁可丢这一帧 */
            n_txfail++;
            return 1;
        }
    }

    if(CAN_TransmitStatus(CAN1, mbox) != CAN_TxStatus_Ok) { n_txfail++; return 1; }
    n_tx++;
    return 0;
}

void CAN_Bus_SetFeedback(uint8_t on) { fb_on = on ? 1u : 0u; }
uint8_t CAN_Bus_FeedbackOn(void)     { return fb_on; }

/* 主动上报总闸 (2026-09-29 新增): 0 = 被动 (默认) */
void    CAN_Bus_SetDebug(uint8_t on) { dbg_on = on ? 1u : 0u; }
uint8_t CAN_Bus_DebugOn(void)        { return dbg_on; }

/* ---- 收到命令: PV ---- */
static void send_status_frame(uint16_t id);
static void send_cfg_frame(void);
static void send_motorcfg_frame(void);

static void handle_pv(const uint8_t *d, uint8_t dlc)
{
    uint8_t f;
    float   tgt, vlim;
    int32_t mrad;
    int16_t vc;

    if(dlc < 7u) return;

    f    = d[6];
    mrad = rd_i32(d + 0);
    vc   = rd_i16(d + 4);

    if(f & 0x08u) Motion_ClearFault();

    if(!(f & 0x02u))                       /* 停止 */
    {
        Motion_Stop((f & 0x04u) ? 1u : 0u);
        return;
    }

    if(!Cmd_IsAligned())
    {
        printf("CAN: PV rejected - not aligned (run 'A' first)\r\n");
        Fault_ReportReject(CAN_ID_CMD_PV, CAN_FAULT_NOT_ALIGNED);
        return;
    }

    tgt  = (float)mrad * 0.001f;
    vlim = (float)vc * 0.01f;
    if(vlim < 0.0f) vlim = -vlim;

    if(f & 0x01u)
    {
        tgt += shaft_angle;                /* 相对: 相对收到命令这一刻 */
    }
    else if(f & 0x20u)
    {
        /* 机械坐标: 目标 = 机械位置 + 机械零位 */
        if(!Motion_HomeSet())
        {
            printf("CAN: PV rejected - no mechanical zero (send 0x112 first)\r\n");
            Fault_ReportReject(CAN_ID_CMD_PV, CAN_FAULT_CMD_REJECT);
            return;
        }
        tgt += Motion_HomeRad();
    }

    Motion_RunPV(tgt, vlim);
    if(f & 0x10u) Motion_SaveNow();
}

/* ---- 收到命令: 动作脚本加一段 (0x120, 2026-09-29) ----
 *   b0..1 uint16 开始前等待 [100us]   b2..3 int16 速度 [rpm, 符号 = 方向]
 *   b4..5 int16 角度 [0.1 度] (必须正数)   b6..7 uint16 跑完停留 [10ms]
 *   例: 100rpm 正转 90 度, 停 0.5s, 不等 -> 00 00 64 00 84 03 32 00
 *   上限: rpm 1432 / 角度 +-9.1 圈 (int16 x0.1 度) / 等待 6.5535s / 停留 655.35s
 *   要单段 >3276.7 度 -> 用 0x122 (同布局, 角度 1 度/LSB) */
static void handle_seg(const uint8_t *d, uint8_t dlc)
{
    long  wait_us, dwell_ms;
    int   rpm;
    float deg;

    if(dlc < 8u) return;

    wait_us  = (long)((uint16_t)(d[0] | ((uint16_t)d[1] << 8))) * 100L;
    rpm      = (int)rd_i16(d + 2);
    deg      = (float)rd_i16(d + 4) * 0.1f;
    dwell_ms = (long)((uint16_t)(d[6] | ((uint16_t)d[7] << 8))) * 10L;

    if(Motion_Seq_AddSeg(wait_us, rpm, deg, dwell_ms) != 0)
    {
        Fault_ReportReject(CAN_ID_CMD_SEG, CAN_FAULT_CMD_REJECT);
        return;
    }
    send_status_frame(CAN_ID_FB_QUERY);
}

/* ---- 收到命令: 动作脚本加一段 · 大角度版 (0x122, 2026-10-06) ----
 *   布局与 0x120 **逐字节相同**, 只有角度字段的刻度不同:
 *     0x120: b4..5 int16 角度 [0.1 度] -> 单段上限 3276.7 度 (细, 适合短段)
 *     0x122: b4..5 int16 角度 [1   度] -> 单段上限 32767 度 (粗, 适合长段 >3276.7 度)
 *   8 个字节已经被 wait/rpm/角度/dwell 占满, 角度要更大就只能牺牲分辨率 ——
 *   长程段用 1 度粒度没有实际损失(1 度 = 0.0028 圈), 短段继续用 0x120。
 *   两条最终都进同一个 Motion_Seq_AddSeg(): 落点/同向接续行为完全一致。
 *   上限: rpm 1432 / 角度 +-32767 度 / 等待 6.5535s / 停留 655.35s */
static void handle_seg_hr(const uint8_t *d, uint8_t dlc)
{
    long  wait_us, dwell_ms;
    int   rpm;
    float deg;

    if(dlc < 8u) return;

    wait_us  = (long)((uint16_t)(d[0] | ((uint16_t)d[1] << 8))) * 100L;
    rpm      = (int)rd_i16(d + 2);
    deg      = (float)rd_i16(d + 4) * 1.0f;
    dwell_ms = (long)((uint16_t)(d[6] | ((uint16_t)d[7] << 8))) * 10L;

    if(Motion_Seq_AddSeg(wait_us, rpm, deg, dwell_ms) != 0)
    {
        Fault_ReportReject(CAN_ID_CMD_SEG_HR, CAN_FAULT_CMD_REJECT);
        return;
    }
    send_status_frame(CAN_ID_FB_QUERY);
}

/* ---- 收到命令: 动作脚本控制 (0x121) ----
 *   b0 0=清空队列  1=开始  2=打印队列;   b1 = 遍数 (0 或 1 = 一遍) */
static void handle_segctl(const uint8_t *d, uint8_t dlc)
{
    uint8_t sub   = (dlc > 0u) ? d[0] : 0u;
    int     loops = (dlc > 1u) ? (int)d[1] : 1;

    if(sub == 0u)
    {
        Motion_Seq_Clear();
    }
    else if(sub == 1u)
    {
        if(Motion_Seq_Start(loops) != 0)
        {
            Fault_ReportReject(CAN_ID_CMD_SEGCTL, CAN_FAULT_CMD_REJECT);
            return;
        }
    }
    else
    {
        Motion_Seq_Print();
    }
    send_status_frame(CAN_ID_FB_QUERY);
}

/* ---- 收到命令: 纯速度 ---- */
static void handle_vel(const uint8_t *d, uint8_t dlc)
{
    uint8_t f;
    int16_t vc;

    if(dlc < 3u) return;
    f  = d[2];
    vc = rd_i16(d + 0);

    if(f & 0x08u) Motion_ClearFault();

    if(!(f & 0x02u))
    {
        Motion_Stop((f & 0x04u) ? 1u : 0u);
        return;
    }

    if(!Cmd_IsAligned())
    {
        printf("CAN: VEL rejected - not aligned (run 'A' first)\r\n");
        Fault_ReportReject(CAN_ID_CMD_VEL, CAN_FAULT_NOT_ALIGNED);
        return;
    }

    Motion_RunVEL((float)vc * 0.01f);
}

/* ---- 收到命令: 设角度环速度上限 (0x115) ----
 * b0..b1 = int16 x0.01 rad/s。这就是 PID_Angle 输出的钳位值,
 * 也是 CAN 的 vlim 给再大也没用的那个天花板。 */
static void handle_vlim(const uint8_t *d, uint8_t dlc)
{
    int16_t vc;

    if(dlc < 2u) return;
    vc = rd_i16(d + 0);
    if(vc <= 0) return;

    Motor_SetVelocityLimit((float)vc * 0.01f);
    printf("CAN: VLIM(angle) = %s rad/s\r\n", FmtF(Motor_GetVelocityLimit(), 3));
    send_status_frame(CAN_ID_FB_QUERY);
}

/* ---- 收到命令: 设电机参数 (0x116) ----
 * b0 = 极对数 (0 = 不改), b1..2 = int16 堵转阈值 0.01A (0 = 不改),
 * b4..5 = int16 母线电压 0.01V (0 = 不改; dlc < 6 时当没给),
 * b3 bit0 = 1 才是"确认要改" —— 这几个参数会一直留在 Flash 里, 加个防呆位。
 * 范围校验 / 越界拒绝 / 改极对数要求先失能 / 作废对齐, 全在 Motion_SetMotorCfg()。 */
static void handle_motorcfg(const uint8_t *d, uint8_t dlc)
{
    int     pp;
    float   amp, vbus;
    int16_t ca, cv;

    if(dlc < 4u)
    {
        printf("CAN: MOTORCFG rejected - dlc=%u < 4\r\n", (unsigned)dlc);
        Fault_ReportReject(CAN_ID_CMD_MOTORCFG, CAN_FAULT_CMD_REJECT);
        return;
    }

    if(!(d[3] & 0x01u))
    {
        printf("CAN: MOTORCFG rejected - b3 bit0=0 (set 1 to confirm a param write)\r\n");
        Fault_ReportReject(CAN_ID_CMD_MOTORCFG, CAN_FAULT_CMD_REJECT);
        return;
    }

    pp  = (int)d[0];
    ca  = rd_i16(d + 1);
    amp = (float)ca * 0.01f;

    /* 母线电压只有 dlc>=6 才带; 0/负数都当"不改" */
    vbus = 0.0f;
    if(dlc >= 6u)
    {
        cv = rd_i16(d + 4);
        if(cv > 0) vbus = (float)cv * 0.01f;
    }

    if(Motion_SetMotorCfg(pp, amp, vbus) != 0)
    {
        printf("CAN: MOTORCFG rejected (see reason line above)\r\n");
        Fault_ReportReject(CAN_ID_CMD_MOTORCFG, CAN_FAULT_CMD_REJECT);
        return;
    }

    send_motorcfg_frame();
    send_cfg_frame();
    send_status_frame(CAN_ID_FB_QUERY);   /* 改极对数会清 aligned, 让主机立刻看到 */
}

/* ---- 收到命令: 使能 / 失能 (0x110) ---- */
static void handle_enable(const uint8_t *d, uint8_t dlc)
{
    uint8_t on = (dlc >= 1u) ? (d[0] ? 1u : 0u) : 1u;

    if(Motion_SetEnabled(on) != 0)
    {
        Fault_ReportReject(CAN_ID_CMD_ENABLE, CAN_FAULT_NOT_ALIGNED);
        return;
    }

    send_status_frame(CAN_ID_FB_QUERY);
}

/* ---- 收到命令: 回机械原点 (0x111) ---- */
static void handle_home(const uint8_t *d, uint8_t dlc)
{
    float vlim = 0.0f;

    if(dlc >= 2u) vlim = (float)rd_i16(d) * 0.01f;

    if(Motion_GotoHome(vlim) != 0)
    {
        Fault_ReportReject(CAN_ID_CMD_HOME, CAN_FAULT_CMD_REJECT);
        return;
    }

    send_status_frame(CAN_ID_FB_QUERY);
}

/* ---- 收到命令: 这里设为机械 0 点 (0x112) ---- */
static void handle_setzero(const uint8_t *d, uint8_t dlc)
{
    (void)d; (void)dlc;

    if(!Cmd_IsAligned())
    {
        printf("CAN: SETZERO rejected - not aligned (run 'A' first)\r\n");
        Fault_ReportReject(CAN_ID_CMD_SETZERO, CAN_FAULT_NOT_ALIGNED);
        return;
    }

    Motion_SetHomeHere();
    send_status_frame(CAN_ID_FB_QUERY);
    send_cfg_frame();
}

/* ---- 收到命令: 询问 (0x113) ---- */
static void handle_query(const uint8_t *d, uint8_t dlc)
{
    uint8_t what = (dlc >= 1u) ? d[0] : 0u;

    if(what == 0xFFu)
    {
        send_status_frame(CAN_ID_FB_QUERY);
        send_cfg_frame();
        Fault_SendBitmap(CAN_ID_FB_FAULT);
        send_motorcfg_frame();
        return;
    }

    switch(what)
    {
        case 1u:  send_cfg_frame();                  break;
        case 2u:  Fault_SendBitmap(CAN_ID_FB_FAULT); break;
        case 3u:  send_motorcfg_frame();             break;
        default:  send_status_frame(CAN_ID_FB_QUERY); break;
    }
}

/* ---- 收到命令: 故障上报配置 (0x114) ---- */
static void handle_faultcfg(const uint8_t *d, uint8_t dlc)
{
    uint8_t on = 1u, per = 10u;

    Fault_GetReport(&on, &per);
    if(dlc >= 1u) on  = d[0] ? 1u : 0u;
    if(dlc >= 2u) per = d[1];
    Fault_SetReport(on, per);

    printf("CAN: fault report %s period=%u (x100ms)\r\n", on ? "ON" : "OFF", (unsigned)per);
    send_cfg_frame();
}

/* 即时状态帧 0x181 (布局与 0x180 相同; status 见 can_bus.h) */
static void send_status_frame(uint16_t id)
{
    uint8_t d[8];
    int32_t pos_m;
    int16_t vel_c;
    float   amp;
    int32_t cur;

    pos_m = (int32_t)(Motion_PosMechRad() * 1000.0f);   /* 机械坐标 */
    vel_c = (int16_t)(Motion_VelRad() * 100.0f);
    amp   = Motion_PeakAmp();
    cur   = (int32_t)(amp * 10.0f);
    if(cur >  127) cur =  127;
    if(cur < -127) cur = -127;

    d[0] = (uint8_t)(pos_m & 0xFF);
    d[1] = (uint8_t)((pos_m >> 8) & 0xFF);
    d[2] = (uint8_t)((pos_m >> 16) & 0xFF);
    d[3] = (uint8_t)((pos_m >> 24) & 0xFF);
    d[4] = (uint8_t)(vel_c & 0xFF);
    d[5] = (uint8_t)((vel_c >> 8) & 0xFF);
    d[6] = Motion_StatusBits();
    d[7] = (uint8_t)(int8_t)cur;

    (void)CAN_Bus_SendStd(id, d, 8);
}

/* 配置帧 0x182 */
static void send_cfg_frame(void)
{
    uint8_t  d[8];
    uint8_t  on = 1u, per = 10u;
    int32_t  home_m;
    float    v;
    uint16_t vv;

    home_m = (int32_t)(Motion_HomeRad() * 1000.0f);
    Fault_GetReport(&on, &per);

    v = CS_VM_Volt() * 10.0f;
    if(v < 0.0f)      v = 0.0f;
    if(v > 65535.0f)  v = 65535.0f;
    vv = (uint16_t)v;

    d[0] = (uint8_t)(home_m & 0xFF);
    d[1] = (uint8_t)((home_m >> 8) & 0xFF);
    d[2] = (uint8_t)((home_m >> 16) & 0xFF);
    d[3] = (uint8_t)((home_m >> 24) & 0xFF);
    d[4] = (uint8_t)(vv & 0xFF);
    d[5] = (uint8_t)((vv >> 8) & 0xFF);

    d[6] = 0u;
    if(Motion_HomeSet())   d[6] |= 0x01u;
    if(Cmd_IsAligned())    d[6] |= 0x02u;
    if(CS_VM_Available())  d[6] |= 0x04u;
    if(Motion_IsEnabled()) d[6] |= 0x08u;
    if(on)                 d[6] |= 0x10u;

    d[7] = per;

    (void)CAN_Bus_SendStd(CAN_ID_FB_CFG, d, 8);
}

/* 电机参数帧 0x184 (2026-09-28 新增; b4..5 母线电压是同日后来加的) */
static void send_motorcfg_frame(void)
{
    uint8_t  d[8];
    int16_t  ca, cv;
    float    a;

    a  = Motion_StallAmp();
    if(a < 0.0f) a = 0.0f;
    ca = (int16_t)(a * 100.0f + 0.5f);

    cv = (int16_t)(Motion_Vbus() * 100.0f + 0.5f);

    d[0] = (uint8_t)Motion_PolePairs();
    d[1] = (uint8_t)(ca & 0xFF);
    d[2] = (uint8_t)((ca >> 8) & 0xFF);

    d[3] = 0u;
    if(Motion_CfgFromFlash()) d[3] |= 0x01u;
    if(Cmd_IsAligned())       d[3] |= 0x02u;
    if(Motion_IsEnabled())    d[3] |= 0x04u;

    d[4] = (uint8_t)(cv & 0xFF);
    d[5] = (uint8_t)((cv >> 8) & 0xFF);
    d[6] = (uint8_t)Motion_Seq_Remain();   /* 脚本: 这一遍还剩几段 */
    d[7] = (uint8_t)Motion_Seq_Pass();     /* 脚本: 正在跑第几遍 (0 = 没跑) */

    (void)CAN_Bus_SendStd(CAN_ID_FB_MOTORCFG, d, 8);
}

static void send_feedback(void) { send_status_frame(CAN_ID_FB); }

void CAN_Bus_Task(void)
{
    /* 1) 先处理收到的命令 */
    while(rxq_tail != rxq_head)
    {
        CanRxItem_t it;
        uint8_t i;

        it.id  = rxq[rxq_tail].id;
        it.dlc = rxq[rxq_tail].dlc;
        for(i = 0; i < 8u; i++) it.data[i] = rxq[rxq_tail].data[i];
        rxq_tail = (uint8_t)((rxq_tail + 1u) % CAN_RXQ_LEN);

        if(it.id == CAN_ID_CMD_PV)            handle_pv(it.data, it.dlc);
        else if(it.id == CAN_ID_CMD_VEL)      handle_vel(it.data, it.dlc);
        else if(it.id == CAN_ID_CMD_ENABLE)   handle_enable(it.data, it.dlc);
        else if(it.id == CAN_ID_CMD_HOME)     handle_home(it.data, it.dlc);
        else if(it.id == CAN_ID_CMD_SETZERO)  handle_setzero(it.data, it.dlc);
        else if(it.id == CAN_ID_CMD_QUERY)    handle_query(it.data, it.dlc);
        else if(it.id == CAN_ID_CMD_FAULTCFG) handle_faultcfg(it.data, it.dlc);
        else if(it.id == CAN_ID_CMD_VLIM)     handle_vlim(it.data, it.dlc);
        else if(it.id == CAN_ID_CMD_MOTORCFG) handle_motorcfg(it.data, it.dlc);
        else if(it.id == CAN_ID_CMD_SEG)      handle_seg(it.data, it.dlc);
        else if(it.id == CAN_ID_CMD_SEGCTL)   handle_segctl(it.data, it.dlc);
        else if(it.id == CAN_ID_CMD_SEG_HR)   handle_seg_hr(it.data, it.dlc);
    }

    /* 2) 10ms 反馈 —— **只在 debug 打开时发** (默认被动: 上位机不问就不发) */
    if(dbg_on && fb_on)
    {
        uint32_t now = Micros();
        if((int32_t)(now - next_fb_us) >= 0)
        {
            next_fb_us += CAN_FB_PERIOD_US;
            /* 落后太多 (比如刚做完 flash 擦写) 就重新对齐, 免得追着补发一串 */
            if((int32_t)(now - next_fb_us) > (int32_t)CAN_FB_PERIOD_US)
                next_fb_us = now + CAN_FB_PERIOD_US;
            send_feedback();
        }
    }
    else
    {
        next_fb_us = Micros() + CAN_FB_PERIOD_US;
    }
}

void USB_LP_CAN1_RX0_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));

void USB_LP_CAN1_RX0_IRQHandler(void)
{
    if(CAN_GetITStatus(CAN1, CAN_IT_FMP0) != RESET)
    {
        while(CAN_MessagePending(CAN1, CAN_FIFO0))
        {
            CanRxMsg rx;
            uint8_t  next;

            CAN_Receive(CAN1, CAN_FIFO0, &rx);

            next = (uint8_t)((rxq_head + 1u) % CAN_RXQ_LEN);
            if(next != rxq_tail)
            {
                rxq[rxq_head].id  = (uint16_t)(rx.StdId & 0x7FFu);
                rxq[rxq_head].dlc = (uint8_t)rx.DLC;
                memcpy((void *)rxq[rxq_head].data, rx.Data, 8);
                rxq_head = next;
            }
            else
            {
                n_overrun++;
            }

            n_rx++;
            last_id  = (uint16_t)(rx.StdId & 0x7FFu);
            last_dlc = (uint8_t)rx.DLC;
        }
        CAN_ClearITPendingBit(CAN1, CAN_IT_FMP0);
    }
}

uint32_t CAN_Bus_RxCount(void)  { return n_rx; }
uint32_t CAN_Bus_TxCount(void)  { return n_tx; }
uint32_t CAN_Bus_TxFail(void)   { return n_txfail; }
uint32_t CAN_Bus_Overrun(void)  { return n_overrun; }
uint32_t CAN_Bus_LastId(void)   { return (uint32_t)last_id; }
int      CAN_Bus_LastRxDlc(void) { return (int)last_dlc; }

void CAN_Bus_ResetCounters(void)
{
    n_rx = 0; n_tx = 0; n_txfail = 0; n_overrun = 0;
}

void CAN_Bus_SendTest(void)
{
    uint8_t d[8] = { 0xA5, 0x5A, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06 };
    int r = CAN_Bus_SendStd(CAN_ID_FB, d, 8);

    printf("CAN: test frame id=0x%03X -> %s (tx=%lu fail=%lu)\r\n",
           (unsigned)CAN_ID_FB, r ? "FAIL" : "sent",
           (unsigned long)n_tx, (unsigned long)n_txfail);
}

void CAN_Bus_PrintStatus(void)
{
    uint32_t errsr = CAN1->ERRSR;
    uint32_t statr = CAN1->STATR;

    printf("CAN: ready=%d fb=%s upstream(debug)=%s n_rx=%lu n_tx=%lu tx_fail=%lu overrun=%lu\r\n",
           (int)ready, fb_on ? "ON" : "OFF", dbg_on ? "ON" : "OFF",
           (unsigned long)n_rx, (unsigned long)n_tx,
           (unsigned long)n_txfail, (unsigned long)n_overrun);
    printf("CAN: bitrate=%luHz presc=%u bs1=%u bs2=%u (PCLK1=%luHz) | last rx id=0x%03lX dlc=%d\r\n",
           (unsigned long)(SystemCoreClock / 2u / CAN_PRESCALER / 24u),
           (unsigned)CAN_PRESCALER, 15u, 8u,
           (unsigned long)(SystemCoreClock / 2u),
           (unsigned long)last_id, (int)last_dlc);
    printf("CAN: STATR=0x%08lX ERRSR=0x%08lX TEC=%lu REC=%lu LEC=0x%lX\r\n",
           (unsigned long)statr, (unsigned long)errsr,
           (unsigned long)((errsr >> 16) & 0xFFu), (unsigned long)((errsr >> 24) & 0xFFu),
           (unsigned long)(errsr & 0x07u));
    /* BTIMR 直接读出来核对波特率: 位定义见 ch32l103_can.c CAN_Init() 里那两行 */
    printf("CAN: BTIMR=0x%08lX -> mode=%lu sjw=%lu bs2=%lu(+1) bs1=%lu(+1) presc=%lu  -> %lu bit/s\r\n",
           (unsigned long)CAN1->BTIMR,
           (unsigned long)((CAN1->BTIMR >> 30) & 0x3u),
           (unsigned long)((CAN1->BTIMR >> 24) & 0x3u),
           (unsigned long)((CAN1->BTIMR >> 20) & 0x7u),
           (unsigned long)((CAN1->BTIMR >> 16) & 0xFu),
           (unsigned long)((CAN1->BTIMR & 0x3FFu) + 1u),
           (unsigned long)((SystemCoreClock / 2u) /
                           (((CAN1->BTIMR & 0x3FFu) + 1u) *
                            (1u + ((CAN1->BTIMR >> 20) & 0x7u) + 1u + ((CAN1->BTIMR >> 16) & 0xFu) + 1u))));
    printf("CAN: AFIO_PCFR1=0x%08lX (CAN_RM[14:13]=%lu)\r\n",
           (unsigned long)AFIO->PCFR1,
           (unsigned long)((AFIO->PCFR1 >> 13) & 0x3u));
}
