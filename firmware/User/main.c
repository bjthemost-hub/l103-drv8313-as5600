#include "debug.h"
#include "board.h"
#include "drv8313.h"
#include "csense.h"
#include "foc_utils.h"
#include "foc_pid.h"
#include "foc_sensor.h"
#include "foc_motor.h"
#include "cmd.h"
#include "wave.h"
#include "can_bus.h"
#include "motion_pv.h"
#include "pos_log.h"
#include "faults.h"

/* =============================================================================
 * CH32L103 + DRV8313 FOC 台架固件
 *
 * 通道:
 *   UART1 (PA9=TX / PA10=RX, COM15 @115200)  debug 输出 + 命令输入
 *   USB CDC (PA11/PA12, COM22)               命令输入 (待接)
 *   CAN1 (PB8/PB9, SN65HVD230, 1Mbps)        位置-速度命令 + 10ms 反馈
 *   注意: UART2 (PA2/PA3) 板上没接线, 不使用
 *
 * 上电默认: **不转** (力矩 0V, 三相 50% 占空 = 不励磁), 等命令。
 *           改掉原来"上电就速度开环 10rad/s"的行为: 现在由 CAN 命令驱动,
 *           上电自转既有安全风险, 也让 CAN 命令没有确定的起点。
 *           想要立刻转: UART 敲 'G<turns>@<rpm>' 或从 CAN 发 0x100。
 * 命令见 H
 * ========================================================================== */

#define BOOT_OPENLOOP_SPEED   10.0f        /* rad/s (机械角); 14 极对 -> 22.3Hz 电频率 */

static void LED_OK_Init(void)
{
    GPIO_InitTypeDef gpio;

    RCC_PB2PeriphClockCmd(RCC_PB2Periph_GPIOA, ENABLE);
    gpio.GPIO_Pin   = GPIO_Pin_15;
    gpio.GPIO_Mode  = GPIO_Mode_Out_PP;
    gpio.GPIO_Speed = GPIO_Speed_10MHz;
    GPIO_Init(GPIOA, &gpio);
    GPIO_ResetBits(GPIOA, GPIO_Pin_15);
}

static void LED_OK_Toggle(void)
{
    if(GPIO_ReadOutputDataBit(GPIOA, GPIO_Pin_15)) GPIO_ResetBits(GPIOA, GPIO_Pin_15);
    else                                           GPIO_SetBits(GPIOA, GPIO_Pin_15);
}

/* 'ST': 运行期手动踢一次 TICK 统计 -------------------------------------------
 * S0(默认) 下电机使能期间整队不跑 -> 主循环一次都不为打印停下来, 但也就
 * 没有 TICK 可看。运行中想量"从上次清零到现在主循环最长卡了多少", 敲一次
 * ST: 下一拍只做 TICK 那一件, loopmax 就是累积值, 打完 default 分支清零。
 * 只卡这一次 (~百 us), 不影响已经记录下来的最大值。 */
static volatile uint8_t tq_kick = 0;
void Main_RequestTick(void) { tq_kick = 1u; }

int main(void)
{
    uint32_t t_start, t_now, next_tick, next_sensor;
    uint32_t ctrl_loops = 0;
    uint32_t next_nfault_us = 0;
    uint32_t t_prev, t_loop, loop_gap_max = 0;
    uint8_t  tick_q = 0;                     /* 1Hz 家务队列游标 (0 = 空闲) */
    uint32_t tick_item_max = 0;              /* 本轮 1Hz 家务里"单件最长"耗时 (us) */
    uint16_t raw = 0;
    uint8_t  scl_lvl, sda_lvl, step;
    uint16_t s1, s2;
    uint8_t  i;

    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
    SystemCoreClockUpdate();
    Delay_Init();
    USART_Printf_Init(115200);
    UsartTx_Init();                 /* 打印改成非阻塞, 否则 1Hz 那一坨卡主循环 30ms */
    Wave_Init();                    /* 相电流波形流 (默认关; 'SW1' 开) */
    LED_OK_Init();

    printf("\r\n=== CH32L103 + DRV8313 FOC (build %s %s) ===\r\n", __DATE__, __TIME__);
    printf("SystemClk:%lu\r\n", (unsigned long)SystemCoreClock);

    /* ---- 1MHz 计时器 + 自检 ---- */
    Micros_Init();
    t_start = Micros();
    Delay_Ms(1000);
    t_now = Micros();
    printf("TIM4 1MHz selftest: 1000ms -> %lu us\r\n", (unsigned long)(t_now - t_start));

    /* ---- DRV8313 ---- */
    DRV8313_Init();
    printf("DRV8313: pwm=%luHz nFAULT=%d\r\n",
           (unsigned long)DRV_PWM_FREQ_HZ, (int)DRV8313_Faulted());

    /* ---- 10kHz 驱动中断: 开环时相位由它推进, 不被主循环的打印拖住 ---- */
    Motor_DriveTimerInit();
    printf("DRIVE: TIM1 10kHz ISR armed\r\n");

    /* ---- INA240 相电流反馈 (ADC1: PA1=MOT_A_V/W相, PA4=MOT_B_V/V相) ---- */
    CS_Init();
    /* 电流零点实测 (此刻 DRV8313 还没使能, 相电流为 0) —— 不再硬用 1.65V */
    CS_CalZero();
    printf("INA240: A_V=PA1(W) B_V=PA4(V) 50V/V x 10mR = 0.5V/A, zero W=%sV V=%sV\r\n",
           FmtF(CS_ZeroVolt(0), 4), FmtF(CS_ZeroVolt(1), 4));

    /* ---- AS5600 / I2C1 ---- */
    Sensor_Init();
    Sensor_BusLevels(&scl_lvl, &sda_lvl);
    printf("I2C1 idle: PB6(SCL)=%u PB7(SDA)=%u\r\n", (unsigned)scl_lvl, (unsigned)sda_lvl);
    if(Sensor_Ping(AS5600_ADDR) == 0)
    {
        printf("AS5600: ACK ok\r\n");
    }
    else
    {
        Sensor_GetDiag(&step, &s1, &s2);
        printf("AS5600: NO ACK step=%u STAR1=0x%04X STAR2=0x%04X  scan:",
               (unsigned)step, (unsigned)s1, (unsigned)s2);
        for(i = 0x08; i < 0x78; i++) if(Sensor_Ping(i) == 0) printf(" 0x%02X", (unsigned)i);
        printf(" (none)\r\n");
        Sensor_BusRecover();
    }
    if(Sensor_ReadRaw(&raw) == 0) printf("AS5600: raw=%u\r\n", (unsigned)raw);

    /* ---- FOC 参数 ---- */
    voltage_power_supply = MOTOR_VOLTAGE_SUPPLY;
    /* MOTOR_VOLTAGE_LIMIT == 0 是"跟母线走"的哨兵值, **不能直接当限幅用**。
     * 坑(2026-09-28 实测): Motor_Init() 里有一句
     *     if(voltage_sensor_align > voltage_limit) voltage_sensor_align = voltage_limit;
     * 而 Motion_ApplyBootVoltageLimit() 要等 Flash 参数恢复完才跑 (在 Motor_Init 之后)。
     * 中间这段空窗里 voltage_limit = 0, 会把 voltage_sensor_align 钳成 0V ->
     * 'A' 扫场加 0V、转子一动不动, 报 "Failed to notice movement", 且失败路径又
     * DRV8313_Disable(), 于是 'A' 再也成功不了 (死锁)。
     * 这里先给一个与母线一致的安全初值 (vbus/sqrt3), 之后 BootVoltageLimit 再按
     * Flash 里的真 vbus 精修。 */
    voltage_limit        = (MOTOR_VOLTAGE_LIMIT > 0.0f) ? MOTOR_VOLTAGE_LIMIT
                                                        : (MOTOR_VOLTAGE_SUPPLY / _SQRT3);
    voltage_sensor_align = MOTOR_VOLTAGE_ALIGN;
    velocity_limit       = MOTOR_VELOCITY_LIMIT;
    torque_controller    = Type_voltage;
    controller           = Type_torque;          /* 上电不转 (见文件头说明) */

    Motor_Init();
    PID_Init();
    Cmd_Init();

    /* ---- 位置掉电保存 / PV 运动 / CAN ---- */
    PosLog_Init();
    Motion_Init();
    (void)Motion_RestoreFromLog();
    Motion_ApplyBootVoltageLimit();     /* 上电默认 Uq 上限 = vbus/sqrt(3) */
    CAN_Bus_Init();
    Fault_Init();                       /* 故障采集/上报 (要在 CAN 起来之后) */
    printf("CAN1: ready=%d 1Mbps PB8(RX)/PB9(TX)  fb=0x%03X @%ums\r\n",
           CAN_Bus_Ready(), (unsigned)CAN_ID_FB,
           (unsigned)(CAN_FB_PERIOD_US / 1000u));
    CAN_Bus_PrintStatus();

    printf("MOT: pole_pairs=%d VM=%sV Uq_limit=%sV\r\n",
           pole_pairs, FmtF(voltage_power_supply, 1), FmtF(voltage_limit, 1));

    cmd_target  = 0.0f;                         /* 上电不转, 等命令 */
    next_tick   = Micros() + 1000000u;          /* 1s: LED 心跳 / 周期状态 */
    next_sensor = Micros() + 5000u;             /* 5ms: 持续访问 AS5600 */

    t_prev = Micros();
    while(1)
    {
        uint8_t want_isr;

        /* 主循环自己最长能卡多久 (1Hz 打印那一坨就卡在这) */
        t_loop = Micros();
        if((uint32_t)(t_loop - t_prev) > loop_gap_max) loop_gap_max = (uint32_t)(t_loop - t_prev);
        t_prev = t_loop;

        Cmd_Poll();
        CAN_Bus_Task();          /* 收 CAN 命令 + 每 10ms 发一帧反馈 */
        CS_Task();               /* 限速采相电流, 维护 5ms 窗口峰值 */
        Motion_Task();           /* PV 轨迹 / 多圈位置 / 周期存 Flash */
        Fault_Task();            /* 故障采集 + 主动上报 (0x1A0 事件 / 0x183 位图) */

        /* 开环: 驱动交给 10kHz 中断推进相位, 主循环只搬数据/打印
           闭环: 仍旧主循环驱动 (每圈要读编码器, 不适合放中断里) */
        want_isr = (controller == Type_velocity_openloop ||
                    controller == Type_angle_openloop) ? 1u : 0u;
        Motor_DriveTimerEnable(want_isr);

        /* 挂起期间 (比如 'A' 对齐正在自己写 PWM) 两边都别动占空比 */
        if(!Motor_DriveSuspended())
        {
            Motor_SetTarget(cmd_target);
            if(!want_isr)
            {
                Move(cmd_target);
                LoopFOC();
            }
        }
        ctrl_loops++;

        /* nFAULT 低: 停驱动, 但**不再死等** —— 以前那个 while(1) 会把 CAN 一起弄哑,
           故障反而报不出去。现在只禁用 + 限速打印, 故障由 faults.c 主动上报。 */
        if(DRV8313_Faulted())
        {
            DRV8313_Disable();
            if((int32_t)(Micros() - next_nfault_us) >= 0)
            {
                next_nfault_us = Micros() + 1000000u;
                printf("MOT: nFAULT LOW -> DRV disabled (reported on CAN 0x1A0/0x183)\r\n");
            }
        }

        t_now = Micros();

        /* 每 5ms 访问一次 AS5600: 示波器在 SCL/SDA 上能看到持续波形,
           同时保持累计角度连续 (闭环时由 LoopFOC 接管, 重复读也无害) */
        if((int32_t)(t_now - next_sensor) > 0)
        {
            next_sensor += 5000u;
            if(Cmd_SensorPoll()) (void)Sensor_GetAngle();
        }

        /* ---- 1Hz 家务: 排队, 下面每拍只做一件 (2026-10-06 改) --------------
         * 原来是一整块一次做完, 实测 block ~ 4.0ms, 其中 Cmd_SensorDiag 一个人就
         * ~2.8ms (4 次阻塞 I2C 读 + 浮点打印)。而**闭环 FOC 就在主循环里跑**:
         * 4ms 卡住 = 500rpm 下 12 度机械 / 132 度电角度整个偏掉 -> 电流冲 2A+
         * -> 看着/听着就是"固定每秒卡一下"。
         * 实测 (2026-10-06, 本板 500rpm 连转 12s, 0x180 里 |峰值电流| >= 2.0A 的帧):
         *   1Hz 诊断开 (X1) 19 帧   1Hz 诊断关 (X0) 1 帧
         * 所以: (1) 拆成 8 件, 每拍只做一件; (2) **电机使能期间不做传感器诊断** --
         * 那是排障用的, 停机时自会做, 运行中想看随时敲 'Y'。
         * block 含义随之改成:**本轮家务里单件最长耗时** (原来 = 整块耗时)。
         *
         * 2026-10-06 再收紧 (家人实听: "声音变得很小了, 但是还是有"):
         * 拆散之后"单件最长"仍有 ~680us —— 就是 Cmd_CurrentSense 那一行浮点打印;
         * 500rpm 下 = 2 度机械 / 23 度电角度, 还是听得见。
         * 所以**电机使能期间整队都不跑**: 转着的时候主循环一次都不为打印停下来。
         * 要看周期上报: 停机时自会有; 或敲 'S1' 强制开着 (那时会有 ~0.7ms/秒 的
         * 相位误差, 是排障手段, 别常开)。 */
        if((int32_t)(t_now - next_tick) > 0)
        {
            next_tick += 1000000u;
            if(!Motion_IsEnabled() || Cmd_AutoStatus())
            {
                tick_q        = 1u;
                tick_item_max = 0;
            }
        }

        /* 'ST' 踢一拍: 只做 TICK 那一件, 用来在运行期读累积的 loopmax */
        if(tq_kick && tick_q == 0u) { tq_kick = 0u; tick_q = 7u; }

        if(tick_q != 0u && !Motor_DriveSuspended())
        {
            uint32_t tb0 = Micros();

            switch(tick_q)
            {
                case 1u:  LED_OK_Toggle();                    break;
                case 2u:  Cmd_CurrentSense();                 break;
                case 3u:  if(Cmd_AutoStatus()) Cmd_Status();  break;
                case 4u:
                    if(Cmd_SensorPoll() && !Motion_IsEnabled()) Cmd_SensorDiag();
                    break;
                case 5u:
                    printf("FAULT: act=0x%04X lat=0x%04X ev=%lu | mech=%s rad home=%s rad | en=%d aligned=%d VM=%sV(sense=%d)\r\n",
                           (unsigned)Fault_Active(), (unsigned)Fault_Latched(),
                           (unsigned long)Fault_EventCount(),
                           FmtF(Motion_PosMechRad(), 4),
                           Motion_HomeSet() ? FmtF(Motion_HomeRad(), 4) : "n/a",
                           Motion_IsEnabled(), Cmd_IsAligned(),
                           FmtF(CS_VM_Volt(), 2), CS_VM_Available());
                    break;
                case 6u:
                    printf("CAN: rx=%lu tx=%lu txfail=%lu ovr=%lu lastid=0x%03lX | pos=%s rad vel=%s rad/s stall=%d pkw=%lu/%lu\r\n",
                           (unsigned long)CAN_Bus_RxCount(), (unsigned long)CAN_Bus_TxCount(),
                           (unsigned long)CAN_Bus_TxFail(), (unsigned long)CAN_Bus_Overrun(),
                           (unsigned long)CAN_Bus_LastId(),
                           FmtF(Motion_PosRad(), 4), FmtF(Motion_VelRad(), 4),
                           Motion_IsStalled(),
                           (unsigned long)PosLog_WriteFail(), (unsigned long)PosLog_WriteOk());
                    break;
                case 7u:
                    /* loopmax = 主循环最长一次迭代间隔
                       isrmax  = 驱动中断最长一次间隔 (开环时应约 100us, 卡住就是中断被挡) */
                    printf("TICK: block=%luus loopmax=%luus isrmax=%luus isrus=%luus loops=%lu txdrop=%lu\r\n",
                           (unsigned long)tick_item_max, (unsigned long)loop_gap_max,
                           (unsigned long)Motor_DriveGapMax(),
                           (unsigned long)Motor_DriveIsrUsMax(), (unsigned long)ctrl_loops,
                           (unsigned long)UsartTx_Drop());
                    break;
                default:
                    loop_gap_max = 0;
                    Motor_DriveGapReset();
                    ctrl_loops   = 0;
                    break;
            }

            {
                uint32_t d = Micros() - tb0;
                if(d > tick_item_max) tick_item_max = d;
            }
            tick_q = (tick_q >= 8u) ? 0u : (uint8_t)(tick_q + 1u);
        }
    }
}
