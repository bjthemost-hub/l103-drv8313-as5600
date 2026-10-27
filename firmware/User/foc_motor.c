#include "foc_motor.h"
#include "foc_pid.h"
#include "foc_sensor.h"
#include "drv8313.h"
#include "board.h"
#include "motion_pv.h"
#include "debug.h"

/* =============================================================================
 * FOC 核心: SVPWM / 对齐 / 开环 / 闭环
 * 来源: loop222 @郑州 SimpleFOC C 移植 (FOCMotor.c + BLDCMotor.c)
 * 改动:
 *   1) 出 PWM 改成 DRV8313_SetDuty(Ta,Tb,Tc)  (DRV8313: EN 常高, IN 打 PWM)
 *   2) Ts 用 TIM4 的 Micros()
 *   3) 极对数 / 母线电压改成 HT4310 的 14 极对 / 20V
 *      (2026-09-28 起极对数与堵转阈值是**默认值**, 真值存 Flash, 见 board.h)
 *   4) _sqrt 用 sqrtf (原版是快速近似, 只在 Ud != 0 时用到)
 * 注意: loop222 原版代码无开源许可证声明, 本文件仅本地学习用, 不外传.
 * ========================================================================== */

long  sensor_direction;
float voltage_power_supply;
float motor_stall_amp;
float voltage_limit;
float voltage_sensor_align;
int   pole_pairs;
int   pole_pairs_check;             /* 对齐估计值, 只打印/诊断用 */
float velocity_limit;

/* 角度环输出的速度上限 (在 foc_pid.c:80 的 PID_Angle 里钳位)。
 * 2026-09-19 实测: 这个值才是转速天花板, 不是电压也不是电流。 */
void Motor_SetVelocityLimit(float w)
{
    if(w < 0.1f)                     w = 0.1f;
    if(w > MOTOR_VELOCITY_LIMIT_MAX) w = MOTOR_VELOCITY_LIMIT_MAX;
    velocity_limit = w;
}

float Motor_GetVelocityLimit(void)
{
    return velocity_limit;
}

float shaft_angle;
float electrical_angle;
float shaft_velocity;
float current_sp;
float shaft_velocity_sp;
float shaft_angle_sp;
DQVoltage_s voltage;
DQCurrent_s current;
float sensor_offset      = 0.0f;
float zero_electric_angle = 0.0f;

TorqueControlType torque_controller;
MotionControlType controller;

static uint32_t open_loop_timestamp;

static int   AlignSensor(void);
static float VelocityOpenloop(float target_velocity);
static float AngleOpenloop(float target_angle);

float ShaftAngle(void)
{
    return (float)sensor_direction * Sensor_GetAngle() - sensor_offset;
}

float ShaftVelocity(void)
{
    return (float)sensor_direction * LPF_Velocity(Sensor_GetVelocity());
}

float ElectricalAngle(void)
{
    return _normalizeAngle((shaft_angle + sensor_offset) * (float)pole_pairs - zero_electric_angle);
}

void Motor_Init(void)
{
    printf("MOT: Init\r\n");

    /* 0 是"跟母线走"的哨兵, 不是"限幅 0V"; 只有真给了正限幅才允许钳对齐电压。
     * (2026-09-28: 这条 guard 是上面那种死锁的第二道防线, 见 main.c 里的说明) */
    if(voltage_limit > 0.0f && voltage_sensor_align > voltage_limit) voltage_sensor_align = voltage_limit;

    /* 下面两个是**编译默认值**; 上电时 Motion_RestoreFromLog() 会用 Flash 里的
     * 真值覆盖它们 (顺序: Motor_Init() 在 Motion_RestoreFromLog() 之前调)。 */
    pole_pairs       = MOTOR_POLE_PAIRS;
    motor_stall_amp  = MOTOR_STALL_AMP;

    /* sensor_direction 默认给 CW(+1), **不再用 UNKNOWN(0)**。
     * 原因(2026-09-19 实测): ShaftAngle() = sensor_direction*angle - offset,
     * direction=0 时它恒等于 0 -> 没对齐之前 CAN 反馈的 pos 永远是 0.0000,
     * Motion_RestoreFromLog() 从 Flash 恢复出来的位置也会被下一句 shaft_angle
     * 直接抹成 0。"是否已对齐"由 Cmd_IsAligned() 单独把关, 不靠 direction==0 表达。 */
    sensor_direction = CW;

    DRV8313_Enable();
    printf("MOT: Enable driver.\r\n");
}

/* 对齐: 电角转一圈 -> 看编码器变化方向 -> 校验极对数 -> 求零点电角 */
static int AlignSensor(void)
{
    long  i;
    float angle, mid_angle, end_angle, moved;

    printf("MOT: Align sensor.\r\n");

    /* 开扫之前**必须自己把 EN 合上**。
       2026-09-19 实测踩到的坑: 只要之前发生过一次失能
       (CAN 0x110 b0=0 / 串口 'D' / 'B1'松轴 / 上一次对齐失败),
       EN 就是低的、输出高阻。这时候扫场等于对着空气加电压:
       相电流恒为 0, 转子一动不动, 报 "mid_angle==end_angle" +
       "Failed to notice movement"; 而失败路径又调一次 DRV8313_Disable(),
       于是 'A' 再也成功不了, 死锁。
       (2026-09-19 对比实测: EN=0 时必失败; 复位后 EN=1 同一段代码
        ALIGN OK, moved=0.4341 rad, PP 14/14。)
       20ms 是留裕量的设计值, 未实测最小需要多久。 */
    DRV8313_Enable();
    Delay_Ms(20);

    /* 扫场速度不能太快: 转子要跟着电角走, 跟不上就"走得少", 极对数会被估大。
       2026-09-19: 1V/2ms 时实测只走了 0.4066 rad (理论 2pi/14=0.4488), 估成 15。 */
    for(i = 0; i <= 500; i++)
    {
        angle = _3PI_2 + _2PI * (float)i / 500.0f;
        SetPhaseVoltage(voltage_sensor_align, 0.0f, angle);
        Delay_Ms(3);
    }
    mid_angle = Sensor_GetAngle();

    for(i = 500; i >= 0; i--)
    {
        angle = _3PI_2 + _2PI * (float)i / 500.0f;
        SetPhaseVoltage(voltage_sensor_align, 0.0f, angle);
        Delay_Ms(3);
    }
    end_angle = Sensor_GetAngle();

    SetPhaseVoltage(0.0f, 0.0f, 0.0f);
    Delay_Ms(200);

    printf("MOT: mid_angle=%s end_angle=%s\r\n", FmtF(mid_angle, 4), FmtF(end_angle, 4));

    moved = fabsf(mid_angle - end_angle);
    if((mid_angle == end_angle) || (moved < 0.02f))
    {
        printf("MOT: Failed to notice movement.\r\n");
        DRV8313_Disable();
        return 0;
    }
    else if(mid_angle < end_angle)
    {
        printf("MOT: sensor_direction==CCW\r\n");
        sensor_direction = CCW;
    }
    else
    {
        printf("MOT: sensor_direction==CW\r\n");
        sensor_direction = CW;
    }

    /* 极对数: **只估不改**。
       原版 SimpleFOC 会直接用估计值覆盖 pole_pairs, 但扫场时转子可能没完全跟上
       ("走得少" -> 估得大), 覆盖会静默把电角度算错, 表现为"能走一小段就顶死"。
       电机规格已知时以 MOTOR_POLE_PAIRS 为准, 这里只把估计值打出来做核对。 */
    pole_pairs_check = (int)(_2PI / moved + 0.5f);
    printf("MOT: PP check: configured=%d estimated=%d (moved=%s rad, %s deg mech) %s\r\n",
           pole_pairs, pole_pairs_check, FmtF(moved, 4),
           FmtF(moved * 180.0f / 3.14159265f, 2),
           (pole_pairs_check == pole_pairs) ? "OK" : "WARN-mismatch (kept configured)");

    SetPhaseVoltage(voltage_sensor_align, 0.0f, _3PI_2);
    Delay_Ms(700);
    zero_electric_angle = _normalizeAngle(_electricalAngle((float)sensor_direction * Sensor_GetAngle(), pole_pairs));
    Delay_Ms(20);
    printf("MOT: Zero elec. angle:%s\r\n", FmtF(zero_electric_angle, 4));

    SetPhaseVoltage(0.0f, 0.0f, 0.0f);
    Delay_Ms(200);
    return 1;
}

int Motor_InitFOC(void)
{
    if(!AlignSensor()) return 0;

    angle_prev = Sensor_GetAngle();
    Delay_Ms(5);
    shaft_velocity = ShaftVelocity();
    Delay_Ms(5);
    shaft_angle = ShaftAngle();

    Delay_Ms(200);
    return 1;
}

void LoopFOC(void)
{
    if(controller == Type_angle_openloop || controller == Type_velocity_openloop) return;

    shaft_angle      = ShaftAngle();
    electrical_angle = ElectricalAngle();

    SetPhaseVoltage(voltage.q, voltage.d, electrical_angle);
}

void Move(float new_target)
{
    /* 开环模式不需要测速: 跳过编码器读取, 环路频率高很多 */
    if(controller != Type_velocity_openloop && controller != Type_angle_openloop)
    {
        shaft_velocity = ShaftVelocity();
    }

    switch(controller)
    {
        case Type_torque:
            if(torque_controller == Type_voltage) voltage.q = new_target;
            else                                   current_sp = new_target;
            voltage.d = 0.0f;
            break;

        case Type_velocity:
            shaft_velocity_sp = new_target;
            current_sp = PID_Velocity(shaft_velocity_sp - shaft_velocity);
            if(torque_controller == Type_voltage)
            {
                voltage.q = current_sp;
                voltage.d = 0.0f;
            }
            break;

        case Type_velocity_openloop:
            shaft_velocity_sp = new_target;
            voltage.q = VelocityOpenloop(shaft_velocity_sp);
            voltage.d = 0.0f;
            break;

        case Type_angle_openloop:
            shaft_angle_sp = new_target;
            voltage.q = AngleOpenloop(shaft_angle_sp);
            voltage.d = 0.0f;
            break;

        case Type_angle:
            shaft_angle_sp = new_target;
            /* 位置环 + **轨迹速度前馈** (2026-10-06):
             * 只有 P 的话, 跟一条速度 v 的斜坡要落后 v/P 才出力 —— P=25,
             * v_ref 峰值 12.5rad/s -> 落后 0.5rad, 跟整个 30 度行程一样大,
             * 于是全程在追, 尾巴 (最后那 0.3 度) 只能靠速度环积分器爬 (实测
             * t90~145ms 却要 t99~370ms)。把轨迹自己的参考速度直接加到速度
             * 给定上, P 只修残差。Motion_RefVel() 在非 PV_POS 时返回 0。
             * 加完再钳一次: P 输出本来就钳到 ±velocity_limit, 前馈再叠上去
             * 可能超, 超了就白白让速度环饱和。 */
            shaft_velocity_sp = PID_Angle(shaft_angle_sp - shaft_angle)
                              + MOTOR_ANG_FF_GAIN * Motion_RefVel();
            if(shaft_velocity_sp >  velocity_limit) shaft_velocity_sp =  velocity_limit;
            if(shaft_velocity_sp < -velocity_limit) shaft_velocity_sp = -velocity_limit;
            current_sp = PID_Velocity(shaft_velocity_sp - shaft_velocity);
            if(torque_controller == Type_voltage)
            {
                voltage.q = current_sp;
                voltage.d = 0.0f;
            }
            break;

        default:
            break;
    }
}

/* =============================================================================
 * SVPWM: 由 Uq/Ud + 电角算出三相占空比
 * 输入 angle_el 单位 rad; Uq/Ud 为电压 (V)
 * ========================================================================== */
void SetPhaseVoltage(float Uq, float Ud, float angle_el)
{
    float    Uout;
    uint32_t sector;
    float    T0, T1, T2;
    float    Ta, Tb, Tc;

    if(Ud != 0.0f)
    {
        Uout     = SqrtF(Ud * Ud + Uq * Uq) / voltage_power_supply;
        angle_el = _normalizeAngle(angle_el + atan2f(Uq, Ud));
    }
    else
    {
        Uout     = Uq / voltage_power_supply;
        angle_el = _normalizeAngle(angle_el + _PI_2);
    }

    /* 线性调制区上限 = 1/sqrt(3) */
    if(Uout > 0.577f)  Uout = 0.577f;
    if(Uout < -0.577f) Uout = -0.577f;

    /* 乘倒数代替 / _PI_3: 软浮点 __divsf3 一次好几百周期, 中断里每秒几千次不划算。
       边界上 SVPWM 是连续的, 常数倒数带来的舍入不会改变波形; 只防 sector 越界 */
    sector = (uint32_t)(angle_el * (1.0f / _PI_3)) + 1u;
    if(sector > 6u) sector = 6u;
    T1 = _SQRT3 * _sin((float)sector * _PI_3 - angle_el) * Uout;
    T2 = _SQRT3 * _sin(angle_el - ((float)sector - 1.0f) * _PI_3) * Uout;
    T0 = 1.0f - T1 - T2;

    switch(sector)
    {
        case 1: Ta = T1 + T2 + T0 / 2.0f; Tb = T2 + T0 / 2.0f;      Tc = T0 / 2.0f;             break;
        case 2: Ta = T1 + T0 / 2.0f;      Tb = T1 + T2 + T0 / 2.0f; Tc = T0 / 2.0f;             break;
        case 3: Ta = T0 / 2.0f;           Tb = T1 + T2 + T0 / 2.0f; Tc = T2 + T0 / 2.0f;        break;
        case 4: Ta = T0 / 2.0f;           Tb = T1 + T0 / 2.0f;      Tc = T1 + T2 + T0 / 2.0f;   break;
        case 5: Ta = T2 + T0 / 2.0f;      Tb = T0 / 2.0f;           Tc = T1 + T2 + T0 / 2.0f;   break;
        case 6: Ta = T1 + T2 + T0 / 2.0f; Tb = T0 / 2.0f;           Tc = T1 + T0 / 2.0f;        break;
        default: Ta = 0.0f; Tb = 0.0f; Tc = 0.0f; break;
    }

    DRV8313_SetDuty(Ta, Tb, Tc);
}

/* 开环速度: 电角按 target_velocity 匀速转, 幅值用 voltage_limit */
static float VelocityOpenloop(float target_velocity)
{
    uint32_t now_us = Micros();
    uint32_t d      = now_us - open_loop_timestamp;
    float    Ts     = (float)d * 1e-6f;
    float    Uq;

    open_loop_timestamp = now_us;
    if(Ts <= 0.0f || Ts > 0.5f) Ts = 1e-3f;

    shaft_angle = _normalizeAngle(shaft_angle + target_velocity * Ts);

    Uq = voltage_limit;
    SetPhaseVoltage(Uq, 0.0f, _electricalAngle(shaft_angle, pole_pairs));

    return Uq;
}

static float AngleOpenloop(float target_angle)
{
    uint32_t now_us = Micros();
    uint32_t d      = now_us - open_loop_timestamp;
    float    Ts     = (float)d * 1e-6f;
    float    Uq;

    open_loop_timestamp = now_us;
    if(Ts <= 0.0f || Ts > 0.5f) Ts = 1e-3f;

    if(fabsf(target_angle - shaft_angle) > velocity_limit * Ts)
    {
        shaft_angle += (float)_sign(target_angle - shaft_angle) * velocity_limit * Ts;
    }
    else
    {
        shaft_angle = target_angle;
    }

    Uq = voltage_limit;
    SetPhaseVoltage(Uq, 0.0f, _electricalAngle(shaft_angle, pole_pairs));

    return Uq;
}

/* =============================================================================
 * TIM1 @10kHz: 开环模式下由中断推进电角度并刷 PWM
 *   drive_isr_on = 0 时中断只清标志立刻返回, 闭环仍旧走主循环 (要读编码器, 不适合进中断)
 * ========================================================================== */
/* 10kHz 试过了: 中断体 ~几十 us 的软浮点, 直接把主循环饿死 (实测 2026-09-19)。
   3kHz 给主循环留足时间; 开环 22Hz 电频率, 3kHz 每周期还有 ~130 个台阶, 够细 */
#define DRIVE_ISR_FREQ_HZ   3000u
#define DRIVE_ISR_ARR()     ((uint16_t)(SystemCoreClock / DRIVE_ISR_FREQ_HZ - 1u))

static volatile uint8_t  drive_isr_on  = 0;
static volatile uint32_t drive_last_us = 0;
static volatile uint32_t drive_gap_max = 0;
static volatile uint32_t drive_isr_us_max = 0;
static volatile float    motor_target  = 0.0f;
static volatile uint8_t  drive_suspend = 0;

/* 对齐/标定这类"我要自己写 PWM"的场合用: 挂起后中断和主循环都不再碰占空比 */
void Motor_DriveSuspend(uint8_t on) { drive_suspend = on ? 1u : 0u; }
uint8_t Motor_DriveSuspended(void)  { return drive_suspend; }

void Motor_SetTarget(float t)
{
    motor_target = t;
}

void Motor_DriveTimerInit(void)
{
    TIM_TimeBaseInitTypeDef tb;
    NVIC_InitTypeDef        nvic;

    RCC_PB2PeriphClockCmd(RCC_PB2Periph_TIM1, ENABLE);

    TIM_DeInit(TIM1);
    tb.TIM_Prescaler         = 0;
    tb.TIM_Period            = DRIVE_ISR_ARR();
    tb.TIM_CounterMode       = TIM_CounterMode_Up;
    tb.TIM_ClockDivision     = TIM_CKD_DIV1;
    tb.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM1, &tb);

    TIM_ClearITPendingBit(TIM1, TIM_IT_Update);
    TIM_ITConfig(TIM1, TIM_IT_Update, ENABLE);

    nvic.NVIC_IRQChannel                   = TIM1_UP_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 1;
    nvic.NVIC_IRQChannelSubPriority        = 0;
    nvic.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init(&nvic);

    TIM_Cmd(TIM1, ENABLE);
}

void Motor_DriveTimerEnable(uint8_t on)
{
    if(on && !drive_isr_on)                   /* 刚打开: 清掉统计, 免得第一笔把历史算进去 */
    {
        drive_last_us = Micros();
        drive_gap_max = 0u;
        drive_isr_us_max = 0u;
    }
    drive_isr_on = on ? 1u : 0u;
}

uint32_t Motor_DriveGapMax(void)    { return drive_gap_max; }
uint32_t Motor_DriveIsrUsMax(void)  { return drive_isr_us_max; }
void     Motor_DriveGapReset(void)
{
    drive_gap_max     = 0u;
    drive_isr_us_max  = 0u;
}

void TIM1_UP_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void TIM1_UP_IRQHandler(void)
{
    uint32_t now, d, t0;

    TIM_ClearITPendingBit(TIM1, TIM_IT_Update);

    if(!drive_isr_on || drive_suspend) return;

    t0 = Micros();
    now = t0;
    d   = now - drive_last_us;
    drive_last_us = now;
    if(d > drive_gap_max) drive_gap_max = d;

    Move(motor_target);

    /* 这一拍中断自己花了多久 (1us 分辨率): 超过周期就必须降频 */
    d = Micros() - t0;
    if(d > drive_isr_us_max) drive_isr_us_max = d;
}
