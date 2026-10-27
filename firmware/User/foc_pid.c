#include "foc_pid.h"
#include "foc_utils.h"
#include "drv8313.h"
#include "board.h"
#include "csense.h"
#include "foc_motor.h"

/* =============================================================================
 * 速度环 (P&I) / 角度环 (P&D) + 速度低通
 * 来源: loop222 @郑州 SimpleFOC C 移植 (pid.c / lowpass_filter.c)
 * 改动: Ts 由 TIM4 的 Micros() 提供 (原来用 STM32 的 SysTick->VAL)
 * ========================================================================== */

extern float voltage_limit;
extern float velocity_limit;

static float volt_limit_now(void);

static float pid_vel_P, pid_vel_I;
static float pid_ang_P, pid_ang_D;
static float integral_vel_prev;
static float error_vel_prev, error_ang_prev;
static float output_vel_ramp;
static float output_vel_prev;
static uint32_t pid_vel_timestamp, pid_ang_timestamp;
static float y_vel_prev = 0.0f;

static float Ts_Update(uint32_t *stamp)
{
    uint32_t now = Micros();
    uint32_t d   = now - *stamp;          /* 无符号差值, 自动处理 32 位回绕 */
    float Ts     = (float)d * 1e-6f;

    *stamp = now;
    if(Ts <= 0.0f || Ts > 0.5f) Ts = 1e-3f;
    return Ts;
}

void PID_Init(void)
{
    pid_vel_P       = 0.3f;      /* 2026-10-06: 0.1 -> 0.3。实测 200rpm 阶跃 t90=109ms 且只用到
                                  * 2.15V/7.51V (力矩余量很大) -> 就是 P 太小、末段全靠积分器爬。 */
    pid_vel_I       = 2.0f;
    output_vel_ramp = 100.0f;             /* V/s */
    integral_vel_prev = 0.0f;
    error_vel_prev    = 0.0f;
    output_vel_prev   = 0.0f;
    pid_vel_timestamp = Micros();

    pid_ang_P = 10.0f;
    pid_ang_D = 1.0f;
    error_ang_prev    = 0.0f;
    pid_ang_timestamp = Micros();
}

/* ---- 软电流限幅 (2026-10-06 加) --------------------------------------------
 * 这套是**电压模式** FOC: PID_Velocity 的输出直接就是相电压, 后面没有电流环。
 * 低速时反电势为 0, 环路一旦要满压, 电流就是 U/R 级 —— 2026-10-06 实测: 22V 母线、
 * L=12.70V 下, +700rpm 阶跃让电压顶到 12.7V, 相电流冲到 3A 阈值 -> 跳堵转。
 * 以前唯一的保护是"事后跳闸", 所以环路只能整体调软, 这就是"肉"的来源之一。
 *
 * 这里按**实测相电流** (CS_PeakAbs, 三相里 |i| 最大那相, 5ms 窗口峰值) 给原来的电压钳位
 * 换一个动态上限: |i| <= knee 不限; knee..stall 之间线性收到 0V。
 * 它不是电流环(没有积分、没有 d/q 分解), 只是一个"别把电流顶到跳闸"的电压天花板。
 */
static float volt_limit_now(void)
{
    float i  = CS_PeakAbs();
    float i0 = motor_stall_amp * MOTOR_ILIM_KNEE;
    float v  = voltage_limit;

    if(i <= i0)              return v;
    if(i >= motor_stall_amp) return 0.0f;
    return v * (motor_stall_amp - i) / (motor_stall_amp - i0);
}

float PID_Velocity(float error)
{
    float Ts = Ts_Update(&pid_vel_timestamp);
    float vlim = volt_limit_now();
    float proportional = pid_vel_P * error;
    float integral = integral_vel_prev + pid_vel_I * Ts * 0.5f * (error + error_vel_prev);
    float output, output_rate;

    integral = _constrain(integral, -vlim, vlim);   /* 积分上限跟着电流走: 顺便起到抗饱和 */
    output   = proportional + integral;
    output   = _constrain(output, -vlim, vlim);

    output_rate = (output - output_vel_prev) / Ts;
    if(output_rate > output_vel_ramp)       output = output_vel_prev + output_vel_ramp * Ts;
    else if(output_rate < -output_vel_ramp) output = output_vel_prev - output_vel_ramp * Ts;

    /* 斜率限幅之后必须再钳一次: 否则 output_vel_prev 会把上面的电流限幅一点点顶回去
     * (output = prev - ramp*Ts 这条分支根本不看 vlim)。电流限幅要能**立刻**把电压压下来。 */
    output = _constrain(output, -vlim, vlim);

    integral_vel_prev = integral;
    output_vel_prev   = output;
    error_vel_prev    = error;

    return output;
}

float PID_Angle(float error)
{
    float Ts = Ts_Update(&pid_ang_timestamp);
    float proportional = pid_ang_P * error;
    float derivative   = pid_ang_D * (error - error_ang_prev) / Ts;
    float output       = proportional + derivative;

    output = _constrain(output, -velocity_limit, velocity_limit);
    error_ang_prev = error;

    return output;
}

/* 速度低通: y = alpha*y_prev + (1-alpha)*x */
float LPF_Velocity(float x)
{
    float y = MOTOR_VEL_FILTER_ALPHA * y_vel_prev + (1.0f - MOTOR_VEL_FILTER_ALPHA) * x;

    y_vel_prev = y;
    return y;
}
