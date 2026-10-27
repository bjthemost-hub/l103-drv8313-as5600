#ifndef __FOC_UTILS_H
#define __FOC_UTILS_H

#include <math.h>
#include <stdint.h>

/* =============================================================================
 * 定点/查表数学 (SimpleFOC 风格)
 * 来源: loop222 @郑州 的 SimpleFOC C 移植 (4_closeloop_M1/SimpleFOC/foc_utils.c)
 *       教程 https://blog.csdn.net/loop222/article/details/120471390
 * ========================================================================== */

#define _sign(a)        ( ( (a) < 0 ) ? -1 : ( (a) > 0 ) )
#define _round(x)       ((x) >= 0 ? (long)((x) + 0.5f) : (long)((x) - 0.5f))
#define _constrain(amt, low, high) ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)))

/* 注意: 这些常数**必须**带 f 后缀。
 * 不带的话是 double, 表达式里 float*double 会整条升级成双精度,
 * 在 RV32 (无 FPU) 上会去调 __extendsfdf2/__muldf3/__truncdfsf2,
 * 周期是 float 版的好几倍 —— 10kHz 中断里放不下 (2026-09-19 实测踩过) */
#define _2_SQRT3        1.15470053838f
#define _SQRT3          1.73205080757f
#define _1_SQRT3        0.57735026919f
#define _SQRT3_2        0.86602540378f
#define _PI             3.14159265359f
#define _PI_2           1.57079632679f
#define _PI_3           1.0471975512f
#define _2PI            6.28318530718f
#define _3PI_2          4.71238898038f
#define _PI_6           0.52359877559f

typedef struct { float d; float q; } DQVoltage_s;
typedef struct { float d; float q; } DQCurrent_s;
typedef struct { float a; float b; float c; } PhaseCurrent_s;

/* ---- 定点打印: 不依赖 newlib 的浮点 printf (省 ~5.7KB Flash) ----
 * FmtF(v, dec)  -> "123.456"   (等价 "%.3f")
 * FmtFS(v, dec) -> 正数也带 '+' (等价 "%+.4f")
 * 返回静态环形缓冲里的字符串, 一次 printf 里最多同时用 6 个 */
const char *FmtF(float v, uint8_t dec);
const char *FmtFS(float v, uint8_t dec);

float _sin(float a);
float _cos(float a);

/* ---- 自带 sqrtf (取代 newlib 的 sqrtf) ----
 * 为什么: newlib 的 sqrtf 内部是 (float)sqrt((double)x) (lib_a-wf_sqrt.o), 会连锁
 * 拉进 __divdf3 / __truncdfsf2 / __extendsfdf2 等约 2 KB Flash, 而且在没有 FPU 的
 * RV32 上每次都要算一遍软双精度 —— SetPhaseVoltage() 是在 10kHz 里跑的。
 * 做法: 位技巧给 1/sqrt(x) 初值, 3 次牛顿迭代 (只用乘法), 最后 x*y。
 * 精度: PC 上 float32 仿真比对 math.sqrt, x 在 [1e-20, 1e20] 最坏相对误差 < 1e-6。 */
float SqrtF(float x);
/* 不用 fmodf: 软浮点下一次 fmodf 上千周期, 10kHz 中断里放不下。
   输入已是 [0,2pi) 时直接短路返回 (开环每拍就是这条快路) */
static inline float _normalizeAngle(float angle)
{
    if(angle >= _2PI || angle < 0.0f)
    {
        angle -= _2PI * (float)(long)(angle * (1.0f / _2PI));
        if(angle >= _2PI)      angle -= _2PI;
        else if(angle < 0.0f)  angle += _2PI;
    }
    return angle;
}

float _electricalAngle(float shaft_angle, int pole_pairs);

#endif
