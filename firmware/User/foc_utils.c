#include "foc_utils.h"

/* =============================================================================
 * 正弦查表: 200 点 / 90 度, 值 = sin * 10000 (int 存储省一半空间)
 * 来源: loop222 @郑州 SimpleFOC C 移植, 精度约 +-0.005
 * ========================================================================== */
const int sine_array[200] = {
    0,79,158,237,316,395,473,552,631,710,789,867,946,1024,1103,1181,1260,1338,1416,1494,
    1572,1650,1728,1806,1883,1961,2038,2115,2192,2269,2346,2423,2499,2575,2652,2728,2804,2879,2955,3030,
    3105,3180,3255,3329,3404,3478,3552,3625,3699,3772,3845,3918,3990,4063,4135,4206,4278,4349,4420,4491,
    4561,4631,4701,4770,4840,4909,4977,5046,5113,5181,5249,5316,5382,5449,5515,5580,5646,5711,5775,5839,
    5903,5967,6030,6093,6155,6217,6279,6340,6401,6461,6521,6581,6640,6699,6758,6815,6873,6930,6987,7043,
    7099,7154,7209,7264,7318,7371,7424,7477,7529,7581,7632,7683,7733,7783,7832,7881,7930,7977,8025,8072,
    8118,8164,8209,8254,8298,8342,8385,8428,8470,8512,8553,8594,8634,8673,8712,8751,8789,8826,8863,8899,
    8935,8970,9005,9039,9072,9105,9138,9169,9201,9231,9261,9291,9320,9348,9376,9403,9429,9455,9481,9506,
    9530,9554,9577,9599,9621,9642,9663,9683,9702,9721,9739,9757,9774,9790,9806,9821,9836,9850,9863,9876,
    9888,9899,9910,9920,9930,9939,9947,9955,9962,9969,9975,9980,9985,9989,9992,9995,9997,9999,10000,10000
};

/* 角度 a 必须已归一化到 [0, 2PI) */
float _sin(float a)
{
    if(a < _PI_2)
    {
        return 0.0001f * sine_array[_round(126.6873f * a)];
    }
    else if(a < _PI)
    {
        return 0.0001f * sine_array[398 - _round(126.6873f * a)];
    }
    else if(a < _3PI_2)
    {
        return -0.0001f * sine_array[-398 + _round(126.6873f * a)];
    }
    else
    {
        return -0.0001f * sine_array[796 - _round(126.6873f * a)];
    }
}

float _cos(float a)
{
    float a_sin = a + _PI_2;
    a_sin = a_sin > _2PI ? a_sin - _2PI : a_sin;
    return _sin(a_sin);
}

/* _normalizeAngle 已挪到 foc_utils.h 做成 static inline (见那边的注释) */

float _electricalAngle(float shaft_angle, int pole_pairs)
{
    return shaft_angle * (float)pole_pairs;
}

/* =============================================================================
 * 定点打印 (替代 newlib 浮点 printf)
 *   先把数值四舍五入成「缩放后的整数」, 再拆成 整数部分 / 小数部分 输出
 *   负数保留符号 (-0.500 也正确), FmtFS 正数补 '+'
 * ========================================================================== */
#define FMT_RING    6

static char    fmt_ring[FMT_RING][20];
static uint8_t fmt_idx;

static const char *fmt_impl(float v, uint8_t dec, uint8_t force_sign)
{
    char     *o = fmt_ring[fmt_idx];
    char     *p = o;
    char      tmp[12];
    uint32_t  scale = 1u;
    uint32_t  a, ip, fp;
    int32_t   s;
    uint8_t   i, n;

    fmt_idx = (uint8_t)((fmt_idx + 1u) % FMT_RING);

    if(dec > 6u) dec = 6u;
    for(i = 0; i < dec; i++) scale *= 10u;

    s = (int32_t)(v * (float)scale + (v < 0.0f ? -0.5f : 0.5f));
    if(s < 0) { *p++ = '-'; a = (uint32_t)(-s); }
    else      { if(force_sign) *p++ = '+'; a = (uint32_t)s; }

    ip = a / scale;
    fp = a % scale;

    n = 0;
    if(ip == 0u) tmp[n++] = '0';
    while(ip > 0u) { tmp[n++] = (char)('0' + (ip % 10u)); ip /= 10u; }
    while(n > 0u) *p++ = tmp[--n];

    if(dec > 0u)
    {
        *p++ = '.';
        n = dec;
        while(n > 0u) { n--; tmp[n] = (char)('0' + (fp % 10u)); fp /= 10u; }
        for(i = 0; i < dec; i++) *p++ = tmp[i];
    }
    *p = '\0';
    return o;
}

const char *FmtF(float v, uint8_t dec)  { return fmt_impl(v, dec, 0); }
const char *FmtFS(float v, uint8_t dec) { return fmt_impl(v, dec, 1); }

/* ---- 自带 sqrtf, 见 foc_utils.h 的说明 ---- */
float SqrtF(float x)
{
    union { float f; uint32_t u; } v;
    float y;

    if(x != x)      return x;               /* NaN: 原样传出去 */
    if(!(x > 0.0f)) return 0.0f;            /* 0 / 负数 -> 0 */

    v.f = x;
    v.u = 0x5f3759dfu - (v.u >> 1);         /* 1/sqrt(x) 的初值: 指数减半 */
    y   = v.f;
    y   = y * (1.5f - 0.5f * x * y * y);    /* 牛顿迭代 3 次 */
    y   = y * (1.5f - 0.5f * x * y * y);
    y   = y * (1.5f - 0.5f * x * y * y);
    return x * y;
}
