#include "motion_pv.h"
#include "board.h"
#include "debug.h"
#include "can_bus.h"
#include "cmd.h"
#include "csense.h"
#include "drv8313.h"
#include "foc_motor.h"
#include "foc_sensor.h"
#include "foc_utils.h"
#include "pos_log.h"
#include <math.h>

/* =============================================================================
 * 位置-速度 (PV) 轨迹 + 多圈位置 + 堵转保护
 *
 * 轨迹: 参考速度 v = min(速度上限, sqrt(2*a_dec*|剩余距离|)) 朝目标推,
 *       参考位置积分出来交给角度环跟。目标速度是"上限"而不是"恒定速度",
 *       所以"转到 X 度, 用不超过 Y rpm"这句话可以直接映射成一条命令。
 *
 * 多圈: 位置以 Sensor_GetAngle() 的累计角为准 (AS5600 本身只有单圈)。
 *
 * 保存策略 (2026-10-06 再定案: **跑动中一次都不写; 连"停稳后补一条"也去掉**) ——
 *   只在这几个**显式事件**上写, 一个不多:
 *     停止('B') / 堵转跳闸 / 失能 / 设机械零位('F') / 清零位 / 改电机参数('#'、0x116)
 *     —— 就是 motion_save_internal() 的那几个调用点。
 *   **不再有任何周期性/自动保存**: Motion_Task 里那条"停稳 0.5s 后补一条"已删。
 *   为什么 (2026-10-06 本机实测): 页擦除典型 7.5ms、页编程典型 2.5ms
 *   (数据手册 V2.1 表3-18), 擦写期间 CPU 取指停摆; 而**闭环 FOC 是在主循环里
 *   跑的** (只有开环搬进了 10kHz 中断, 见 foc_motor.h), 所以每写一次 LoopFOC()
 *   就断一拍, 轴自由滑一下。500rpm 连续转时听感就是**每秒一声"咔哒"**:
 *   实测位置下陷严格每 1.000s 一次、串口 pkw 每秒 +1、
 *   loopmax 3.7ms(编程) / 9.6ms(每 9 次一次整页擦)。
 *   2026-10-06 家人当场否掉"跑动中顺手存一下"这种做法 —— 高速运转的电机旁边,
 *   一次都不能写。这条自动记录本来也只用于上电恢复"上次停在哪",
 *   **没写上就算了, 它不重要**。
 *   少写还有第二个好处: 擦写寿命有限 (表3-19: 25C 300K 次)。
 * ========================================================================== */

enum { PV_IDLE = 0, PV_POS, PV_VEL, PV_HOLD };

static uint8_t  mode;
static uint8_t  arrived;
static uint8_t  restored;
static uint8_t  stalled;
static float    tgt_rad;
static float    vlim_rad;
static float    ref_rad;
static float    vref_rad;
static uint32_t t_prev_us;
static uint32_t last_save_us;
static uint32_t sens_err_seen;
static uint32_t sens_err_us;
static float    home_off_rad;       /* 机械零位在原始坐标系里的位置 */
static uint8_t  home_set;           /* 1 = 用户设过机械零位 */
static uint8_t  cfg_from_flash;     /* 1 = 极对数/堵转阈值取自 Flash 记录 (2026-09-28) */

/* ---- 板载动作脚本: 常量/结构/变量 (函数实现在 Motion_Seq_* 那一段) ---- */

#define SEQ_MAX_SEG          MOTION_SEQ_MAX_SEG
#define SEQ_WAIT_MAX_US      10000000L      /* 单段"开始前等待"上限 10s (设计值) */
#define SEQ_DWELL_MAX_MS     60000L         /* 单段"跑完停留"上限 60s (设计值) */
#define SEQ_DEG_MAX          36000.0f       /* 单段角度上限 = 100 圈 (设计值) */
#define SEQ_SETTLE_RAD       0.02f          /* 参考到位后再等轴进到 0.02rad (设计值) */
#define SEQ_SETTLE_MAX_US    800000UL       /* 等不到最多再等 800ms 就走 (原 100ms)
                                             * 2026-10-06 实测 (500rpm 3200 度, 10Hz 采 S):
                                             * 参考到位那一刻轴还差 4.99rad, 之后 |残差| 按
                                             * 4.99 -> 0.63(t+0.29s) -> 0.24(+0.40) -> 0.09(+0.51)
                                             * -> 0.017(+0.61s) 收敛, 之后稳在 0.011rad。
                                             * 也就是**要 ~0.6s 才真到位**; 100ms 的兜底只够走到
                                             * 0.63rad, 等于每次都在"轴差半米"的地方换向。
                                             * 兜底放到 800ms: 正常情况 0.61s 就提前退出, 不白等;
                                             * 只有轴真卡住时才走满。 */
#define SEQ_MOVE_EXTRA_US    500000UL       /* 运动兜底超时 = 估算时长x2 + 0.5s */
#define SEQ_MOVE_TMAX_S      60.0f          /* 估算时长本身的上限, 防低速长程算出天文数字 */
#define SEQ_D2R              0.017453292f   /* pi/180 */
#define SEQ_R2D              57.2957795f    /* 180/pi */

enum { SEQ_PH_IDLE = 0, SEQ_PH_WAIT, SEQ_PH_MOVE, SEQ_PH_DWELL };

typedef struct
{
    uint32_t wait_us;       /* 这一段开跑前等多久 */
    uint32_t dwell_us;      /* 这一段跑完停多久 */
    float    delta_rad;     /* 本段位移 (带符号), 相对本遍起点 */
    float    vlim_rad_s;    /* 速度上限 (正数) */
} SeqSeg_t;

static SeqSeg_t seq[SEQ_MAX_SEG];
static uint8_t  seq_n;          /* 装了几段 */
static uint8_t  seq_head;       /* 下一段的下标 */
static uint8_t  seq_run;        /* 1 = 正在跑 */
static uint8_t  seq_phase;
static uint8_t  seq_pass;       /* 正在跑第几遍 */
static uint8_t  seq_loops;      /* 一共几遍 */
static uint32_t seq_t_us;       /* 当前相位的截止时刻 */
static uint32_t seq_seg_us;     /* 本段运动开始时刻 (算耗时) */
static uint32_t seq_move_to_us; /* 本段运动兜底超时 */
static uint32_t seq_arr_us;     /* 参考到位时刻 (0 = 还没到) */
static float    seq_tgt_rad;    /* 命令链累进目标 (shaft 坐标, 绝对)。
                                 * 遍锚点只在 'RUN' 那一刻从实测位置取**一次**, 跑起来之后
                                 * **不许再用实测角重锚** (2026-10-06, 见 SEQ_PH_DWELL)。 */
/* 段内诊断 (2026-10-06): 参考轨迹是在主循环里按 dt 积分的, 主循环一停摆就被
 * Motion_Task 里那条 if(dt>0.005f) dt = 0.005f; 吃掉时间。这三条就是拿来量
 * "这一段到底丢了多长时间"的 —— 实测上行段比运动学模型固定慢 93ms, 下行段不慢。 */
static uint32_t seq_dbg_ticks;      /* 本段 Motion_Task 拍数 */
static uint32_t seq_dbg_stall_us;   /* 本段被 dt 钳位吃掉的微秒 */
static uint32_t seq_dbg_ierr0;      /* 本段起点时的传感器错误计数 */

static void motion_save_internal(void)
{
    PosLog_State_t st;

    st.pos_rad             = Sensor_GetAngle();
    st.sensor_offset       = sensor_offset;
    /* 这里写 0 表示"没对齐", 存的是对齐状态而不是 sensor_direction 本身:
     * 运行期 sensor_direction 默认就是 CW(+1), 直接存它会让下次上电误以为
     * 已经对齐(zero_electric_angle 却是 0) -> 电角度全错。 */
    st.sensor_direction    = Cmd_IsAligned() ? (int)sensor_direction : 0;
    st.zero_electric_angle = zero_electric_angle;
    st.home_off_rad        = home_set ? home_off_rad : POSLOG_HOME_NONE;
    /* 电机参数: 每次快照都带上当前运行值, 这样"改参数"和"改位置"走同一条路 */
    st.pole_pairs          = pole_pairs;
    st.stall_amp           = motor_stall_amp;
    st.vbus_v              = voltage_power_supply;

    (void)PosLog_Save(&st);

    last_save_us    = Micros();
}

void Motion_Init(void)
{
    mode       = PV_IDLE;
    arrived    = 0;
    restored   = 0;
    stalled    = 0;
    tgt_rad    = 0.0f;
    vlim_rad   = 0.0f;
    ref_rad    = 0.0f;
    vref_rad   = 0.0f;
    t_prev_us  = Micros();
    last_save_us = t_prev_us;
    sens_err_seen = Sensor_ErrorCount();
    sens_err_us   = 0u;
    home_off_rad  = 0.0f;
    home_set      = 0u;
    cfg_from_flash = 0u;
}

int Motion_RestoreFromLog(void)
{
    PosLog_State_t st;
    float raw_now, f;
    long  k;

    if(!PosLog_Load(&st)) return 0;

    /* 位置恢复: AS5600 只有单圈, 靠保存的累计角反推"圈数偏置"。
     * 只要断电期间轴转过的角度小于半圈, 就能精确接上;
     * 超过半圈是单圈编码器的原理性上限, 接不上。 */
    raw_now = Sensor_GetAngle();                       /* 此时 full_rotation_offset 还是 0 */
    f = raw_now - full_rotation_offset;                /* 单圈部分, [0,2pi) */
    k = _round((st.pos_rad - f) / _2PI);
    full_rotation_offset = (float)k * _2PI;

    if(st.sensor_direction != 0)
    {
        sensor_direction    = (long)st.sensor_direction;
        sensor_offset       = st.sensor_offset;
        zero_electric_angle = st.zero_electric_angle;
        Cmd_MarkAligned(1);
        restored = 1;
    }

    if(st.home_off_rad <= 1e29f)
    {
        home_off_rad = st.home_off_rad;
        home_set     = 1u;
    }

    /* 电机参数 (2026-09-28): 极对数 / 堵转阈值。
     * 这一条记录是**整张快照** —— zero_electric_angle 与 pole_pairs 是同一时刻
     * 写下来的, 配套有效, 所以这里**不作废对齐**, 一起用就行。
     * (会作废对齐的只有"运行期间改极对数", 见 Motion_SetPolePairs。) */
    if(st.pole_pairs >= MOTOR_CFG_PP_MIN && st.pole_pairs <= MOTOR_CFG_PP_MAX)
    {
        pole_pairs     = st.pole_pairs;
        cfg_from_flash = 1u;
    }
    if(st.stall_amp >= MOTOR_CFG_AMP_MIN && st.stall_amp <= MOTOR_CFG_AMP_MAX)
    {
        motor_stall_amp = st.stall_amp;
        cfg_from_flash  = 1u;
    }
    if(st.vbus_v >= MOTOR_CFG_VBUS_MIN && st.vbus_v <= MOTOR_CFG_VBUS_MAX)
    {
        voltage_power_supply = st.vbus_v;
        /* 上电时 voltage_limit 还是编译默认的 3.0V, 一般小于 vbus/sqrt3;
         * 只有默认值本身就超了才钳一下。 */
        if(voltage_limit > voltage_power_supply / _SQRT3)
            voltage_limit = voltage_power_supply / _SQRT3;
        cfg_from_flash = 1u;
    }

    shaft_angle = ShaftAngle();
    ref_rad     = shaft_angle;
    tgt_rad     = shaft_angle;
    last_save_us    = Micros();

    printf("POSLOG: restored pos=%s rad (raw=%s, k=%ld) dir=%d zero_elec=%s%s\r\n",
           FmtF(Sensor_GetAngle(), 4), FmtF(f, 4), k,
           (int)sensor_direction, FmtF(zero_electric_angle, 4),
           restored ? "" : "  [no align params]");
    printf("POSLOG: home=%s (mech pos=%s rad)\r\n",
           home_set ? FmtF(home_off_rad, 4) : "not set",
           FmtF(shaft_angle - home_off_rad, 4));
    printf("MOTCFG: %s pole_pairs=%d stall=%s A vbus=%s V (compiled default %d / %s A / %s V)\r\n",
           cfg_from_flash ? "from flash" : "compiled default",
           pole_pairs, FmtF(motor_stall_amp, 2), FmtF(voltage_power_supply, 2),
           MOTOR_POLE_PAIRS, FmtF(MOTOR_STALL_AMP, 2), FmtF(MOTOR_VOLTAGE_SUPPLY, 1));
    return 1;
}

/* 内部: 真正开一条 PV 轨迹。脚本也只走这里 (绕开下面那层"用户命令让脚本让位") */
static void motion_runpv_internal(float target_rad, float vlim_rad_s)
{
    if(!Cmd_IsAligned())
    {
        printf("MOT: PV refused - not aligned\r\n");
        return;
    }

    if(vlim_rad_s < 0.0f)  vlim_rad_s = -vlim_rad_s;
    if(vlim_rad_s < MOTION_PV_VLIM_MIN) vlim_rad_s = MOTION_PV_VLIM_MIN;

    controller = Type_angle;
    ref_rad    = shaft_angle;          /* 从当前实际位置起步, 不跳变 */
    tgt_rad    = target_rad;
    vlim_rad   = vlim_rad_s;
    vref_rad   = 0.0f;
    arrived    = 0;
    mode       = PV_POS;
    t_prev_us  = Micros();

    printf("MOT: PV target=%s rad (%s turns) vlim=%s rad/s  from=%s rad\r\n",
           FmtF(tgt_rad, 4), FmtF(tgt_rad / _2PI, 3), FmtF(vlim_rad, 3), FmtF(ref_rad, 4));
}

/* 用户/上位机直接下 PV: 让脚本先让位 (否则两边抢 tgt_rad) */
void Motion_RunPV(float target_rad, float vlim_rad_s)
{
    if(seq_run) Motion_Seq_Abort();
    motion_runpv_internal(target_rad, vlim_rad_s);
}

/* =============================================================================
 * 板载动作脚本 (2026-09-29) —— 让"没有 shell 的另一颗单片机"发一串动作
 * -----------------------------------------------------------------------------
 * 语法 (UART 控制台):  *{<等待us>,<±rpm>,<角度deg>,<停留ms>}{...}{...}
 *   4 个字段位置固定, 逗号分隔, 空格随便加; 段与段首尾相连 (也接受 ';' 分隔)。
 *   - 方向 = rpm 那一栏的符号 (负数 = 反转)。**deg 必须写正数** —— 负角度一律拒绝:
 *     方向只能在一处给, 两处都能给就迟早出事 (2026-09-28 的单位/符号事故)。
 *   - 角度是相对量: 相对**本遍起点**(按下开始那一刻的位置)算, 不是相对上一段终点,
 *     所以中间有误差也不会跨段累积; 与机械零位无关, 不用先设 'F'。
 *   - "等待" = 这一段开跑前先等多久; "停留" = 这一段跑完停多久再进下一段。
 * 命令: *<脚本> 追加(可多行累加) | *! 跑一遍 | *!<n> 跑 n 遍 | *0 清空 | *? 打印
 * CAN : 0x120 追加一段(8B) | 0x122 追加一段·大角度版(同布局, 角度 1 度/LSB)
 *       0x121 控制(b0 0=清空 1=开始 2=打印; b1 = 遍数)
 *       剩余段数 / 当前遍数报在 0x184 的 b6 / b7。
 *
 * Flash 写入策略 (刻意这么设计 —— 动作中间不能被打断):
 *   - 装脚本 / 开始 / 每一段: 一个字节都不写。
 *   - 一遍跑完写一次 (记下停在哪儿, 仅作参考); 停止/失能/堵转各写一次(沿用原逻辑)。
 *   - 跑脚本期间, 连原来那条"Δt>=1s 且动了 0.05rad 才写"的周期保存也**跳过** ——
 *     页擦除典型 7.5ms (手册 V2.1 表3-18), 卡在那几毫秒里动作会一顿一顿。
 *   - 代价: 跑到一半掉电, 上电恢复出来的是上一处保存点, 仅供参考。
 * ========================================================================== */

int  Motion_Seq_Running(void) { return seq_run ? 1 : 0; }
int  Motion_Seq_Loaded(void)  { return (int)seq_n; }
int  Motion_Seq_Remain(void)  { return seq_run ? (int)(seq_n - seq_head) : 0; }
int  Motion_Seq_Pass(void)    { return seq_run ? (int)seq_pass : 0; }

void Motion_Seq_Clear(void)
{
    seq_n    = 0;
    seq_head = 0;
    printf("SEQ: queue cleared\r\n");
}

void Motion_Seq_Abort(void)
{
    if(!seq_run) return;                /* 没在跑就什么都不做(不刷屏) */

    seq_run   = 0;
    seq_phase = SEQ_PH_IDLE;
    seq_head  = 0;
    seq_n     = 0;                      /* 停下就把脚本丢掉: 免得"以为停了又自己跑" */
    printf("SEQ: aborted, script dropped (re-send to run again)\r\n");
}

/* 装一段进队列 —— 文字脚本和 CAN 帧都走这里, 校验与语义只有一份 */
static int seq_push(float wait_us, float rpm, float deg, float dwell_ms)
{
    if(seq_n >= (uint8_t)SEQ_MAX_SEG)
    {
        printf("SEQ ERR: queue full (%d segs max, '*0' clears)\r\n", (int)SEQ_MAX_SEG);
        return -1;
    }
    if(wait_us < 0.0f || wait_us > (float)SEQ_WAIT_MAX_US)
    {
        printf("SEQ ERR: wait must be 0..%ld us\r\n", SEQ_WAIT_MAX_US);
        return -1;
    }
    if(dwell_ms < 0.0f || dwell_ms > (float)SEQ_DWELL_MAX_MS)
    {
        printf("SEQ ERR: dwell must be 0..%ld ms\r\n", SEQ_DWELL_MAX_MS);
        return -1;
    }
    if(deg <= 0.0f)
    {
        printf("SEQ ERR: deg must be POSITIVE (direction lives in the rpm sign)\r\n");
        return -1;
    }
    if(deg > SEQ_DEG_MAX)
    {
        printf("SEQ ERR: deg too big (max %s deg = 100 turns)\r\n", FmtF(SEQ_DEG_MAX, 0));
        return -1;
    }
    if(rpm == 0.0f)
    {
        printf("SEQ ERR: rpm 0 - a segment with no speed does nothing\r\n");
        return -1;
    }
    if(fabsf(rpm) * _2PI / 60.0f > MOTOR_VELOCITY_LIMIT_MAX)
    {
        printf("SEQ ERR: rpm too big (max %s rpm = board.h MOTOR_VELOCITY_LIMIT_MAX)\r\n",
               FmtF(MOTOR_VELOCITY_LIMIT_MAX * 60.0f / _2PI, 0));
        return -1;
    }

    seq[seq_n].wait_us    = (uint32_t)wait_us;
    seq[seq_n].dwell_us   = (uint32_t)(dwell_ms * 1000.0f + 0.5f);
    seq[seq_n].delta_rad  = (rpm < 0.0f) ? -(deg * SEQ_D2R) : (deg * SEQ_D2R);
    seq[seq_n].vlim_rad_s = fabsf(rpm) * _2PI / 60.0f;
    seq_n++;
    return 0;
}

/* 取一个字段: 成功返回 0, 并把 p 停在字段后面那个分隔符上 */
static int seq_field(const char **pp, float *out)
{
    const char *p = *pp, *q;

    while(*p == ' ' || *p == '\t') p++;
    if(!((*p >= '0' && *p <= '9') || *p == '+' || *p == '-' || *p == '.')) return -1;

    for(q = p; *q && *q != ',' && *q != '}' && *q != ';'; q++)
    {
        char c = *q;
        if(!((c >= '0' && c <= '9') || c == '.' || c == '+' || c == '-' ||
             c == 'e' || c == 'E' || c == ' ' || c == '\t'))
            return -1;                  /* 字段里有垃圾 -> 不猜, 报错 */
    }

    *out = (float)Cmd_ParseFloat(p);
    *pp  = q;
    return 0;
}

int Motion_Seq_Append(const char *script)
{
    const char *p = script;
    int n0 = seq_n, bad = 0;

    for(;;)
    {
        float f0, f1, f2, f3;

        while(*p == ' ' || *p == '\t' || *p == ';' || *p == ',') p++;
        if(*p == '\0') break;

        if(*p != '{')
        {
            printf("SEQ ERR: expect '{' at offset %d, got '%c'\r\n", (int)(p - script), *p);
            bad = 1;
            break;
        }
        p++;

        if(seq_field(&p, &f0) != 0 || *p != ',')
        { printf("SEQ ERR: field 1 (wait_us) bad at offset %d\r\n", (int)(p - script)); bad = 1; break; }
        p++;
        if(seq_field(&p, &f1) != 0 || *p != ',')
        { printf("SEQ ERR: field 2 (rpm) bad at offset %d\r\n", (int)(p - script)); bad = 1; break; }
        p++;
        if(seq_field(&p, &f2) != 0 || *p != ',')
        { printf("SEQ ERR: field 3 (deg) bad at offset %d\r\n", (int)(p - script)); bad = 1; break; }
        p++;
        if(seq_field(&p, &f3) != 0)
        { printf("SEQ ERR: field 4 (dwell_ms) bad at offset %d\r\n", (int)(p - script)); bad = 1; break; }
        if(*p != '}')
        {
            /* 多半是这一行被 LINE_MAX 截断了 */
            printf("SEQ ERR: '}' expected after field 4 (got '%c') - line too long?\r\n",
                   *p ? *p : ' ');
            bad = 1;
            break;
        }
        p++;

        if(seq_push(f0, f1, f2, f3) != 0) { bad = 1; break; }
    }

    if(bad)
    {
        seq_n = (uint8_t)n0;            /* 整行回退: 半行脚本比没有更坑 */
        printf("SEQ: line rejected, nothing added (queue still %d/%d)\r\n",
               (int)seq_n, (int)SEQ_MAX_SEG);
        return -1;
    }

    printf("SEQ: +%d seg(s) -> queue %d/%d\r\n", (int)seq_n - n0, (int)seq_n, (int)SEQ_MAX_SEG);
    return (int)seq_n - n0;
}

/* CAN 0x120: 一帧一段 (已换算成 wait_us / rpm / deg / dwell_ms) */
int Motion_Seq_AddSeg(long wait_us, int rpm, float deg, long dwell_ms)
{
    int rc = seq_push((float)wait_us, (float)rpm, deg, (float)dwell_ms);

    if(rc == 0) printf("SEQ: +1 seg (CAN) -> queue %d/%d\r\n", (int)seq_n, (int)SEQ_MAX_SEG);
    return rc;
}

int Motion_Seq_Start(int loops)
{
    if(seq_n == 0)
    {
        printf("SEQ: nothing loaded - send *{wait_us,+-rpm,deg,dwell_ms} first\r\n");
        return -1;
    }
    if(!Cmd_IsAligned())
    {
        printf("SEQ: refused - not aligned (run 'A' first)\r\n");
        return -1;
    }
    if(!Motion_IsEnabled())
    {
        printf("SEQ: refused - driver disabled (press 'E' first)\r\n");
        return -1;
    }
    if(loops < 1)  loops = 1;
    if(loops > 99) loops = 99;

    seq_loops   = (uint8_t)loops;
    seq_pass    = 1;
    seq_head    = 0;
    seq_run     = 1;
    seq_phase   = SEQ_PH_WAIT;
    seq_arr_us  = 0;
    seq_tgt_rad = shaft_angle;          /* 本遍起点 = 按下开始这一刻的实际位置 */
    seq_t_us    = Micros() + seq[0].wait_us;

    printf("SEQ: RUN %d seg(s) x %d pass(es), anchor=%s rad (all targets from here)\r\n",
           (int)seq_n, loops, FmtF(seq_tgt_rad, 4));
    return 0;
}

void Motion_Seq_Print(void)
{
    int   i;
    float tot = 0.0f;

    printf("SEQ: %d/%d segs, %s", (int)seq_n, (int)SEQ_MAX_SEG,
           seq_run ? "RUNNING" : (seq_n ? "loaded" : "empty"));
    if(seq_run) printf(", pass %d/%d, next seg %d", (int)seq_pass, (int)seq_loops, (int)seq_head + 1);
    printf("\r\n");

    for(i = 0; i < (int)seq_n; i++)
    {
        tot += seq[i].delta_rad;
        printf("  %2d: wait %luus, %s deg @ %s rpm, dwell %lums\r\n",
               i + 1, (unsigned long)seq[i].wait_us,
               FmtF(seq[i].delta_rad * SEQ_R2D, 1),
               FmtF(fabsf(seq[i].vlim_rad_s) * 60.0f / _2PI, 0),
               (unsigned long)(seq[i].dwell_us / 1000u));
    }
    if(seq_n) printf("  total from anchor: %s deg = %s turns\r\n",
                     FmtF(tot * SEQ_R2D, 1), FmtF(tot / _2PI, 3));
}

/* 接在后面的那一段是不是"**同方向** + 不等(0us) + 不停(0ms)"?
 * 是 -> 这次参考到位后**不进**那个定点窗口, 直接进下一段: 段与段之间一点死等都没有。
 * 同向同速的两段本来就已经并成一次运动(见 SEQ_PH_WAIT 里的合并), 所以这里管的是
 * "同向、但速度上限不同"那种接续。
 *
 * 为什么**换向不算** (2026-10-06 改): 换向的段轴本来就得停下来再回头; 参考一到位就把
 * 目标抽走, 轴根本没走到命令点就被拉回去 —— 实测 500rpm 换向到位那一刻轴还差
 * 2.5~4.2rad (上行段), 于是每一遍的顶点比命令值少 ~200 度。换向必须等轴真进到
 * SEQ_SETTLE_RAD 或超时。这一遍跑完还要再跑一遍时, 看的是第 1 段的 wait/dwell 和方向。 */
static int seq_next_is_immediate(void)
{
    const SeqSeg_t *cur = &seq[seq_head];
    const SeqSeg_t *nxt;

    if((int)seq_head + 1 < (int)seq_n)  nxt = &seq[seq_head + 1u];
    else if(seq_pass < seq_loops)       nxt = &seq[0];
    else                                return 0;

    if(nxt->wait_us != 0u || nxt->dwell_us != 0u) return 0;
    return ((nxt->delta_rad > 0.0f) == (cur->delta_rad > 0.0f)) ? 1 : 0;
}

/* 脚本状态机 —— 由 Motion_Task 每拍调一次 (主循环, 实测 2000~2500 次/秒) */
static void Motion_Seq_Task(void)
{
    uint32_t now;

    if(!seq_run) return;
    now = Micros();

    switch(seq_phase)
    {
        case SEQ_PH_WAIT:
        {
            SeqSeg_t *s;
            float     t_est;
            float     d_rad;
            uint8_t   seg_first;

            if((int32_t)(now - seq_t_us) < 0) return;      /* 还在"开始前等待" */

            /* ---- 同向接续 (2026-10-06) ------------------------------------
             * 紧跟其后的段如果"同方向 + 同速度上限 + 不等(0us) + 不停(0ms)", 就并进
             * 这一次运动: 目标一次给到底, PV 参考的 err 不归零 -> **中途不减速停车**。
             * 原行为: 每一段都是独立的"静止→静止", 段末必然减速到停, 而且还要等最多
             * 100ms 的定点窗口(那是**当时**的 SEQ_SETTLE_MAX_US: 残差 6~8 度永远够不到
             * 0.02rad 门槛, 所以那一支每次都走满)。该窗口 2026-10-06 已改 800ms —— 上面这几句
             * 是写补丁那天的现场。实测 3000 度 @500rpm 单段 2365ms, 其中 ~100ms 是白等。
             * 方向或速度上限变了**不合并** —— 那本来就该减速/换向, 是两次运动;
             * 但只要后面那段也是 wait=0/dwell=0, 两次运动之间**一个 tick 的死等都没有**
             * (见 seq_next_is_immediate)。要"连速度都不掉"就得同向同速, 才能并成一次。 */
            seg_first = seq_head;
            s     = &seq[seq_head];
            d_rad = s->delta_rad;

            while(((int)seq_head + 1 < (int)seq_n) &&
                  seq[seq_head].dwell_us       == 0u &&   /* 本段自己要求停一下 -> 不能并 */
                  seq[seq_head + 1u].wait_us  == 0u &&
                  seq[seq_head + 1u].dwell_us == 0u &&
                  ((seq[seq_head + 1u].delta_rad > 0.0f) == (d_rad > 0.0f)) &&
                  fabsf(seq[seq_head + 1u].vlim_rad_s - s->vlim_rad_s) < 0.001f)
            {
                seq_head++;
                d_rad += seq[seq_head].delta_rad;
            }

            seq_tgt_rad += d_rad;
            motion_runpv_internal(seq_tgt_rad, s->vlim_rad_s);

            t_est = fabsf(d_rad) / s->vlim_rad_s + s->vlim_rad_s / MOTION_PV_DECEL;
            if(t_est > SEQ_MOVE_TMAX_S) t_est = SEQ_MOVE_TMAX_S;
            seq_move_to_us = now + (uint32_t)(t_est * 2.0f * 1e6f) + SEQ_MOVE_EXTRA_US;

            seq_seg_us       = now;
            seq_arr_us       = 0;
            seq_dbg_ticks    = 0u;
            seq_dbg_stall_us = 0u;
            seq_dbg_ierr0    = Sensor_ErrorCount();
            seq_phase        = SEQ_PH_MOVE;

            printf("SEQ: pass %d/%d seg %d-%d/%d  %s deg @ %s rpm -> target %s rad\r\n",
                   (int)seq_pass, (int)seq_loops,
                   (int)seg_first + 1, (int)seq_head + 1, (int)seq_n,
                   FmtF(d_rad * SEQ_R2D, 1),
                   FmtF(s->vlim_rad_s * 60.0f / _2PI, 0),
                   FmtF(seq_tgt_rad, 4));
            if(seq_head != seg_first)
                printf("SEQ:   ^ %d 段接续成一次运动 (中途不停)\r\n", (int)(seq_head - seg_first) + 1);
            break;
        }

        case SEQ_PH_MOVE:
            if(Motion_Arrived())
            {
                if(seq_arr_us == 0u) seq_arr_us = now ? now : 1u;
                if(seq_next_is_immediate() ||
                   fabsf(shaft_angle - seq_tgt_rad) < SEQ_SETTLE_RAD ||
                   (uint32_t)(now - seq_arr_us) >= SEQ_SETTLE_MAX_US)
                {
                    printf("SEQ:   done in %lums, rest err %s rad  (dwell %lums) [n=%lu stall=%lums ierr=%lu]\r\n",
                           (unsigned long)((now - seq_seg_us) / 1000u),
                           FmtFS(shaft_angle - seq_tgt_rad, 4),
                           (unsigned long)(seq[seq_head].dwell_us / 1000u),
                           (unsigned long)seq_dbg_ticks,
                           (unsigned long)(seq_dbg_stall_us / 1000u),
                           (unsigned long)(Sensor_ErrorCount() - seq_dbg_ierr0));
                    seq_t_us  = now + seq[seq_head].dwell_us;
                    seq_phase = SEQ_PH_DWELL;
                    break;
                }
            }
            else
            {
                seq_arr_us = 0;
            }

            if((int32_t)(now - seq_move_to_us) >= 0)
            {
                printf("SEQ: !! segment timeout, %s rad away -> stop + hold, script dropped\r\n",
                       FmtF(shaft_angle - seq_tgt_rad, 4));
                Motion_Stop(0);                 /* 停下保持; 它内部会把脚本丢掉 */
            }
            break;

        case SEQ_PH_DWELL:
            if((int32_t)(now - seq_t_us) < 0) return;

            seq_head++;
            if(seq_head < seq_n)
            {
                seq_phase = SEQ_PH_WAIT;
                seq_t_us  = now + seq[seq_head].wait_us;
            }
            else if(seq_pass < seq_loops)
            {
                seq_pass++;
                seq_head  = 0;
                /* seq_tgt_rad **不动** (2026-10-06 改)。
                 * 原代码这里是 seq_tgt_rad = shaft_angle;, 注释写"每遍重新锚定, 误差不跨遍
                 * 累积" —— 实际**恰好相反**: 拿**实测**角当下一遍的基准, 等于把上一遍还没消掉
                 * 的跟随误差吃进命令链, 每遍累积一份。实测 ±3200deg @500rpm 连跑 50 遍:
                 * 每遍锚点往下 -0.33rad (-19deg), 50 遍累计 -15.7rad (-900deg, 2.5 圈),
                 * 就是家人看到的"越走越远"。
                 * 脚本两段是 +D/-D 闭环时, 第 2 段的命令值本来就回到遍起点, seq_tgt_rad 保持
                 * 不动即可; 不闭环(比如两段同向)时它自然接着往下累, 那也是想要的语义。
                 * 一句话: **锚点必须是命令链上的量, 不能是测量值**。 */
                seq_phase = SEQ_PH_WAIT;
                seq_t_us  = now + seq[0].wait_us;
                printf("SEQ: pass %d/%d done, next pass from %s rad (command chain)\r\n",
                       (int)seq_pass - 1, (int)seq_loops, FmtF(seq_tgt_rad, 4));
            }
            else
            {
                seq_run   = 0;
                seq_phase = SEQ_PH_IDLE;
                seq_head  = 0;
                Motion_SaveNow();               /* 跑完只写这一次位置 (仅供参考) */
                printf("SEQ: all done (%d pass(es)) - queue kept, '*!' reruns, '*0' clears, pos saved\r\n",
                       (int)seq_loops);
            }
            break;

        default:
            break;
    }
}

void Motion_RunVEL(float vel_rad_s)
{
    Motion_Seq_Abort();
    if(!Cmd_IsAligned())
    {
        printf("MOT: VEL refused - not aligned\r\n");
        return;
    }

    controller = Type_velocity;
    vlim_rad   = vel_rad_s;
    vref_rad   = vel_rad_s;
    arrived    = 0;
    mode       = PV_VEL;
    t_prev_us  = Micros();

    printf("MOT: VEL %s rad/s (%s rpm)\r\n",
           FmtF(vel_rad_s, 3), FmtF(vel_rad_s * 60.0f / _2PI, 2));
}

void Motion_Stop(uint8_t release)
{
    Motion_Seq_Abort();
    if(release || !Cmd_IsAligned())
    {
        mode     = PV_IDLE;
        arrived  = 0;
        vref_rad = 0.0f;
        /* 松轴停: controller 可能还是 Type_angle, 它的 cmd_target 是绝对多圈角度.
           这里不能无条件清 0, 否则下次使能时角度环会去追 0 rad 而满压冲一下. */
        cmd_target = (controller == Type_angle) ? shaft_angle : 0.0f;
        DRV8313_Disable();
        printf("MOT: STOP (release)\r\n");
    }
    else
    {
        controller = Type_angle;
        ref_rad    = shaft_angle;
        tgt_rad    = shaft_angle;
        vref_rad   = 0.0f;
        mode       = PV_HOLD;
        arrived    = 1;
        cmd_target = ref_rad;
        printf("MOT: STOP (hold at %s rad)\r\n", FmtF(ref_rad, 4));
    }

    motion_save_internal();
}

void Motion_SaveNow(void)
{
    motion_save_internal();
}

void Motion_CalCurrentZero(void)
{
    CS_CalZero();
}

static void motion_trip(void)
{
    Motion_Seq_Abort();
    stalled = 1;
    mode    = PV_IDLE;
    vref_rad = 0.0f;
    cmd_target = 0.0f;
    DRV8313_Disable();
    motion_save_internal();

    printf("MOT: STALL! peak=%s A -> PWM off (DRV disabled). clear with 'O' or CAN flag bit3\r\n",
           FmtF(CS_PeakAmp(), 2));
}

void Motion_Task(void)
{
    uint32_t now = Micros();
    uint32_t d   = now - t_prev_us;
    float    dt;

    t_prev_us = now;

    if((uint32_t)(Sensor_ErrorCount() - sens_err_seen) != 0u)
    {
        sens_err_seen = Sensor_ErrorCount();
        sens_err_us   = now;
    }

    /* 堵转: 窗口由 CS_Task 维护, 连续 N 个窗口超标就跳 */
    if(!stalled && (CS_OverWins() >= MOTOR_STALL_WINS))
    {
        motion_trip();
    }

    dt = (float)d * 1e-6f;
    if(dt <= 0.0f) return;
    /* Flash 擦写会把 CPU 卡住几毫秒, 别让轨迹用这个超长 dt 一步跳过去 */
    if(dt > 0.005f) dt = 0.005f;

    Motion_Seq_Task();

    /* 段内诊断: 只统计"正在跑一段运动"的拍子, 看参考轨迹被 dt 钳位吃了多少时间 */
    if(seq_run && seq_phase == SEQ_PH_MOVE)
    {
        seq_dbg_ticks++;
        if(d > 5000u) seq_dbg_stall_us += d - 5000u;
    }

    switch(mode)
    {
        case PV_POS:
        {
            /* 只有**角度环**才吃位置参考。手动模式 (力矩/速度) 的 cmd_target 是
             * 用户给的电压/转速, 拿 ref_rad 去覆盖就把 'V10' 冲成 0 了。
             * 2026-09-28 实测: 'G0@40' (留下 mode=PV_POS) 之后 'M1'+'V10',
             * 'S' 里的 target 立刻回到 0.000, 电机不转。 */
            if(controller != Type_angle) break;

            float err = tgt_rad - ref_rad;
            float v   = SqrtF(2.0f * MOTION_PV_DECEL * fabsf(err));

            if(v > vlim_rad) v = vlim_rad;
            if(err < 0.0f)   v = -v;

            ref_rad += v * dt;
            vref_rad = v;

            if(fabsf(tgt_rad - ref_rad) < MOTION_PV_ARRIVE)
            {
                ref_rad  = tgt_rad;
                vref_rad = 0.0f;
                arrived  = 1;
            }
            else
            {
                arrived = 0;
            }

            cmd_target = ref_rad;
            break;
        }

        case PV_VEL:
            vref_rad   = vlim_rad;
            cmd_target = vlim_rad;
            break;

        case PV_HOLD:
            /* 位置保持只有角度环才需要: 它的 cmd_target 是"绝对多圈角度".
               速度环的 cmd_target 是 rad/s, 力矩模式的是电压 —— 拿 ref_rad
               (位置) 去覆盖它们会变成"目标转速 = 当前角度 / 满压".
               2026-09-28 实测: 'E' 进 PV_HOLD 后 'V10' 被覆盖成 4.481 rad/s. */
            if(controller == Type_angle)
            {
                cmd_target = ref_rad;
            }
            vref_rad   = 0.0f;
            break;

        case PV_IDLE:
        default:
            break;
    }

}

/* 对齐 / 重新使能之后调用: 角度环的 cmd_target 是"绝对多圈角度", 必须
 * 先跟当前轴位置对齐。否则它可能还停在 0 rad (或别处的旧值), 一合 EN
 * 就要求轴瞬间跑到那个绝对位置 -> 满压冲一下 -> 2A 跳堵转。
 * 只对角度环做; 手动力矩/速度模式下 cmd_target 是用户给的电压/转速, 不能动。*/
void Motion_SyncAngleTarget(void)
{
    if(controller != Type_angle) return;   /* 手动力矩/速度: 目标归用户管 */
    if(mode == PV_POS)           return;   /* 正在跑位置轨迹: 别动它的参考 */

    ref_rad    = shaft_angle;
    tgt_rad    = shaft_angle;
    vref_rad   = 0.0f;
    cmd_target = shaft_angle;
    arrived    = 1;
    mode       = PV_HOLD;
}

/* 'M<n>' 切换控制模式时调用 (cmd.c): 把**上一条 PV 轨迹/状态**整个丢掉。
 * 两个必须做的理由 (都是 2026-09-28 实测抓到的):
 *   1) mode 还停在 PV_POS 时, Motion_Task 每拍都 `cmd_target = ref_rad;`,
 *      把用户刚下的 'V10'(1.047 rad/s) / 'T' 冲掉 -> 手动速度模式实际目标 0, 电机纹丝不动;
 *   2) mode 还停在 PV_VEL 时切回 'M2' (角度环), `cmd_target = vlim_rad;` 会把
 *      当初那条轨迹的**速度上限(rad/s)当成绝对位置(rad)**塞给角度环 -> 轴自己跑。
 * 切到角度环时调用方随后会走 Motion_SyncAngleTarget() 把目标拉回当前轴位置。 */
void Motion_AbortPV(void)
{
    Motion_Seq_Abort();
    mode     = PV_IDLE;
    arrived  = 0;
    vref_rad = 0.0f;
}

/* ---- 使能 / 失能 ----
 * 使能 = 合上 EN + 进角度环保持当前位置 (松手就飘的话说明没对齐, 直接拒)。
 * 失能 = 断 EN (输出高阻, 轴自由) + 存一次位置。
 * 返回 0 成功 / -1 被拒 (原因由调用方上报)。 */
int Motion_SetEnabled(uint8_t on)
{
    Motion_Seq_Abort();
    if(on)
    {
        if(!Cmd_IsAligned())
        {
            printf("MOT: ENABLE refused - not aligned (run 'A' first)\r\n");
            return -1;
        }

        DRV8313_Enable();
        controller = Type_angle;
        ref_rad    = shaft_angle;
        tgt_rad    = shaft_angle;
        vref_rad   = 0.0f;
        mode       = PV_HOLD;
        arrived    = 1;
        cmd_target = ref_rad;
        printf("MOT: ENABLED (hold at %s rad)\r\n", FmtF(ref_rad, 4));
    }
    else
    {
        DRV8313_Disable();
        mode       = PV_IDLE;
        arrived    = 0;
        vref_rad   = 0.0f;
        cmd_target = 0.0f;
        motion_save_internal();
        printf("MOT: DISABLED (coast, pos saved)\r\n");
    }
    return 0;
}

int Motion_IsEnabled(void) { return DRV8313_IsEnabled(); }
int Motion_HomeSet(void)   { return home_set ? 1 : 0; }
float Motion_HomeRad(void) { return home_off_rad; }

/* 机械坐标 = 原始累计位置 - 机械零位。没设过零位时二者相同 */
float Motion_PosMechRad(void) { return shaft_angle - home_off_rad; }

void Motion_SetHomeHere(void)
{
    home_off_rad = shaft_angle;
    home_set     = 1u;
    motion_save_internal();
    printf("HOME: mech zero = current shaft pos (raw=%s rad, saved to Flash)\r\n",
           FmtF(home_off_rad, 4));
}

void Motion_ClearHome(void)
{
    home_off_rad = 0.0f;
    home_set     = 0u;
    motion_save_internal();
    printf("HOME: mech zero cleared (mech coord = raw coord)\r\n");
}

/* =============================================================================
 * 电机参数 (极对数 / 堵转阈值 / 母线电压) —— 2026-09-28 新增
 * -----------------------------------------------------------------------------
 * 为什么要能改: 家里电机一颗一个极对数, 母线也有 24V/36V 两种, 以前每换一次都要
 * 改 board.h 重编烧写。现在存 Flash (跟位置/对齐参数同一条 32B 快照, 见 pos_log.c
 * 字段表), 上电时 Motion_RestoreFromLog() 用它覆盖 board.h 的编译默认值。
 * 串口 '#' 与 CAN 0x116 都能改, 改完**立刻写一次 Flash**。
 *
 * 改极对数为什么要作废对齐:
 *   zero_electric_angle 是用**当时的 pole_pairs** 算出来的 (foc_motor.c 的
 *   AlignSensor(): normalize(_electricalAngle(dir*angle, pole_pairs)))。
 *   极对数一换, 同一个机械角对应的电角就变了 —— 拿旧零点电角继续用 =
 *   电角度整圈都错, 表现是没劲 / 抖 / 发热 / 根本不走。所以强制下次先跑 'A'。
 *
 * 为什么改极对数要求先失能:
 *   带电改 = 下一个 10kHz 节拍就用新极对数算电角, 而转子还在旧磁场里 -> 立刻
 *   错误换相。失能后输出高阻, 改完参数是安全的。
 *
 * 改母线电压不作废对齐 (它跟电角度无关), 也不要求先失能:
 *   它进的是 SVPWM 归一化 Uout = |U|/vbus。改的一瞬间同样的 Uq 对应的真实相电压
 *   会按新 vbus 变 (24 -> 36V 时降到 2/3, 力矩变小; 反之变大), 所以下面会把
 *   voltage_limit 重新钳到 vbus/sqrt(3), 并在打印里说清楚。
 * ========================================================================== */

/* =============================================================================
 * 上电默认相电压上限 (2026-09-28 新增)
 * -----------------------------------------------------------------------------
 * 语义: 上电时 voltage_limit = vbus / sqrt(3) —— 就是 SVPWM 线性调制的相电压上限。
 *   vbus 可能来自 Flash (见 Motion_RestoreFromLog), 所以这个函数必须在它之后调。
 *   13V 母线 -> 7.51V, 24V -> 13.86V, 36V -> 20.78V。
 * MOTOR_VOLTAGE_LIMIT > 0 时改用那个固定值 (台面想强制小限幅就改宏)。
 * 为什么不再固定 3.0V: 母线换挡时它不变, 位置环的出力被钉死在 ~0.45A,
 *   平衡关节看不出来 (保持只要 0.03V), 一带负载/一要加速就暴露。
 * ========================================================================== */
void Motion_ApplyBootVoltageLimit(void)
{
    float lim = voltage_power_supply / _SQRT3;

    if(MOTOR_VOLTAGE_LIMIT > 0.0f && MOTOR_VOLTAGE_LIMIT < lim)
        lim = MOTOR_VOLTAGE_LIMIT;
    if(lim < 0.0f) lim = 0.0f;

    voltage_limit = lim;
}

int Motion_SetMotorCfg(int pp, float amp, float vbus)
{
    int change_pp   = (pp > 0) ? 1 : 0;             /* pp   = 0 -> 不改极对数 */
    int change_amp  = (amp != 0.0f) ? 1 : 0;        /* amp  = 0 -> 不改阈值 */
    int change_vbus = (vbus > 0.0f) ? 1 : 0;        /* vbus = 0 -> 不改母线电压 */
    int dirty       = 0;

    if(!change_pp && !change_amp && !change_vbus)
    {
        printf("CFG: nothing to change (pp/amp/vbus = 0 means keep)\r\n");
        return -1;
    }

    /* 先全部校验: 有一条不合法就整条不作数, 别改一半 */
    if(change_pp && (pp < MOTOR_CFG_PP_MIN || pp > MOTOR_CFG_PP_MAX))
    {
        printf("CFG: pole_pairs=%d out of range (%d..%d), rejected\r\n",
               pp, MOTOR_CFG_PP_MIN, MOTOR_CFG_PP_MAX);
        return -1;
    }
    if(change_amp && !(amp >= MOTOR_CFG_AMP_MIN && amp <= MOTOR_CFG_AMP_MAX))
    {
        printf("CFG: stall=%s A out of range (%s..%s), rejected\r\n", FmtF(amp, 2),
               FmtF(MOTOR_CFG_AMP_MIN, 2), FmtF(MOTOR_CFG_AMP_MAX, 2));
        return -1;
    }
    if(change_vbus && !(vbus >= MOTOR_CFG_VBUS_MIN && vbus <= MOTOR_CFG_VBUS_MAX))
    {
        printf("CFG: vbus=%s V out of range (%s..%s), rejected\r\n", FmtF(vbus, 2),
               FmtF(MOTOR_CFG_VBUS_MIN, 1), FmtF(MOTOR_CFG_VBUS_MAX, 1));
        return -1;
    }
    /* 改极对数必须停: 带电改 = 下一个 10kHz 节拍就用新极对数换相 */
    if(change_pp && (pp != pole_pairs) && Motion_IsEnabled())
    {
        printf("CFG: driver enabled -- disable first ('D' / CAN 0x110 b0=0) before changing pole_pairs\r\n");
        return -1;
    }

    if(change_pp && (pp != pole_pairs))
    {
        printf("CFG: pole_pairs %d -> %d  (alignment void, re-run 'A')\r\n", pole_pairs, pp);
        pole_pairs = pp;
        Cmd_MarkAligned(0);
        dirty = 1;
    }

    if(change_amp && (amp != motor_stall_amp))
    {
        motor_stall_amp = amp;
        printf("CFG: stall = %s A\r\n", FmtF(amp, 2));
        if(amp > MOTOR_CFG_AMP_WARN)
            printf("CFG: note -- above %s A the DRV8313 peak limit (~2.5A) trips first\r\n",
                   FmtF(MOTOR_CFG_AMP_WARN, 2));
        dirty = 1;
    }

    if(change_vbus && (vbus != voltage_power_supply))
    {
        printf("CFG: vbus %s -> %s V\r\n", FmtF(voltage_power_supply, 2), FmtF(vbus, 2));
        voltage_power_supply = vbus;
        /* 相电压上限跟着走: 线性调制上限就是 vbus/sqrt(3) */
        if(voltage_limit > vbus / _SQRT3)
        {
            voltage_limit = vbus / _SQRT3;
            printf("CFG: voltage_limit clamped to %s V (vbus/sqrt3)\r\n", FmtF(voltage_limit, 2));
        }
        /* voltage_sensor_align 不用管: foc_motor.c 的 FOC 初始化每次都会重钳它 */
        dirty = 1;
    }

    if(!dirty)
    {
        printf("CFG: unchanged, Flash not written\r\n");
        return 0;
    }

    motion_save_internal();                     /* 只写这一次 */
    /* 刚存进去的这套值, 下次上电就是从 Flash 恢复的 -> 来源标签也得跟着改,
     * 否则 '#' 会一直说 "[compiled default]" (值和来源对不上, 2026-09-28 实测)。 */
    cfg_from_flash = 1u;
    printf("CFG: saved to Flash\r\n");
    return 0;
}

int Motion_SetPolePairs(int pp)    { return Motion_SetMotorCfg(pp, 0.0f, 0.0f); }
int Motion_SetStallAmp(float a)    { return Motion_SetMotorCfg(0, a, 0.0f); }
int Motion_SetVbus(float v)        { return Motion_SetMotorCfg(0, 0.0f, v); }

int   Motion_PolePairs(void)    { return pole_pairs; }
float Motion_StallAmp(void)     { return motor_stall_amp; }
float Motion_Vbus(void)         { return voltage_power_supply; }
int   Motion_CfgFromFlash(void) { return (int)cfg_from_flash; }

void Motion_PrintMotorCfg(void)
{
    printf("CFG: pole_pairs = %d   (compiled default %d)  [%s]\r\n",
           pole_pairs, MOTOR_POLE_PAIRS, cfg_from_flash ? "Flash" : "compiled default");
    printf("CFG: stall      = %s A (compiled default %s A)  [%s]\r\n",
           FmtF(motor_stall_amp, 2), FmtF(MOTOR_STALL_AMP, 2),
           cfg_from_flash ? "Flash" : "compiled default");
    printf("CFG: vbus       = %s V (compiled default %s V)  [%s]  range %s..%s V\r\n",
           FmtF(voltage_power_supply, 2), FmtF(MOTOR_VOLTAGE_SUPPLY, 1),
           cfg_from_flash ? "Flash" : "compiled default",
           FmtF(MOTOR_CFG_VBUS_MIN, 1), FmtF(MOTOR_CFG_VBUS_MAX, 1));
    printf("CFG: how to set: UART '#<pp>,<A>,<V>' or CAN 0x116; re-run 'A' after changing pole_pairs\r\n");
    PosLog_PrintInfo();
}

int Motion_GotoHome(float vlim_rad_s)
{
    if(!home_set)
    {
        printf("HOME: no mech zero set; send 0x112 / press F first\r\n");
        return -1;
    }

    if(vlim_rad_s <= 0.0f) vlim_rad_s = MOTION_HOME_VLIM_DEF;

    printf("HOME: go to mech origin (raw target=%s rad, current mech coord=%s rad)\r\n",
           FmtF(home_off_rad, 4), FmtF(Motion_PosMechRad(), 4));
    Motion_RunPV(home_off_rad, vlim_rad_s);
    return 0;
}

void Motion_PrintHome(void)
{
    printf("HOME: set=%d home=%s rad | mech coord=%s rad (%s turns) raw=%s rad\r\n",
           (int)home_set, FmtF(home_off_rad, 4),
           FmtF(Motion_PosMechRad(), 4), FmtF(Motion_PosMechRad() / _2PI, 3),
           FmtF(shaft_angle, 4));
}

int     Motion_IsRunning(void) { return (mode == PV_POS || mode == PV_VEL) ? 1 : 0; }
int     Motion_Arrived(void)   { return arrived ? 1 : 0; }
int     Motion_IsStalled(void) { return stalled ? 1 : 0; }
float   Motion_RefRad(void)    { return ref_rad; }

/* 轨迹参考速度 —— 给 foc_motor.c 的 Type_angle 做前馈。
 * **只在 PV 位置轨迹跑着的时候才给值**: 手动 'P'/'V'/'T' 路径不会每拍刷新
 * vref_rad, 拿旧值当前馈会在切模式后莫名冲一下。 */
float   Motion_RefVel(void)    { return (mode == PV_POS) ? vref_rad : 0.0f; }
float   Motion_PosRad(void)    { return shaft_angle; }
float   Motion_VelRad(void)    { return shaft_velocity; }
float   Motion_PeakAmp(void)   { return CS_PeakAmp(); }

void Motion_ClearFault(void)
{
    stalled = 0;
    arrived = 0;
    CS_ResetWins();

    if(!Cmd_IsAligned())
    {
        printf("MOT: fault cleared (not aligned, run 'A')\r\n");
        return;
    }

    DRV8313_Enable();
    controller = Type_angle;
    ref_rad    = shaft_angle;
    tgt_rad    = shaft_angle;
    mode       = PV_HOLD;
    cmd_target = ref_rad;
    printf("MOT: fault cleared, hold at %s rad\r\n", FmtF(ref_rad, 4));
}

uint8_t Motion_StatusBits(void)
{
    uint8_t b = 0;

    if(Motion_IsRunning()) b |= CAN_ST_RUN;
    if(arrived)            b |= CAN_ST_ARRIVED;
    if(stalled)            b |= CAN_ST_STALL;
    if((uint32_t)(Micros() - sens_err_us) < 1000000u) b |= CAN_ST_SENSOR_ERR;
    if(Cmd_IsAligned())    b |= CAN_ST_ALIGNED;
    if(restored)           b |= CAN_ST_POS_RESTORED;
    if(Motion_IsEnabled()) b |= CAN_ST_ENABLED;
    if(home_set)           b |= CAN_ST_ZERO_SET;
    return b;
}

void Motion_PrintStatus(void)
{
    const char *mn = "IDLE";
    uint32_t now = Micros();

    if(mode == PV_POS)  mn = "POS";
    if(mode == PV_VEL)  mn = "VEL";
    if(mode == PV_HOLD) mn = "HOLD";

    printf("MOT: mode=%s stalled=%d arrived=%d\r\n",
           mn, (int)stalled, (int)arrived);
    printf("MOT: pos=%s rad (%s turns) vel=%s rad/s ref=%s rad\r\n",
           FmtF(shaft_angle, 4), FmtF(shaft_angle / _2PI, 3),
           FmtF(shaft_velocity, 4), FmtF(ref_rad, 4));
    printf("MOT: target=%s rad vlim=%s rad/s\r\n", FmtF(tgt_rad, 4), FmtF(vlim_rad, 4));
    printf("MOT: enabled=%d home_set=%d home=%s rad | mech=%s rad (%s turns)\r\n",
           Motion_IsEnabled(), (int)home_set, FmtF(home_off_rad, 4),
           FmtF(Motion_PosMechRad(), 4), FmtF(Motion_PosMechRad() / _2PI, 3));
    printf("MOT: cur peak=%s A (windows_over=%lu/%u) since_save=%lums\r\n",
           FmtF(CS_PeakAmp(), 3), (unsigned long)CS_OverWins(), (unsigned)MOTOR_STALL_WINS,
           (unsigned long)((now - last_save_us) / 1000u));
    PosLog_PrintInfo();
}
