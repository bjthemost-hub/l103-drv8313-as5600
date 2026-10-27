#include "pos_log.h"
#include "board.h"
#include "ch32l103_flash.h"
#include "debug.h"
#include "foc_utils.h"
#include <math.h>

/* =============================================================================
 * 位置掉电保存: Flash 页日志 (说明见 pos_log.h)
 *
 * 记录 32 字节, 一页 256B 放 8 条:
 *   off  0  uint32 magic  0x4C503330 ('LP30')
 *   off  4  uint32 seq    记录序号, 越大越新
 *   off  8  int32  pos_mrad       Sensor_GetAngle() 累计角 * 1000
 *   off 12  int32  offset_mrad    sensor_offset * 1000
 *   off 16  int16  direction      +1 / -1 / 0(未对齐)
 *   off 18  int16  zero_elec_mrad zero_electric_angle * 1000
 *   off 20  int32  home_mrad      机械零位 * 1000 (0x7FFFFFFF = 没设过)
 *   off 24  int16  stall_ca       堵转/过流阈值 * 100  (0.01A 单位)
 *   off 26  uint16 pole_pairs     电机极对数 (1..MOTOR_CFG_PP_MAX)
 *   off 26  uint16 pole_pairs     电机极对数 (1..MOTOR_CFG_PP_MAX)
 *   off 28  uint16 vbus_cv        母线电压 * 100 (0.01V 单位)
 *   off 30  uint16 reserved       预留 (写 0xFFFF)
 *
 *   ★ 2026-09-19: 20B -> 24B (加了机械零位), magic 'LP10' -> 'LP20'。
 *   ★ 2026-09-28: 24B -> 28B (加了电机参数: 极对数 + 堵转阈值), 'LP20' -> 'LP30'。
 *   ★ 2026-09-28: 28B -> 32B (加了**母线电压**), 'LP30' -> 'LP31'。
 *     老记录不再被认, 所以升级后第一次上电的位置恢复会丢一次, 这是预期的;
 *     电机参数(极对数/阈值/母线电压)也要用 '#' / CAN 0x116 重新设一次。
 *
 * 4 页轮转 (0x0800FC00 ~ 0x0800FFFF, 64KB Flash 的最末 1KB):
 *   一页 8 条 -> 一整轮 32 次保存才擦一次页。
 *
 * ★ 日志区必须待在代码区之上 ★
 *   PosLog_Init() 会先算一次"程序镜像的 Flash 末尾地址"
 *   (= _data_lma + sizeof(.data), 从 Link.ld 的符号推), 一旦顶到日志区就
 *   自动停用日志并打印原因 —— 免得哪天代码长大了把程序自己擦掉。
 *   2026-09-19 加这个功能时就是这样: 代码已经到了 0xE104, 原来放在 0xE000 的
 *   日志区正好压在程序上。
 *
 * 擦写粒度依据: ch32l103_flash.c 中 FLASH_ErasePage_Fast 官方注释
 *               "Erases a specified FLASH page (1page = 256Byte)";
 *               FLASH_ProgramPage_Fast + FLASH_BufLoad 一次写满整页
 *               (WCH IAP 例程 iap.c: CH32_IAP_Program() 就是这个用法)。
 * 耗时依据:     数据手册 V2.1 表3-18 (页擦除 6.2/7.5ms, 页编程 2.0/2.5ms 典型值)。
 * 寿命依据:     数据手册 V2.1 表3-19 (擦写次数 25C 300K 次 / 70C 100K 次)。
 * ========================================================================== */

/* 2026-09-19: 8 页 -> 4 页, base 0x0800F800 -> 0x0800FC00。
 * 原因: 加上 faults.c 之后代码涨到 62908 B, 离 0x0800F800 只剩 580 B
 * (PosLog_Init 有"代码顶到日志区就自动停用"的保护, 但那样位置就不保存了)。
 * 4 页 = 40 条/轮, 按自动保存最快 1 次/秒算, 40 秒才擦一次页;
 * 手册表3-19 给的是 25C 300K 次擦写 -> 足够用, 换回 1KB 代码空间。 */
#define POSLOG_BASE         0x0800FC00u
#define POSLOG_PAGES        4u
#define POSLOG_REC_SIZE     32u
#define POSLOG_REC_WORDS    (POSLOG_REC_SIZE / 4u)  /* 8 个字 */
#define POSLOG_RECS         8u                      /* 256 / 32 = 8 条 (整页用满) */
#define POSLOG_MAGIC        0x4C503331u             /* 'LP31' */

static uint32_t shadow[64];                          /* 当前页 256B 影子 */
static int      inited;
static int      cur_page;
static int      cur_slot;
static uint32_t next_seq;
static uint32_t n_ok, n_fail, n_erase;

static uint32_t page_addr(int p)
{
    return POSLOG_BASE + 256u * (uint32_t)p;
}

/* 0x0800_xxxx (pos_log 用的真实 Flash 地址) 与 0x0000_xxxx (Link.ld 的 ORIGIN=0)
 * 是同一块 Flash 的两个别名 —— 所以比"撞上日志区没有"之前必须先归一到物理偏移
 * (低 24 位)。2026-09-28 修正: 以前直接拿 code_flash_end() 跟 POSLOG_BASE 比,
 * 一个是 0x0000_xxxx、一个是 0x0800_xxxx, 不等式永远为假, 这个保护形同虚设
 * (当时程序已经长到 0x0000_FC1C, 真的压进日志区 28 字节了)。 */
static uint32_t flash_off(uint32_t a) { return a & 0x00FFFFFFu; }

/* 程序镜像在 Flash 里的末尾地址。
 * Link.ld 里 .data 是 ">RAM AT>FLASH", 它的加载映像紧跟在 .text/.rodata 后面,
 * 所以 末尾 = _data_lma + (_edata - _data_vma)。 */
static uint32_t code_flash_end(void)
{
    extern char _data_lma[];
    extern char _data_vma[];
    extern char _edata[];

    return (uint32_t)_data_lma + (uint32_t)(_edata - _data_vma);
}

static uint32_t rd32(uint32_t a) { return *(volatile uint32_t *)a; }
static uint16_t rd16(uint32_t a) { return *(volatile uint16_t *)a; }

static void page_fill_erased(void)
{
    uint32_t i;
    for(i = 0; i < 64u; i++) shadow[i] = 0xFFFFFFFFu;
}

static void page_program(int p)
{
    uint32_t i;

    FLASH_Unlock_Fast();
    FLASH_BufReset();
    for(i = 0; i < 64u; i++) FLASH_BufLoad(page_addr(p) + 4u * i, shadow[i]);
    FLASH_ProgramPage_Fast(page_addr(p));
    FLASH_Lock_Fast();
}

static void page_erase(int p)
{
    FLASH_Unlock_Fast();
    FLASH_ErasePage_Fast(page_addr(p));
    FLASH_Lock_Fast();
    n_erase++;
}

static int rec_valid(int p, int slot)
{
    uint32_t a = page_addr(p) + POSLOG_REC_SIZE * (uint32_t)slot;
    return (rd32(a) == POSLOG_MAGIC) ? 1 : 0;
}

static void rec_unpack(int p, int slot, PosLog_State_t *st, uint32_t *seq)
{
    uint32_t a = page_addr(p) + POSLOG_REC_SIZE * (uint32_t)slot;
    int32_t  pos_m, off_m, home_m;
    int16_t  dir, ze_m, stall_ca;
    uint16_t pp, vbus_cv;

    *seq   = rd32(a + 4u);
    pos_m  = (int32_t)rd32(a + 8u);
    off_m  = (int32_t)rd32(a + 12u);
    dir    = (int16_t)rd16(a + 16u);
    ze_m   = (int16_t)rd16(a + 18u);
    home_m = (int32_t)rd32(a + 20u);
    stall_ca = (int16_t)rd16(a + 24u);
    pp       = rd16(a + 26u);
    vbus_cv  = rd16(a + 28u);

    st->pos_rad             = (float)pos_m * 0.001f;
    st->sensor_offset       = (float)off_m * 0.001f;
    st->sensor_direction    = (int)dir;
    st->zero_electric_angle = (float)ze_m * 0.001f;
    st->home_off_rad        = (home_m == 0x7FFFFFFF)
                              ? POSLOG_HOME_NONE : (float)home_m * 0.001f;
    st->pole_pairs          = (int)pp;
    st->stall_amp           = (float)stall_ca * 0.01f;
    st->vbus_v              = (float)vbus_cv * 0.01f;
}

/* 扫所有页, 找序号最大的有效记录; 返回 1 = 找到 */
static int scan_best(PosLog_State_t *st, uint32_t *best_seq, int *best_page)
{
    uint32_t p, s;
    uint32_t best = 0;
    int      found = 0;

    for(p = 0; p < POSLOG_PAGES; p++)
    {
        for(s = 0; s < POSLOG_RECS; s++)
        {
            uint32_t seq;
            PosLog_State_t tmp;

            if(!rec_valid((int)p, (int)s)) continue;
            rec_unpack((int)p, (int)s, &tmp, &seq);

            if(!found || (int32_t)(seq - best) > 0)
            {
                best = seq;
                *best_seq  = seq;
                *best_page = (int)p;
                *st = tmp;
                found = 1;
            }
        }
    }
    return found;
}

void PosLog_Init(void)
{
    PosLog_State_t st;
    uint32_t best_seq  = 0;
    int      best_page = 0;

    inited   = 0;
    n_ok     = 0;
    n_fail   = 0;
    n_erase  = 0;
    cur_slot = 0;

    if(flash_off(code_flash_end()) > flash_off(POSLOG_BASE))
    {
        printf("POSLOG: DISABLED - code ends at 0x%08lX, log base=0x%08lX (overlap!)\r\n",
               (unsigned long)code_flash_end(), (unsigned long)POSLOG_BASE);
        return;
    }

    if(scan_best(&st, &best_seq, &best_page))
    {
        next_seq = best_seq + 1u;
        cur_page = (best_page + 1) % (int)POSLOG_PAGES;
    }
    else
    {
        next_seq = 1u;
        cur_page = 0;
    }

    page_erase(cur_page);
    page_fill_erased();
    inited = 1;
}

int PosLog_Load(PosLog_State_t *st)
{
    uint32_t best_seq  = 0;
    int      best_page = 0;

    if(!scan_best(st, &best_seq, &best_page)) return 0;
    return 1;
}

int PosLog_Save(const PosLog_State_t *st)
{
    uint32_t base, magic, v, w6, w7;
    int32_t  pos_m, off_m, home_m;
    int16_t  dir, ze_m, stall_ca;
    uint16_t pp, vbus_cv;
    int      i, ok;

    if(!inited) return 0;

    /* 本页写满 -> 轮转到下一页: 擦掉它, 从头写 */
    if(cur_slot >= (int)POSLOG_RECS)
    {
        cur_page = (cur_page + 1) % (int)POSLOG_PAGES;
        page_erase(cur_page);
        page_fill_erased();
        cur_slot = 0;
    }

    pos_m  = (int32_t)(st->pos_rad * 1000.0f);
    off_m  = (int32_t)(st->sensor_offset * 1000.0f);
    dir    = (int16_t)st->sensor_direction;
    ze_m   = (int16_t)(st->zero_electric_angle * 1000.0f);
    home_m = (st->home_off_rad > 1e29f)
             ? 0x7FFFFFFF : (int32_t)(st->home_off_rad * 1000.0f);

    /* 电机参数 (2026-09-28): 越界就退回编译默认, 免得把坏值写进去 */
    pp       = (uint16_t)((st->pole_pairs >= MOTOR_CFG_PP_MIN &&
                           st->pole_pairs <= MOTOR_CFG_PP_MAX)
                          ? st->pole_pairs : MOTOR_POLE_PAIRS);
    {   float a = st->stall_amp;
        if(!(a >= MOTOR_CFG_AMP_MIN && a <= MOTOR_CFG_AMP_MAX)) a = MOTOR_STALL_AMP;
        stall_ca = (int16_t)(a * 100.0f + (a >= 0.0f ? 0.5f : -0.5f));
    }
    /* 母线电压 (2026-09-28 后加): 0.01V 单位 */
    {   float vv = st->vbus_v;
        if(!(vv >= MOTOR_CFG_VBUS_MIN && vv <= MOTOR_CFG_VBUS_MAX)) vv = MOTOR_VOLTAGE_SUPPLY;
        vbus_cv = (uint16_t)(vv * 100.0f + 0.5f);
    }

    i = cur_slot * (int)POSLOG_REC_WORDS;
    shadow[i + 0] = POSLOG_MAGIC;
    shadow[i + 1] = next_seq;
    shadow[i + 2] = (uint32_t)pos_m;
    shadow[i + 3] = (uint32_t)off_m;
    shadow[i + 4] = ((uint32_t)(uint16_t)ze_m << 16) | (uint32_t)(uint16_t)dir;
    shadow[i + 5] = (uint32_t)home_m;
    shadow[i + 6] = ((uint32_t)pp << 16) | (uint32_t)(uint16_t)stall_ca;
    shadow[i + 7] = (0xFFFFu << 16) | (uint32_t)vbus_cv;
    next_seq++;

    page_program(cur_page);

    /* 写完回读校验: 整页重编程(在已写过的页上追加)到底行不行, 用这个计数说话 */
    ok    = 1;
    base  = page_addr(cur_page) + POSLOG_REC_SIZE * (uint32_t)cur_slot;
    magic = rd32(base + 0u);
    v     = rd32(base + 4u);
    if(magic != POSLOG_MAGIC || v != next_seq - 1u) ok = 0;
    if((int32_t)rd32(base + 8u) != pos_m) ok = 0;
    if((int32_t)rd32(base + 20u) != home_m) ok = 0;
    w6 = rd32(base + 24u);
    if((int16_t)(w6 & 0xFFFFu) != stall_ca) ok = 0;
    if((uint16_t)(w6 >> 16) != pp) ok = 0;
    w7 = rd32(base + 28u);
    if((uint16_t)(w7 & 0xFFFFu) != vbus_cv) ok = 0;

    if(ok) n_ok++; else n_fail++;

    cur_slot++;
    return ok;
}

uint32_t PosLog_WriteOk(void)    { return n_ok; }
uint32_t PosLog_WriteFail(void)  { return n_fail; }
uint32_t PosLog_EraseCount(void) { return n_erase; }
uint32_t PosLog_Seq(void)        { return next_seq; }

void PosLog_PrintInfo(void)
{
    PosLog_State_t st;
    int have = PosLog_Load(&st);

    printf("POSLOG: base=0x%08lX pages=%u page=%d slot=%d/%u seq=%lu | code_end=0x%08lX\r\n",
           (unsigned long)POSLOG_BASE, (unsigned)POSLOG_PAGES,
           cur_page, cur_slot, (unsigned)POSLOG_RECS, (unsigned long)next_seq,
           (unsigned long)code_flash_end());
    printf("POSLOG: write_ok=%lu write_fail=%lu erase=%lu\r\n",
           (unsigned long)n_ok, (unsigned long)n_fail, (unsigned long)n_erase);

    if(have)
    {
        printf("POSLOG: stored pos=%s rad offset=%s dir=%d zero_elec=%s\r\n",
               FmtF(st.pos_rad, 4), FmtF(st.sensor_offset, 4), st.sensor_direction,
               FmtF(st.zero_electric_angle, 4));
        if(st.home_off_rad > 1e29f)
            printf("POSLOG: home = (never set)\r\n");
        else
            printf("POSLOG: home = %s rad\r\n", FmtF(st.home_off_rad, 4));
        printf("POSLOG: motor cfg: pole_pairs=%d stall=%s A vbus=%s V\r\n",
               st.pole_pairs, FmtF(st.stall_amp, 2), FmtF(st.vbus_v, 2));
    }
    else
    {
        printf("POSLOG: no valid record (first run)\r\n");
    }
}
