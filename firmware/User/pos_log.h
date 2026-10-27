#ifndef __POS_LOG_H
#define __POS_LOG_H

#include "ch32l103.h"

/* =============================================================================
 * 位置掉电保存 (Flash 页日志)
 * -----------------------------------------------------------------------------
 * 为什么不是"每次写同一个地址": Flash 只能 1->0, 原地改要整页擦除, 而擦一页
 * 要 6.2~7.5ms (数据手册 V2.1 表3-18)。一秒擦一次很快就写穿了。
 * 做法: 256B 一页切成 8 条 32B 记录 (每页记录见 PosLog_State_t / pos_log.c 的字段表),
 *       从前往后顺写; 一页写满就换下一页擦掉接着写 -> 一次擦除换 9 次保存。
 *       上电时扫所有页, 取"序号最大"的那条 = 最后一次保存的值。
 * 记录 32B:
 *   2026-09-19 20B -> 24B (加机械零位, magic 'LP10' -> 'LP20')
 *   2026-09-28 24B -> 28B (加电机参数: 极对数 + 堵转阈值, magic 'LP20' -> 'LP30')
 *   2026-09-28 28B -> 32B (加**母线电压**, magic 'LP30' -> 'LP31')
 * 每次扩字段都换 magic, 老记录不再被认 -> 升级固件后第一次上电位置恢复会丢一次,
 * 属预期 (位置/机械零位要重设; 电机参数也要重新设一次)。
 * 地址: 0x0800FC00 ~ 0x0800FFFF (64KB Flash 的最末 1KB, 4 页)
 * 粒度依据: ch32l103_flash.c 里 FLASH_ErasePage_Fast 官方注释
 *           "Erases a specified FLASH page (1page = 256Byte)"。
 * ========================================================================== */

typedef struct {
    float pos_rad;              /* Sensor_GetAngle() 的累计值 (方向无关) */
    float sensor_offset;        /* ShaftAngle() 用的偏置 */
    int   sensor_direction;     /* +1 / -1 / 0(未对齐) */
    float zero_electric_angle;  /* 对齐得到的零点电角 (rad) */
    float home_off_rad;         /* 机械零位在原始坐标系里的位置 (rad);
                                 * POSLOG_HOME_NONE = 从没设过机械零位 */
    int   pole_pairs;           /* 电机极对数 (2026-09-28 新增): 0 = 用 board.h 默认 */
    float stall_amp;            /* 堵转/过流阈值 A (2026-09-28 新增): 0 = 用 board.h 默认 */
    float vbus_v;               /* 母线电压 V (2026-09-28 新增): 0 = 用 board.h 默认 */
} PosLog_State_t;

/* "没设过机械零位" 的哨兵 (存 Flash 时用 int32 0x7FFFFFFF, 读出来换成它) */
#define POSLOG_HOME_NONE   1e30f

void     PosLog_Init(void);
int      PosLog_Load(PosLog_State_t *st);    /* 1 = 找到有效记录 */
int      PosLog_Save(const PosLog_State_t *st);
void     PosLog_PrintInfo(void);             /* 'W' */
uint32_t PosLog_WriteOk(void);
uint32_t PosLog_WriteFail(void);             /* 写完回读不一致的次数 (0 才对) */
uint32_t PosLog_EraseCount(void);
uint32_t PosLog_Seq(void);

#endif
