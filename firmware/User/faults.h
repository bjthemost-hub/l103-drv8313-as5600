#ifndef __FAULTS_H
#define __FAULTS_H

#include "ch32l103.h"

/* =============================================================================
 * 故障采集 + 主动上报
 * -----------------------------------------------------------------------------
 * 采集 (Fault_Task, 主循环调用):
 *   堵转 / 电流超限 / 失能 / 母线电压低 / AS5600 失败 / 未对齐 /
 *   Flash 保存失败 / DRV8313 nFAULT 低 / 命令被拒
 *
 * 上报 (CAN):
 *   0x1A0  事件帧: 某个故障"发生"或"消失"的瞬间各发一帧
 *   0x183  位图帧: 活动位 + 锁存位 + 累计事件数 (周期重报 / 查询应答)
 *
 * 几条判定上的讲究:
 *   失能   : 只在"使能过 -> 没使能"的下降沿算故障。上电本来就没使能, 不报。
 *   未对齐 : 只在"有命令因为没对齐被拒"之后才算, 一旦对齐成功自动撤。
 *   命令被拒: 脉冲, 保持 1s; 事件帧的 info 里给的是被拒的命令 ID。
 *   电流超限: 用窗口峰值, 掉到阈值的 80% 以下才撤 (回差), 免得在阈值上抖。
 *   VM_LOW : 本版硬件没有 VM 分压 (见 board.h), 采不到就不报, 也不会误报。
 * ========================================================================== */

#define CAN_FAULT_STALL         0x0001u   /* 堵转: 已停 PWM + 松轴 */
#define CAN_FAULT_OVERCURRENT   0x0002u   /* 相电流超限 */
#define CAN_FAULT_DISABLED      0x0004u   /* 使能过又被失能 */
#define CAN_FAULT_VM_LOW        0x0008u   /* 母线电压低 (需 VM 采样) */
#define CAN_FAULT_SENSOR        0x0010u   /* AS5600 读失败 */
#define CAN_FAULT_NOT_ALIGNED   0x0020u   /* 没对齐 (有命令因此被拒) */
#define CAN_FAULT_FLASH_FAIL    0x0040u   /* 位置保存 Flash 写失败 */
#define CAN_FAULT_DRV_NFAULT    0x0080u   /* DRV8313 nFAULT 被拉低 */
#define CAN_FAULT_CMD_REJECT    0x0100u   /* 命令被拒 (脉冲 1s) */

/* 事件帧 (0x1A0) 的 b3: 说明 b4..5 那个数是什么 */
#define FAULT_INFO_NONE         0u
#define FAULT_INFO_AMP          1u        /* 0.01 A */
#define FAULT_INFO_VOLT         2u        /* 0.1 V */
#define FAULT_INFO_CMDID        3u        /* 被拒的命令 ID */

void     Fault_Init(void);
void     Fault_Task(void);                       /* 主循环调用 */
uint16_t Fault_Active(void);
uint16_t Fault_Latched(void);
uint32_t Fault_EventCount(void);
void     Fault_Clear(uint16_t bits);
void     Fault_SetReport(uint8_t on, uint8_t period_100ms);
void     Fault_GetReport(uint8_t *on, uint8_t *period_100ms);
void     Fault_SendBitmap(uint16_t id);          /* 0x183 那一帧 */
void     Fault_ReportReject(uint16_t cmd_id, uint16_t reason_bit);
void     Fault_Print(void);

#endif
