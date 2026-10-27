#ifndef __CAN_BUS_H
#define __CAN_BUS_H

#include "ch32l103.h"

/* =============================================================================
 * CAN1 应用层 (SN65HVD230 收发器, PB8=RX / PB9=TX, 1Mbps)
 * -----------------------------------------------------------------------------
 * 报文约定 (全部标准帧 11bit, 数据小端)
 *
 *   0x100  主机 -> 板 : 位置-速度命令 (PV)
 *           b0..3  int32  目标位置, 单位 mrad (0.001 rad), 可正可负, 可多圈
 *           b4..5  int16  目标速度上限, 单位 0.01 rad/s
 *           b6     flags  bit0 1=相对位置(相对收到命令那一刻) 0=绝对位置
 *                         bit1 1=运行 0=停止
 *                         bit2 1=停止时释放(松轴, 断 EN) 0=停止时保持在原位
 *                         bit3 1=清除堵转故障(顺便重新使能)
 *                         bit4 1=立刻把当前位置存进 Flash
 *                         bit5 1=目标位置用机械坐标(需先设过机械零位, 见 0x112)
 *                              bit0 和 bit5 同时置位时按 bit0 (相对) 处理
 *           b7     保留 (0)
 *
 *   0x101  主机 -> 板 : 纯速度命令
 *           b0..1  int16  速度, 单位 0.01 rad/s
 *           b2     flags  同 0x100 的 b6 (bit0 / bit5 忽略)
 *           b3..7  保留 (0)
 *
 *   0x110  主机 -> 板 : 使能 / 失能
 *           b0     1=使能(合上 EN, 进入角度环保持当前位置) 0=失能(松轴, 输出高阻)
 *                  * 使能要求已对齐; 否则拒绝并回 0x1A0(NOT_ALIGNED)
 *           b1..7  保留 (0)
 *           回复: 0x181
 *
 *   0x111  主机 -> 板 : 回机械原点 (走到 0x112 设定的那个零点)
 *           b0..1  int16  速度上限, 单位 0.01 rad/s (0 -> 默认 30rpm)
 *           b2..7  保留 (0)
 *           回复: 0x181   没设过零位 -> 拒绝, 回 0x1A0(CMD_REJECT)
 *
 *   0x112  主机 -> 板 : 把当前位置设为机械 0 点 (立即存 Flash)
 *           b0..7  保留 (0)
 *           回复: 0x181 + 0x182
 *
 *   0x113  主机 -> 板 : 询问
 *           b0     0=位置+速度(回 0x181)  1=配置(回 0x182)
 *                  2=故障(回 0x183)      3=电机参数(回 0x184)
 *                  0xFF=以上全回 (0x181 + 0x182 + 0x183 + 0x184)
 *           回复: 按 b0
 *
 *   0x114  主机 -> 板 : 故障主动上报配置
 *           b0     0=关 1=开 (默认开)
 *           b1     周期重报间隔, 单位 100ms (0 = 只在变化时报; 默认 10 = 1s)
 *           回复: 0x182
 *
 *   0x115  主机 -> 板 : 设角度环的速度上限 (PID_Angle 输出的钳位)
 *           b0..1  int16  速度上限, 单位 0.01 rad/s (<=0 忽略)
 *           回复: 0x181
 *
 *   0x116  主机 -> 板 : 设**电机参数** (极对数 / 堵转阈值 / 母线电压), 立即存 Flash
 *                      (2026-09-28 新增: 换电机/换母线电源不用再改代码重编)
 *           b0     uint8  极对数 (1..64); **0 = 这一条不改**
 *           b1..2  int16  堵转/过流阈值, 单位 0.01 A; **0 = 这一条不改**
 *           b3     flags  bit0 **必须为 1** —— 这是"确认要改"的防呆位,
 *                         因为这些参数会一直留在 Flash 里; 给 0 直接拒
 *           b4..5  int16  母线电压, 单位 0.01 V; **0 = 这一条不改** (dlc < 6 时当没给)
 *           b6..7  保留 (0)
 *           几条一起给时: 先一起校验, 只要有一条越界就整条不作数 (不会改一半),
 *           通过后**只写一次 Flash**。
 *           拒绝的条件: 参数越界 / b3 bit0=0 / 要改极对数时驱动还在使能中。
 *           ★ **改极对数会把"已对齐"作废** —— zero_electric_angle 是用旧极对数
 *             算出来的 (foc_motor.c 的 AlignSensor)。所以改完必须重跑对齐 'A',
 *             否则电角度整圈都错 (没劲/抖/发热/不走)。
 *           回复: 0x184 (电机参数帧) + 0x182 (配置帧)
 *
 *   0x180  板 -> 主机 : 周期反馈, 每 10ms 一帧
 *           b0..3  int32  当前机械坐标位置, 单位 mrad (多圈)
 *                         (= 原始累计位置 - 机械零位; 没设过零位时二者相同)
 *           b4..5  int16  当前速度, 单位 0.01 rad/s
 *           b6     status 见 CAN_ST_*
 *           b7     int8   相电流, 单位 0.1 A (取三相重构后 |i| 最大的那相, 带符号)
 *
 *   0x181  板 -> 主机 : 即时状态帧 (= 0x113 的应答, 也是 0x110/0x111/0x112 的 ACK)
 *           布局与 0x180 完全一样
 *
 *   0x182  板 -> 主机 : 配置帧
 *           b0..3  int32  机械零位在原始坐标系里的位置, 单位 mrad
 *           b4..5  uint16 母线电压, 单位 0.1 V (本版硬件没有 VM 采样 -> 配置值)
 *           b6     flags  bit0 已设机械零位  bit1 已对齐   bit2 有 VM 采样
 *                         bit3 驱动已使能      bit4 故障主动上报开
 *           b7     uint8  上报周期, 单位 100ms
 *
 *   0x183  板 -> 主机 : 故障帧
 *           b0..1  uint16 当前活动故障位 (CAN_FAULT_*, 定义见 faults.h)
 *           b2..3  uint16 锁存故障位 (出现过就保持 1, 直到被清)
 *           b4..7  uint32 累计故障事件帧数
 *
 *   0x184  板 -> 主机 : **电机参数帧** (2026-09-28 新增)
 *           b0     uint8  当前极对数
 *           b1..2  int16  当前堵转/过流阈值, 单位 0.01 A
 *           b3     flags  bit0 参数来自 Flash 记录 (不是编译默认值)
 *                         bit1 已对齐        bit2 驱动已使能
 *           b4..5  int16  当前母线电压, 单位 0.01 V
 *           b6..7  保留 (0)
 *
 *   0x1A0  板 -> 主机 : 故障事件 (主动上报)
 *           b0..1  uint16 故障位
 *           b2     0=发生  1=消失
 *           b3     b4..5 是什么: 0 无 / 1 电流(0.01A) / 2 电压(0.1V) / 3 被拒命令ID
 *           b4..5  int16  附加数值
 *           b6..7  uint16 事件时刻, 单位 ms (上电起, 会回绕)
 *
 *   0x122  主机 -> 板 : 动作脚本加一段 · **大角度版** (2026-10-06)
                      布局与 0x120 逐字节相同, 只有角度那个字段的刻度不同:
                        0x120: b4..5 int16 角度 [0.1 度] -> 单段上限 3276.7 度
                        0x122: b4..5 int16 角度 [1   度] -> 单段上限 32767 度 (再按 SEQ_DEG_MAX 36000 判)
                      要在一条帧里给 >3276.7 度的段就用 0x122; 短段继续用 0x120 的 0.1 度。
                      回复: 0x181

   保留 ID: 0x102..0x10F, 0x117..0x11F, 0x123..0x17F (主机 -> 板)
 *            0x185..0x19F (应答), 0x1A1..0x1AF (事件), 0x1B0..0x1FF
 *
 *   ---- 主动上报总闸 (2026-09-29 新增) ----
 *   默认**被动**: 上电后 0x180 周期反馈、0x1A0 故障事件、周期 0x183 位图 **一帧都不主动发**,
 *   只有"被问才答" (0x113 -> 0x181/0x182/0x183/0x184, 以及各条命令的 ACK 帧)。
 *   要开周期上报: 串口发 `K1` (关: `K0`)。**不存 Flash**, 掉电即回到被动。
 *   主机想要数据就自己用 0x113 轮询 —— 一块 CAN 上挂十几个设备时, 本来就该这样。
 *
 *   0x180  板 -> 主机 : 反馈, 每 10ms 一帧 —— **只在 debug 打开时发** (2026-09-29 起)
 *           b0..3  int32  当前机械坐标位置, 单位 mrad (多圈, 上电从 Flash 恢复)
 *           b4..5  int16  当前速度, 单位 0.01 rad/s
 *           b6     status 见 CAN_ST_*
 *           b7     int8   相电流, 单位 0.1 A (取三相重构后 |i| 最大的那相, 带符号)
 * ========================================================================== */

/* ---- 主机 -> 板 (0x100 ~ 0x17F) ---- */
#define CAN_ID_CMD_PV        0x100u     /* 位置-速度 */
#define CAN_ID_CMD_VEL       0x101u     /* 纯速度 */
#define CAN_ID_CMD_ENABLE    0x110u     /* 使能 / 失能 */
#define CAN_ID_CMD_HOME      0x111u     /* 回机械原点 */
#define CAN_ID_CMD_SETZERO   0x112u     /* 把当前位置设为机械 0 点 */
#define CAN_ID_CMD_QUERY     0x113u     /* 询问 (位置速度 / 配置 / 故障) */
#define CAN_ID_CMD_FAULTCFG  0x114u     /* 故障主动上报开关 + 周期 */
#define CAN_ID_CMD_VLIM      0x115u     /* 设角度环速度上限 (rad/s) */
#define CAN_ID_CMD_MOTORCFG  0x116u     /* 设电机参数 (极对数/堵转阈值/母线电压) 并存 Flash */

/* ---- 板 -> 主机 (0x180 ~ 0x1FF) ---- */
#define CAN_ID_FB            0x180u     /* 10ms 周期反馈 */
#define CAN_ID_FB_QUERY      0x181u     /* 即时状态帧 (ACK / 询问应答) */
#define CAN_ID_FB_CFG        0x182u     /* 配置帧 */
#define CAN_ID_FB_FAULT      0x183u     /* 故障帧 (位图) */
#define CAN_ID_FB_MOTORCFG   0x184u     /* 电机参数帧 (极对数/堵转阈值/母线电压); b6/b7 = 队列剩余段数/遍数 */
#define CAN_ID_CMD_SEG       0x120u     /* 动作脚本: 追加一段 (2026-09-29) */
#define CAN_ID_CMD_SEGCTL    0x121u     /* 动作脚本: 控制 (b0 0=清空 1=开始 2=打印, b1=遍数) */
#define CAN_ID_CMD_SEG_HR    0x122u     /* 动作脚本: 追加一段·大角度版 (布局同 0x120, 角度 int16 x 1 度) */
#define CAN_ID_EVT_FAULT     0x1A0u     /* 故障事件 (主动上报) */

/* 反馈状态位 */
#define CAN_ST_RUN           0x01u      /* 位置/速度环正在跑 */
#define CAN_ST_ARRIVED       0x02u      /* 已到目标位置 */
#define CAN_ST_STALL         0x04u      /* 堵转过流锁存, PWM 已停 */
#define CAN_ST_SENSOR_ERR    0x08u      /* AS5600 读失败 */
#define CAN_ST_ALIGNED       0x10u      /* 有可用对齐参数 (方向/零点电角) */
#define CAN_ST_POS_RESTORED  0x20u      /* 当前位置来自 Flash 恢复 */
#define CAN_ST_ENABLED       0x40u      /* DRV8313 EN 已合上 */
#define CAN_ST_ZERO_SET      0x80u      /* 已设过机械零位 */

#define CAN_FB_PERIOD_US     10000u     /* 10ms */

void     CAN_Bus_Init(void);
int      CAN_Bus_Ready(void);
void     CAN_Bus_Task(void);                 /* 主循环调用: 收命令 + 10ms 反馈 */
int      CAN_Bus_SendStd(uint16_t id, const uint8_t *data, uint8_t dlc);
void     CAN_Bus_SetFeedback(uint8_t on);
uint8_t  CAN_Bus_FeedbackOn(void);
void     CAN_Bus_SetDebug(uint8_t on);       /* 主动上报总闸: 0=被动(默认) 1=debug */
uint8_t  CAN_Bus_DebugOn(void);
void     CAN_Bus_PrintStatus(void);          /* 'K' */
void     CAN_Bus_SendTest(void);             /* 'J' */
uint32_t CAN_Bus_RxCount(void);
uint32_t CAN_Bus_TxCount(void);
uint32_t CAN_Bus_TxFail(void);
uint32_t CAN_Bus_Overrun(void);
uint32_t CAN_Bus_LastId(void);
void     CAN_Bus_ResetCounters(void);
int      CAN_Bus_LastRxDlc(void);

#endif
