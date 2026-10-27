#ifndef __MOTION_PV_H
#define __MOTION_PV_H

#include "ch32l103.h"

/* =============================================================================
 * 位置-速度 (PV) 运动 + 多圈位置 + 堵转保护
 * -----------------------------------------------------------------------------
 * 语义: CAN 命令给的"速度"是**速度上限**, 位置是终点。内部生成一条加减速参考
 *       轨迹 (梯形/三角), 参考位置交给已有的角度环 (Type_angle) 去跟。
 * 位置: 用 Sensor_GetAngle() 的累计角做多圈, 单位 rad, 可正可负,
 *       掉电保存/恢复由 pos_log 负责。
 * ========================================================================== */

#define MOTION_PV_DECEL      300.0f     /* 轨迹加/减速度 rad/s^2 (2026-10-06 实测定的, 见 docs)
                                         * 参考速度 v_ref=sqrt(2*a*err)。原来是 20 —— 30 度这么短的
                                         * 行程参考峰值只有 4.58 rad/s (43.7rpm), 短程"肉"的主要来源。
                                         * 实测 (22V 母线, 30 度阶跃, ang_ff=1/ang_p=10/ang_d=1):
                                         *   a=20  : 老固件, t90=418ms
                                         *   a=150 : t90=74ms  t99=322ms  超调<=3.8%
                                         *   a=300 : t90=55ms  t99=171ms  超调<=0.6%  (干净跑)
                                         *   a=500 : t90=44ms  t99=49ms   但干净跑也有 7.8~11% 超调 -> 否掉
                                         * 所以 300 是"再快就刹不住"的那一档; 真正上限取决于能出多少电流,
                                         * 由软电流限幅 (foc_pid.c volt_limit_now) 兜底。 */
#define MOTION_PV_ARRIVE     0.005f     /* 到位判定阈值 rad (设计值) */
#define MOTION_PV_VLIM_MIN   0.1f       /* 命令给 0 速度时用的兜底上限 rad/s */

void    Motion_Init(void);
void    Motion_Task(void);                      /* 主循环调用, 越高频越好 */
int     Motion_RestoreFromLog(void);            /* 上电恢复位置/对齐参数 */

void    Motion_RunPV(float target_rad, float vlim_rad_s);
void    Motion_RunVEL(float vel_rad_s);
void    Motion_Stop(uint8_t release);           /* release=1 松轴(断 EN) */
void    Motion_SyncAngleTarget(void);        /* 对齐/使能后把角度目标拉到当前轴位置 */
void    Motion_AbortPV(void);                /* 切手动模式前丢掉位置轨迹 (cmd.c 'M' 调用) */
void    Motion_SaveNow(void);
void    Motion_CalCurrentZero(void);            /* 电机必须停转 */

/* ---- 使能 / 机械原点 (CAN 命令 0x110~0x112, 控制台 E/D/N/F) ---- */
int     Motion_SetEnabled(uint8_t on);          /* 1=使能并保持当前位置 0=失能松轴; -1=被拒 */
int     Motion_IsEnabled(void);                 /* = DRV8313 的 EN 引脚状态 */
void    Motion_SetHomeHere(void);               /* 把当前位置设为机械 0 点, 并存 Flash */
/* ---- 电机参数 (2026-09-28 起进 Flash): 极对数 / 堵转阈值 / 母线电压 ----
 * 换电机、换母线电源都不用重编: 设一次就存在 Flash 里, 上电覆盖 board.h 的编译默认值。
 * 改极对数会**作废对齐** (零点电角是用旧极对数算的) -> 必须重跑 'A'。
 * 改母线电压不作废对齐 (它只进 SVPWM 归一化), 但会跟着重钳相电压上限。 */
int     Motion_SetMotorCfg(int pp, float amp, float vbus);
void    Motion_ApplyBootVoltageLimit(void);   /* 上电默认 Uq 上限 = vbus/sqrt(3) */
                                                 /* 0 = 成功; -1 = 越界 / 改极对数时驱动使能中
                                                  * 每项给 0 = 不改这一项
                                                  * 几项都给 = 一次校验 + 只写一次 Flash */
int     Motion_SetPolePairs(int pp);            /* = Motion_SetMotorCfg(pp, 0, 0)  */
int     Motion_SetStallAmp(float amp);          /* = Motion_SetMotorCfg(0, amp, 0)  */
int     Motion_SetVbus(float v);                /* = Motion_SetMotorCfg(0, 0, v)    */
void    Motion_PrintMotorCfg(void);             /* 串口 '#' */
int     Motion_PolePairs(void);                 /* 当前极对数 (给 CAN 0x184 用) */
float   Motion_StallAmp(void);                  /* 当前堵转阈值 A */
float   Motion_Vbus(void);                      /* 当前母线电压 V (给 CAN 0x184 用) */
int     Motion_CfgFromFlash(void);              /* 1 = 参数来自 Flash 记录 */
void    Motion_ClearHome(void);                 /* 取消机械零位 (机械坐标 = 原始坐标) */
int     Motion_HomeSet(void);
float   Motion_HomeRad(void);                   /* 机械零位在原始坐标系里的位置 rad */
float   Motion_PosMechRad(void);                /* 机械坐标 = 原始累计位置 - 机械零位 */
int     Motion_GotoHome(float vlim_rad_s);      /* 回机械原点; -1 = 没设过零位 */
void    Motion_PrintHome(void);

int     Motion_IsRunning(void);
int     Motion_Arrived(void);
int     Motion_IsStalled(void);
void    Motion_ClearFault(void);
uint8_t Motion_StatusBits(void);
float   Motion_PosRad(void);
float   Motion_VelRad(void);
float   Motion_RefRad(void);
float   Motion_RefVel(void);                /* 轨迹参考速度 rad/s —— **只给角度环做前馈用**,
                                             * mode!=PV_POS 时恒返回 0 */
float   Motion_PeakAmp(void);
void    Motion_PrintStatus(void);               /* 'W' */

/* ---- 板载动作脚本 (2026-09-29) ----
 * 语法 / 语义 / Flash 策略写在 motion_pv.c 顶部那段注释里, 这里是给外面的入口。
 * 用途: 让"没有 shell 的另一颗单片机"(UART 或 CAN)发一串动作, 板子自己一段段跑完。
 * 脚本里的角度是**相对量**(相对本遍起点), 不依赖机械零位。 */
#define MOTION_SEQ_MAX_SEG   16
int     Motion_Seq_Append(const char *script);      /* UART '*<脚本>': 加段; 返回新增段数, <0 = 出错 */
int     Motion_Seq_AddSeg(long wait_us, int rpm, float deg, long dwell_ms);  /* CAN 0x120 用 */
int     Motion_Seq_Start(int loops);                /* 开始; 0 = 成功, <0 = 被拒(原因已打印) */
void    Motion_Seq_Clear(void);                     /* '*0' / CAN 0x121 b0=0 */
void    Motion_Seq_Abort(void);                     /* 停脚本; 停/失能/堵转/手动命令时内部会调 */
void    Motion_Seq_Print(void);                     /* '*?' / CAN 0x121 b0=2 */
int     Motion_Seq_Running(void);
int     Motion_Seq_Loaded(void);
int     Motion_Seq_Remain(void);                    /* 这一遍还剩几段 (报在 0x184 b6) */
int     Motion_Seq_Pass(void);                      /* 正在跑第几遍 (报在 0x184 b7) */

#endif
