#ifndef __BOARD_H
#define __BOARD_H

/* =============================================================================
 * L103_CANUSB_DRV8313 板级定义 (引脚来自网表核对 2026-09-18)
 * -----------------------------------------------------------------------------
 * PWM_A = PA6 -> U6.27 (IN1)   TIM3_CH1
 * PWM_B = PA7 -> U6.25 (IN2)   TIM3_CH2
 * PWM_C = PB0 -> U6.23 (IN3)   TIM3_CH3
 * EN_A  = PB3 -> U6.26 (EN1)
 * EN_B  = PB4 -> U6.24 (EN2)
 * EN_C  = PB5 -> U6.22 (EN3)
 * nFAULT= PC14 <- U6.18        (R5 10k 上拉到 +3V3)
 *         **注意**: R5 上拉的是 MCU 侧的 +3V3, 与 DRV8313 是否在板/是否供电无关
 *         -> PC14=1 不能证明 8313 健康, 只能说明它没把线拉低
 * 3V3   = +3V3DC --FB3--> +3V3  (来自 24V->5V->3.3V 支路)
 *         **不是** U6.15(V3P3OUT): 网表核对 2026-09-19, U6.15 只接 C20/R6/R7,
 *         即 DRV8313 自己的 nRESET/nSLEEP 上拉, 不供给板子
 * AS5600= U5   I2C1: PB6=SCL / PB7=SDA
 * UART1 = PA9(TX)  / PA10(RX)   debug 输出
 * UART2 = PA2(TX)  / PA3(RX)    命令输入 (预留)
 * USB   = PA11(D-) / PA12(D+)   CDC 命令输入
 * CAN1  = PB8(RX)  / PB9(TX)
 * RV1   = PA5 (ADC5)            调速电位器 (装在减速输出轴上)
 * MOT_A_V = PA1 (INA240A2 U10)  相电流反馈
 * MOT_B_V = PA4 (INA240A2 U8)   相电流反馈
 * ========================================================================== */

#include "ch32l103.h"

/* ---- 电机参数 (2026-09-28 起是"编译默认值", 可被 Flash 里的参数覆盖) ----
 * 为什么改成参数: 换颗电机 / 换个母线电源就要改代码重编太笨。现在极对数 / 堵转阈值 /
 * 母线电压存 Flash, 上电时 Flash 里的值覆盖这里的默认值
 * (见 pos_log 的记录格式 + motion_pv 的读取)。
 *   串口  #                看当前参数
 *         #<pp>            只设极对数
 *         #<pp>,<A>        设极对数 + 堵转阈值 (A)
 *         #0,<A>           只设堵转阈值
 *         #<pp>,<A>,<V>    三个一起设
 *         #0,0,<V>         只设母线电压 (换 36V 电源时用)
 *   CAN   0x116            同上的 CAN 版本: b0 极对数, b1..2 阈值 x0.01A,
 *                          b4..5 母线电压 x0.01V, b3 bit0=1 才是确认 (回 0x184)
 * 三条都**立刻写 Flash**。改极对数会把"已对齐"作废 (零点电角是用旧极对数算的),
 * 必须重跑对齐 ('A') —— 见 Motion_SetMotorCfg()。改母线电压不作废对齐。
 * 出厂默认值 HT4310: 用户规格表 2026-09-18 */
#define MOTOR_POLE_PAIRS        14          /* 编译默认: 14 极对 = 28 极 */

/* 参数合法范围 (越界直接拒, 不写 Flash) */
#define MOTOR_CFG_PP_MIN        1           /* 极对数下限 */
#define MOTOR_CFG_PP_MAX        64          /* 极对数上限 (设计值; 实际电机 1~50 就够) */
#define MOTOR_CFG_AMP_MIN       0.1f        /* 堵转阈值下限 A */
#define MOTOR_CFG_AMP_MAX       3.0f        /* 堵转阈值上限 A —— 采样链满量程就 ±3.3A
                                             * (INA240A2 增益 50 x 分流 10mR = 0.5 V/A,
                                             * 出处 交接文档_2026-09-19_CH32L103_FOC台架.md §2),
                                             * 再往上设也测不到 */
#define MOTOR_CFG_AMP_WARN      2.5f        /* 超过它就提示: 这颗 DRV8313 自己的峰值限流
                                             * 约 2.5A (出处 DRV8313 datasheet 首页),
                                             * 首页), 硬件会先跳 -> 固件阈值再高也没用 */

/* ---- PWM (TIM3, 中心对齐) ---- */
#define DRV_PWM_FREQ_HZ         20000u
#define DRV_PWM_ARR()           (SystemCoreClock / 2u / DRV_PWM_FREQ_HZ - 1u)

/* ---- CAN1 (PB8=RX / PB9=TX, 收发器 SN65HVD230) ----
 * 复用到 PB8/PB9 的依据: 数据手册 V2.1 表2-1-1 里 PB8 重映射功能写的是
 * "CAN1_RX_2", PB9 写的是 "CAN1_TX_2"; 注10 明确 "下划线后的数值 = AFIO 对应位
 * 的配置值", 即 CAN_RM[1:0] = 10b。
 * 波特率 1Mbps 的出处: 同板子的 CH32L103_USB2CAN_DM 工程 can_bsp.c 头注释
 * (PCLK1=48MHz, Prescaler=2 -> tq=24MHz, 1+BS1(15)+BS2(8)=24tq -> 1MHz),
 * 该工程此前已和 COM5 上的 USB-CAN 跑通过收发。
 * PCLK1=48MHz 的出处: User/system_ch32l103.c SetSysClockTo96_HSI() 第 709 行
 * RCC_PPRE1_DIV2 (HCLK 96MHz / 2)。 */
#define CAN_PRESCALER           2u
#define CAN_BS1_TQ              CAN_BS1_15tq
#define CAN_BS2_TQ              CAN_BS2_8tq

/* 发一帧的等待上限 (2026-09-29 新增)。
 * 为什么要有: 总线上没人给 ACK 时, 硬件的 TxMailbox 会一直停在 Pending, 旧代码用
 *   "空转 200000 圈" 兜底 —— 实测一次要 162 ms (最高一次 480 ms = 连堵 3 帧)。
 *   CAN_Bus_Task() 在主循环里, 每 10ms 想发一帧反馈, 于是主循环被吃满:
 *   闭环 FOC 从 ~2000 次/秒 掉到 3~7 次/秒 (2026-09-29 实测, 见命令表 铁律 #9)。
 * 1 Mbps 下一帧最多约 130us, 给 2ms 是十几倍余量; 真发不出去就 CAN_CancelTransmit 丢掉这一帧,
 *   宁可丢一帧, 也不许堵住控制环。 */
#define CAN_TX_TIMEOUT_US       2000u

/* ---- 软电流限幅的"拐点"(2026-10-06 加, 实现见 foc_pid.c: volt_limit_now) ----
 * 相电流到 motor_stall_amp 的这个比例之前**不限制**允许的相电压; 从 knee 到 stall 阈值
 * 之间线性收到 0V。目的: 这套是电压模式 FOC, 没有电流环 —— 低速反电势为 0 时环路一要
 * 满压, 电流就是 U/R 级 (2026-10-06 实测: 22V 母线 / L=12.70V 下 +700rpm 阶跃直接跳堵转)。
 * 0.75 是**设计值, 未实测**: 定得太低会误伤高速稳态 (13V 母线实测 440rpm 时稳态相电流
 * 1.8A, 占 3.0A 阈值的 60%)。
 * 2026-10-06 实测补记: knee=0.9 与 0.75 在 22V 下**顶速一样**
 * (700/1200rpm 命令都只到 ~490rpm), 也就是 0.9 换不来速度, 只多放电流
 * -> 保持 0.75 更保守。 */
#define MOTOR_ILIM_KNEE         0.75f

/* ---- 堵转保护 (阈值默认 2A, 2026-09-28 起可被 Flash 参数覆盖; 窗口/次数是设计值, 非实测) ---- */
#define MOTOR_STALL_AMP         2.0f        /* 编译默认值, 范围见 MOTOR_CFG_AMP_* */
#define MOTOR_STALL_WIN_MS      5u          /* 每个统计窗口 5ms */
#define MOTOR_STALL_WINS        3u          /* 连续 3 个窗口都超 -> 判堵转 (15ms) */

/* ---- 母线电压 / 限幅 (设计值, 非实测) ---- */
#define MOTOR_VOLTAGE_SUPPLY    24.0f       /* VM = 24V —— **编译默认值**, 2026-09-28 起
                                             * 可被 Flash 参数覆盖 (串口 '#0,0,<V>' / CAN 0x116 b4..5),
                                             * 换 36V 母线不用重编。
                                             * 2026-09-19 实测: 万用表量 VM 对 GND = 23.8V (台架电源 36V15A 调到 24V 档)。
                                             * 这个值只用于两处: (a) SVPWM 归一化 Uout = |U|/VM; (b) 上报的 VM。
                                             * 注意: 占空比乘的是**真实母线**, 真正决定最大相电压的是
                                             * `Uout <= 0.577` (线性调制上限), 即 Vbus/sqrt(3)。改这个常数不会
                                             * 提高转速上限, 只是让 L 的数值变成"真伏特"。 */
/* 母线电压参数的合法范围 (越界直接拒, 不写 Flash) */
#define MOTOR_CFG_VBUS_MIN      8.0f        /* 下限出处: DRV8313 数据手册 6.3 Recommended
                                             * Operating Conditions —— VM 8..60V
                                             * (DRV8313 datasheet 第 5 页) */
#define MOTOR_CFG_VBUS_MAX      36.0f       /* 上限出处: 用户给定的关节母线 24~36V。
                                             * DRV8313 自己允许到 60V (同上), 但本板是按 24V 母线
                                             * 设计的, 母线电容/降压支路/防反接都没核到 36V 以上,
                                             * 所以先卡 36V; 要更高先核硬件再放。 */
/* 上电默认相电压上限 (Uq/Ud 幅值), 0 = 用 vbus/sqrt(3)。
 * 2026-09-28 改: 原来固定 3.0V, 母线从 12V 换到 24V 它也不跟着变, 于是不改 'L'
 * 就一直是 3V 的出力 —— 实测台面 13V 时平衡关节只靠 0.03V 就能保持, 看着没事,
 * 可一旦要加速/抗扰/带载就不够 (3V / 6.7Ω ≈ 0.45A 相电流)。
 * 现在默认跟母线走: 13V->7.51V, 24V->13.86V, 36V->20.78V (见 Motion_ApplyBootVoltageLimit)。
 * 想强制小限幅(比如台面调试怕烧)就把这里填个正数, 运行期 'L<volts>' 仍可压小。 */
#define MOTOR_VOLTAGE_LIMIT     0.0f
#define MOTOR_VELOCITY_LIMIT    130.0f      /* rad/s 角度环输出的速度钳位 = 1241rpm
                                             * 2026-09-19 实测: 原来 20 (=191rpm) 就是转速天花板,
                                             * 跟电压/电流都无关 (CAN 的 vlim 给到 1200rpm 也只到 191rpm)。
                                             * 放开后 CAN 的 vlim 才有意义。 */
#define MOTOR_VELOCITY_LIMIT_MAX 150.0f     /* rad/s 运行时 0x115 能设到的上限 = 1432rpm */
#define MOTOR_VOLTAGE_ALIGN     2.5f        /* 对齐用电压 (设计值, 非实测)
                                             * 2026-09-19: 原来是 1.0V, 实测转子跟不上对齐扫场
                                             * (mid/end 差 0.4066 rad < 理论 2pi/14=0.4488 rad),
                                             * 导致极对数被估成 15。抬到 2.5V 让转子跟得住。 */
#define MOTOR_VEL_FILTER_ALPHA  0.9f        /* 速度低通系数 */
#define MOTOR_ANG_FF_GAIN       1.0f        /* 角度环的**轨迹速度前馈**增益 (2026-10-06 加)
                                             * 0 = 关 (退回老行为, 纯 P 跟轨迹)
                                             * 1 = 把 PV 轨迹自己的参考速度整个加到速度给定上
                                             * 只对角度环 + mode==PV_POS 生效; 手动 'P'/'V'/'T' 无影响。
                                             * 由来: 30 度阶跃实测 t90~145ms 但 t99~370ms, 尾巴是
                                             * 纯 P 的跟踪误差 (v/Pa), 不是轨迹慢。见
                                             * docs\工作报告_2026-10-06_FOC肉_角度环前馈.md */

/* ---- 母线电压 (VM) 采样 ----
 * 网表核对 2026-09-19 (work/l103_v2.net): +24VSRC 只连到 DRV8313 的 VM(4/11 脚)、
 * C13/C14/C18/C28/C30/C4/C5/C6、防反接 Q1、FB2(buck 输入)、R25(电源指示 LED) 和 TP19,
 * **没有任何分压电阻引到 MCU 的 ADC 脚** -> 这一版硬件量不到 VM。
 * 所以默认 VM_SENSE_ENABLE = 0: CS_VM_Volt() 返回配置值 voltage_power_supply,
 * 并且 VM_LOW 这条故障会标成"无采样"而不上报 (见 faults.c)。
 * 想真正量: 下版硬件从 +24VSRC 拉一级分压到 PA0 (焊盘6, 现在只接 TP17/TP24),
 * 再把这里改成 1、按实际电阻填 VM_DIVIDER_RATIO。 */
#define VM_SENSE_ENABLE     0
#define VM_ADC_CHANNEL      ADC_Channel_0
#define VM_DIVIDER_RATIO    11.0f       /* 分压比: 设计值, 硬件未做, 未实测 */
#define VM_LOW_THRESHOLD    20.0f       /* 用户给定: 母线低于 20V 报 VM_LOW */
#define VM_LOW_CLEAR        21.0f       /* 回差上限: 设计值, 未实测 */

/* ---- 回机械原点 ---- */
#define MOTION_HOME_VLIM_DEF  3.14159f  /* 默认速度上限 rad/s = 30rpm (设计值, 未实测) */

#endif
