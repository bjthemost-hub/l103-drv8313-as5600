#ifndef __FOC_MOTOR_H
#define __FOC_MOTOR_H

#include "foc_utils.h"

typedef enum { Type_torque, Type_velocity, Type_angle, Type_velocity_openloop, Type_angle_openloop } MotionControlType;
typedef enum { Type_voltage, Type_dc_current, Type_foc_current } TorqueControlType;
typedef enum { CW = 1, CCW = -1, UNKNOWN = 0 } Direction;

/* ---- 可调参数 ---- */
extern float voltage_power_supply;
extern float voltage_limit;
extern float voltage_sensor_align;
extern float velocity_limit;
extern int   pole_pairs;
extern int   pole_pairs_check;      /* 对齐时估出来的极对数 (仅诊断, 不用它改配置值) */
extern float motor_stall_amp;       /* 堵转/过流阈值 A (默认 MOTOR_STALL_AMP,
                                     * 上电会被 Flash 里的参数覆盖, 见 motion_pv) */

extern TorqueControlType torque_controller;
extern MotionControlType controller;

/* ---- 状态 ---- */
extern long  sensor_direction;
extern float shaft_angle;
extern float electrical_angle;
extern float shaft_velocity;
extern float current_sp;
extern float shaft_velocity_sp;
extern float shaft_angle_sp;
extern DQVoltage_s voltage;
extern DQCurrent_s current;
extern float sensor_offset;
extern float zero_electric_angle;

float ShaftAngle(void);
float ShaftVelocity(void);
float ElectricalAngle(void);

void  Motor_SetVelocityLimit(float w);   /* 角度环速度钳位 (rad/s), 见 board.h */
float Motor_GetVelocityLimit(void);
void  Motor_Init(void);
int   Motor_InitFOC(void);          /* 对齐 + 取初值; 没接电机/编码器时会失败并停驱动 */

/* ---- 10kHz 固定节拍驱动: 把开环相位推进从主循环里拆出来 ----
 * 起因(实测 2026-09-19 晚): 主循环 1Hz 那一坨 printf/I2C/ADC 会卡住循环几十 ms,
 * 期间开环相位不前进 -> 每次打日志电机顿一下, 并且每秒丢一个极对(25.8deg/s)。
 * 搬进定时器中断后, 主循环再慢也不影响相位。 */
void     Motor_DriveTimerInit(void);            /* TIM1 更新中断 @10kHz */
void     Motor_DriveTimerEnable(uint8_t on);    /* 1 = 中断推进相位; 0 = 主循环自己驱动 */
void     Motor_SetTarget(float t);              /* 给中断用的目标值(锁存) */
void     Motor_DriveSuspend(uint8_t on);        /* 1 = 谁都别写 PWM (对齐/标定期间) */
uint8_t  Motor_DriveSuspended(void);
uint32_t Motor_DriveGapMax(void);               /* 诊断: 中断两次驱动之间的最大间隔 us */
uint32_t Motor_DriveIsrUsMax(void);             /* 诊断: 一次中断体自己最长耗时 us */
void     Motor_DriveGapReset(void);
void  LoopFOC(void);
void  Move(float new_target);
void  SetPhaseVoltage(float Uq, float Ud, float angle_el);

#endif
