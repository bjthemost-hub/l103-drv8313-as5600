#ifndef __CMD_H
#define __CMD_H

#include "ch32l103.h"

/* 命令接口: UART1 (PA9/PA10, 与 printf 共用) + 后续 USB CDC
 * 两边都调用同一个 Cmd_Exec(), 输出走 Cmd_SetOutHook() 指定的通道.
 */

extern float cmd_target;        /* 当前控制目标 (单位随模式) */

void Cmd_Init(void);            /* 使能 USART1 接收 + 中断 */
void Cmd_Poll(void);            /* 主循环里调用: 拆行 + 执行 */
void Cmd_Exec(const char *line);
float Cmd_ParseFloat(const char *s);  /* 轻量十进制解析, 取代 atof (省 Flash) */
void Cmd_SetOutHook(void (*fn)(const char *buf));
int  Cmd_AutoStatus(void);
int  Cmd_IsAligned(void);                   /* 有没有可用的对齐参数 */
void Cmd_MarkAligned(int ok);
void Cmd_Status(void);
void Cmd_CurrentSense(void);
void Cmd_SensorDiag(void);
int  Cmd_SensorPoll(void);

#endif
