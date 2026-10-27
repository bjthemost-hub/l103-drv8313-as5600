#ifndef WAVE_H
#define WAVE_H

#include <stdint.h>

/* =============================================================================
 * 相电流波形流 (2026-10-06 新增) —— 往 UART1 送实时曲线, **绝不影响闭环 FOC**
 *
 * 硬前提 (家人 2026-10-06 定): 电机正常运转优先, 一切给它让路。
 * 所以这里只做一件事: 把定长小帧塞进 debug.c 那个 2048 B 环形缓冲, 由 TXE 中断
 * 在后台慢慢发。**缓冲满了就丢帧 + 计数, 主循环一次都不等。**
 *   - 主循环代价: 每帧一次 9 B 拷贝 + 3 次浮点转整, 实测约 1~2 us;
 *     1 kHz 下 = 2 ms/s, 约主循环预算的 0.2%。
 *   - 发送中断优先级 2 (低于电机中断 1), 抢不到电机的中断。
 *
 * 帧格式 (小端):
 *   off 0  0xAA 0x55            帧头
 *   off 2  type (uint8)         0x01 = 三相相电流
 *   off 3  iU  int16, 0.001 A/LSB
 *   off 5  iV  int16
 *   off 7  iW  int16
 *   共 9 字节。
 *
 *   type 0x02 预留给下一版 PCB 的母线电流/电压 (采样通道还没焊, 现在发不出来)。
 *
 * 丢帧识别: 帧内三相满足 iU+iV+iW ~= 0, 上位机拿它校验; 不满足就重新找帧头。
 * ========================================================================== */

#define WAVE_T_IMON     0x01u       /* 三相相电流 */

void     Wave_Init(void);
void     Wave_SetOn(int on);
int      Wave_IsOn(void);
void     Wave_SetDiv(uint16_t div);     /* 每 div 个采样点发一帧 (1 = 全发) */
uint16_t Wave_GetDiv(void);
uint32_t Wave_Samples(void);            /* CS_Task 累计采样点数 */
uint32_t Wave_Frames(void);             /* 真正塞进缓冲的帧数 */

/* CS_Task() 每次采完样调一次; 波形关着时立刻返回。 */
void     Wave_Sample(float iu, float iv, float iw);

void     Wave_PrintInfo(void);          /* 'SW' 打印 */
int      Wave_SetBaud(uint32_t baud);   /* 改 UART1 波特率; 成功返回新波特率, 失败 0 */

#endif /* WAVE_H */
