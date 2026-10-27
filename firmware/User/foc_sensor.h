#ifndef __FOC_SENSOR_H
#define __FOC_SENSOR_H

#include "ch32l103.h"

/* AS5600 (U5, SOIC-8) 走 I2C1: PB6 = SCL / PB7 = SDA (板上 R3/R4 上拉)
 * 手册 v1-06 第18页寄存器表:
 *   0x0B STATUS  bit5=MD 检测到磁铁 / bit4=ML 太弱 / bit3=MH 太强
 *   0x0C-0x0D RAW ANGLE (12bit)
 *   0x1A AGC (3.3V 供电时范围 0~128)
 *   0x1B-0x1C MAGNITUDE (12bit)
 */
#define AS5600_ADDR             0x36
#define AS5600_REG_STATUS       0x0B
#define AS5600_REG_RAW_ANGLE_H  0x0C
#define AS5600_REG_AGC          0x1A
#define AS5600_REG_MAGNITUDE_H  0x1B
#define AS5600_CPR              4096        /* 12 bit */

extern long  sensor_cpr;
extern float full_rotation_offset;
extern long  angle_data_prev;
extern float angle_prev;

void  Sensor_Init(void);
int   Sensor_Ping(uint8_t addr7);                        /* 0 = 从机 ACK */
int   Sensor_ReadRegs(uint8_t reg, uint8_t *buf, uint8_t len);
int   Sensor_ReadRaw(uint16_t *raw);
int   Sensor_ReadStatus(uint8_t *st);                    /* 0x0B */
int   Sensor_ReadAGC(uint8_t *agc);                      /* 0x1A */
int   Sensor_ReadMagnitude(uint16_t *mag);               /* 0x1B/0x1C */
float Sensor_ReadSingle(void);
float Sensor_GetAngle(void);
float Sensor_GetVelocity(void);
uint32_t Sensor_ErrorCount(void);
uint16_t Sensor_RecoverCount(void);                        /* 自动总线恢复做了几次 */
void Sensor_GetHwState(uint8_t *pe, uint16_t *star1, uint8_t *cfg6, uint8_t *cfg7, uint8_t *clk);

void  Sensor_GetDiag(uint8_t *step, uint16_t *star1, uint16_t *star2);
void  Sensor_BusLevels(uint8_t *scl, uint8_t *sda);
void  Sensor_BusRecover(void);
void  Sensor_ForceRecover(void);                           /* 手动强制总线恢复 */

#endif
