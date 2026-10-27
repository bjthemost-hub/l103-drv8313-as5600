#ifndef __FOC_PID_H
#define __FOC_PID_H

void  PID_Init(void);
float PID_Velocity(float error);
float PID_Angle(float error);
float LPF_Velocity(float x);

#endif
