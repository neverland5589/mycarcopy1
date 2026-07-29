#ifndef MOTOR_H
#define MOTOR_H

#include "ti_msp_dl_config.h"

void PWM_SetCompareA(uint16_t compare);
void PWM_SetCompareB(uint16_t compare);
void motor_init(void);
void Motor_SetPWM(uint8_t motor, int8_t pwm);

#endif
