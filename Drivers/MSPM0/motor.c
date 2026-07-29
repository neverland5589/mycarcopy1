#include "ti_msp_dl_config.h"
#include "motor.h"

/*
 * 两路电机硬件对应关系：
 * 电机1：PWM PA3（TIMA0_C2），方向 PA31
 * 电机2：PWM PA4（TIMA0_C3），方向 PA28
 */

void PWM_SetCompareA(uint16_t compare)
{
    if (compare > 100)
        compare = 100;

    DL_Timer_setCaptureCompareValue(PWMAB_INST,compare,GPIO_PWMAB_C3_IDX);
}

void PWM_SetCompareB(uint16_t compare)
{
    if (compare > 100)
        compare = 100;

    DL_Timer_setCaptureCompareValue(PWMAB_INST,compare,GPIO_PWMAB_C2_IDX);
}

void motor_init(void)
{
    DL_GPIO_clearPins(DIR_PORT, DIR_DIR1_PIN);
    DL_GPIO_clearPins(DIR_PORT, DIR_DIR2_PIN);

    PWM_SetCompareA(0);
    PWM_SetCompareB(0);

    DL_Timer_startCounter(PWMAB_INST);
}

void Motor_SetPWM(uint8_t motor, int8_t pwm)
{
    if (motor == 1)
    {
        if (pwm >= 0)
        {
            DL_GPIO_setPins(DIR_PORT, DIR_DIR1_PIN);
            
            PWM_SetCompareA(pwm);
        }
        else
        {
            DL_GPIO_clearPins(DIR_PORT, DIR_DIR1_PIN);
            PWM_SetCompareA(-pwm);
        }
    }
    else if (motor == 2)
    {
        if (pwm >= 0)
        {
            DL_GPIO_clearPins(DIR_PORT, DIR_DIR2_PIN);
            PWM_SetCompareB(pwm);
        }
        else
        {
            DL_GPIO_setPins(DIR_PORT, DIR_DIR2_PIN);
           
            PWM_SetCompareB(-pwm);
        }
    }
}
