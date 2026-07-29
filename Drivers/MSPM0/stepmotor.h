#ifndef STEPMOTOR_H
#define STEPMOTOR_H

#include "ti_msp_dl_config.h"
#include <stdint.h>

/* PD42S1默认从机地址。 */
#define STEPMOTOR_DEFAULT_ADDRESS 1

/* PD42S1规定电机转一圈对应51200个位置单位。 */
#define STEPMOTOR_POSITION_PER_TURN 51200

/* PWM周期为20000微秒，也就是20毫秒、50Hz。 */
#define STEPMOTOR_PWM_PERIOD 20000

/* 默认脉宽范围为500到2500微秒，中间位置为1500微秒。 */
#define STEPMOTOR_DEFAULT_MAX_PULSE_WIDTH 2500
#define STEPMOTOR_DEFAULT_MIN_PULSE_WIDTH 500
#define STEPMOTOR_DEFAULT_CENTER_PULSE_WIDTH 1500

/* 1422个位置单位约等于10度，因此默认控制范围为负10度到正10度。 */
#define STEPMOTOR_DEFAULT_MAX_POSITION 1422
#define STEPMOTOR_DEFAULT_MIN_POSITION -1422

/* 初始化驱动器通信、发送F5配置命令，并让PWM回到中间位置。 */
void StepMotor_Init(void);

/* 修改PD42S1从机地址。 */
void StepMotor_SetAddress(uint8_t Address);

/* 发送手册中的F5命令，配置脉宽与电机位置的对应关系。 */
void StepMotor_ConfigurePulseWidthPosition(uint16_t MaxPulseWidth, uint16_t MinPulseWidth, int32_t MaxPosition, int32_t MinPosition);

/* 直接设置PA10输出的高电平脉宽，单位为微秒。 */
void StepMotor_SetPulseWidth(uint16_t PulseWidth);

/* 使用PD42S1的位置单位设置目标位置，51200为一圈。 */
void StepMotor_SetTargetPosition(int32_t TargetPosition);

/* 使用角度设置目标位置，适合控制程序直接调用。 */
void StepMotor_SetAngle(double Angle);

#endif
