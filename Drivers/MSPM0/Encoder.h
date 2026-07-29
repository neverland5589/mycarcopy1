#ifndef ENCODER_H
#define ENCODER_H

#include "ti_msp_dl_config.h"

#define PI                  3.1415926f
#define WHEEL_DIAMETER_M    0.068f
#define ENCODER_RESOLUTION  11.0f
#define GEAR_RATIO          30.0f
#define MULTIPLE            2.0f

#define PULSE_PER_TURN \
    (ENCODER_RESOLUTION * GEAR_RATIO * MULTIPLE)

extern volatile int32_t encoderA_cnt;
extern volatile int32_t encoderB_cnt;
extern volatile int32_t total_pulse_A;
extern volatile int32_t total_pulse_B;

extern volatile float speed_A;
extern volatile float speed_B;
extern volatile float distance_A;
extern volatile float distance_B;

void Encoder_Update(void);
uint8_t get_key_state(uint32_t key);

#endif
