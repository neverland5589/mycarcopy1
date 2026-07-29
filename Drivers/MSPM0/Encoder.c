#include "Encoder.h"
#include "clock.h"

extern volatile unsigned long tick_ms;
static unsigned long key1_last_time = 0;
static unsigned long key2_last_time = 0;

extern volatile int task;
extern volatile uint8_t task_confirmed;
extern volatile int status;
uint8_t get_key_state(uint32_t key) {
    uint32_t high_bits = DL_GPIO_readPins(KEY_PORT, key); //0x00000040 0b01000000 PB6 0~31
    if((high_bits & key) == 0) return 1;
    else return 0;
}

/* 20ms内由编码器中断累加的脉冲数。 */
volatile int Get_Encoder_countA = 0;
volatile int Get_Encoder_countB = 0;

/* 本周期脉冲数和上电后的总脉冲数。 */
volatile int32_t encoderA_cnt = 0;
volatile int32_t encoderB_cnt = 0;
volatile int32_t total_pulse_A = 0;
volatile int32_t total_pulse_B = 0;

/* 两个车轮的速度和累计路程。 */
volatile float speed_A = 0.0f;
volatile float speed_B = 0.0f;
volatile float distance_A = 0.0f;
volatile float distance_B = 0.0f;

/* 每20ms调用一次，计算两路编码器速度。 */
void Encoder_Update(void)
{
    encoderA_cnt = -Get_Encoder_countA;
    encoderB_cnt = Get_Encoder_countB;

    Get_Encoder_countA = 0;
    Get_Encoder_countB = 0;

    total_pulse_A += encoderA_cnt;
    total_pulse_B += encoderB_cnt;

    distance_A =total_pulse_A * (PI * WHEEL_DIAMETER_M / PULSE_PER_TURN);
    distance_B =total_pulse_B * (PI * WHEEL_DIAMETER_M / PULSE_PER_TURN);

    speed_A =encoderA_cnt *(PI * WHEEL_DIAMETER_M / PULSE_PER_TURN) *50.0f;
    speed_B =encoderB_cnt *(PI * WHEEL_DIAMETER_M / PULSE_PER_TURN) *50.0f;
}

/* 两路AB相编码器的GPIO中断。 */
void GROUP1_IRQHandler(void)
{
    uint32_t encoderA_interrupt;
    uint32_t encoderB_interrupt;

    encoderA_interrupt = DL_GPIO_getEnabledInterruptStatus(ENCODERA_PORT,ENCODERA_E1A_PIN | ENCODERA_E1B_PIN);

    encoderB_interrupt = DL_GPIO_getEnabledInterruptStatus(ENCODERB_PORT,ENCODERB_E2A_PIN | ENCODERB_E2B_PIN);
    switch (DL_GPIO_getPendingInterrupt(GPIOA)) 
    {
        case KEY_KEY1_IIDX:
            if ((tick_ms - key1_last_time) >= 20)
            {   
                key1_last_time = tick_ms;
                status = status % 5 + 1;
            }
            
             
            break;
        case KEY_KEY2_IIDX:
            if ((tick_ms - key2_last_time) >= 20)
            {
                key2_last_time = tick_ms;
                task = status;
                task_confirmed = 1;                
            }
            break;
        default:
        break;
    }

    if ((encoderA_interrupt & ENCODERA_E1A_PIN) ==ENCODERA_E1A_PIN)
    {
        if (!DL_GPIO_readPins(ENCODERA_PORT, ENCODERA_E1B_PIN))
            Get_Encoder_countA--;
        else
            Get_Encoder_countA++;

        DL_GPIO_clearInterruptStatus(
            ENCODERA_PORT,
            ENCODERA_E1A_PIN);
    }
    else if ((encoderA_interrupt & ENCODERA_E1B_PIN) ==ENCODERA_E1B_PIN)
    {
        if (!DL_GPIO_readPins(ENCODERA_PORT, ENCODERA_E1A_PIN))
            Get_Encoder_countA++;
        else
            Get_Encoder_countA--;

        DL_GPIO_clearInterruptStatus(
            ENCODERA_PORT,
            ENCODERA_E1B_PIN);
    }

    if ((encoderB_interrupt & ENCODERB_E2A_PIN) ==ENCODERB_E2A_PIN)
    {
        if (!DL_GPIO_readPins(ENCODERB_PORT, ENCODERB_E2B_PIN))
            Get_Encoder_countB--;
        else
            Get_Encoder_countB++;

        DL_GPIO_clearInterruptStatus(
            ENCODERB_PORT,
            ENCODERB_E2A_PIN);
    }
    else if ((encoderB_interrupt & ENCODERB_E2B_PIN) ==ENCODERB_E2B_PIN)
    {
        if (!DL_GPIO_readPins(ENCODERB_PORT, ENCODERB_E2A_PIN))
            Get_Encoder_countB++;
        else
            Get_Encoder_countB--;

        DL_GPIO_clearInterruptStatus(
            ENCODERB_PORT,
            ENCODERB_E2B_PIN);
    }
}
