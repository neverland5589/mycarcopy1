#include "ti_msp_dl_config.h"
#include "main.h"
#include "PID.h"
#include "Encoder.h"
#include "motor.h"
#include "Serial.h"
#include "GRAY.h"
#include "TRACKING.h"
#include "GrayCalibration.h"
#include <stdio.h>

PID_t speed_pid_A;
PID_t speed_pid_B;

volatile int status = 0;
volatile int task = 0;
volatile uint8_t task_confirmed = 0;
No_MCU_Sensor sensor;//传感器参数数组
unsigned short white[8] = {1587, 1785, 2299, 1875, 2946, 2583, 1856, 1386};
unsigned short black[8] = {103, 107, 112, 109, 293, 130, 110, 110};

unsigned short Anolog[8] = {0};
volatile unsigned char Digtal;

#define SPEED_KP 245.0f
#define SPEED_KI  12.0f

uint8_t oled_buffer[32];

volatile float pwm_A;
volatile float pwm_B;
static volatile uint8_t gray_scan_channel = 0;
static volatile uint8_t gray_scan_busy = 0;
static volatile uint32_t gray_frame_count = 0;
static volatile uint16_t gray_frame_snapshot[8] = {0};


float target_base_speed_A = 1.4f; 
float target_base_speed_B = 1.4f;

/* 循迹外环计算出的两个车轮目标速度。 */
volatile float target_speed_A = 0.0f;
volatile float target_speed_B = 0.0f;

/*
 * I2C屏幕每100ms刷新一次。
 * 速度PID仍然每20ms运行，屏幕刷新不会改变速度环周期。
 */
volatile uint8_t oled_update_request = 1;

/* 任务1状态：从A点出发，离开A点后再次检测到A点停车。 */
static volatile uint8_t task1_running = 0;
static volatile uint8_t task1_left_start = 0;
static volatile uint8_t task1_finished = 0;
static volatile unsigned long task1_start_time = 0;
static volatile unsigned long task1_elapsed_time = 0;

/* 任务2状态：从A点循迹行驶到1.5米处的B点停车。 */
static volatile uint8_t task2_running = 0;
static volatile uint8_t task2_finished = 0;
static volatile float task2_distance = 0.0f;

#define TASK2_TARGET_DISTANCE 1.75
/////123343
/*
 * 只检查中间六路灰度。
 * 中间六路至少有三路为0时，认为小车位于A点。
 */
static uint8_t Task1_IsAtA(unsigned char Digital)
{
    uint8_t black_count = 0;
    uint8_t i = 0;

    for (i = 1; i < 7; i++)
    {
        if (((Digital >> i) & 1) == 0)
        {
            black_count++;
        }
    }

    return black_count >= 3;
}

/* 立即停止任务1并清除速度环积分。 */
static void Task1_StopMotor(void)
{
    target_speed_A = 0.0f;
    target_speed_B = 0.0f;
    pwm_A = 0.0f;
    pwm_B = 0.0f;
    speed_pid_A.ErrorInt = 0.0f;
    speed_pid_B.ErrorInt = 0.0f;
    Motor_SetPWM(1, 0);
    Motor_SetPWM(2, 0);
}

/* KEY2确认任务1时，从零开始计时并等待小车先离开起点A。 */
static void Task1_Start(void)
{
    task1_start_time = tick_ms;
    task1_elapsed_time = 0;
    task1_left_start = 0;
    task1_finished = 0;
    task1_running = 1;
    speed_pid_A.ErrorInt = 0.0f;
    speed_pid_B.ErrorInt = 0.0f;
    oled_update_request = 1;
}

/* 每20毫秒执行一次任务1循迹和回到A点停车逻辑。 */
static void Task1_Control(void)
{
    uint8_t at_a = Task1_IsAtA(Digtal);

    if (!task1_running)
    {
        Task1_StopMotor();
        return;
    }

    if (!task1_left_start)
    {
        if (!at_a)
        {
            task1_left_start = 1;
        }
    }
    else if (at_a)
    {
        task1_elapsed_time = tick_ms - task1_start_time;
        task1_running = 0;
        task1_finished = 1;
        Task1_StopMotor();
        oled_update_request = 1;
        return;
    }

    task1_elapsed_time = tick_ms - task1_start_time;
    Tracking_Update(sensor.Normal_value, Digtal);
    pwm_A = PID_Positional_Update(&speed_pid_A, speed_A, target_speed_A);
    pwm_B = PID_Positional_Update(&speed_pid_B, speed_B, target_speed_B);
    Motor_SetPWM(1, (int8_t)pwm_A);
    Motor_SetPWM(2, (int8_t)pwm_B);
}

/* KEY2确认任务2时，将左右编码器累计路程清零。 */
static void Task2_Start(void)
{
    uint32_t interrupt_state = __get_PRIMASK();

    __disable_irq();
    encoderA_cnt = 0;
    encoderB_cnt = 0;
    total_pulse_A = 0;
    total_pulse_B = 0;
    distance_A = 0.0f;
    distance_B = 0.0f;
    if (interrupt_state == 0)
    {
        __enable_irq();
    }

    task2_distance = 0.0f;
    task2_finished = 0;
    task2_running = 1;
    speed_pid_A.ErrorInt = 0.0f;
    speed_pid_B.ErrorInt = 0.0f;
    oled_update_request = 1;
}

/* 每20毫秒执行一次任务2，平均里程达到1.5米后停车。 */
static void Task2_Control(void)
{
    if (!task2_running)
    {
        Task1_StopMotor();
        return;
    }

    task2_distance = (distance_A + distance_B) / 2.0f;

    if (task2_distance >= TASK2_TARGET_DISTANCE)
    {
        task2_running = 0;
        task2_finished = 1;
        Task1_StopMotor();
        oled_update_request = 1;
        return;
    }

    Tracking_Update(sensor.Normal_value, Digtal);
    pwm_A = PID_Positional_Update(&speed_pid_A, speed_A, target_speed_A);
    pwm_B = PID_Positional_Update(&speed_pid_B, speed_B, target_speed_B);
    Motor_SetPWM(1, (int8_t)pwm_A);
    Motor_SetPWM(2, (int8_t)pwm_B);
}

static void OLED_ShowCalibrationStatus(void)
{
    uint8_t calibration_step = GrayCalibration_GetStep();
    uint8_t calibration_sample_count = GrayCalibration_GetSampleCount();

    if (calibration_step == 0)
    {
        sprintf((char *)oled_buffer, "CAL:KEY2 ENTER  ");
        OLED_ShowString(0, 6, oled_buffer, 8);
        sprintf((char *)oled_buffer, "TASK:5          ");
        OLED_ShowString(0, 7, oled_buffer, 8);
    }
    else if (calibration_step == 1)
    {
        sprintf((char *)oled_buffer, "CAL:PUT WHITE   ");
        OLED_ShowString(0, 6, oled_buffer, 8);
        sprintf((char *)oled_buffer, "KEY2 TO START   ");
        OLED_ShowString(0, 7, oled_buffer, 8);
    }
    else if (calibration_step == 2)
    {
        sprintf((char *)oled_buffer, "CAL:WHITE %02d/50", calibration_sample_count);
        OLED_ShowString(0, 6, oled_buffer, 8);
        sprintf((char *)oled_buffer, "KEEP STILL      ");
        OLED_ShowString(0, 7, oled_buffer, 8);
    }
    else if (calibration_step == 3)
    {
        sprintf((char *)oled_buffer, "CAL:PUT BLACK   ");
        OLED_ShowString(0, 6, oled_buffer, 8);
        sprintf((char *)oled_buffer, "KEY2 TO START   ");
        OLED_ShowString(0, 7, oled_buffer, 8);
    }
    else if (calibration_step == 4)
    {
        sprintf((char *)oled_buffer, "CAL:BLACK %02d/50", calibration_sample_count);
        OLED_ShowString(0, 6, oled_buffer, 8);
        sprintf((char *)oled_buffer, "KEEP STILL      ");
        OLED_ShowString(0, 7, oled_buffer, 8);
    }
    else if (calibration_step == 5)
    {
        sprintf((char *)oled_buffer, "CAL:SAVED       ");
        OLED_ShowString(0, 6, oled_buffer, 8);
        sprintf((char *)oled_buffer, "SELECT:%d KEY2   ", status);
        OLED_ShowString(0, 7, oled_buffer, 8);
    }
    else
    {
        sprintf((char *)oled_buffer, "CAL:ERROR       ");
        OLED_ShowString(0, 6, oled_buffer, 8);
        sprintf((char *)oled_buffer, "CHECK WHITE/BLK ");
        OLED_ShowString(0, 7, oled_buffer, 8);
    }
}

static void OLED_ShowControlStatus(void)
{
    sprintf(
        (char *)oled_buffer,
        "GRAY:%u%u%u%u%u%u%u%u        ",
        (Digtal >> 0) & 1U,
        (Digtal >> 1) & 1U,
        (Digtal >> 2) & 1U,
        (Digtal >> 3) & 1U,
        (Digtal >> 4) & 1U,
        (Digtal >> 5) & 1U,
        (Digtal >> 6) & 1U,
        (Digtal >> 7) & 1U);
    OLED_ShowString(0, 0, oled_buffer, 8);

    sprintf((char *)oled_buffer, "A TAR:%-12.3f", target_speed_A);
    OLED_ShowString(0, 1, oled_buffer, 8);

    sprintf((char *)oled_buffer, "A ACT:%-12.3f", speed_A);
    OLED_ShowString(0, 2, oled_buffer, 8);

    sprintf((char *)oled_buffer, "B TAR:%-12.3f", target_speed_B);
    OLED_ShowString(0, 3, oled_buffer, 8);

    sprintf((char *)oled_buffer, "B ACT:%-12.3f", speed_B);
    OLED_ShowString(0, 4, oled_buffer, 8);

    sprintf((char *)oled_buffer, "LINE :%-12.3f", line_error);
    OLED_ShowString(0, 5, oled_buffer, 8);

    if (task == 1)
    {
        sprintf((char *)oled_buffer, "TIME:%lu.%03lus   ", task1_elapsed_time / 1000, task1_elapsed_time % 1000);
        OLED_ShowString(0, 6, oled_buffer, 8);
        if (task1_finished)
        {
            sprintf((char *)oled_buffer, "TASK1:STOP      ");
        }
        else
        {
            sprintf((char *)oled_buffer, "TASK1:RUN       ");
        }
        OLED_ShowString(0, 7, oled_buffer, 8);
    }
    else if (task == 2)
    {
        sprintf((char *)oled_buffer, "DIST:%-10.3f", task2_distance);
        OLED_ShowString(0, 6, oled_buffer, 8);
        if (task2_finished)
        {
            sprintf((char *)oled_buffer, "TASK2:STOP      ");
        }
        else
        {
            sprintf((char *)oled_buffer, "TASK2:RUN       ");
        }
        OLED_ShowString(0, 7, oled_buffer, 8);
    }
    else if (task == 5)
    {
        OLED_ShowCalibrationStatus();
    }
    else
    {
        sprintf((char *)oled_buffer, "status:  %-12d", status);
        OLED_ShowString(0, 6, oled_buffer, 8);
        sprintf((char *)oled_buffer, "task:    %-12d", task);
        OLED_ShowString(0, 7, oled_buffer, 8);
    }
}

static void Gray_SelectChannel(uint8_t channel)
{
    Switch_Address_0(!(channel & 0x01U));
    Switch_Address_1(!(channel & 0x02U));
    Switch_Address_2(!(channel & 0x04U));
}

static void Gray_StartScan(void)
{
    if (gray_scan_busy)
    {
        return;
    }

    gray_scan_channel = 0;
    gray_scan_busy = 1;
    Gray_SelectChannel(gray_scan_channel);
    DL_Common_delayCycles(CPUCLK_FREQ / 100000);
    DL_DMA_setSrcAddr(DMA, DMA_GRAY_CHAN_ID, (uint32_t)&ADC0->ULLMEM.MEMRES[0]);
    DL_DMA_setDestAddr(DMA, DMA_GRAY_CHAN_ID, (uint32_t)&ADC_VALUE[0]);
    DL_DMA_setTransferSize(DMA, DMA_GRAY_CHAN_ID, 40);
    DL_DMA_enableChannel(DMA, DMA_GRAY_CHAN_ID);
}

int main(void)
{
    SYSCFG_DL_init();
    SysTick_Init();
    // sudu0.Kp=1;
    // MPU6050_Init();
    OLED_Init();
    // Ultrasonic_Init();
    BNO08X_Init();

    GrayCalibration_Load(white, black);
    No_MCU_Ganv_Sensor_Init(&sensor, white, black);

    /* 灰度使用DMA1后台采集，BNO08X继续使用DMA0。 */
    DL_DMA_enableInterrupt(DMA, DL_DMA_INTERRUPT_CHANNEL1);
    NVIC_ClearPendingIRQ(DMA_INT_IRQn);
    NVIC_EnableIRQ(DMA_INT_IRQn);
    DL_ADC12_startConversion(ADC12_0_INST);
    Gray_StartScan();


    PID_Init(&speed_pid_A, SPEED_KP, SPEED_KI, 0.0f, 100.0f, -100.0f);
    PID_Init(&speed_pid_B, SPEED_KP, SPEED_KI, 0.0f, 100.0f, -100.0f);
    
    
    Tracking_Init();

    /* Don't remove this! */
    Interrupt_Init();
    // motor_init();
    DL_Timer_startCounter(PWMAB_INST);
    //DL_UART_Main_enableInterrupt(UART_DEBUG_INST, DL_UART_MAIN_INTERRUPT_RX);
    NVIC_ClearPendingIRQ(UART_DEBUG_INST_INT_IRQN);
    NVIC_EnableIRQ(UART_DEBUG_INST_INT_IRQN);

    NVIC_ClearPendingIRQ(GPIOA_INT_IRQn);
    NVIC_EnableIRQ(GPIOA_INT_IRQn);

    NVIC_ClearPendingIRQ(GPIOB_INT_IRQn);
    NVIC_EnableIRQ(GPIOB_INT_IRQn);
    DL_Timer_startCounter(TIMER_0_INST);
    //清除定时器中断标志
    NVIC_ClearPendingIRQ(TIMER_0_INST_INT_IRQN);
    NVIC_EnableIRQ(TIMER_0_INST_INT_IRQN );
    // Motor_SetPWM(1,50);
    float temp_value = 0.0f;

    while (1)
    {
        if (task_confirmed)
        {
            task_confirmed = 0;
            if (task == 1)
            {
                GrayCalibration_Reset();
                task2_running = 0;
                Task1_Start();
            }
            else if (task == 2)
            {
                GrayCalibration_Reset();
                task1_running = 0;
                Task2_Start();
            }
            else if (task == 5)
            {
                task1_running = 0;
                task2_running = 0;
                GrayCalibration_Confirm(gray_frame_count);
            }
            else
            {
                GrayCalibration_Reset();
                task1_running = 0;
                task2_running = 0;
                Task1_StopMotor();
            }
        }

        if (task == 5)
        {
            if (GrayCalibration_Process(&gray_frame_count, gray_frame_snapshot, white, black))
            {
                uint32_t interrupt_state = __get_PRIMASK();
                __disable_irq();
                No_MCU_Ganv_Sensor_Init(&sensor, white, black);
                if (interrupt_state == 0)
                {
                    __enable_irq();
                }
            }
        }

        if (oled_update_request)
        {
            oled_update_request = 0;
            OLED_ShowControlStatus();
        }

        if(Serial_GetRxFlag())
        {
            if(sscanf(Serial_RxString, "P:%f", &temp_value) == 1)
            {
                speed_pid_A.Kp = temp_value;
                speed_pid_B.Kp = temp_value;
            }
            else if(sscanf(Serial_RxString, "I:%f", &temp_value) == 1)
            {
                speed_pid_A.Ki = temp_value;
                speed_pid_B.Ki = temp_value;
                speed_pid_A.ErrorInt = 0.0f;
                speed_pid_B.ErrorInt = 0.0f;
            }
            else if(sscanf(Serial_RxString, "T:%f", &temp_value) == 1)
            {
                target_speed_A = temp_value;
                target_speed_B = temp_value;
                speed_pid_A.ErrorInt = 0.0f;
                speed_pid_B.ErrorInt = 0.0f;
            }
            else if(sscanf(Serial_RxString, "LP:%f", &temp_value) == 1)
            {
                Tracking_SetKp(temp_value);
            }
            else if(sscanf(Serial_RxString, "LD:%f", &temp_value) == 1)
            {
                Tracking_SetKd(temp_value);
            }
            else if(sscanf(Serial_RxString, "V:%f", &temp_value) == 1)
            {
                Tracking_SetBaseSpeed(temp_value);
            }
            else if(Serial_RxString[0] == 'G' &&
                    Serial_RxString[1] == '\0')
            {
                if (Get_Anolog_Value(&sensor, Anolog))
                {
                    Serial_Printf(
                        "GRAY:%d,%d,%d,%d,%d,%d,%d,%d\r\n",
                        Anolog[0],
                        Anolog[1],
                        Anolog[2],
                        Anolog[3],
                        Anolog[4],
                        Anolog[5],
                        Anolog[6],
                        Anolog[7]);
                }
            }
        }

     }
}
void TIMER_0_INST_IRQHandler()
{
    static uint8_t oled_update_count = 0;

    switch (DL_Timer_getPendingInterrupt(TIMER_0_INST))
    {
        case DL_TIMER_IIDX_ZERO:
           {           
            
            Encoder_Update();
            switch (task)
            {
                case 1:
                    Task1_Control();
                break;
                case 2:
                    Task2_Control();
                break;
                case 5:
                    target_speed_A = 0.0f;
                    target_speed_B = 0.0f;
                    pwm_A = 0.0f;
                    pwm_B = 0.0f;
                    speed_pid_A.ErrorInt = 0.0f;
                    speed_pid_B.ErrorInt = 0.0f;
                    Motor_SetPWM(1, 0);
                    Motor_SetPWM(2, 0);
                break;
            }
            // Tracking_Update(sensor.Normal_value, Digtal);

           

            Gray_StartScan();
            oled_update_count++;
            if (oled_update_count >= 5)
            {
                oled_update_count = 0;
                oled_update_request = 1;
            }

            break;
           }
        default:
        break;
    }
}

void DMA_IRQHandler(void)
{
    uint32_t adc_sum = 0;
    uint16_t adc_average = 0;
    uint8_t i = 0;

    if (DL_DMA_getPendingInterrupt(DMA) != DL_DMA_EVENT_IIDX_DMACH1)
    {
        return;
    }

    DL_DMA_disableChannel(DMA, DMA_GRAY_CHAN_ID);

    for (i = 0; i < 40U; i++)
    {
        adc_sum += ADC_VALUE[i];
    }

    adc_average = (uint16_t)(adc_sum / 40U);

    if (!Direction)
    {
        sensor.Analog_value[gray_scan_channel] = adc_average;
    }
    else
    {
        sensor.Analog_value[7U - gray_scan_channel] = adc_average;
    }

    gray_scan_channel++;

    if (gray_scan_channel < 8U)
    {
        Gray_SelectChannel(gray_scan_channel);
        DL_Common_delayCycles(CPUCLK_FREQ / 100000);
        DL_DMA_setSrcAddr(DMA, DMA_GRAY_CHAN_ID, (uint32_t)&ADC0->ULLMEM.MEMRES[0]);
        DL_DMA_setDestAddr(DMA, DMA_GRAY_CHAN_ID, (uint32_t)&ADC_VALUE[0]);
        DL_DMA_setTransferSize(DMA, DMA_GRAY_CHAN_ID, 40);
        DL_DMA_enableChannel(DMA, DMA_GRAY_CHAN_ID);
    }
    else
    {
        No_Mcu_Ganv_Sensor_Process(&sensor);
        Digtal = Get_Digtal_For_User(&sensor);
        for (i = 0; i < 8; i++)
        {
            gray_frame_snapshot[i] = sensor.Analog_value[i];
        }
        gray_frame_count++;
        gray_scan_busy = 0;
    }
}
