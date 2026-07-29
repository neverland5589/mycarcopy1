#include "TRACKING.h"
#include "PID.h"


#define LINE_KP             0.06f
#define LINE_KI             0.0f
#define LINE_KD             0.02f

/* 外环最多给左右车轮增加或减少0.18m/s。 */
#define LINE_OUTPUT_LIMIT   0.9f

PID_t line_pid;


const float line_weight[8] = {
    -7.0f, -5.0f, -3.0f, -1.0f,
     1.0f,  3.0f,  5.0f,  7.0f
};

/* 调试时可以直接在表达式窗口观察这三个变量。 */
volatile float line_error = 0.0f;
volatile float turn_speed = 0.0f;
volatile unsigned char line_found = 0;

void Calculate_Line_Error(const unsigned short *Normal_value)
{
    float weighted_sum = 0.0f;
    float strength_sum = 0.0f;
    unsigned char i;

    for (i = 0; i < 8; i++)
    {
        float black_strength = 4096.0f - Normal_value[i];

        if (black_strength < 0.0f)
            black_strength = 0.0f;

        weighted_sum += line_weight[i] * black_strength;
        strength_sum += black_strength;
    }

    if (strength_sum > 0.0f)
        line_error = weighted_sum / strength_sum;
}

void Tracking_Init(void)
{
    PID_Init(&line_pid, LINE_KP, LINE_KI, LINE_KD, LINE_OUTPUT_LIMIT, -LINE_OUTPUT_LIMIT);

    /*
     * 上电初始化阶段先按基础速度直行。
     * 之后每次Tracking_Update()都会根据循迹偏差重新计算目标速度。
     */
    target_speed_A = target_base_speed_A;
    target_speed_B = target_base_speed_B;

    line_error = 0.0f;
    turn_speed = 0.0f;
    line_found = 0;
}

void Tracking_Update(const unsigned short *Normal_value, unsigned char Digital)
{
    /*
     * 原厂Digtal每一位为1表示白底，为0表示黑线。
     * 0xFF表示8路全白，也就是已经丢线。
     */
    if (Digital == 0xFF)
    {
        line_found = 0;
    }
    else
    {
        line_found = 1;
        Calculate_Line_Error(Normal_value);
    }

    /*
     * 第二步：外环方向PID。
     * 实际值是line_error，目标值是0，也就是让黑线回到传感器中央。
     */
    turn_speed = PID_Positional_Update(&line_pid, line_error, 0.0f);

    /*
     * 第三步：固定基础速度加减转向量，得到左右轮目标速度。
     * A为左轮，B为右轮。
     */
    float turn_speed_abs = 0.0f;
    float corner_base_speed = 0.0f;

    turn_speed_abs = turn_speed >= 0.0f ? turn_speed : -turn_speed;
    corner_base_speed = target_base_speed_A - turn_speed_abs;

    if (corner_base_speed < turn_speed_abs)
    {
        corner_base_speed = turn_speed_abs;
    }

    target_speed_A = corner_base_speed - turn_speed;
    target_speed_B = corner_base_speed + turn_speed;
}

void Tracking_SetKp(float Kp)
{
    line_pid.Kp = Kp;
}

void Tracking_SetKd(float Kd)
{
    line_pid.Kd = Kd;
}

void Tracking_SetBaseSpeed(float speed)
{
    if (speed < 0.0f)
        speed = 0.0f;

    target_base_speed_A = speed;
    target_base_speed_B = speed;
}

float Tracking_GetKp(void)
{
    return line_pid.Kp;
}

float Tracking_GetKd(void)
{
    return line_pid.Kd;
}
