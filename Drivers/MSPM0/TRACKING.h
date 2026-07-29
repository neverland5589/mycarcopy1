#ifndef TRACKING_H
#define TRACKING_H

/*
 * 初始化循迹外环PID，并将左右轮目标速度设为各自的基础速度。
 * 在main()中完成灰度传感器初始化后调用一次。
 */
void Tracking_Init(void);

/*
 * 执行一次循迹外环计算。
 *
 * Normal_value：
 *     原厂GRAY.c输出的8路归一化模拟量；
 *     黑线接近0，白底接近4096。
 *
 * Digital：
 *     只用于判断是否8路全白丢线；
 *     正常循迹偏差由Normal_value模拟量计算。
 */
void Tracking_Update(
    const unsigned short *Normal_value,
    unsigned char Digital);

/* 串口在线调节循迹外环参数。 */
void Tracking_SetKp(float Kp);
void Tracking_SetKd(float Kd);
void Tracking_SetBaseSpeed(float speed);

float Tracking_GetKp(void);
float Tracking_GetKd(void);

/*
 * 外环给两个速度内环的目标速度，单位为m/s。
 * A为左轮，B为右轮。
 */
extern volatile float target_speed_A;
extern volatile float target_speed_B;

/*
 * 两个车轮的基础速度在main.c中设置。
 * 普通直行时两个值相同；如果机械结构存在差异，也可以分别微调。
 */
extern float target_base_speed_A;
extern float target_base_speed_B;

/* 循迹调试变量。 */
extern volatile float line_error;
extern volatile float turn_speed;
extern volatile unsigned char line_found;

#endif
