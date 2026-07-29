#include "stepmotor.h"

/* 保存当前驱动器地址以及F5命令配置的脉宽和位置范围。 */
static uint8_t stepmotor_address = STEPMOTOR_DEFAULT_ADDRESS;
static uint16_t stepmotor_max_pulse_width = STEPMOTOR_DEFAULT_MAX_PULSE_WIDTH;
static uint16_t stepmotor_min_pulse_width = STEPMOTOR_DEFAULT_MIN_PULSE_WIDTH;
static int32_t stepmotor_max_position = STEPMOTOR_DEFAULT_MAX_POSITION;
static int32_t stepmotor_min_position = STEPMOTOR_DEFAULT_MIN_POSITION;

/* 计算PD42S1通信帧的校验码，校验码为前面所有字节累加后的低8位。 */
static uint8_t StepMotor_Checksum(const uint8_t *Data, uint8_t Length)
{
    uint8_t Checksum = 0;
    uint8_t i = 0;

    for (i = 0; i < Length; i++)
    {
        Checksum += Data[i];
    }

    return Checksum;
}

/*
 * 按照PD42S1手册组装并发送通信帧。
 * 帧格式为：C5 从机地址 功能码 指令数据 CHECKSUM 5C。
 * PA0配置为UART0_TX，所有字节通过PA0发送给驱动器RX。
 */
static void StepMotor_SendFrame(uint8_t Function, const uint8_t *Data, uint8_t DataLength)
{
    uint8_t Frame[18];
    uint8_t FrameLength = DataLength + 5;
    uint8_t i = 0;

    /* 写入帧头、从机地址和功能码。 */
    Frame[0] = 0xC5;
    Frame[1] = stepmotor_address;
    Frame[2] = Function;

    /* 将指令数据复制到发送帧。 */
    for (i = 0; i < DataLength; i++)
    {
        Frame[i + 3] = Data[i];
    }

    /* 在数据后面添加校验码和帧尾。 */
    Frame[DataLength + 3] = StepMotor_Checksum(Frame, DataLength + 3);
    Frame[DataLength + 4] = 0x5C;

    /* 通过PA0对应的UART0_TX阻塞发送完整数据帧。 */
    for (i = 0; i < FrameLength; i++)
    {
        DL_UART_Main_transmitDataBlocking(UART_STEPMOTOR_INST, Frame[i]);
    }
}

/* 将uint16_t数据按手册要求转换为高字节在前的两个字节。 */
static void StepMotor_WriteUint16(uint8_t *Data, uint16_t Value)
{
    Data[0] = (uint8_t)(Value >> 8);
    Data[1] = (uint8_t)Value;
}

/* 将int32_t数据按手册要求转换为高字节在前的四个字节。 */
static void StepMotor_WriteInt32(uint8_t *Data, int32_t Value)
{
    uint32_t UnsignedValue = (uint32_t)Value;

    Data[0] = (uint8_t)(UnsignedValue >> 24);
    Data[1] = (uint8_t)(UnsignedValue >> 16);
    Data[2] = (uint8_t)(UnsignedValue >> 8);
    Data[3] = (uint8_t)UnsignedValue;
}

/*
 * 步进电机模块初始化。
 * 首先发送F5命令，将500到2500微秒映射为负10度到正10度。
 * 然后把PA10的PWM脉宽设为1500微秒，使电机回到中间位置。
 */
void StepMotor_Init(void)
{
    stepmotor_address = STEPMOTOR_DEFAULT_ADDRESS;
    StepMotor_ConfigurePulseWidthPosition(STEPMOTOR_DEFAULT_MAX_PULSE_WIDTH, STEPMOTOR_DEFAULT_MIN_PULSE_WIDTH, STEPMOTOR_DEFAULT_MAX_POSITION, STEPMOTOR_DEFAULT_MIN_POSITION);
    StepMotor_SetPulseWidth(STEPMOTOR_DEFAULT_CENTER_PULSE_WIDTH);
    DL_Timer_startCounter(PWM_STEPMOTOR_INST);
}

/* 设置需要通信的PD42S1从机地址，默认地址为1。 */
void StepMotor_SetAddress(uint8_t Address)
{
    stepmotor_address = Address;
}

/*
 * 发送手册4.4.6中的F5脉宽位置模式配置命令。
 * MaxPulseWidth和MinPulseWidth的单位为微秒。
 * MaxPosition和MinPosition使用PD42S1位置单位，51200表示一圈。
 */
void StepMotor_ConfigurePulseWidthPosition(uint16_t MaxPulseWidth, uint16_t MinPulseWidth, int32_t MaxPosition, int32_t MinPosition)
{
    uint8_t Data[12];

    /* 手册规定脉宽取值不能超过50000微秒。 */
    if (MaxPulseWidth > 50000)
    {
        MaxPulseWidth = 50000;
    }
    if (MinPulseWidth > 50000)
    {
        MinPulseWidth = 50000;
    }

    /* 最大脉宽必须大于最小脉宽，否则不发送错误配置。 */
    if (MaxPulseWidth <= MinPulseWidth)
    {
        return;
    }

    /* 最大位置必须大于最小位置，否则无法进行线性换算。 */
    if (MaxPosition <= MinPosition)
    {
        return;
    }

    /* Byte1到Byte2为最长脉宽，Byte3到Byte4为最短脉宽。 */
    StepMotor_WriteUint16(&Data[0], MaxPulseWidth);
    StepMotor_WriteUint16(&Data[2], MinPulseWidth);

    /* Byte5到Byte8为最长脉宽位置，Byte9到Byte12为最短脉宽位置。 */
    StepMotor_WriteInt32(&Data[4], MaxPosition);
    StepMotor_WriteInt32(&Data[8], MinPosition);

    /* 功能码F5表示脉宽位置模式配置。 */
    StepMotor_SendFrame(0xF5, Data, 12);

    /* 保存本次配置，后面的位置到脉宽换算必须使用相同范围。 */
    stepmotor_max_pulse_width = MaxPulseWidth;
    stepmotor_min_pulse_width = MinPulseWidth;
    stepmotor_max_position = MaxPosition;
    stepmotor_min_position = MinPosition;
}

/*
 * 直接设置STP引脚的PWM高电平时间。
 * PA10使用1MHz定时器时钟，因此一个计数正好等于一微秒。
 */
void StepMotor_SetPulseWidth(uint16_t PulseWidth)
{
    /* 将脉宽限制在F5命令已经配置的范围内。 */
    if (PulseWidth > stepmotor_max_pulse_width)
    {
        PulseWidth = stepmotor_max_pulse_width;
    }
    if (PulseWidth < stepmotor_min_pulse_width)
    {
        PulseWidth = stepmotor_min_pulse_width;
    }

    /* EDGE_ALIGN_UP模式下比较值加1等于高电平时间，因此这里减1。 */
    DL_Timer_setCaptureCompareValue(PWM_STEPMOTOR_INST, PulseWidth - 1, GPIO_PWM_STEPMOTOR_C0_IDX);
}

/*
 * 根据目标位置计算对应的PWM脉宽。
 * 计算关系与发送给驱动器的F5配置完全一致。
 */
void StepMotor_SetTargetPosition(int32_t TargetPosition)
{
    int64_t PulseWidth = 0;

    /* 目标位置超过配置范围时限制到最大或最小位置。 */
    if (TargetPosition > stepmotor_max_position)
    {
        TargetPosition = stepmotor_max_position;
    }
    if (TargetPosition < stepmotor_min_position)
    {
        TargetPosition = stepmotor_min_position;
    }

    /* 将位置范围线性映射为脉宽范围，使用int64_t防止乘法溢出。 */
    PulseWidth = stepmotor_min_pulse_width;
    PulseWidth += (int64_t)(TargetPosition - stepmotor_min_position) * (stepmotor_max_pulse_width - stepmotor_min_pulse_width) / (stepmotor_max_position - stepmotor_min_position);

    /* 把计算得到的脉宽输出到PA10的STP信号。 */
    StepMotor_SetPulseWidth((uint16_t)PulseWidth);
}

/* 将角度转换为PD42S1位置单位，然后设置目标位置。 */
void StepMotor_SetAngle(double Angle)
{
    int32_t TargetPosition = (int32_t)(Angle * STEPMOTOR_POSITION_PER_TURN / 360.0);

    StepMotor_SetTargetPosition(TargetPosition);
}
