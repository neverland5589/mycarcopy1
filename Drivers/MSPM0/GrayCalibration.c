#include "GrayCalibration.h"
#include "ti_msp_dl_config.h"

#define GRAY_CALIBRATION_FLASH_ADDRESS 0x0001FC00
#define GRAY_CALIBRATION_MAGIC 0x47524159
#define GRAY_CALIBRATION_MIN_DIFF 100
#define GRAY_CALIBRATION_SAMPLES 50

typedef struct
{
    uint32_t magic;
    uint32_t checksum;
    uint16_t white[8];
    uint16_t black[8];
} GrayCalibrationRecord;

static uint8_t gray_calibration_step = 0;
static uint8_t gray_calibration_sample_count = 0;
static uint32_t gray_calibration_last_frame = 0;
static uint32_t gray_calibration_sum[8] = {0};
static uint16_t gray_calibration_white[8] = {0};
static uint16_t gray_calibration_black[8] = {0};

static uint32_t GrayCalibration_Checksum(const uint16_t *white_value, const uint16_t *black_value)
{
    uint32_t checksum = 0x13572468;
    uint8_t i = 0;

    for (i = 0; i < 8; i++)
    {
        checksum = checksum * 33 + white_value[i];
        checksum = checksum * 33 + black_value[i];
    }

    return checksum;
}

static uint8_t GrayCalibration_ValuesValid(const uint16_t *white_value, const uint16_t *black_value)
{
    uint8_t i = 0;
    int difference = 0;

    for (i = 0; i < 8; i++)
    {
        if (white_value[i] > 4095 || black_value[i] > 4095)
        {
            return 0;
        }

        difference = (int)white_value[i] - (int)black_value[i];
        if (difference < 0)
        {
            difference = -difference;
        }

        if (difference < GRAY_CALIBRATION_MIN_DIFF)
        {
            return 0;
        }
    }

    return 1;
}

uint8_t GrayCalibration_Load(uint16_t *white_value, uint16_t *black_value)
{
    const GrayCalibrationRecord *record = (const GrayCalibrationRecord *)GRAY_CALIBRATION_FLASH_ADDRESS;
    uint8_t i = 0;

    if (record->magic != GRAY_CALIBRATION_MAGIC)
    {
        return 0;
    }

    if (record->checksum != GrayCalibration_Checksum(record->white, record->black))
    {
        return 0;
    }

    if (!GrayCalibration_ValuesValid(record->white, record->black))
    {
        return 0;
    }

    for (i = 0; i < 8; i++)
    {
        white_value[i] = record->white[i];
        black_value[i] = record->black[i];
    }

    return 1;
}

static uint8_t GrayCalibration_Save(const uint16_t *white_value, const uint16_t *black_value)
{
    GrayCalibrationRecord record;
    const GrayCalibrationRecord *saved_record = (const GrayCalibrationRecord *)GRAY_CALIBRATION_FLASH_ADDRESS;
    DL_FLASHCTL_COMMAND_STATUS flash_status;
    uint32_t *record_words = (uint32_t *)&record;
    uint32_t interrupt_state = __get_PRIMASK();
    uint8_t i = 0;

    record.magic = GRAY_CALIBRATION_MAGIC;
    for (i = 0; i < 8; i++)
    {
        record.white[i] = white_value[i];
        record.black[i] = black_value[i];
    }
    record.checksum = GrayCalibration_Checksum(record.white, record.black);

    __disable_irq();
    DL_FlashCTL_executeClearStatus(FLASHCTL);
    DL_FlashCTL_unprotectSector(FLASHCTL, GRAY_CALIBRATION_FLASH_ADDRESS, DL_FLASHCTL_REGION_SELECT_MAIN);
    flash_status = DL_FlashCTL_eraseMemoryFromRAM(FLASHCTL, GRAY_CALIBRATION_FLASH_ADDRESS, DL_FLASHCTL_COMMAND_SIZE_SECTOR);
    if (flash_status == DL_FLASHCTL_COMMAND_STATUS_FAILED)
    {
        if (interrupt_state == 0)
        {
            __enable_irq();
        }
        return 0;
    }

    for (i = 0; i < sizeof(GrayCalibrationRecord) / 8; i++)
    {
        DL_FlashCTL_executeClearStatus(FLASHCTL);
        DL_FlashCTL_unprotectSector(FLASHCTL, GRAY_CALIBRATION_FLASH_ADDRESS + i * 8, DL_FLASHCTL_REGION_SELECT_MAIN);
        flash_status = DL_FlashCTL_programMemoryFromRAM64WithECCGenerated(FLASHCTL, GRAY_CALIBRATION_FLASH_ADDRESS + i * 8, &record_words[i * 2]);
        if (flash_status == DL_FLASHCTL_COMMAND_STATUS_FAILED)
        {
            if (interrupt_state == 0)
            {
                __enable_irq();
            }
            return 0;
        }
    }

    if (interrupt_state == 0)
    {
        __enable_irq();
    }

    if (saved_record->magic != GRAY_CALIBRATION_MAGIC)
    {
        return 0;
    }

    if (saved_record->checksum != GrayCalibration_Checksum(saved_record->white, saved_record->black))
    {
        return 0;
    }

    return GrayCalibration_ValuesValid(saved_record->white, saved_record->black);
}

static void GrayCalibration_StartCapture(uint8_t step, uint32_t current_frame_count)
{
    uint8_t i = 0;

    gray_calibration_step = step;
    gray_calibration_sample_count = 0;
    gray_calibration_last_frame = current_frame_count;
    for (i = 0; i < 8; i++)
    {
        gray_calibration_sum[i] = 0;
    }
}

void GrayCalibration_Reset(void)
{
    gray_calibration_step = 0;
    gray_calibration_sample_count = 0;
}

void GrayCalibration_Confirm(uint32_t current_frame_count)
{
    if (gray_calibration_step == 0)
    {
        gray_calibration_step = 1;
    }
    else if (gray_calibration_step == 1)
    {
        GrayCalibration_StartCapture(2, current_frame_count);
    }
    else if (gray_calibration_step == 3)
    {
        GrayCalibration_StartCapture(4, current_frame_count);
    }
}

uint8_t GrayCalibration_Process(volatile const uint32_t *frame_count_source, volatile const uint16_t *frame_snapshot, uint16_t *white_value, uint16_t *black_value)
{
    uint16_t frame[8];
    uint32_t frame_count = 0;
    uint32_t interrupt_state = 0;
    uint8_t i = 0;

    if (gray_calibration_step != 2 && gray_calibration_step != 4)
    {
        return 0;
    }

    if (*frame_count_source == gray_calibration_last_frame)
    {
        return 0;
    }

    interrupt_state = __get_PRIMASK();
    __disable_irq();
    frame_count = *frame_count_source;
    for (i = 0; i < 8; i++)
    {
        frame[i] = frame_snapshot[i];
    }
    if (interrupt_state == 0)
    {
        __enable_irq();
    }

    gray_calibration_last_frame = frame_count;
    for (i = 0; i < 8; i++)
    {
        gray_calibration_sum[i] += frame[i];
    }
    gray_calibration_sample_count++;

    if (gray_calibration_sample_count < GRAY_CALIBRATION_SAMPLES)
    {
        return 0;
    }

    if (gray_calibration_step == 2)
    {
        for (i = 0; i < 8; i++)
        {
            gray_calibration_white[i] = gray_calibration_sum[i] / GRAY_CALIBRATION_SAMPLES;
        }
        gray_calibration_step = 3;
        return 0;
    }

    for (i = 0; i < 8; i++)
    {
        gray_calibration_black[i] = gray_calibration_sum[i] / GRAY_CALIBRATION_SAMPLES;
    }

    if (!GrayCalibration_ValuesValid(gray_calibration_white, gray_calibration_black))
    {
        gray_calibration_step = 6;
        return 0;
    }

    if (!GrayCalibration_Save(gray_calibration_white, gray_calibration_black))
    {
        gray_calibration_step = 6;
        return 0;
    }

    if (!GrayCalibration_Load(white_value, black_value))
    {
        gray_calibration_step = 6;
        return 0;
    }

    gray_calibration_step = 5;
    return 1;
}

uint8_t GrayCalibration_GetStep(void)
{
    return gray_calibration_step;
}

uint8_t GrayCalibration_GetSampleCount(void)
{
    return gray_calibration_sample_count;
}
