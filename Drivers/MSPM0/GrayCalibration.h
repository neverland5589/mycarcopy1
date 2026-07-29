#ifndef GRAY_CALIBRATION_H
#define GRAY_CALIBRATION_H

#include <stdint.h>

uint8_t GrayCalibration_Load(uint16_t *white_value, uint16_t *black_value);
void GrayCalibration_Reset(void);
void GrayCalibration_Confirm(uint32_t current_frame_count);
uint8_t GrayCalibration_Process(volatile const uint32_t *frame_count_source, volatile const uint16_t *frame_snapshot, uint16_t *white_value, uint16_t *black_value);
uint8_t GrayCalibration_GetStep(void);
uint8_t GrayCalibration_GetSampleCount(void);

#endif
