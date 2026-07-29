#ifndef ADC_H
#define ADC_H

#include "ti_msp_dl_config.h"

extern volatile uint16_t ADC_VALUE[40];
unsigned int adc_getValue(unsigned int number); //读取ADC的数据

#endif
