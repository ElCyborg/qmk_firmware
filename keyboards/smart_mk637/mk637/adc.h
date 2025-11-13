#pragma once

#include "analog.h"

//测量电量的参数
#define BATT_OFF 3000
#define BATT_5   3130
#define BATT_10 3180
#define BATT_40 3630
#define BATT_60 3760
#define BATT_80 3930
#define BATT_85 3980
#define BATT_99 4150


void adc_init(void);
void adc_channel_set(ADC_TypeDef *adcx, uint8_t ch, uint8_t stime);
uint32_t adc_get_result(uint8_t ch);
uint32_t adc_get_result_average(uint8_t ch, uint8_t times);
uint16_t get_adc_value(void);
uint16_t get_adc_vref(void);
uint8_t batt_level(void);