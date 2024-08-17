#ifndef __XOS_CURRENT_SENSOR_H
#define __XOS_CURRENT_SENSOR_H

#include "math.h"
#include "stm32f7xx_hal.h"

void ADC_Select_CH0(ADC_HandleTypeDef);
void ADC_Select_CH1(ADC_HandleTypeDef);
void ADC_Select_CH2(ADC_HandleTypeDef);
void read_ADC_voltage(ADC_HandleTypeDef ,uint16_t *);
void calibrateOffsets(ADC_HandleTypeDef,uint16_t *);

#endif