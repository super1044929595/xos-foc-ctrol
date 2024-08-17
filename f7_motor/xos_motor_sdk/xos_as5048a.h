#ifndef __XOS_AS5048A_H
#define __XOS_AS5048A_H

#include <stdint.h>

#include "stm32f7xx_hal.h"

#define AS5048A_CLEAR_ERROR_FLAG             0x0001
#define AS5048A_PROGRAMMING_CONTROL          0x0003
#define AS5048A_OTP_REGISTER_ZERO_POS_HIGH   0x0016
#define AS5048A_OTP_REGISTER_ZERO_POS_LOW    0x0017
#define AS5048A_DIAG_AGC                     0x3FFD
#define AS5048A_MAGNITUDE                    0x3FFE
#define AS5048A_ANGLE                        0x3FFF
#define MAX_ANGLE_VALUE                      8192
#define EN_SPI HAL_GPIO_WritePin(_ps, _cs, GPIO_PIN_RESET);
#define DIS_SPI HAL_GPIO_WritePin(_ps, _cs, GPIO_PIN_SET);

uint8_t spiCalcEvenParity(uint16_t value);
uint16_t read(SPI_HandleTypeDef* _spi, GPIO_TypeDef* _ps, uint16_t _cs,uint16_t registerAddress);

#endif