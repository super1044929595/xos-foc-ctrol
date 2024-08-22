#ifndef __XOS_SOFTWARE_I2C_H
#define __XOS_SOFTWARE_I2C_H

#ifdef __cplusplus
	extern "C"{
#endif

#include "main.h"

#define ACK                 (0)
	
#define NACK                (1)
	
	
	
#define SCL_PIN             GPIO_PIN_9
	
#define SCL_PORT            GPIOB
	
#define SCL_PIN_CLK_EN()    __HAL_RCC_GPIOB_CLK_ENABLE()
	
	
	
#define SDA_PIN             GPIO_PIN_8
	
#define SDA_PORT            GPIOB
	
#define SDA_PIN_CLK_EN()    __HAL_RCC_GPIOB_CLK_ENABLE()
	
	
	
#define SCL_H()             HAL_GPIO_WritePin(SCL_PORT, SCL_PIN, GPIO_PIN_SET)
	
#define SCL_L()             HAL_GPIO_WritePin(SCL_PORT, SCL_PIN, GPIO_PIN_RESET)
	
#define SCL_INPUT()         HAL_GPIO_ReadPin(SCL_PORT, SCL_PIN)
	
	
	
#define SDA_H()             HAL_GPIO_WritePin(SDA_PORT, SDA_PIN, GPIO_PIN_SET)
	
#define SDA_L()             HAL_GPIO_WritePin(SDA_PORT, SDA_PIN, GPIO_PIN_RESET)
	
#define SDA_INPUT()         HAL_GPIO_ReadPin(SDA_PORT, SDA_PIN) 

//method
void xos_I2C_Init(void);
void EEPROM_Read_NBytes(uint16_t addr, uint8_t *pdata, uint16_t sz);


#ifdef __cplusplus
}
#endif
#endif

