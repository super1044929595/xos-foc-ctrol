#ifndef __XOS_AS5600_H
#define __XOS_AS5600_H

#ifdef __cplusplus
	extern "C" {
#endif

#include "main.h"

//#include "stm32f7xx_hal_i2c.h"
float GetAngle(void);
float GetAngle_Without_Track(void);
void xos_as5600_Init(void);

#define AS5600_RAW_ADDR    0x36
#define AS5600_ADDR        (AS5600_RAW_ADDR << 1)
#define AS5600_WRITE_ADDR  (AS5600_RAW_ADDR << 1)
#define AS5600_READ_ADDR   ((AS5600_RAW_ADDR << 1) | 1)
#define _2PI 6.28318530718
	
#define AS5600_RESOLUTION 4096 //12bit Resolution
	
#define AS5600_RAW_ANGLE_REGISTER  0x0C
uint16_t bsp_as5600GetRawAngle(void) ;
float bsp_as5600GetAngle(void) ;
float Sensor_update(void);
float getVelocity(void);
float xosGetAngle_Without_Track(void);
float getMechanicalAngle(void) ;
float getAngle(void);

#ifdef __cplusplus
}
#endif


#endif
