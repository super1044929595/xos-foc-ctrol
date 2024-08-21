#ifndef __XOS_MOTOR_H
#define __XOS_MOTOR_H

#ifdef __cplusplus
extern "C"{
#endif



#include "stm32f7xx_hal.h"
#include "xos_motor_pid.h"

	void xos_motor_time_init(void);
	void xos_sg90_update(void);
	//void xos_VelocityopenLoop(float target_velocity);
	uint32_t xos_GetSystick(void);
	void xo_SetSysTick(void);

#define _constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))
#define _SQRT3 1.73205080757f
#define _SQRT3_2 0.86602540378f
#define _1_SQRT3 0.57735026919f
#define _2_SQRT3 1.15470053838f

	extern float exp2f (float);
	extern float scalblnf (float, long int);
	extern float tgammaf (float);
	extern float nearbyintf (float);
	extern long int lrintf (float);
	extern long long int llrintf (float);
	extern float roundf (float);
	extern long int lroundf (float);
	extern long long int llroundf (float);
	extern float truncf (float);
	extern float remquof (float, float, int *);
	extern float fdimf (float, float);
	extern float fmaxf (float, float);
	extern float fminf (float, float);
	extern float fmaf (float, float, float);

	float _normalizeAngle(float angle);
	float _electricalAngle(float shaft_angle, int pole_pairs);
	float velocityOpenloop(float target_velocity, float Uq, TIM_TypeDef * TIM_BASE);
	void setPwm(float Ua, float Ub, float Uc, TIM_TypeDef * TIM_BASE);
	void setPhaseVoltage(float Uq,float Ud, float angle_el, TIM_TypeDef * TIM_BASE);

	void xos_motor_loop(void);
	uint32_t xos_GetSystick(void);

#ifdef __cplusplus
}
#endif

#endif