#ifndef __XOS_MOTOR_PID_H
#define __XOS_MOTOR_PID_H


#include "stm32f7xx_hal.h"

#define MAXFLOAT	3.40282347e+38F

#define M_E		    2.7182818284590452354
#define M_LOG2E		1.4426950408889634074
#define M_LOG10E	0.43429448190325182765
#define M_LN2		_M_LN2
#define M_LN10		2.30258509299404568402
#define M_PI		3.14159265358979323846
#define M_PI_2		1.57079632679489661923
#define M_PI_4		0.78539816339744830962
#define M_1_PI		0.31830988618379067154
#define M_2_PI		0.63661977236758134308
#define M_2_SQRTPI	1.12837916709551257390
#define M_SQRT2		1.41421356237309504880
#define M_SQRT1_2	0.70710678118654752440


struct PIDController{
	float P; //!< 比例增益(P环增益)
	float I; //!< 积分增益（I环增益）
	float D; //!< 微分增益（D环增益）
	float output_ramp;
	float limit;
    float error_prev; //!< 最后的跟踪误差值
    float output_prev;  //!< 最后一个 pid 输出值
    float integral_prev; //!< 最后一个积分分量值
//    unsigned long timestamp_prev; //!< 上次执行时间戳
};
float PID_operator(float error, struct PIDController* pid);

void xos_PeriodElapsedCallback(void);

#endif