#ifndef PID_H
#define PID_H
#include "main.h"

#ifdef __cplusplus
	extern "C"{
#endif
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
float PID_xos_operator(float error, struct PIDController* pid);

#ifdef __cplusplus
}
#endif

#endif


