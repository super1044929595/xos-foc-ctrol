

#include "xos_lowpass_filter.h"
#include "xos_motor.h"

float LowPassFilter_operator(float x, struct LowPassFilter* filter){
	float dt=5E-3f;
	float alpha = filter->Tf/(filter->Tf + dt);
	float y = alpha*filter->y_prev + (1.0f - alpha)*x;
	filter->y_prev = y;
	return y;
}


