#include "xos_lowpass_filter.h"

float LowPassFilter_operator(float x, struct LowPassFilter* filter)
{
	float dt=5E-3f;
	#if 0
		   uint32_t xos_nowtime=0;
	static uint32_t xos_lasttime=0;
		   uint32_t xos_dt=0;
	if(xos_nowtime>=xos_lasttime){
		   xos_dt=(xos_nowtime-xos_lasttime)
	}else{
		   xos_dt=(0xFFFFFFFF-xos_lasttime+xos_nowtime);
	}
	if(xos_dt<=5){
			x=filter->y_prev;
	}else{
			float alpha=filter->Tf/(filter->Tf + dt);
			float y = alpha*filter->y_prev + (1.0f - alpha)*xos_dt;
			filter->y_prev=y;
	}
	xos_lasttime=xos_nowtime;
	return y;
	#endif
	float alpha = filter->Tf/(filter->Tf + dt);
	float y = alpha*filter->y_prev + (1.0f - alpha)*x;
	filter->y_prev = y;
	return y;
}
