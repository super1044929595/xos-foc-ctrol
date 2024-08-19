#include "xos_motor_pid.h"
#include "xos_lowpass_filter.h"
#include "math.h"
#include "stm32f7xx_hal.h"
#include "xos_motor.h"
#include "xos_as5048a.h"
#include "stdio.h"


extern SPI_HandleTypeDef hspi2;

char data[50];
uint32_t open_loop_timestamp=0;
float zero_electric_angle=0;
float shaft_angle=0;
float voltage_limit=5;
float voltage_power_supply=5;
int period=200; // period for the PWM
int dir=1; // anti clockwise direction is 1 , clockwise is -1
int pole_pairs=7;
int index1=0;
uint16_t raw1,raw2,raw3;
float motor_target= M_PI/6;
float Ts=5E-3f;
float Kp=0.167f;
float angle_prev=-1.0f;
float target_vel=5;
uint16_t ADC_VAL[3];
uint16_t current_offset[3];
double current_phase[3];
const int adc_range=4095;
const double Vref=3.3;    // Voltage
const double Rsense=0.33; // Ohm
//const double KV= 2375.0/12.0; //KV number (RPM is 2149 - 2375, when operating voltage is 12V)

struct LowPassFilter filter= {.Tf=0.01,.y_prev=0.0f}; //Tf=10ms
struct LowPassFilter filter_current= {.Tf=0.05,.y_prev=0.0f}; //Tf=5ms
// limit=voltage_power_supply/2;
struct PIDController pid_controller = {.P=0.5,.I=0.1,.D=0.0,.output_ramp=100.0,.limit=6,.error_prev=0,.output_prev=0,.integral_prev=0};

struct PIDController pid_controller_current = {.P=1.0,.I=0.1,.D=0.0,.output_ramp=100.0,.limit=6,.error_prev=0,.output_prev=0,.integral_prev=0};


float cal_angular_vel(float angle_now){
    if (angle_prev < 0){
    	angle_prev=angle_now;
    	return 0;
    }
    float delta_angle=angle_now -angle_prev;
    if (delta_angle >= 1.6*M_PI){
    	delta_angle-=2*M_PI;
    }
    if (delta_angle <= -1.6*M_PI){
        	delta_angle+=2*M_PI;
    }
    angle_prev=angle_now;
    return delta_angle/Ts;


}
float cal_Iq(double* current_phase, float angle_el){
	 angle_el = _normalizeAngle(angle_el + zero_electric_angle);
	 float I_alpha=current_phase[0];
	 float I_beta=_1_SQRT3*(2*current_phase[1]+current_phase[0]);
//	 float Iq=-sin(angle_el)*I_alpha+cos(angle_el)*I_beta;
//	 float Id=cos(angle_el)*I_alpha+sin(angle_el)*I_beta;
	 return -sin(angle_el)*I_alpha+cos(angle_el)*I_beta;
}


void xos_PeriodElapsedCallback(void)
{
	  uint16_t read_raw=0;//read(&hspi2, SPI1_CSn_GPIO_Port,SPI1_CSn_Pin,AS5048A_ANGLE);
    float angle_now=(float)read_raw /(float)MAX_ANGLE_VALUE *2*M_PI*dir;

//    close loop position control
//    float angle_error=motor_target-angle_now;
//
//    angle_error=_normalizeAngle(angle_error);
//    if (angle_error > M_PI){
//    	angle_error-=2*M_PI;
//    }
//    setPhaseVoltage(_constrain(Kp*(angle_error)/M_PI*180,-voltage_power_supply/2,voltage_power_supply/2),0,_electricalAngle(angle_now,pole_pairs),TIM1);
//    sprintf(data, "angle_error : %i \n", (int) floor(angle_error/M_PI*180));
//       CDC_Transmit_FS((uint8_t*) data, strlen(data));
    //    open loop speed control
//        velocityOpenloop(5,5.5,TIM1);
//    closed loop speed control
    float angular_vel=cal_angular_vel(angle_now);
    float filtered_vel=LowPassFilter_operator(angular_vel,&filter);
    float velo_controller_out=PID_operator(target_vel-filtered_vel,&pid_controller);
    //read_ADC_voltage(hspi2,ADC_VAL);
    for (int i=0;i<3;i++){
       	current_phase[i] =(float)  ( ADC_VAL[i]-current_offset[i])/(float) adc_range*Vref/Rsense;
    }

    float Iq=cal_Iq(current_phase, _electricalAngle(angle_now, pole_pairs));
    float filtered_Iq=LowPassFilter_operator(Iq,&filter_current);

    float current_controller_output=PID_operator(velo_controller_out-filtered_Iq,&pid_controller_current);

    setPhaseVoltage(_constrain(current_controller_output,-voltage_power_supply/2,voltage_power_supply/2),  0, _electricalAngle(angle_now, pole_pairs),TIM2);
    sprintf(data, "angle_now : %i \t angle_prev : %i \n", (int) floor(angle_now/M_PI*180), (int) floor(angle_prev/M_PI*180));
//    CDC_Transmit_FS((uint8_t*) data, strlen(data));
   // sprintf(data, "angular_vel : %i \n", (int) floor(angular_vel/M_PI*180));
}



float PID_operator(float error, struct PIDController* pid){
    float Ts = 2.5E-3f;
    // P环

    float proportional = pid->P * error;
    // Tustin 散点积分（I环）
    float integral = pid->integral_prev + pid->I*Ts*0.5f*(error + pid->error_prev);
    integral = _constrain(integral, -pid->limit, pid->limit);
    // D环（微分环节）
    float derivative = pid->D*(error - pid->error_prev)/Ts;

    // 将P,I,D三环的计算值加起来
    float output = proportional + integral + derivative;
    output = _constrain(output, -pid->limit, pid->limit);

    if(pid->output_ramp > 0){
        // 对PID的变化速率进行限制
        float output_rate = (output - pid->output_prev)/Ts;
        if (output_rate > pid->output_ramp)
            output = pid->output_prev + pid->output_ramp*Ts;
        else if (output_rate < -pid->output_ramp)
            output = pid->output_prev - pid->output_ramp*Ts;
    }
    // 保存值（为了下一次循环）
    pid->integral_prev = integral;
    pid->output_prev = output;
    pid->error_prev = error;
//  pid->timestamp_prev = timestamp_now;
    return output;
}
