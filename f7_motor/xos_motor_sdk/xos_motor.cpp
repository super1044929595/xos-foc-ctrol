#include "main.h"
#include "xos_motor.h"
#include "stm32f7xx_hal.h"
#include "stm32f7xx_hal_tim.h"
#include "stdio.h"
#include "stdlib.h"
#include "math.h"
#include "xos_as5600.h"
#include "xos_pid.h"
#include "xos_lowpass_filter.h"


void xos_Foc_SetVol(float uq, float angle);
void xos_Foc_SetPwm( float ua, float ub,float  uc);
float _electricalAngle(float shaft_angle, int pole_pairs);
float _normalizeAngle(float angle);
float _electricalAngle(float shaft_angle, int pole_pairs) ;

//#define _constrain(x,a,b)      ((x<a)?a:(x<b?x:b))
//#define voltage_power_supply 12
//#define PI 3.1415926
extern TIM_HandleTypeDef htim2;
static int32_t xos_inter=0;
static int32_t xos_pwm_enable=0;
static __IO uint32_t xos_SysTick_CNt=0;
float zero_electric_angle=0;
float shaft_angle=0;
float voltage_limit=12.6;
float voltage_power_supply=12.6;

//PID--------------------------------->
//PIDController xos_vel_pid=PIDController;//{.P=2,.I=0,.D=0,ramp=100000,.limit=12};


void xos_motor_time_init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */
	voltage_power_supply=5;
  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 3;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 2400-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;

  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_OC_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
	
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};
  /* USER CODE BEGIN TIM2_Init 2 */
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;

  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
	sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim2, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE END TIM2_Init 2 */
	HAL_TIM_MspPostInit(&htim2);
	//HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_3);

	#if 0
	HAL_TIM_OC_Start(&htim2,TIM_CHANNEL_1);
	HAL_TIM_OC_Start(&htim2,TIM_CHANNEL_3);
	HAL_TIM_OC_Start(&htim2,TIM_CHANNEL_4);
    #else
	
    HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_3);	
    HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_4);
	#endif
#define _3PI_2 4.71238898038f

	//setPhaseVoltage(5, 0,_3PI_2,TIM2);
	//HAL_Delay(1000);
	zero_electric_angle=Sensor_update();
	//setPhaseVoltage(0, 0,_3PI_2,TIM2);
}

uint32_t xos_GetSystick(void)
{
	return xos_SysTick_CNt;
}



static int xossi=0;
float xos_jwangle=0;
void xo_SetSysTick(void)
{
	++xos_SysTick_CNt;
	if(xos_SysTick_CNt%120==0){		

	}
}

void set_angle(int    angle )
{
	__HAL_TIM_SetCompare(&htim2,TIM_CHANNEL_1,angle);
	__HAL_TIM_SetCompare(&htim2,TIM_CHANNEL_3,angle);
	__HAL_TIM_SetCompare(&htim2,TIM_CHANNEL_4,angle);
	//HAL_Delay(500);
}

void xos_sg90_update(void)
{
	if(xos_pwm_enable==0){
		xos_inter+=5;
		if(xos_inter>=35){
		xos_pwm_enable=1;
		}
	}else{
		xos_inter-=5;
		if(xos_inter<=0){
			xos_pwm_enable=0;
		}
	}   
  xos_GetSystick();
  __HAL_TIM_SetCompare(&htim2,TIM_CHANNEL_3,xos_inter);
  __HAL_TIM_SetCompare(&htim2,TIM_CHANNEL_4,xos_inter);
  //HAL_Delay(100);
}

#define M_PI 3.1415926
extern float zero_electric_angle;
extern int pole_pairs;
extern float shaft_angle;
int dir=-1;
extern float voltage_limit;
extern float voltage_power_supply;
extern int period;

float _normalizeAngle(float angle){
  float a = fmod(angle, 2*M_PI);   //取余运算可以用于归一化，列出特殊值例子算便知
  return a >= 0 ? a : (a + 2*M_PI);
  //三目运算符。格式：condition ? expr1 : expr2
  //其中，condition 是要求值的条件表达式，如果条件成立，则返回 expr1 的值，否则返回 expr2 的值。
  //可以将三目运算符视为 if-else 语句的简化形式。
  //fmod 函数的余数的符号与除数相同。因此，当 angle 的值为负数时，余数的符号将与 _2M_PI 的符号相反。
  //也就是说，如果 angle 的值小于 0 且 _2M_PI 的值为正数，则 fmod(angle, _2M_PI) 的余数将为负数。
  //例如，当 angle 的值为 -M_PI/2，_2M_PI 的值为 2M_PI 时，fmod(angle, _2M_PI) 将返回一个负数。
  //在这种情况下，可以通过将负数的余数加上 _2M_PI 来将角度归一化到 [0, 2M_PI] 的范围内，以确保角度的值始终为正数。
}

float _electricalAngle(float shaft_angle, int pole_pairs) {
	//return _normalizeAngle(((float)7)*shaft_angle-zero_electric_angle);
	return _normalizeAngle(((float)7)*Sensor_update()-zero_electric_angle);
}

uint32_t getCurrentMicros(void)
{
  /* Ensure COUNTFLAG is reset by reading SysTick control and status register */
  //LL_SYSTICK_IsActiveCounterFlag();
  uint32_t m = HAL_GetTick();
  const uint32_t tms = SysTick->LOAD + 1;
  __IO uint32_t u = tms - SysTick->VAL;
 // if (LL_SYSTICK_IsActiveCounterFlag()) {
    m = HAL_GetTick();
    u = tms - SysTick->VAL;
 // }
  return (m * 1000 + (u * 1000) / tms);
}
#define PI				 3.14159265358979f

float _electricalAnglexos()
{
  return  _normalizeAngle((float) getMechanicalAngle());
}

//开环速度函数
float xos_time_monitor_time=0;
float xos_angle_monitor=0;
struct LowPassFilter filter= {.Tf=0.01,.y_prev=0.0f}; //Tf=10ms
struct LowPassFilter angle_lowpass_filter= {.Tf=0.01,.y_prev=0.0f}; //Tf=10ms

struct PIDController pid_controller = {.P=0.1,.I=0.1,.D=0.0,.output_ramp=100000.0,.limit=6,.error_prev=0,.output_prev=0,.integral_prev=0};

float vec_out_filter=0;
float velocityOpenloop(float target_velocity, float Uq, TIM_TypeDef * TIM_BASE)
{
	static uint32_t pre_us=0;
	float Ts;
	float sensor_angle;

	uint32_t now_us =xos_GetSystick();
	//Provides a tick value in microseconds.
	//计算当前每个Loop的运行时间间隔
	now_us=now_us-pre_us;
	
	if(now_us<2000) return 0;
	Ts = (float)(now_us - pre_us)*1e-2f ;
	if(Ts<=0 || Ts> 0.5f){
		Ts=1e-3f;
	}
    Ts = (float)(1)*1e-2f ;
	shaft_angle=Sensor_update();
	//shaft_angle=LowPassFilter_operator(shaft_angle,&angle_lowpass_filter);
	//shaft_angle+=0.03;	
	// 通过乘以时间间隔和目标速度来计算需要转动的机械角度，存储在 shaft_angle 变量中。
	//在此之前，还需要对轴角度进行归一化，以确保其值在 0 到 2π 之间。
	shaft_angle = _normalizeAngle(shaft_angle + target_velocity*Ts);
	//以目标速度为 10 rad/s 为例，如果时间间隔是 1 秒，则在每个循环中需要增加 10 * 1 = 10 弧度的角度变化量，才能使电机转动到目标速度。
	//如果时间间隔是 0.1 秒，那么在每个循环中需要增加的角度变化量就是 10 * 0.1 = 1 弧度，才能实现相同的目标速度。
	//因此，电机轴的转动角度取决于目标速度和时间间隔的乘积。
	// Uq is not related to voltage limit
	//xos_time_monitor_time=LowPassFilter_operator(getVelocity(),&filter);
	//float velo_controller_out=PID_xos_operator(xos_time_monitor_time,&pid_controller);
	xos_angle_monitor=shaft_angle;
	float Kp=0.133;
	vec_out_filter=LowPassFilter_operator(Kp*(3-Sensor_update())*180/PI,&filter);
	float vec_out_pit=PID_xos_operator(vec_out_filter,&pid_controller);
	xos_time_monitor_time=_electricalAngle(shaft_angle,7);
	//vec_out_filter=-voltage_power_supply/3*180/PI;
	setPhaseVoltage(vec_out_pit,0,_electricalAngle(shaft_angle,7),TIM2);
	pre_us = now_us;  //用于计算下一个时间间隔
  return Uq;
}



float xos_ua=0;
float xos_ub=0;
float xos_uc=0;

float xos_pwma=0;
float xos_pwmb=0;
float xos_pwmc=0;
void setPwm(float Ua, float Ub, float Uc, TIM_TypeDef * TIM_BASE)
{
	//限制上限
	Ua = _constrain(Ua, 0.0f, voltage_limit);
	Ub = _constrain(Ub, 0.0f, voltage_limit);
	Uc = _constrain(Uc, 0.0f, voltage_limit);
	xos_ua=Ua;
	xos_ub=Ub;
	xos_uc=Uc;
	
	// 计算占空比
	// 限制占空比从0到1
	float dc_a = _constrain(Ua / voltage_power_supply, 0.0f , 1.0f );
	float dc_b = _constrain(Ub / voltage_power_supply, 0.0f , 1.0f );
	float dc_c = _constrain(Uc / voltage_power_supply, 0.0f , 1.0f );
	
	xos_ua=dc_a;
	xos_ub=dc_b;
	xos_uc=dc_c;
	
	TIM2->CCR1 = (uint32_t) roundf(dc_a*2400);
	TIM2->CCR3 = (uint32_t) roundf(dc_b*2400);	
	TIM2->CCR4 = (uint32_t) roundf(dc_c*2400);
	
	xos_pwma=TIM2->CCR1;
	xos_pwmb=TIM2->CCR3;
	xos_pwmc=TIM2->CCR4;
}

//---------------------------------------------------------------------------------------------

float xos_ua_ori=0;
float xos_ub_ori=0;
float xos_uc_ori=0;
float xos_angle_setphase=0;
void setPhaseVoltage(float Uq,float Ud, float angle_el, TIM_TypeDef * TIM_BASE) 
{
	
	Uq=_constrain(Uq,-(voltage_power_supply)/2,(voltage_power_supply)/2);
	
	angle_el = _normalizeAngle(angle_el+zero_electric_angle);
	xos_angle_setphase=angle_el;
	// 帕克逆变换
	float Ualpha =  -Uq*sin(angle_el);
	float Ubeta =   Uq*cos(angle_el);

	// 克拉克逆变换
	float Ua = Ualpha + voltage_power_supply/2;
	float Ub = (sqrt(3)*Ubeta-Ualpha)/2 + voltage_power_supply/2;
	float Uc = (-Ualpha-sqrt(3)*Ubeta)/2 + voltage_power_supply/2;

	xos_ua_ori=Ua;
	xos_ub_ori=Ub;
	xos_uc_ori=Uc;

	setPwm(Ua,Ub,Uc,TIM_BASE);

}

//---------------------------------------------------------------------------------------------


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef * htim)
{
	//xos_sg90_update();
	//__HAL_TIM_SetCompare(&htim2,TIM_CHANNEL_3,100);
	//__HAL_TIM_SetCompare(&htim2,TIM_CHANNEL_4,100);

	//xos_PeriodElapsedCallback();
	//velocityOpenloop(1,5,TIM2);
	
	//xos_motor_loop();
	velocityOpenloop(1,5,TIM2);

}


//速度环
float vel_P; //!< 比例增益(P环增益)
float vel_I; //!< 积分增益（I环增益）
float vel_D; //!< 微分增益（D环增益）
float vel_output_ramp; 
float vel_limit; 

float vel_error_prev; //!< 最后的跟踪误差值
float vel_output_prev;	//!< 最后一个 pid 输出值
float vel_integral_prev; //!< 最后一个积分分量值
uint32_t vel_timestamp_prev; //!< 上次执行时间戳


//位置环
float pos_P; //!< 比例增益(P环增益)
float pos_I; //!< 积分增益（I环增益）
float pos_D; //!< 微分增益（D环增益）
float pos_output_ramp; 
float pos_limit; 

float pos_error_prev; //!< 最后的跟踪误差值
float pos_output_prev;	//!< 最后一个 pid 输出值
float pos_integral_prev; //!< 最后一个积分分量值
uint32_t pos_timestamp_prev; //!< 上次执行时间戳



float xos_vec_init(float error)
{

 return 0;
}

float xos_vec_set_value(float error)
{

	// 计算两次循环中间的间隔时间
	uint32_t timestamp_now = xos_GetSystick();
	float Ts = (timestamp_now - vel_timestamp_prev) * 1e-6f;
	if(Ts <= 0 || Ts > 0.5f) Ts = 1e-3f;

	// P环
	float proportional= vel_P*error;
	// Tustin 散点积分（I环）
	float integral=vel_integral_prev+vel_I*Ts*0.5f*(error+vel_error_prev);
	// D环（微分环节）
	float derivative=vel_D*(error-vel_error_prev)/Ts;

	// 将P,I,D三环的计算值加起来
	float output=proportional+integral+derivative;
	output=_constrain(output,-vel_limit,vel_limit);

	 if(vel_output_ramp>0){
		// 对PID的变化速率进行限制
		float output_rate=(output-vel_output_prev)/Ts;
		if(output_rate>vel_output_ramp){
			output=vel_output_prev+vel_output_ramp*Ts;
		}else if(output_rate<-vel_output_ramp){
			output=vel_output_prev-vel_output_ramp*Ts;
		}
	 }

	 // 保存值（为了下一次循环）
	 vel_integral_prev=integral;
	 vel_output_prev=output;
	 vel_error_prev=error;
	 vel_timestamp_prev=timestamp_now;
	 return output;
}

float xos_angle_set_value(float error)
{

	// 计算两次循环中间的间隔时间
	uint32_t timestamp_now = xos_GetSystick();
	float Ts = (timestamp_now - pos_timestamp_prev) * 1e-6f;
	if(Ts <= 0 || Ts > 0.5f) Ts = 1e-3f;

	// P环
	float proportional= vel_P*error;
	// Tustin 散点积分（I环）
	float integral=pos_integral_prev+pos_I*Ts*0.5f*(error+pos_error_prev);
	// D环（微分环节）
	float derivative=pos_D*(error-pos_error_prev)/Ts;

	// 将P,I,D三环的计算值加起来
	float output=proportional+integral+derivative;
	output=_constrain(output,-pos_limit,pos_limit);

	 if(pos_output_ramp>0){
		// 对PID的变化速率进行限制
		float output_rate=(output-pos_output_prev)/Ts;
		if(output_rate>pos_output_ramp){
			output=pos_output_prev+pos_output_ramp*Ts;
		}else if(output_rate<-pos_output_ramp){
			output=pos_output_prev-pos_output_ramp*Ts;
		}
	 }

	 // 保存值（为了下一次循环）
	 pos_integral_prev=integral;
	 pos_output_prev=output;
	 pos_error_prev=error;
	 pos_timestamp_prev=timestamp_now;
	 return output;
}

//低通滤波初始化

//PID
//PIDController vel_loop_M0 = PIDController{.P = 2, .I = 0, .D = 0,  .limit = voltage_power_supply/2};
//PIDController angle_loop_M0 = PIDController{.P = 2, .I = 0, .D = 0,.limit = 100 };




void xos_motor_loop(void)
{
	float angle=getAngle();
	float Kp=0.133;

	angle=LowPassFilter_operator(shaft_angle,&filter);
	float velo_controller_out=PID_xos_operator(angle,&pid_controller);

	//setPhaseVoltage(3,0,_electricalAngle(shaft_angle,7),TIM2);	   //角度闭环
	//setPhaseVoltage(xos_vec_set_value(xos_angle_set_value((0-getAngle())*180/PI)),0,_electricalAngle(0,7),TIM2);	 //角度闭环
	//setPhaseVoltage(0,0,_electricalAngle(0,7),TIM2);

}
