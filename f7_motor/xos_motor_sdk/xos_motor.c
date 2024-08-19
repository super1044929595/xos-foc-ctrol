#include "main.h"
#include "xos_motor.h"
#include "stm32f7xx_hal.h"
#include "stm32f7xx_hal_tim.h"
#include "stdio.h"
#include "stdlib.h"
#include "math.h"
#include "xos_as5600.h"
//#include "xos_pid.hpp"



void xos_Foc_SetVol(float uq, float angle);
void xos_Foc_SetPwm( float ua, float ub,float  uc);
float _electricalAngle(float shaft_angle, int pole_pairs);
float _normalizeAngle(float angle);

//#define _constrain(x,a,b)      ((x<a)?a:(x<b?x:b))
//#define voltage_power_supply 12
//#define PI 3.1415926
extern TIM_HandleTypeDef htim2;
static int32_t xos_inter=0;
static int32_t xos_pwm_enable=0;
static uint32_t xos_SysTick_CNt=0;
float zero_electric_angle=0;
float shaft_angle=0;
float voltage_limit=5;
float voltage_power_supply=5;

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
  htim2.Init.Prescaler = 36-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 200-1;
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
  /* USER CODE BEGIN TIM2_Init 2 */

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

}

uint32_t xos_GetSystick(void)
{
	return xos_SysTick_CNt;
}

void xo_SetSysTick(void)
{
	++xos_SysTick_CNt;

	if(xos_SysTick_CNt%120==0){
		velocityOpenloop(2,voltage_power_supply/3,TIM2);
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
  return _normalizeAngle(((float)(dir * pole_pairs)*shaft_angle)-zero_electric_angle);
}



//开环速度函数
uint32_t xos_time_monitor_time=0;
float velocityOpenloop(float target_velocity, float Uq, TIM_TypeDef * TIM_BASE)
{
	static uint32_t pre_us=0;

	uint32_t now_us = xos_GetSystick();
	//Provides a tick value in microseconds.
	//计算当前每个Loop的运行时间间隔
	float Ts = (float)(now_us - pre_us) ;
	xos_time_monitor_time=(uint32_t)Ts;
	Ts=Ts*1e-6f;//5E-3f;
	// 通过乘以时间间隔和目标速度来计算需要转动的机械角度，存储在 shaft_angle 变量中。
	//在此之前，还需要对轴角度进行归一化，以确保其值在 0 到 2π 之间。
	shaft_angle = _normalizeAngle(shaft_angle + target_velocity*Ts);
	//以目标速度为 10 rad/s 为例，如果时间间隔是 1 秒，则在每个循环中需要增加 10 * 1 = 10 弧度的角度变化量，才能使电机转动到目标速度。
	//如果时间间隔是 0.1 秒，那么在每个循环中需要增加的角度变化量就是 10 * 0.1 = 1 弧度，才能实现相同的目标速度。
	//因此，电机轴的转动角度取决于目标速度和时间间隔的乘积。
	// Uq is not related to voltage limit
	setPhaseVoltage(voltage_power_supply/3,  0, _electricalAngle(shaft_angle,7),TIM_BASE);
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
	//Ua = _constrain(Ua/, 0.0f, voltage_limit);
	//Ub = _constrain(Ub, 0.0f, voltage_limit);
	//Uc = _constrain(Uc, 0.0f, voltage_limit);
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
	
	TIM2->CCR1 = (uint32_t) roundf(dc_a*200);
	TIM2->CCR3 = (uint32_t) roundf(dc_c*200);	
	TIM2->CCR4 = (uint32_t) roundf(dc_b*200);
	
	xos_pwma=TIM2->CCR1;
	xos_pwmb=TIM2->CCR4;
	xos_pwmc=TIM2->CCR3;
}

//---------------------------------------------------------------------------------------------

float xos_ua_ori=0;
float xos_ub_ori=0;
float xos_uc_ori=0;

void setPhaseVoltage(float Uq,float Ud, float angle_el, TIM_TypeDef * TIM_BASE) {

	angle_el = _normalizeAngle(angle_el + zero_electric_angle);
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


static int xossi=0;
float xos_jwangle=0;
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef * htim)
{
	//xos_sg90_update();
	//__HAL_TIM_SetCompare(&htim2,TIM_CHANNEL_3,100);
	//__HAL_TIM_SetCompare(&htim2,TIM_CHANNEL_4,100);

	//xos_PeriodElapsedCallback();
	//velocityOpenloop(2,5,TIM2);
	
	GetAngle_Without_Track();
	xos_jwangle=GetAngle();
}

