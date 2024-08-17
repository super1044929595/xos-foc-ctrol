#include "xos_current_sensor.h"

void xos_adc_init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* Peripheral clock enable */
    __HAL_RCC_ADC1_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**ADC1 GPIO Configuration
    PA0-WKUP     ------> ADC1_IN0
    PA1     ------> ADC1_IN1
    PA2     ------> ADC1_IN2
    */
    GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  
}

void ADC_Select_CH0 (ADC_HandleTypeDef hadc1)
{
	ADC_ChannelConfTypeDef sConfig = {0};
	  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	  */
	  sConfig.Channel = ADC_CHANNEL_0;
	  sConfig.Rank = 1;
	  //sConfig.SamplingTime = ADC_SAMPLETIME_28CYCLES_5;
	  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	  {
	 
	  }
}

void ADC_Select_CH1 (ADC_HandleTypeDef hadc1)
{
	ADC_ChannelConfTypeDef sConfig = {0};
	  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	  */
	  sConfig.Channel = ADC_CHANNEL_1;
	  sConfig.Rank = 1;
	 // sConfig.SamplingTime = ADC_SAMPLETIME_28CYCLES_5;
	  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	  {
	  
	  }
}

void ADC_Select_CH2 (ADC_HandleTypeDef hadc1)
{
	ADC_ChannelConfTypeDef sConfig = {0};
	  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	  */
	  sConfig.Channel = ADC_CHANNEL_2;
	  sConfig.Rank = 1;
	  //sConfig.SamplingTime = ADC_SAMPLETIME_28CYCLES_5;
	  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	  {
	  
	  }
}
void read_ADC_voltage(ADC_HandleTypeDef hadc1,uint16_t *ADC_VAL){
	ADC_Select_CH0(hadc1);
	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, 1000);
	ADC_VAL[0] = HAL_ADC_GetValue(&hadc1);
	HAL_ADC_Stop(&hadc1);

	ADC_Select_CH1(hadc1);
	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, 1000);
	ADC_VAL[1] = HAL_ADC_GetValue(&hadc1);
	HAL_ADC_Stop(&hadc1);

	ADC_Select_CH2(hadc1);
	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, 1000);
	ADC_VAL[2] = HAL_ADC_GetValue(&hadc1);
	HAL_ADC_Stop(&hadc1);
}
void calibrateOffsets(ADC_HandleTypeDef hadc1,uint16_t *offset){
	const int calibration_rounds = 1000;

	// 查找0电流时候的电压

	uint16_t cal_current[3];
	for (int i = 0; i < calibration_rounds; i++) {
		read_ADC_voltage(hadc1,cal_current);
		for (int j=0;j<3;j++){
			offset[j]=(offset[j]*i+cal_current[j])/(i+1);
		}
		HAL_Delay(1);
	}
}
