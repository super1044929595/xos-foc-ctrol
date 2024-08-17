#include "xos_as5600.h"

//extern I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c1;

#define PI					3.14159265358979f
#define cpr (float)(2.0f*PI)
#define AS5600_ADDRESS 0x36<<1
#define Angle_Hight_Register_Addr 0x0C //寄存器高位地址
#define Angle_Low_Register_Addr   0x0D 

void AS5600_Write_Reg(uint16_t reg, uint8_t *value)
{
	HAL_I2C_Mem_Write(&hi2c1, AS5600_ADDRESS, reg, I2C_MEMADD_SIZE_8BIT, value, 1, 50);
}
 
 
//???????
void AS5600_Write_Regs(uint16_t reg, uint8_t *value, uint8_t len)
{
	HAL_I2C_Mem_Write(&hi2c1, AS5600_ADDRESS, reg, I2C_MEMADD_SIZE_8BIT, value, len, 50);
}
 
 
//IIC????
void AS5600_Read_Reg(uint16_t reg, uint8_t* buf, uint8_t len)
{
	HAL_I2C_Mem_Read(&hi2c1, AS5600_ADDRESS, reg, I2C_MEMADD_SIZE_8BIT, buf, len, 50);
}


 
float angle_prev1=0; 
int full_rotations=0; // full rotation tracking;
float angle_d;				//GetAngle_Without_Track()????
float angle_cd;				//GetAngle()????
 

#define DATA_SIZE 2
                        


int myabs(int n)
{

return n * ( (n>>31<<1) +1);

}
//???????????
float GetAngle(void)
{
	float val = angle_d;
	float d_angle = val - angle_prev1;
	//????????
	//????????????80%???(0.8f*6.28318530718f)??????????,?????,??full_rotations??1(??d_angle??0)???1(??d_angle??0)?
	if(myabs(d_angle) > (0.8f*2.0f*PI) ) full_rotations += ( d_angle > 0 ) ? -1 : 1; 
	angle_prev1 = val;

	angle_cd = full_rotations * (2.0f*PI) + angle_prev1;
	return angle_cd;
	//    return (float)full_rotations * 6.28318530718f + angle_prev;
}

//????????,???0-6.28
float GetAngle_Without_Track(void)
{   
	int16_t in_angle;
	uint8_t temp[DATA_SIZE]={0};
	AS5600_Read_Reg( Angle_Hight_Register_Addr, temp, DATA_SIZE);
	in_angle = ((int16_t)temp[0] <<8) | (temp[1]);
	
	angle_d = (float)in_angle * (2.0f*PI) / 4096;
//angle_d????,???0-6.28	
	return angle_d;
}


void xos_as5600_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x20404768;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}
