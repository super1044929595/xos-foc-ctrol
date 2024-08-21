#include "xos_as5600.h"
#include "main.h"

extern I2C_HandleTypeDef hi2c1;
extern I2C_HandleTypeDef hi2c2;

#define I2C_DEVICE_OBJ  hi2c1

#define PI					3.14159265358979f
#define cpr (float)(2.0f*PI)
#define AS5600_ADDRESS (0x36<<1)
#define Angle_Hight_Register_Addr 0x0C //寄存器高位地址
#define Angle_Low_Register_Addr   0x0D 

void AS5600_Write_Reg(uint16_t reg, uint8_t *value,uint32_t count)
{
	HAL_I2C_Mem_Write(&I2C_DEVICE_OBJ, AS5600_ADDRESS, reg, I2C_MEMADD_SIZE_8BIT, value, count , 200);
}
 
 
//???????
void AS5600_Write_Regs(uint16_t reg, uint8_t *value, uint8_t len)
{
	HAL_I2C_Mem_Write(&I2C_DEVICE_OBJ, AS5600_ADDRESS, reg, I2C_MEMADD_SIZE_8BIT, value, len, 200);
}
 
 
//IIC????
void AS5600_Read_Reg(uint16_t reg, uint8_t* buf, uint8_t len)
{
	HAL_I2C_Mem_Read(&I2C_DEVICE_OBJ, AS5600_ADDRESS, reg,I2C_MEMADD_SIZE_8BIT,(uint8_t*)buf, len, 500);
	//HAL_I2C_Master_Receive(&I2C_DEVICE_OBJ,AS5600_ADDRESS|1, buf,len,500);
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
	uint8_t rawangle_register=0x0C;
	//AS5600_Write_Reg(0x0C, &rawangle_register, 1);
	AS5600_Read_Reg( Angle_Hight_Register_Addr, temp, DATA_SIZE);
	in_angle = ((int16_t)temp[0] <<8) | (temp[1]);
	
	angle_d = (float)in_angle * (2.0f*PI) / 4096;
//angle_d????,???0-6.28	
	return angle_d;
}


float xos_angelupdate(void)
{
	GetAngle_Without_Track();
	return GetAngle();
}



void xos_as5600_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  I2C_DEVICE_OBJ.Instance = I2C1;
  I2C_DEVICE_OBJ.Init.Timing = 0x20404768;
  I2C_DEVICE_OBJ.Init.OwnAddress1 = 0;
  I2C_DEVICE_OBJ.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  I2C_DEVICE_OBJ.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  I2C_DEVICE_OBJ.Init.OwnAddress2 = 0;
  I2C_DEVICE_OBJ.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  I2C_DEVICE_OBJ.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  I2C_DEVICE_OBJ.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  
  if (HAL_I2C_Init(&I2C_DEVICE_OBJ) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&I2C_DEVICE_OBJ, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&I2C_DEVICE_OBJ, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}






#define AS5600_I2C_HANDLE I2C_DEVICE_OBJ


#define I2C_TIME_OUT_BASE   10
#define I2C_TIME_OUT_BYTE   1

#define abs(x) ((x)>0?(x):-(x))
#define _2PI 6.28318530718

#define AS5600_RAW_ADDR    0x36
#define AS5600_ADDR        (AS5600_RAW_ADDR << 1)
#define AS5600_WRITE_ADDR  (AS5600_RAW_ADDR << 1)
#define AS5600_READ_ADDR   ((AS5600_RAW_ADDR << 1) | 1)


#define AS5600_RESOLUTION 4096 //12bit Resolution

#define AS5600_RAW_ANGLE_REGISTER  0x0C

static float angle_data_prev; //上次位置
static float full_rotation_offset; //转过的整圈数

static int i2cWrite(uint8_t dev_addr, uint8_t *pData, uint32_t count) {
  int status;
  int i2c_time_out = I2C_TIME_OUT_BASE + count * I2C_TIME_OUT_BYTE;
  
  status = HAL_I2C_Master_Transmit(&AS5600_I2C_HANDLE, dev_addr, pData, count, i2c_time_out);
  return status;
}

static int i2cRead(uint8_t dev_addr, uint8_t *pData, uint32_t count) {
  int status;
  int i2c_time_out = I2C_TIME_OUT_BASE + count * I2C_TIME_OUT_BYTE;
  
  status = HAL_I2C_Master_Receive(&AS5600_I2C_HANDLE, (dev_addr | 1), pData, count, i2c_time_out);
  return status;
}

uint16_t bsp_as5600GetRawAngle(void) {
  uint16_t raw_angle;
  uint8_t buffer[2] = {0};
  uint8_t raw_angle_register = AS5600_RAW_ANGLE_REGISTER;
  
  i2cWrite(AS5600_ADDR, &raw_angle_register, 1);
  i2cRead(AS5600_ADDR, buffer, 2);
  raw_angle = ((uint16_t)buffer[0] << 8) | (uint16_t)buffer[1];
  return raw_angle;
}

float bsp_as5600GetAngle(void) {
  float angle_data = bsp_as5600GetRawAngle();
  
  float d_angle = angle_data - angle_data_prev;
  if(abs(d_angle) > (0.8 * AS5600_RESOLUTION)) {
    full_rotation_offset += (d_angle > 0 ? -_2PI : _2PI);
  }
  angle_data_prev = angle_data;
  
  return (full_rotation_offset + (angle_data / (float)AS5600_RESOLUTION)*_2PI);
}

