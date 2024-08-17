#include "xos_as5048a.h"
#include "stm32f7xx_hal.h"


extern SPI_HandleTypeDef hspi2;

void xos_as5048a_init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  {
  /* USER CODE BEGIN SPI1_MspInit 0 */

  /* USER CODE END SPI1_MspInit 0 */
    /* Peripheral clock enable */
    __HAL_RCC_SPI1_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**SPI1 GPIO Configuration
    PA5     ------> SPI1_SCK
    PA6     ------> SPI1_MISO
    PA7     ------> SPI1_MOSI
    */
    GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_6;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN SPI1_MspInit 1 */

  /* USER CODE END SPI1_MspInit 1 */
  }
  hspi2.Instance = SPI1;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_16BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
   
  }
  /* USER CODE BEGIN SPI1_Init 2 */
  /* USER CODE END SPI1_Init 2 */
}

uint8_t spiCalcEvenParity(uint16_t value){
	uint8_t cnt = 0;
	uint8_t i;

	for (i = 0; i < 16; i++)
	{
		if (value & 0x1)
		{
			cnt++;
		}
		value >>= 1;
	}
	return cnt & 0x1;
}

uint16_t read(SPI_HandleTypeDef* _spi, GPIO_TypeDef* _ps, uint16_t _cs,uint16_t registerAddress)
{

	uint8_t send_data[2];
	uint8_t recv_data[2];
//	uint16_t data2;
	uint16_t command = 0b0100000000000000; // PAR=0 R/W=R
	command = command | registerAddress;

	//Add a parity bit on the the MSB
	command |= ((uint16_t)spiCalcEvenParity(command)<<15);

	//Split the command into two bytes
	send_data[1] = command & 0xFF;
	send_data[0] = ( command >> 8 ) & 0xFF;

	EN_SPI;
	HAL_SPI_Transmit(_spi, (uint8_t *)&send_data, 2, 0xFFFF);
//	HAL_SPI_Transmit(_spi, (uint8_t *)&command, 1, 0xFFFF);
	while (HAL_SPI_GetState(_spi) != HAL_SPI_STATE_READY) {}
	DIS_SPI;
	send_data[0]=0x00;
	send_data[1]=0x00;
	EN_SPI;
	 HAL_SPI_TransmitReceive(_spi,(uint8_t*)&send_data,(uint8_t*)&recv_data,2, 0xFFFF);
//	HAL_SPI_Receive(_spi, (uint8_t *)&recv_data, 2, 0xFFFF);
//	HAL_SPI_Receive(_spi, (uint8_t *)&data2, 1, 0xFFFF);
	while (HAL_SPI_GetState(_spi) != HAL_SPI_STATE_READY) {}
	DIS_SPI;

//	if (recv_data[1] & 0x40) {
//		errorFlag = 1;
//	} else {
//		errorFlag = 0;
//	}

	//Return the data, stripping the parity and error bits
	return (( ( recv_data[1] & 0xFF ) << 8 ) | ( recv_data[0] & 0xFF )) & ~0xC000;
//	return data2 & ~0xC000;
}

