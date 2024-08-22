#include "xos_softi2c.h"

//#define I2C_Delay()     HAL_Delay(1)  

#define EEPROM_WR  0x00
#define EEPROM_RD  0x01  
#define EEPROM_DEV_ADDR 0x36
#define EEPROM_WORD_ADDR_SIZE 1

void xos_i2c_delay(void)
{
	for(int i=0;i<1000;i++)
	{
	}
}

#define I2C_Delay()     xos_i2c_delay()

/*
 *  函数名：void I2C_Init(void)
 *  输入参数：
 *  输出参数：无
 *  返回值：无
 *  函数作用：初始化模拟I2C的引脚为输出状态且SCL/SDA都初始为高电平
*/
void xos_I2C_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    SCL_PIN_CLK_EN();
    SDA_PIN_CLK_EN();

    GPIO_InitStruct.Mode      = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull      = GPIO_PULLUP;
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_HIGH;

    GPIO_InitStruct.Pin       = SCL_PIN;
    HAL_GPIO_Init(SCL_PORT, &GPIO_InitStruct);

    GPIO_InitStruct.Pin       = SDA_PIN;
    HAL_GPIO_Init(SDA_PORT, &GPIO_InitStruct);

    SCL_H();
    SDA_H();
}

/*
 *  函数名：static void I2C_SDA_OUT(void)
 *  输入参数：
 *  输出参数：无
 *  返回值：无
 *  函数作用：配置SDA引脚为输出
*/
static void I2C_SDA_OUT(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    GPIO_InitStruct.Mode      = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull      = GPIO_PULLUP;
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_HIGH;

    GPIO_InitStruct.Pin       = SDA_PIN;
    HAL_GPIO_Init(SDA_PORT, &GPIO_InitStruct);
}

/*
 *  函数名：static void I2C_SDA_IN(void)
 *  输入参数：
 *  输出参数：无
 *  返回值：无
 *  函数作用：配置SDA引脚为输入
*/
static void I2C_SDA_IN(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    GPIO_InitStruct.Mode      = GPIO_MODE_INPUT;
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_HIGH;

    GPIO_InitStruct.Pin       = SDA_PIN;
    HAL_GPIO_Init(SDA_PORT, &GPIO_InitStruct);
}



/*
 *  函数名：void I2C_Start(void)
 *  输入参数：
 *  输出参数：无
 *  返回值：无
 *  函数作用：I2C开始信号
*/
void I2C_Start(void)
{
    I2C_SDA_OUT();

    SCL_H();
    I2C_Delay();

    SDA_H();
    I2C_Delay();

    SDA_L();
    I2C_Delay();

    SCL_L();
    I2C_Delay();
}

/*
 *  函数名：void I2C_Stop(void)
 *  输入参数：
 *  输出参数：无
 *  返回值：无
 *  函数作用：I2C停止信号
*/
void I2C_Stop(void)
{
    I2C_SDA_OUT();

    SDA_L();
    I2C_Delay();

    SCL_H();
    I2C_Delay();

    SDA_H();
    I2C_Delay();
}




/*
 *  函数名：void I2C_ACK(void)
 *  输入参数：
 *  输出参数：无
 *  返回值：无
 *  函数作用：I2C发出应答信号
*/
void I2C_ACK(void)
{
    I2C_SDA_OUT();

    SCL_L();
    I2C_Delay();

    SDA_L();
    I2C_Delay();

    SCL_H();
    I2C_Delay();

    SCL_L();
    I2C_Delay();
}

/*
 *  函数名：void I2C_NACK(void)
 *  输入参数：
 *  输出参数：无
 *  返回值：无
 *  函数作用：I2C发出非应答信号
*/
void I2C_NACK(void)
{
    I2C_SDA_OUT();

    SCL_L();
    I2C_Delay();

    SDA_H();
    I2C_Delay();

    SCL_H();
    I2C_Delay();

    SCL_L();
    I2C_Delay();
}

/*
 *  函数名：uint8_t I2C_GetACK(void)
 *  输入参数：
 *  输出参数：无
 *  返回值：1无应答，0有应答
 *  函数作用：I2C等待从机的应答信号
*/
uint8_t I2C_GetACK(void)
{
    uint8_t time = 0;
    I2C_SDA_IN();

    SCL_L();
    I2C_Delay();

    SDA_H();
    I2C_Delay();

    SCL_H();
    I2C_Delay();

    while(SDA_INPUT())
    {
        time++;
        if(time>250)
        {
            SCL_L();
            return 1;
        }
    }
    SCL_L();

    return 0;
}


/*
 *  函数名：void I2C_SendByte(uint8_t data)
 *  输入参数：data->发送的数据
 *  输出参数：无
 *  返回值：无
 *  函数作用：I2C发送一个字节
*/
void I2C_SendByte(uint8_t data)
{
    uint8_t cnt = 0;

    I2C_SDA_OUT();

    for(cnt=0; cnt<8; cnt++)
    {
        SCL_L();
        I2C_Delay();

        if(data & 0x80)
        {
            SDA_H();
        }
        else
        {
            SDA_L();
        }
        data = data<<1;
        SCL_H();
        I2C_Delay();
    }

    SCL_L();
    I2C_Delay();
    I2C_GetACK();
}

/*
 *  函数名：uint8_t I2C_ReadByte(uint8_t ack)
 *  输入参数：ack->发送的应答标志，1应答，0非应答
 *  输出参数：无
 *  返回值：返回读到的字节
 *  函数作用：I2C读出一个字节
*/
uint8_t I2C_ReadByte(uint8_t ack)
{
    uint8_t cnt;
    uint8_t data = 0xFF;

    SCL_L();
    I2C_Delay();

    for(cnt=0; cnt<8; cnt++)
    {
        SCL_H();                 //SCL高(读取数据)
        I2C_Delay();

        data <<= 1;
        if(SDA_INPUT())
        {
            data |= 0x01;        //SDA高(数据为1)
        }
        SCL_L();
        I2C_Delay();
    }
    //发送应答信号，为低代表应答，高代表非应答
    if(ack == 0)
    {
        I2C_ACK();
    }
    else
    {
        I2C_NACK();
    }
    return data;                 //返回数据
} 


/*
 *  函数名：uint8_t EEPROM_WriteByte(uint16_t addr, uint8_t data)
 *  输入参数：addr -> 写一个字节的EEPROM初始地址
 *            data -> 要写的数据
 *  输出参数：无
 *  返回值：无
 *  函数作用：EEPROM写一个字节
*/
void EEPROM_WriteByte(uint16_t addr, uint8_t data)
{
    /* 1. Start */
    I2C_Start();

    /* 2. Write Device Address */
    I2C_SendByte( EEPROM_DEV_ADDR | EEPROM_WR );

    /* 3. Data Address */
    if(EEPROM_WORD_ADDR_SIZE==1)
    {
        I2C_SendByte( (uint8_t)(addr & 0x00FF) );
    }
    else
    {
        I2C_SendByte( (uint8_t)(addr>>8) );
        I2C_SendByte( (uint8_t)(addr & 0x00FF) );
    }

    /* 4. Write a byte */
    I2C_SendByte(data);

    /* 5. Stop */
    I2C_Stop();
}

/*
 *  函数名：uint8_t EEPROM_ReadByte(uint16_t addr, uint8_t *pdata)
 *  输入参数：addr -> 读一个字节的EEPROM初始地址
 *            data -> 要读的数据指针
 *  输出参数：无
 *  返回值：无
 *  函数作用：EEPROM读一个字节
*/
void EEPROM_ReadByte(uint16_t addr, uint8_t *pdata)
{
    /* 1. Start */
    I2C_Start();

    /* 2. Write Device Address */
    I2C_SendByte( EEPROM_DEV_ADDR | EEPROM_WR );

    /* 3. Data Address */
    if(EEPROM_WORD_ADDR_SIZE==1)
    {
        I2C_SendByte( (uint8_t)(addr & 0x00FF) );
    }
    else
    {
        I2C_SendByte( (uint8_t)(addr>>8) );
        I2C_SendByte( (uint8_t)(addr & 0x00FF) );
    }

    /* 4. Start Again */
    I2C_Start();

    /* 5. Write Device Address Read */
    I2C_SendByte( EEPROM_DEV_ADDR | EEPROM_RD );

    /* 6.Read a byte */
    *pdata = I2C_ReadByte(NACK);

    /* 7. Stop */
    I2C_Stop();
} 


/*
 *  函数名：void EEPROM_Write_NBytes(uint16_t addr, uint8_t *pdata, uint16_t sz)
 *  输入参数：addr -> 写一个字节的EEPROM初始地址
 *            data -> 要写的数据指针
 *            sz   -> 要写的字节个数
 *  输出参数：无
 *  返回值：无
 *  函数作用：EEPROM写N个字节
*/
void EEPROM_Write_NBytes(uint16_t addr, uint8_t *pdata, uint16_t sz)
{
    uint16_t i = 0;

    for(i=0; i<sz; i++)
    {
        EEPROM_WriteByte(addr, pdata[i]);
        addr++;
        HAL_Delay(10); // Write Cycle Time 5ms
    }
}

/*
 *  函数名：void EEPROM_Read_NBytes(uint16_t addr, uint8_t *pdata, uint16_t sz)
 *  输入参数：addr -> 读一个字节的EEPROM初始地址
 *            data -> 要读的数据指针
 *            sz   -> 要读的字节个数
 *  输出参数：无
 *  返回值：无
 *  函数作用：EEPROM读N个字节
*/
void EEPROM_Read_NBytes(uint16_t addr, uint8_t *pdata, uint16_t sz)
{
    uint16_t i = 0;

    for(i=0; i<sz; i++)
    {
        EEPROM_ReadByte(addr, &pdata[i]);
        addr++;
    }
} 

