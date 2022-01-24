/*
 * Temp.c
 *
 * Created: 2016/3/9 9:24:31
 *  Author: haiqing
 */ 

#include "main.h"

/*******************************************************************************
* Function Name  : SPI_Init
* Description    : SPI初始化函数
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void SPI_Init(void)
{
	/* Enable SPI, Master, set clock rate fck/16 */
	SPCR = (1<<SPE)|(1<<MSTR)|(1<<SPR0);
}

/*******************************************************************************
* Function Name  : spi_write
* Description    : SPI写数据函数
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void spi_write(unsigned char temp)
{
	SPDR=temp;
	while(!(SPSR&(1<<SPIF)));
}

/*******************************************************************************
* Function Name  : spi_read
* Description    : SPI读函数
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
unsigned char spi_read(void)
{
	unsigned char data;
	SPDR=0x00;
	while(!(SPSR&(1<<SPIF)));
	data=SPDR;
	return data;
}

/*******************************************************************************
* Function Name  : Temp_Read
* Description    : 温度采集子函数
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
float Temp_Read(void)
{
	unsigned int TempVal_temp;
	unsigned char MSByte,LSByte;
	float TempVal,tt;
	TEMPCS_L;
	Delay_ms(10);
	
	MSByte=spi_read();//读高位
	LSByte=spi_read();//读低位
	Delay_ms(10);
	TEMPCS_H;
	
	TempVal_temp = (MSByte<<8) | LSByte;
	TempVal      = (float)TempVal_temp;
	if((TempVal_temp & 0x2000) == 0x2000) tt = (TempVal-16384)/32;
	else 
		tt = (TempVal/32);
	return tt;

}





