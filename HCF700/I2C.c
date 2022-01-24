/*
 * I2C.c
 *
 * Created: 2016/1/29 14:17:14
 *  Author: haiqing
 */ 
#include "main.h"

// #define TWI_START()			(TWCR=(1<<TWINT)|(1<<TWSTA)|(1<<TWEN))
// #define TWI_STOP()			(TWCR=(1<<TWINT)|(1<<TWSTO)|(1<<TWEN))
// #define TWI_WAITACK()		do{while(!(TWCR&(1<<TWINT)));}while(0)
// #define TWI_CHKACK()		(TWSR&0xf8)
// #define TWI_SENDACK()		(TWCR |= (1<<TWEA))
// 
//#define TWI_SENDNOACK()		(TWCR &= ~(1<<TWEA))
// #define TWI_SENDDATA(x)		do{TWDR=(x);TWCR=(1<<TWINT)|(1<<TWEN);}while(0)
// #define TWI_RcvNckByte()		(TWCR=(1<<TWINT)|(1<<TWEN))
// #define TWI_RcvAckByte()		(TWCR=(1<<TWINT)|(1<<TWEN)|(1<<TWEA))
// 
// #define TWI_STATUS()		(TWSR&0xf8)
// #define TWI_LOADDATA(x)		do{TWDR=(x);TWCR=(1<<TWINT)|(1<<TWEN);}while(0)
// #define TWI_WAIT()			do{while(!(TWCR&(1<<TWINT)));}while(0)
// 

// 

// 
//#define TWI_ReStart()		TWI_Start()
// /
//?TWCR:?控制寄存器，用来控制TWI操作，说明如下：
//?Bit?7-TWINT：中断标志位，Bit?6-TWEA：使能应答位，Bit?5-TWSTA：START状态位????//?Bit?4-TWSTO：STOP状态位，Bit?3-TWWC:?写冲突标志，Bit?2-TWEN：TWI使能位
//?Bit?1-RES：保留，Bit?0-TWIE：中断使能
//?TWSR:?状态寄存器，Bits?7..3:表示了TWI总线的当前状态，读取时需屏蔽低三位的值，Bits?1..0-TWPS:TWI预分频位
//?TWBR:?比特率寄存器，用来设置TWI的工作频率，计算公式为：SCL=CPU频率/(16+2*(TWBR)*4TWPS),TWPS在4的指数位置
//?TWAR:?从机地址寄存器，Bits?7..1:存放从机地址，Bit?0：最低位为广播识别使能位????//?TWDR:?数据寄存器，用来存放接收或要发送的地址和数据

//.....................以下为底层驱动程序代码段..................//
void TWI_init(void)
{
	TWBR = TWBR_INIT;
	TWSR = 0;
	TWCR = _BV(TWEN);   // #define _BV(n) 1<<n
	//TWCR = 0x44;
}

void TWIStart(void)
{
	TWCR = (1<<TWINT)|(1<<TWSTA)|(1<<TWEN);
	while ((TWCR & (1<<TWINT)) == 0);
}
//send stop signal
void TWIStop(void)
{
	TWCR = (1<<TWINT)|(1<<TWSTO)|(1<<TWEN);
	while(TWCR & (1<<TWSTO));
}

void TWIWrite(uint8_t u8data)
{
	TWDR = u8data;
	TWCR = (1<<TWINT)|(1<<TWEN);
	while ((TWCR & (1<<TWINT)) == 0);
}

uint8_t TWIReadACK(void)
{
	TWCR = (1<<TWINT)|(1<<TWEN)|(1<<TWEA);
	while ((TWCR & (1<<TWINT)) == 0);
	return TWDR;
}
//read byte with NACK
uint8_t TWIReadNACK(void)
{
	TWCR = (1<<TWINT)|(1<<TWEN);
	while ((TWCR & (1<<TWINT)) == 0);
	return TWDR;
}

uint8_t TWIGetStatus(void)
{
	uint8_t status;
	//mask status
	status = TWSR & 0xF8;
	return status;
}

//.....................以下为应用程序代码段..................//

/*******************************************************************************
* Function Name  : EEWriteByte
* Description    : EEPROM写一个字节数据
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
uint8_t EEWriteByte(uint16_t u16addr, uint8_t u8data)
{
	TWIStart();
	//if (TWIGetStatus() != 0x08)
	if (    (TWIGetStatus() != 0x08) & (TWIGetStatus() != 0x10)       )
	return ERROR;
	//select devise and send A2 A1 A0 address bits
	//TWIWrite((EEDEVADR)|(uint8_t)((u16addr & 0x0700)>>7));
	TWIWrite(EEDEVADR&0XFE);
	if (TWIGetStatus() != 0x18)
	return ERROR;
	//send the rest of address
	TWIWrite( ((uint8_t)(u16addr>>8)) );
	if (TWIGetStatus() != 0x28)
	return ERROR;
	TWIWrite( ((uint8_t)(u16addr)) );
	if (TWIGetStatus() != 0x28)
	return ERROR;
	//write byte to eeprom
	TWIWrite(u8data);
	if (TWIGetStatus() != 0x28)
	return ERROR;
	TWIStop();
	Delay_ms(5);    //必须加大于2ms的延时，等待I2C操作完成
	return SUCCESS;
}

/*******************************************************************************
* Function Name  : EEReadByte
* Description    : EEPROM读一个字节数据
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
uint8_t EEReadByte(uint16_t u16addr, uint8_t *u8data)
{
	//uint8_t databyte;
	TWIStart();
	//if (TWIGetStatus() != 0x08)
	if (    (TWIGetStatus() != 0x08) & (TWIGetStatus() != 0x10)       )
	return ERROR;
	//select devise and send A2 A1 A0 address bits
	//TWIWrite((EEDEVADR)|((uint8_t)((u16addr & 0x0700)>>7)));
	TWIWrite(EEDEVADR&0XFE);
	//if (TWIGetStatus() != 0x18)
	if (    (TWIGetStatus() != 0x18) &  (TWIGetStatus() != 0x20)   )
	return ERROR;
	//send the rest of address
	TWIWrite( ((uint8_t)(u16addr>>8)) );
	//if (TWIGetStatus() != 0x28)
	if (    (TWIGetStatus() != 0x28) &  (TWIGetStatus() != 0x30)     )
	return ERROR;
	TWIWrite( ((uint8_t)(u16addr)) );
	//if (TWIGetStatus() != 0x28)
	if (    (TWIGetStatus() != 0x28) &  (TWIGetStatus() != 0x30)     )
	return ERROR;
	//send start
	TWIStart();
	//if (TWIGetStatus() != 0x10)
	if (    (TWIGetStatus() != 0x08) & (TWIGetStatus() != 0x10)       )
	return ERROR;
	//select devise and send read bit
	//TWIWrite((EEDEVADR)|((uint8_t)((u16addr & 0x0700)>>7))|1);
	TWIWrite(EEDEVADR|0X01);
	if (TWIGetStatus() != 0x40)
	return ERROR;
	*u8data = TWIReadNACK();
	if (TWIGetStatus() != 0x58)
	return ERROR;
	TWIStop();
	//Delay_ms(5);
	return SUCCESS;
}

/*******************************************************************************
* Function Name  : EEWriteOnePageNByte
* Description    : EEPROM向某一特定页写若干个数据，长度小于页字节数
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
// unsigned char  EEWriteOnePageNByte(unsigned char addr,unsigned char *pdata,unsigned char length)
// {
// 	unsigned char i;
// 	if(length>32)return ERROR;
// 	for(i=0;i<length;i++)
// 		EEWriteByte(addr++,*pdata++);
// 	return SUCCESS;
// }

/*******************************************************************************
* Function Name  : EEReadOnePageNByte
* Description    : EEPROM向某一特定页读若干个数据，长度小于页字节数
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
// unsigned char  EEReadOnePageNByte(unsigned char addr,unsigned char *pdata,unsigned char length)
// {
// 	unsigned char i;
// 	if(length>32)return ERROR;
// 	for(i=0;i<length;i++)
// 		EEReadByte(addr++,pdata++);
// 	return SUCCESS;
// }

/*******************************************************************************
* Function Name  : EEWritePage
* Description    : EEPROM写特定一页数据，
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
uint8_t EEWritePage(unsigned short addr, unsigned char *pdata,unsigned char length)
{
	//calculate page address
	TWIStart();
	if (TWIGetStatus() != 0x08)
	return ERROR;
	//select page start address and send A2 A1 A0 bits send write command
	TWIWrite(EEDEVADR&0XFE);
	if (TWIGetStatus() != 0x18)
	return ERROR;
	//send the rest of address
	TWIWrite( ((uint8_t)(addr>>8)) );
	if (TWIGetStatus() != 0x28)
	return ERROR;
	TWIWrite( ((uint8_t)(addr)) );
	if (TWIGetStatus() != 0x28)
	return ERROR;
	
	//write page to eeprom
	while(length--)
	{
		TWIWrite(*pdata++);
		if (TWIGetStatus() != 0x28)
		return ERROR;
	}
	TWIStop();
	Delay_ms(5);
	return SUCCESS;
}

/*******************************************************************************
* Function Name  : EEWrite
* Description    : EEPROM多字节写函数，可以跨页连续写
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
uint8_t EEWrite(unsigned short addr, unsigned char *pbuff,unsigned short length)
{
	unsigned char temp;
	temp=addr%I2C_PAGESIZE;
	if(temp)
	{
		temp=I2C_PAGESIZE-temp;
		if(length>=temp)
		{
			EEWritePage(addr,pbuff,temp);
			length -= temp;
			addr += temp;
			pbuff += temp;
		}
		else
		{
			if(EEWritePage(addr,pbuff,length) == ERROR)return ERROR;
			length=0;
		}
	}
	
	while(length)
	{
		if(length>=I2C_PAGESIZE)
		{
			if(EEWritePage(addr,pbuff,I2C_PAGESIZE) == ERROR)return ERROR;
			length-=I2C_PAGESIZE;
			addr+=I2C_PAGESIZE;
			pbuff+=I2C_PAGESIZE;
		}
		else
		{
			if(EEWritePage(addr,pbuff,length) == ERROR)return ERROR;
			length=0;
		}
	}
	return SUCCESS;
}

/*******************************************************************************
* Function Name  : EEWrite
* Description    : EEPROM多字节写函数，可以跨页连续写
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
uint8_t EEWrite_E(unsigned short addr, unsigned char *pbuff,unsigned short length)
{
	unsigned char temp;
	temp=addr%I2C_PAGESIZE;
	if(temp)
	{
		temp=I2C_PAGESIZE-temp;
		if(length>=temp)
		{
			EEWritePage(addr,pbuff,temp);
			length -= temp;
			addr += temp;
			//pbuff += temp;
		}
		else
		{
			if(EEWritePage(addr,pbuff,length) == ERROR)return ERROR;
			length=0;
		}
	}
	
	while(length)
	{
		if(length>=I2C_PAGESIZE)
		{
			if(EEWritePage(addr,pbuff,I2C_PAGESIZE) == ERROR)return ERROR;
			length-=I2C_PAGESIZE;
			addr+=I2C_PAGESIZE;
			//pbuff+=I2C_PAGESIZE;
		}
		else
		{
			if(EEWritePage(addr,pbuff,length) == ERROR)return ERROR;
			length=0;
		}
	}
	return SUCCESS;
}

/*******************************************************************************
* Function Name  : EERead
* Description    : EEPROM多字节读函数，可以跨页连续读
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
uint8_t EERead(unsigned short addr, unsigned char *pbuff,unsigned short length)
{
	//calculate page address
	TWIStart();
	if (TWIGetStatus() != 0x08)
	return ERROR;
	//select page start address and send A2 A1 A0 bits send write command
	TWIWrite(EEDEVADR&0XFE);
	if (TWIGetStatus() != 0x18)
	return ERROR;
	//send the rest of address
	TWIWrite( ((uint8_t)(addr>>8)) );
	if (TWIGetStatus() != 0x28)
	return ERROR;
	TWIWrite( ((uint8_t)(addr)) );
	if (TWIGetStatus() != 0x28)
	return ERROR;
	//send start
	TWIStart();
	if (TWIGetStatus() != 0x10)
	return ERROR;
	//select devise and send read bit
	TWIWrite(((EEDEVADR)|0X01));
	if (TWIGetStatus() != 0x40)
	return ERROR;
	while(length)
	{
		if(length==1)
		{
			*pbuff=TWIReadNACK();
			if (TWIGetStatus() != 0x58)
			return ERROR;
			TWIStop();
			return SUCCESS;
		}
		*pbuff++ = TWIReadACK();
		if (TWIGetStatus() != 0x50)
		return ERROR;
		length--;
	}
	return SUCCESS;
}


/*******************************************************************************
* Function Name  : EEErase
* Description    : EEPROM擦除函数，写0
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void EEErase(unsigned short addr, unsigned short length)
{
	unsigned char Data_temp[512]={0};
	EEWrite_E(addr,&Data_temp,length);
}
