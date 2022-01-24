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
//?TWCR:?���ƼĴ�������������TWI������˵�����£�
//?Bit?7-TWINT���жϱ�־λ��Bit?6-TWEA��ʹ��Ӧ��λ��Bit?5-TWSTA��START״̬λ????//?Bit?4-TWSTO��STOP״̬λ��Bit?3-TWWC:?д��ͻ��־��Bit?2-TWEN��TWIʹ��λ
//?Bit?1-RES��������Bit?0-TWIE���ж�ʹ��
//?TWSR:?״̬�Ĵ�����Bits?7..3:��ʾ��TWI���ߵĵ�ǰ״̬����ȡʱ�����ε���λ��ֵ��Bits?1..0-TWPS:TWIԤ��Ƶλ
//?TWBR:?�����ʼĴ�������������TWI�Ĺ���Ƶ�ʣ����㹫ʽΪ��SCL=CPUƵ��/(16+2*(TWBR)*4TWPS),TWPS��4��ָ��λ��
//?TWAR:?�ӻ���ַ�Ĵ�����Bits?7..1:��Ŵӻ���ַ��Bit?0�����λΪ�㲥ʶ��ʹ��λ????//?TWDR:?���ݼĴ�����������Ž��ջ�Ҫ���͵ĵ�ַ������

//.....................����Ϊ�ײ�������������..................//
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

//.....................����ΪӦ�ó�������..................//

/*******************************************************************************
* Function Name  : EEWriteByte
* Description    : EEPROMдһ���ֽ�����
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
	Delay_ms(5);    //����Ӵ���2ms����ʱ���ȴ�I2C�������
	return SUCCESS;
}

/*******************************************************************************
* Function Name  : EEReadByte
* Description    : EEPROM��һ���ֽ�����
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
* Description    : EEPROM��ĳһ�ض�ҳд���ɸ����ݣ�����С��ҳ�ֽ���
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
* Description    : EEPROM��ĳһ�ض�ҳ�����ɸ����ݣ�����С��ҳ�ֽ���
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
* Description    : EEPROMд�ض�һҳ���ݣ�
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
* Description    : EEPROM���ֽ�д���������Կ�ҳ����д
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
* Description    : EEPROM���ֽ�д���������Կ�ҳ����д
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
* Description    : EEPROM���ֽڶ����������Կ�ҳ������
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
* Description    : EEPROM����������д0
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void EEErase(unsigned short addr, unsigned short length)
{
	unsigned char Data_temp[512]={0};
	EEWrite_E(addr,&Data_temp,length);
}
