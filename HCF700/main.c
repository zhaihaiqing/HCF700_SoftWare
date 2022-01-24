/*
 * DIS100.c
 *
 * Created: 2016/1/28 13:55:54
 *  Author: haiqing
 */ 

#include "main.h"
// Target : M328P
// Crystal: 7.3728Mhz
 int usart_putchar_printf(char var, FILE *stream);
 static FILE mystdout = FDEV_SETUP_STREAM(usart_putchar_printf, NULL, _FDEV_SETUP_WRITE);
 


unsigned char  FactoryResetWord[4]    = {0x33,0x55,0x77,0x99};//��������ָ�����
unsigned char  FactorySetValueWord[4] = {0x44,0x66,0x88,0xaa};//��������ָ�����

unsigned char  FactorySetValueWord2[4] = {0x40,0x66,0x88,0xaa};//��������ָ�����
unsigned char  SuperModeValueWord2[4]  = {0x40,0x60,0x80,0xa0};//�߼�ģʽ����
volatile unsigned int   WorkTime = 0;			//����ʱ���ʱ�����900S����������Ĵ����е�ϵͳ����ʱ��
volatile unsigned char  WorkTimeOutFlag = 0;	//����ʱ�������־λ������900Sδ�յ�ָ���1��λ��ͬʱ����ι�����ȴ���λ
volatile unsigned char  Temp_Flag=1;            //�¶Ȳ�����־λ��5s��һ��
volatile unsigned int   Temp_Count=0;			//�¶Ȳ���������

unsigned char SuperMode_Flag	= 0;

		
const uint8_t __progmem_smartbow_start[64] PROGMEM = {"&&SUPERHHH111401&&2"};

//unsigned char  WorkStatus=0;					//ϵͳ����״̬
//01,��ʼ����ɣ�δ���빤��״̬��02���յ�����ָ�crc�ɹ���03���յ��㲥ָ�04���յ��������ݣ�������ַ
//05���յ��������ݣ��Ǳ�����ַ�ҷǹ㲥ָ�06���յ���

volatile KeepRegister_t		 KeepRegister;		//���屣�ּĴ���
volatile KeepRegister_t		 KeepRegisterTemp;	//���屣�ּĴ�������
volatile InputRegister_t	 InputRegister;		//��������Ĵ���
volatile InputRegister_t	 InputRegisterTemp;	//��������Ĵ�������
volatile ModbusDataPackage_t ModbusDataPackage;	//����modbus���ջ���

/*******************************************************************************
* Function Name  : Delay_ms
* Description    : ms����ʱ������ʹ�ö�ʱ������
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void Delay_ms(unsigned int ms)
{
	_MS=ms;
	while(_MS);
}

/*******************************************************************************
* Function Name  : WatchDog_Init
* Description    : ���Ź���ʼ������
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void WatchDog_Init(void)
{
	_WDR();
	WDTCSR |= (1<<WDCE) | (1<<WDE); //�򿪻�رտ��Ź�ʱ�����뽫WDCEλ��1
	WDTCSR = (1<<WDE)|(1<<WDP3);    //enable wdt,clk = 512��4s
	//WDTCSR = (1<<WDE)|(1<<WDP0)|(1<<WDP3);    //enable wdt,clk = 1024��8s
}

/*******************************************************************************
* Function Name  : __ltobf
* Description    : float�����ݴ�С�˸�ʽת��
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
static float __ltobf(float data)
{
	FLOAT_CONV d1, d2;

	d1.f = data;

	d2.c[0] = d1.c[3];
	d2.c[1] = d1.c[2];
	d2.c[2] = d1.c[1];
	d2.c[3] = d1.c[0];
	return d2.f;
}

/*******************************************************************************
* Function Name  : Port_Init
* Description    : ��ʼ��IO
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void Port_Init(void)
{
	DDRC  = (1<<1) | (1<<2) | (1<<3);	//---<<< ADS_SCK��ADS_CS#��TEMP_CS# >>>---set those pin output mode
	PORTC = (1<<2)|(1<<3);			    //SET ADS_CS#��TEMP_CS#
	
	DDRD = (1<<1)|(1<<2)|(1<<5);//---<<< TXD��485RW��SenPow >>>---set those pin output mode
	PORTD = 0x00;
	
	DDRB  = (1<<0)|(1<<1)|(1<<2)|(1<<3)|(1<<5);//<LED1��LED2��SS��MOSI��SCK>   set those pin output mode
	PORTB |=(1<<0)|(1<<1);
	LED1_OFF;
}

/*******************************************************************************
* Function Name  : Init_Devices
* Description    : ��ʼ��Ӳ������
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void Init_Devices(void)
{
	cli(); //disable all interrupts
	Port_Init();
	uart0_init();
	timer1_init();
	TWI_init();
	SPI_Init();
	WatchDog_Init();
	
	MCUCR  = 0x00;
	TIMSK1 = 0x01; //timer interrupt sources
	PCMSK0 = 0x00;
	PCMSK1 = 0x00;
	PCMSK2 = 0x00;
	EIMSK  = 0x00;
	EICRA  = 0x00; //extended ext ints
	PCICR  = 0x00;
	
	sei();//enable all interrupts
	SenPow_ON;
	Delay_ms(200);
	ADS8325_Start();
	U485RX;Delay_ms(300);
}
// /*******************************************************************************
// * Function Name  : main
// * Description    : ������
// * Input          : None
// * Output         : None
// * Return         : None
// *******************************************************************************/
char Init_Parameter(void)
{
	uint8_t temp[2];
	//��ʼ�����ּĴ�������
	//EEErase(0,512);
	EERead(FPOWERONFLAG_BASEADDR,temp,2);
	if( (temp[0] != 0x12) && (temp[1] != 0x34) )	//����Ƿ����״ο���
	{
		EERead(KREEPROM_BASEADDR,(void *)&KeepRegister,sizeof(KeepRegister));//��ȡEEPROM�д洢�ı��ּĴ���ֵ���ָ����¼���
		KeepRegister.DeviceAddress              = DefaultDeviceADDR;
		KeepRegister.OriginaLiquidAltitude      = 0;
		KeepRegister.Reserve1and2              = DefaultLiquidDensity;
		KeepRegister.Liquid_Sec                = 0x0100;
		KeepRegister.LocalAccelerationOfGravity = DefaultAccelerationOfGravity;
		KeepRegister.DeviceGroupNum = 0;
		KeepRegister.Sensor_Range = DefaultSensor_Range;
		
		EEWrite(KREEPROM_BASEADDR,(void *)&KeepRegister,sizeof(KeepRegister));//����EEPROM
		
		temp[0] = 0x34;
		temp[1] = 0x12;
		EEWrite(SUPERMODE_FLAG_BASEADDR,temp,2);
		SuperMode_Flag=1;										//�״ο�������ģʽΪ����ģʽ
		InputRegister.SystemWorkStatus=(InputRegister.SystemWorkStatus & 0x00ff)|0x0100;
		
		temp[0] = 0x12;
		temp[1] = 0x34;
		EEWrite(FPOWERONFLAG_BASEADDR,temp,2);
	}
	
	temp[0]=0;
	temp[1]=0;
	EERead(SUPERMODE_FLAG_BASEADDR,temp,2);
	if((temp[0] == 0x34) && (temp[1] == 0x12) )
	{
		SuperMode_Flag=1;		//���ݱ�־λ����ģʽ�ж�
		InputRegister.SystemWorkStatus=(InputRegister.SystemWorkStatus & 0x00ff)|0x0100;
	}
	else 
	{
		SuperMode_Flag=0;
		InputRegister.SystemWorkStatus=(InputRegister.SystemWorkStatus & 0x00ff)|0x0200;
	}
	
	EERead(KREEPROM_BASEADDR,(void *)&KeepRegister,sizeof(KeepRegister));
	InputRegister.DeviceType      = DEVICETYPE;
	InputRegister.SoftwareVersion = SOFTWAREVERSION;
	
	return SUCCESS;
}

/*******************************************************************************
* Function Name  : ADCSmaple
* Description    : ADC��������
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
#define ADCBUFF_LENGTH 100
float ADCSmaple(void)
{
	float ADS_Result=0;
	uint16_t ADS_buff[ADCBUFF_LENGTH]={0};
	uint16_t buff=0;
	uint32_t ADCsum=0;
	unsigned char i,j;
	for(i=0;i<ADCBUFF_LENGTH;i++)
	{
		ADS_buff[i]= ADS8325_GetData();
	}
	
	//ð�����򣬲���11�Σ���С��������ȡ�м�5���������ֵ
	for(i=0;i<ADCBUFF_LENGTH;i++)
	{
		for(j=i+1;j<ADCBUFF_LENGTH;j++)
		{
			if(ADS_buff[i]>ADS_buff[j])//��С���󣬸�Ϊ"<"��Ϊ�Ӵ�С
			{
				buff=ADS_buff[i];
				ADS_buff[i]=ADS_buff[j];
				ADS_buff[j]=buff;
			}
		}
	}
	
	ADCsum=0;
	for(i=39;i<59;i++)
	{
		ADCsum+=ADS_buff[i];
	}
	ADS_Result=1.0*ADCsum/20;
	ADS_Result= ADS_Result*4.096/65536;//δ����ʱ�²���Ƶ��5.15Khz,��20��nop�󣬲���Ƶ��1.333K
	ADS_Result= (ADS_Result-0.5)*1000/(52.4/2.4);    //��ȥ0.5V��׼���������棬��ת��ΪmV
	InputRegister.ADCOriginalValue=ADS_Result;

	return ADS_Result;
}

/*******************************************************************************
* Function Name  : CRC16_Check
* Description    : CRCУ��
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
uint16_t CRC16_Check(uint8_t *Pushdata,uint8_t length)
{
	uint16_t Reg_CRC=0xffff;
	uint8_t i,j;
	for( i = 0; i<length; i ++)
	{
		Reg_CRC^= *Pushdata++;
		for (j = 0; j<8; j++)
		{
			if (Reg_CRC & 0x0001)

			Reg_CRC=Reg_CRC>>1^0xA001;
			else
			Reg_CRC >>=1;
		}
	}
	return   Reg_CRC;
}

/*******************************************************************************
* Function Name  : ModbusReturnAckInfo
* Description    : modbus�����쳣����Ϣ
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void ModbusReturnAckInfo(uint8_t err)
{
	uint8_t ErrDat[5];
	uint16_t crc;
	if(err)//�쳣��
	{
		ErrDat[0] = KeepRegister.DeviceAddress;//��ֵ�豸��ַ
		ErrDat[1] = ModbusDataPackage.dat[1] | 0x80;//��ֵ�쳣������,��������+0x80
		ErrDat[2] = err;//��ֵ�쳣��
		crc = CRC16_Check(ErrDat,3);
		ErrDat[3] = (crc & 0xff);//У���8λ
		ErrDat[4] = (crc >> 8);//У���8λ
		
		U485SendData(ErrDat,5);//��485��������
	}
}

/*******************************************************************************
* Function Name  : KeepRegistorDataHton
* Description    : ���ּĴ������ݴ�С��ת��
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void KeepRegistorDataHton(void)
{
	uint8_t i;
	KeepRegisterTemp.DeviceAddress =htons(KeepRegister.DeviceAddress);
	KeepRegisterTemp.DeviceGroupNum      =htons(KeepRegister.DeviceGroupNum);
	KeepRegisterTemp.Sensor_Range      =htons(KeepRegister.Sensor_Range);
	KeepRegisterTemp.Liquid_Sec      =htons(KeepRegister.Liquid_Sec);
	KeepRegisterTemp.OriginaLiquidAltitude =__ltobf(KeepRegister.OriginaLiquidAltitude);
	KeepRegisterTemp.Reserve1and2 =__ltobf(KeepRegister.Reserve1and2);
	KeepRegisterTemp.LocalAccelerationOfGravity =__ltobf(KeepRegister.LocalAccelerationOfGravity);
	KeepRegisterTemp.Sensor_FS_Val =__ltobf(KeepRegister.Sensor_FS_Val);
	for(i=0;i<6;i++)KeepRegisterTemp.MV[i] = __ltobf(KeepRegister.MV[i]);
	KeepRegisterTemp.Temp_T0 = __ltobf(KeepRegister.Temp_T0);
	for(i=0;i<5;i++)KeepRegisterTemp.LTC0[i] = __ltobf(KeepRegister.LTC0[i]);
	for(i=0;i<5;i++)KeepRegisterTemp.LTC1[i] = __ltobf(KeepRegister.LTC1[i]);
	for(i=0;i<5;i++)KeepRegisterTemp.LTC2[i] = __ltobf(KeepRegister.LTC2[i]);
	for(i=0;i<5;i++)KeepRegisterTemp.LTC3[i] = __ltobf(KeepRegister.LTC3[i]);
	for(i=0;i<5;i++)KeepRegisterTemp.LTC4[i] = __ltobf(KeepRegister.LTC4[i]);
	for(i=0;i<5;i++)KeepRegisterTemp.LTC5[i] = __ltobf(KeepRegister.LTC5[i]);	
}

/*******************************************************************************
* Function Name  : InputRegistorDataHton
* Description    : ����Ĵ������ݴ�С��ת��
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void InputRegistorDataHton(void)
{
	InputRegisterTemp.DeviceType =htons(InputRegister.DeviceType);
	InputRegisterTemp.SoftwareVersion =htons(InputRegister.SoftwareVersion);
	InputRegisterTemp.SystemWorkTime =htonl(InputRegister.SystemWorkTime);
	InputRegisterTemp.SystemWorkStatus =htons(InputRegister.SystemWorkStatus);
	InputRegisterTemp.AltitudeDifference =__ltobf(InputRegister.AltitudeDifference);
	InputRegisterTemp.Temperature =__ltobf(InputRegister.Temperature);
	InputRegisterTemp.LiquidAltitudeAbsoluteValue =__ltobf(InputRegister.LiquidAltitudeAbsoluteValue);
	InputRegisterTemp.ADCOriginalValue =__ltobf(InputRegister.ADCOriginalValue);
	InputRegisterTemp.PA               =__ltobf(InputRegister.PA);
}

/********************************
�����ּĴ���
���ݸ�ʽ:������ַ(1�ֽ�)+������(0x03)+��ʼ��ַ(2�ֽ�)+��ȡ�Ĵ�������(2�ֽ�)+У��(2�ֽ�)
���ظ�ʽ:������ַ(1�ֽ�)+������(0x03)+���ݳ���(�ֽ���,1�ֽ�)+����(n�ֽ�)+У��(2�ֽ�)
*********************************/
char ModbusReadKeepRegistor(void)
{
	unsigned char err = 0;
	uint8_t temp[180];
	uint16_t crc;
	
 	uint16_t StartAddress = (ModbusDataPackage.dat[2] << 8) | ModbusDataPackage.dat[3]; //��ȡ��ʼ��ַ
 	uint16_t RegVal = (ModbusDataPackage.dat[4] << 8) | ModbusDataPackage.dat[5];		//��ȡҪ��ȡ�ļĴ�������
 	uint16_t StopAddress = StartAddress + RegVal - 1;									//��ȡ������ַ
 	uint16_t KeepRegistorSize = sizeof(KeepRegister) / 2;								//���㱣�ּĴ���������
 	uint16_t bytes = RegVal*2;															//����Ҫ��ȡ���ֽ���
	
 	if(ModbusDataPackage.DataLen != 8)err = err_OE;										//��Ч���������쳣
 	if(!RegVal)err = err_add;															//�Ĵ�����ַ����ȷ,��ȡ�����������1
 	if( (  (StartAddress == 0x03) || (StartAddress == 0x05) || (StartAddress == 0x09) || (StartAddress == 0x0b)  || (StartAddress == 0x0d) || (StartAddress == 0x0f)  || \
	       (StartAddress == 0x11) || (StartAddress == 0x13) || (StartAddress == 0x15) || (StartAddress == 0x17)  || (StartAddress == 0x19) || (StartAddress == 0x1b)  || (StartAddress == 0x1d) || (StartAddress == 0x1f) || \
		   (StartAddress == 0x21) || (StartAddress == 0x23) || (StartAddress == 0x25) || (StartAddress == 0x27)  || (StartAddress == 0x29) || (StartAddress == 0x2b)  || (StartAddress == 0x2d) || (StartAddress == 0x2f) || \
		   (StartAddress == 0x31) || (StartAddress == 0x33) || (StartAddress == 0x35) || (StartAddress == 0x37)  || (StartAddress == 0x39) || (StartAddress == 0x3b)  || (StartAddress == 0x3d) || (StartAddress == 0x3f) || \
		   (StartAddress == 0x41) || (StartAddress == 0x43) || (StartAddress == 0x45) || (StartAddress == 0x47)  || (StartAddress == 0x49) || (StartAddress == 0x4b)  || (StartAddress == 0x4d) || (StartAddress == 0x4f) || \
		   (StartAddress == 0x51) || (StartAddress == 0x53) || (StartAddress == 0x55)  ) )err = err_add;//�Ĵ�����ַ����ȷ,���ֽ�����Ҫ��ͷ����
 	if( (  (StopAddress == 0x02)  || (StopAddress == 0x04)  || (StopAddress == 0x08)  || (StopAddress == 0x0a)   || (StopAddress == 0x0c)  || (StopAddress == 0x0e)   || \
	       (StopAddress == 0x10)  || (StopAddress == 0x12)  || (StopAddress == 0x14)  || (StopAddress == 0x16)   || (StopAddress == 0x18)  || (StopAddress == 0x1a)   || (StopAddress == 0x1c)  || (StopAddress == 0x1e)  || \
		   (StopAddress == 0x20)  || (StopAddress == 0x22)  || (StopAddress == 0x24)  || (StopAddress == 0x26)   || (StopAddress == 0x28)  || (StopAddress == 0x2a)   || (StopAddress == 0x2c)  || (StopAddress == 0x2e)  || \
		   (StopAddress == 0x30)  || (StopAddress == 0x32)  || (StopAddress == 0x34)  || (StopAddress == 0x36)   || (StopAddress == 0x38)  || (StopAddress == 0x3a)   || (StopAddress == 0x3c)  || (StopAddress == 0x3e)  || \
		   (StopAddress == 0x40)  || (StopAddress == 0x42)  || (StopAddress == 0x44)  || (StopAddress == 0x46)   || (StopAddress == 0x48)  || (StopAddress == 0x4a)   || (StopAddress == 0x4c)  || (StopAddress == 0x4e)  || \
		   (StopAddress == 0x50)  || (StopAddress == 0x52)  || (StopAddress == 0x54)   ) )err = err_add;//�Ĵ�����ַ����ȷ,���ֽ�����û��ȫ������
 	if(StopAddress > (KeepRegistorSize - 1))err = err_add;							//�Ĵ�����ַ����ȷ
 	if(StartAddress > (KeepRegistorSize - 1))err = err_add;							//�Ĵ�����ַ����ȷ
	 
	 
// 	U485TX;Delay_ms(10);
// 	printf("StartAddress:%d\r\n",StartAddress);
// 	printf("StopAddress:%d\r\n",StopAddress);
// 	printf("KeepRegistorSize:%d\r\n",KeepRegistorSize);
// 	printf("err:%d\r\n",err);
// 	printf("ModbusDataPackage.dat[0]:%d\r\n",ModbusDataPackage.dat[0]);
// 	printf("\r\n");
// 	U485RX;Delay_ms(100);
	
	if(  err != 0  )							//�����쳣����Ϣ
	{
		ModbusReturnAckInfo(err);														//��485�����쳣����Ϣ
		return ERROR;
	}
 	
 	KeepRegistorDataHton();																//��С�����ݴ���,���ڻ�����
 	/*��ȡ���ּĴ������ݲ���485��������*/
  	
	temp[0] = KeepRegister.DeviceAddress;								//��ֵ�豸��ַ
  	temp[1] = ModbusDataPackage.dat[1];									//��ֵ������
  	temp[2] = bytes;													//��ֵ���ݳ���(�ֽ���)
  	memcpy(&temp[3],(uint8_t *)&KeepRegisterTemp+StartAddress*2,bytes);	//�ַ���copy
  	
  	crc=CRC16_Check(temp,bytes+3);										//ִ��crcУ��
  	temp[bytes+3]=crc & 0xff;
  	temp[bytes+4]=crc>>8;
 	if(ModbusDataPackage.dat[0]) U485SendData(temp,bytes+5);			//��������
	
	return SUCCESS;	
}

/********************************
������Ĵ���
���ݸ�ʽ:������ַ(1�ֽ�)+������(0x04)+��ʼ��ַ(2�ֽ�)+��ȡ�Ĵ�������(2�ֽ�)+У��(2�ֽ�)
���ظ�ʽ:������ַ(1�ֽ�)+������(0x04)+���ݳ���(�ֽ���,1�ֽ�)+����(n�ֽ�)+У��(2�ֽ�)
*********************************/
char ModbusReadInputRegistor(void)
{
	uint8_t err=0;
	uint8_t temp[40];
	uint16_t crc;
	
	uint16_t StartAddress = (ModbusDataPackage.dat[2] << 8) | ModbusDataPackage.dat[3];	//��ȡ��ʼ��ַ
	uint16_t RegVal = (ModbusDataPackage.dat[4] << 8) | ModbusDataPackage.dat[5];		//��ȡҪ��ȡ�ļĴ�������
	uint16_t StopAddress = StartAddress + RegVal - 1;									//��ȡ������ַ
	uint16_t InputRegistorSize = sizeof(InputRegister) / 2;								//���㱣�ּĴ���������
	uint16_t bytes = RegVal*2;															//����Ҫ��ȡ���ֽ���
	
	if(ModbusDataPackage.DataLen != 8)err = err_OE;										//��Ч���������쳣
	if(!RegVal)err = err_add;															//�Ĵ�����ַ����ȷ,��ȡ�����������1
	if( ((StartAddress == 3) || (StartAddress == 7) || (StartAddress == 9) || (StartAddress == 11) || (StartAddress == 13) || (StartAddress == 15) ) )err = err_add;//�Ĵ�����ַ����ȷ,���ֽ�����Ҫ��ͷ����
	if( ((StopAddress == 2 ) || (StopAddress == 6 ) || (StopAddress == 8 ) || (StopAddress == 10 ) || (StopAddress == 12 ) || (StopAddress == 14 ) ) )err = err_add;//�Ĵ�����ַ����ȷ,���ֽ�����û��ȫ������
	if(StopAddress > (InputRegistorSize - 1))err = err_add;	//�Ĵ�����ַ����ȷ
	if(StartAddress > (InputRegistorSize - 1))err = err_add;	//�Ĵ�����ַ����ȷ
	if(  err != 0 )						//�����쳣����Ϣ
	{
		ModbusReturnAckInfo(err);								//��485�����쳣����Ϣ
		return ERROR;
	}
	
	if( (StartAddress<=5) && (StopAddress>=5) ) EERead(KREEPROM_BASEADDR+2,(void *)&InputRegister.Reserve1,2);   //��ȡ�豸���
	
	InputRegistorDataHton();									//��С�����ݴ���,���ڻ�����
	//��ȡ����Ĵ������ݲ���485��������
	
	temp[0] = KeepRegister.DeviceAddress;						//��ֵ�豸��ַ
	temp[1] = ModbusDataPackage.dat[1];							//��ֵ������
	temp[2] = bytes;											//��ֵ���ݳ���(�ֽ���)
	memcpy(&temp[3],(uint8_t *)&InputRegisterTemp+StartAddress*2,bytes);
	
	crc=CRC16_Check(temp,bytes+3);
	temp[bytes+3]=crc & 0xff;
	temp[bytes+4]=crc>>8;
	if(ModbusDataPackage.dat[0]) U485SendData(temp,bytes+5);
	
	return SUCCESS;	
}

/********************************
д�����Ĵ���,���ּĴ���
���ݸ�ʽ:������ַ(1�ֽ�)+������(0x06)+�Ĵ�����ַ(2�ֽ�)+�Ĵ�����ֵ(2�ֽ�)+У��(2�ֽ�)
���ظ�ʽ:������ַ(1�ֽ�)+������(0x06)+�Ĵ�����ַ(2�ֽ�)+�Ĵ�����ֵ(2�ֽ�)+У��(2�ֽ�)
*********************************/
char ModbusWriteSingleRegistor(void)
{
	uint8_t err=0;
	uint8_t temp[10];
	uint16_t crc,dat;
	//��ȡ��ز���
	uint16_t StartAddress = (ModbusDataPackage.dat[2] << 8) | ModbusDataPackage.dat[3];	//��ȡ��ʼ��ַ
	
	dat = (ModbusDataPackage.dat[4] << 8) | ModbusDataPackage.dat[5];					//��ȡҪд�������
	
	//�����Ϸ����
	if(ModbusDataPackage.DataLen != 8)err = err_OE;	//��Ч���������쳣
	if((StartAddress != 0) && (StartAddress != 1) && (StartAddress != 6) && (StartAddress != 7) )err = err_add;			//�쳣��,�Ĵ�����ʼ��ַ����ȷ,���ֽ����ݲ����ô˹�����
	//������Ч��Χ�жϲ�д��
	if(StartAddress == 0)							//���д�豸��ַ����
	{
		if((dat == 0) || (dat > 247))err = err_add;	//��ַ���ݳ�����Χ,�����쳣������,�Ĵ���ֵ������Χ
		else
		{
			KeepRegister.DeviceAddress = dat;
			EEWrite(KREEPROM_BASEADDR,(void *)&dat,2);//��������
		}
	}
	
	if(StartAddress == 1)							//���д�豸���
	{	
		KeepRegister.DeviceGroupNum = dat;
		EEWrite(KREEPROM_BASEADDR+2,(void *)&dat,2);//��������
	}
	
	if(StartAddress == 6)							//���д����������
	{		
		if( ((dat == 0x14) || (dat == 0x64)) ==0 ){ModbusReturnAckInfo(4);	return ERROR;}						//����������ض�ֵ�����ش���
		
		KeepRegister.Sensor_Range = dat;
		EEWrite(KREEPROM_BASEADDR+12,(void *)&dat,2);//��������
	}
	
	if(StartAddress == 7)							//Һ��ѡ��
	{
		if( ( ((dat >>8) == 0x01) || ((dat >>8) == 0x02) || ((dat >>8) ==0x03) || ((dat >>8) ==0x04) || ((dat >>8)==0x05) ) ==0  ){ModbusReturnAckInfo(4);	return ERROR;}   //����������ض�ֵ�����ش���
		if( ( ((dat & 0x00FF)==0x00) || ((dat & 0x00FF)==0x01))  ==0  ){ModbusReturnAckInfo(4);	return ERROR;}														     //����������ض�ֵ�����ش���
		
		KeepRegister.Liquid_Sec = dat;
		EEWrite(KREEPROM_BASEADDR+14,(void *)&dat,2);//��������
	}
	
	if(  err != 0 )			//�����쳣����Ϣ
	{
		ModbusReturnAckInfo(err);					//��485�����쳣����Ϣ
		return ERROR;
	}
	//ָ���
	
	temp[0] = KeepRegister.DeviceAddress;			//��ֵ�豸��ַ
	temp[1] = ModbusDataPackage.dat[1];				//��ֵ������
	memcpy(&temp[2],(uint8_t *)&ModbusDataPackage.dat[2],4);
	//У��
	crc = CRC16_Check(temp,6);						//crcУ��
	temp[6] = crc & 0xff;							//crc��λ��ǰ
	temp[7] = crc >> 8;								//��λ�ں�
	if(ModbusDataPackage.dat[0]) U485SendData(temp,8);//��������
	
	return SUCCESS;
}

/********************************
д����Ĵ���,���ּĴ���
���ݸ�ʽ:������ַ(1�ֽ�)+������(0x10)+�Ĵ�����ַ(2�ֽ�)+�Ĵ�������(2�ֽ�)+�ֽ���(1�ֽ�)+�Ĵ�����ֵ(N������)+У��(2�ֽ�)
���ظ�ʽ:������ַ(1�ֽ�)+������(0x10)+�Ĵ�����ַ(2�ֽ�)+�Ĵ�������(2�ֽ�)+У��(2�ֽ�)
*********************************/
char ModbusWriteSomeRegistor(void)
{
	uint8_t err=0;
	uint8_t temp[10];
	uint8_t flag=0;
	//uint8_t t[180];
	uint16_t crc;
	//��ȡ��ز���
	uint16_t StartAddress = (ModbusDataPackage.dat[2] << 8) | ModbusDataPackage.dat[3];	//��ȡ��ʼ��ַ
	uint16_t RegVal = (ModbusDataPackage.dat[4] << 8) | ModbusDataPackage.dat[5];		//��ȡҪд��ļĴ�������
	uint8_t bytes = ModbusDataPackage.dat[6];											//��ȡ�ֽ���
	uint16_t StopAddress = StartAddress + RegVal - 1;									//��ȡ������ַ
	uint16_t KeepRegistorSize = sizeof(KeepRegister) / 2;								//���㱣�ּĴ���������
	
	//U485TX;Delay_ms(10);
	
	//printf("����ֵ:mm  ������߶�ֵ:mm\r\n");
	//Delay_ms(100);U485RX;Delay_ms(10);
	
	//�����Ϸ����
	if(!RegVal || !bytes)err = err_add;													//�Ĵ�����ַ����ȷ,��ȡ�����������1
	if( (  (StartAddress == 0x03) || (StartAddress == 0x05) || (StartAddress == 0x09) || (StartAddress == 0x0b)  || (StartAddress == 0x0d) || (StartAddress == 0x0f)  || \
		   (StartAddress == 0x11) || (StartAddress == 0x13) || (StartAddress == 0x15) || (StartAddress == 0x17)  || (StartAddress == 0x19) || (StartAddress == 0x1b)  || (StartAddress == 0x1d) || (StartAddress == 0x1f) || \
	       (StartAddress == 0x21) || (StartAddress == 0x23) || (StartAddress == 0x25) || (StartAddress == 0x27)  || (StartAddress == 0x29) || (StartAddress == 0x2b)  || (StartAddress == 0x2d) || (StartAddress == 0x2f) || \
	       (StartAddress == 0x31) || (StartAddress == 0x33) || (StartAddress == 0x35) || (StartAddress == 0x37)  || (StartAddress == 0x39) || (StartAddress == 0x3b)  || (StartAddress == 0x3d) || (StartAddress == 0x3f) || \
	       (StartAddress == 0x41) || (StartAddress == 0x43) || (StartAddress == 0x45) || (StartAddress == 0x47)  || (StartAddress == 0x49) || (StartAddress == 0x4b)  || (StartAddress == 0x4d) || (StartAddress == 0x4f) || \
	       (StartAddress == 0x51) || (StartAddress == 0x53) || (StartAddress == 0x55)  ) )err = err_add;//�Ĵ�����ַ����ȷ,���ֽ�����Ҫ��ͷ����
	if( (  (StopAddress == 0x02)  || (StopAddress == 0x04)  || (StopAddress == 0x08)  || (StopAddress == 0x0a)   || (StopAddress == 0x0c)  || (StopAddress == 0x0e)   || \
	       (StopAddress == 0x10)  || (StopAddress == 0x12)  || (StopAddress == 0x14)  || (StopAddress == 0x16)   || (StopAddress == 0x18)  || (StopAddress == 0x1a)   || (StopAddress == 0x1c)  || (StopAddress == 0x1e)  || \
	       (StopAddress == 0x20)  || (StopAddress == 0x22)  || (StopAddress == 0x24)  || (StopAddress == 0x26)   || (StopAddress == 0x28)  || (StopAddress == 0x2a)   || (StopAddress == 0x2c)  || (StopAddress == 0x2e)  || \
	       (StopAddress == 0x30)  || (StopAddress == 0x32)  || (StopAddress == 0x34)  || (StopAddress == 0x36)   || (StopAddress == 0x38)  || (StopAddress == 0x3a)   || (StopAddress == 0x3c)  || (StopAddress == 0x3e)  || \
	       (StopAddress == 0x40)  || (StopAddress == 0x42)  || (StopAddress == 0x44)  || (StopAddress == 0x46)   || (StopAddress == 0x48)  || (StopAddress == 0x4a)   || (StopAddress == 0x4c)  || (StopAddress == 0x4e)  || \
	       (StopAddress == 0x50)  || (StopAddress == 0x52)  || (StopAddress == 0x54)   ) )err = err_add;//�Ĵ�����ַ����ȷ,���ֽ�����û��ȫ������
	if(StopAddress > (KeepRegistorSize - 1))err = err_add;		//�Ĵ�����ַ����ȷ
	if(StartAddress > (KeepRegistorSize - 1))err = err_add;		//�Ĵ�����ַ����ȷ
	
	if(StopAddress > 0x09)flag=1;		//�ж�����Σ�ռĴ�����־λ
	if(StartAddress > 0x09)flag=1;		//�ж�����Σ�ռĴ�����־λ
	
	
	if( (SuperMode_Flag==0) && (flag==1) )err=err_mode;		//�������ͨģʽ��ͬʱ������Σ�ռĴ������򷵻ش���
	
	
	
	if(  err != 0 )							//�����쳣����Ϣ
	{
		ModbusReturnAckInfo(err);									//��485�����쳣����Ϣ
		return ERROR;
	}
	
	memcpy((uint8_t *)&KeepRegisterTemp,(uint8_t *)&KeepRegister,sizeof(KeepRegister));					//�Ƚ���ǰ�Ĵ������ݿ�����������
	memcpy((uint8_t *)&KeepRegisterTemp + StartAddress*2,(uint8_t *)&ModbusDataPackage.dat[7],bytes);	//Ȼ�󽫽��յ����ݿ������Ĵ���������
	err = 0;
	//��С��ת��
	if(StartAddress == 0x00)KeepRegisterTemp.DeviceAddress = htons(KeepRegisterTemp.DeviceAddress);														//���豸��ַ���ݴ�С��ת��
	if((StartAddress <= 0x01) && (StopAddress >= 0x01))KeepRegisterTemp.DeviceGroupNum = htons(KeepRegisterTemp.DeviceGroupNum );										//����������1���ݴ�С��ת��
	if((StartAddress <= 0x02) && (StopAddress >= 0x03))KeepRegisterTemp.OriginaLiquidAltitude = __ltobf(KeepRegisterTemp.OriginaLiquidAltitude);				//��Һλ��ʼ�߶����ݴ�С��ת��
	if((StartAddress <= 0x04) && (StopAddress >= 0x05))KeepRegisterTemp.Reserve1and2 = __ltobf(KeepRegisterTemp.Reserve1and2 );							//��Һ���ܶ����ݴ�С��ת��
	if((StartAddress <= 0x06) && (StopAddress >= 0x06))KeepRegisterTemp.Sensor_Range = htons(KeepRegisterTemp.Sensor_Range );									//����������2���ݴ�С��ת��
	if((StartAddress <= 0x07) && (StopAddress >= 0x07))KeepRegisterTemp.Liquid_Sec = htons(KeepRegisterTemp.Liquid_Sec );											//����������3���ݴ�С��ת��
	if((StartAddress <= 0x08) && (StopAddress >= 0x09))KeepRegisterTemp.LocalAccelerationOfGravity = __ltobf(KeepRegisterTemp.LocalAccelerationOfGravity );	//�������������ٶ����ݴ�С��ת��
	if((StartAddress <= 0x0a) && (StopAddress >= 0x0b))KeepRegisterTemp.Sensor_FS_Val = __ltobf(KeepRegisterTemp.Sensor_FS_Val );							//
	if((StartAddress <= 0x0c) && (StopAddress >= 0x0d))KeepRegisterTemp.MV[0] = __ltobf(KeepRegisterTemp.MV[0] );
	if((StartAddress <= 0x0e) && (StopAddress >= 0x0f))KeepRegisterTemp.MV[1] = __ltobf(KeepRegisterTemp.MV[1] );
	if((StartAddress <= 0x10) && (StopAddress >= 0x11))KeepRegisterTemp.MV[2] = __ltobf(KeepRegisterTemp.MV[2] );
	if((StartAddress <= 0x12) && (StopAddress >= 0x13))KeepRegisterTemp.MV[3] = __ltobf(KeepRegisterTemp.MV[3] );
	if((StartAddress <= 0x14) && (StopAddress >= 0x15))KeepRegisterTemp.MV[4] = __ltobf(KeepRegisterTemp.MV[4] );
	if((StartAddress <= 0x16) && (StopAddress >= 0x17))KeepRegisterTemp.MV[5] = __ltobf(KeepRegisterTemp.MV[5] );
	if((StartAddress <= 0x18) && (StopAddress >= 0x19))KeepRegisterTemp.Temp_T0 = __ltobf(KeepRegisterTemp.Temp_T0 );
	if((StartAddress <= 0x1a) && (StopAddress >= 0x1b))KeepRegisterTemp.LTC0[0] = __ltobf(KeepRegisterTemp.LTC0[0] );
	if((StartAddress <= 0x1c) && (StopAddress >= 0x1d))KeepRegisterTemp.LTC0[1] = __ltobf(KeepRegisterTemp.LTC0[1] );
	if((StartAddress <= 0x1e) && (StopAddress >= 0x1f))KeepRegisterTemp.LTC0[2] = __ltobf(KeepRegisterTemp.LTC0[2] );
	if((StartAddress <= 0x20) && (StopAddress >= 0x21))KeepRegisterTemp.LTC0[3] = __ltobf(KeepRegisterTemp.LTC0[3] );
	if((StartAddress <= 0x22) && (StopAddress >= 0x23))KeepRegisterTemp.LTC0[4] = __ltobf(KeepRegisterTemp.LTC0[4] );
	
	if((StartAddress <= 0x24) && (StopAddress >= 0x25))KeepRegisterTemp.LTC1[0] = __ltobf(KeepRegisterTemp.LTC1[0] );
	if((StartAddress <= 0x26) && (StopAddress >= 0x27))KeepRegisterTemp.LTC1[1] = __ltobf(KeepRegisterTemp.LTC1[1] );
	if((StartAddress <= 0x28) && (StopAddress >= 0x29))KeepRegisterTemp.LTC1[2] = __ltobf(KeepRegisterTemp.LTC1[2] );
	if((StartAddress <= 0x2a) && (StopAddress >= 0x2b))KeepRegisterTemp.LTC1[3] = __ltobf(KeepRegisterTemp.LTC1[3] );
	if((StartAddress <= 0x2c) && (StopAddress >= 0x2d))KeepRegisterTemp.LTC1[4] = __ltobf(KeepRegisterTemp.LTC1[4] );
	
	if((StartAddress <= 0x2e) && (StopAddress >= 0x2f))KeepRegisterTemp.LTC2[0] = __ltobf(KeepRegisterTemp.LTC2[0] );
	if((StartAddress <= 0x30) && (StopAddress >= 0x31))KeepRegisterTemp.LTC2[1] = __ltobf(KeepRegisterTemp.LTC2[1] );
	if((StartAddress <= 0x32) && (StopAddress >= 0x33))KeepRegisterTemp.LTC2[2] = __ltobf(KeepRegisterTemp.LTC2[2] );
	if((StartAddress <= 0x34) && (StopAddress >= 0x35))KeepRegisterTemp.LTC2[3] = __ltobf(KeepRegisterTemp.LTC2[3] );
	if((StartAddress <= 0x36) && (StopAddress >= 0x37))KeepRegisterTemp.LTC2[4] = __ltobf(KeepRegisterTemp.LTC2[4] );
	
	if((StartAddress <= 0x38) && (StopAddress >= 0x39))KeepRegisterTemp.LTC3[0] = __ltobf(KeepRegisterTemp.LTC3[0] );
	if((StartAddress <= 0x3a) && (StopAddress >= 0x3b))KeepRegisterTemp.LTC3[1] = __ltobf(KeepRegisterTemp.LTC3[1] );
	if((StartAddress <= 0x3c) && (StopAddress >= 0x3d))KeepRegisterTemp.LTC3[2] = __ltobf(KeepRegisterTemp.LTC3[2] );
	if((StartAddress <= 0x3e) && (StopAddress >= 0x3f))KeepRegisterTemp.LTC3[3] = __ltobf(KeepRegisterTemp.LTC3[3] );
	if((StartAddress <= 0x40) && (StopAddress >= 0x41))KeepRegisterTemp.LTC3[4] = __ltobf(KeepRegisterTemp.LTC3[4] );
	
	if((StartAddress <= 0x42) && (StopAddress >= 0x43))KeepRegisterTemp.LTC4[0] = __ltobf(KeepRegisterTemp.LTC4[0] );
	if((StartAddress <= 0x44) && (StopAddress >= 0x45))KeepRegisterTemp.LTC4[1] = __ltobf(KeepRegisterTemp.LTC4[1] );
	if((StartAddress <= 0x46) && (StopAddress >= 0x47))KeepRegisterTemp.LTC4[2] = __ltobf(KeepRegisterTemp.LTC4[2] );
	if((StartAddress <= 0x48) && (StopAddress >= 0x49))KeepRegisterTemp.LTC4[3] = __ltobf(KeepRegisterTemp.LTC4[3] );
	if((StartAddress <= 0x4a) && (StopAddress >= 0x4b))KeepRegisterTemp.LTC4[4] = __ltobf(KeepRegisterTemp.LTC4[4] );
	
	if((StartAddress <= 0x4c) && (StopAddress >= 0x4d))KeepRegisterTemp.LTC5[0] = __ltobf(KeepRegisterTemp.LTC5[0] );
	if((StartAddress <= 0x4e) && (StopAddress >= 0x4f))KeepRegisterTemp.LTC5[1] = __ltobf(KeepRegisterTemp.LTC5[1] );
	if((StartAddress <= 0x50) && (StopAddress >= 0x51))KeepRegisterTemp.LTC5[2] = __ltobf(KeepRegisterTemp.LTC5[2] );
	if((StartAddress <= 0x52) && (StopAddress >= 0x53))KeepRegisterTemp.LTC5[3] = __ltobf(KeepRegisterTemp.LTC5[3] );
	if((StartAddress <= 0x54) && (StopAddress >= 0x55))KeepRegisterTemp.LTC5[4] = __ltobf(KeepRegisterTemp.LTC5[4] );
	
	//�ж�������Ч��
	if((KeepRegisterTemp.DeviceAddress == 0) || (KeepRegisterTemp.DeviceAddress > 247))err = err_Re_VOR;	//��ַ���ݳ�����Χ,�����쳣������,�Ĵ���ֵ������Χ
	//if((KeepRegisterTemp.Reserve1and2< 0.5) || (KeepRegisterTemp.Reserve1and2 > 2.0))err = err_Re_VOR;
	if((KeepRegisterTemp.LocalAccelerationOfGravity< 9.78) || (KeepRegisterTemp.LocalAccelerationOfGravity > 10.0))err = err_Re_VOR;
	if(!err)																						//����޴���,�򽫻�������ݿ������Ĵ�����
	{
		memcpy((uint8_t *)&KeepRegister,(uint8_t *)&KeepRegisterTemp,sizeof(KeepRegister));
		EEWrite(KREEPROM_BASEADDR,(void *)&KeepRegisterTemp,sizeof(KeepRegister));					//�������ݣ����������Ĵ�����
	}
	else if(ModbusDataPackage.dat[0])//������ݷ�Χ����
	{
		ModbusReturnAckInfo(err);	 //��485�����쳣����Ϣ
		return ERROR;
	}
	
	//ָ���
	temp[0] = KeepRegister.DeviceAddress;					//��ֵ�豸��ַ
	temp[1] = ModbusDataPackage.dat[1];						//��ֵ������
	memcpy(&temp[2],(uint8_t *)&ModbusDataPackage.dat[2],4);//��ֵ�Ĵ�����ַ������,��4�ֽ�
	//У��
	crc = CRC16_Check(temp,6);								//crcУ��
	temp[6] = crc & 0xff;									//crc��λ��ǰ
	temp[7] = crc >> 8;										//��λ�ں�
	if(ModbusDataPackage.dat[0]) U485SendData(temp,8);		//��������
	
	return SUCCESS;
}


float Level_height_conversion()
{
	unsigned char i;
	float SV[6]={0};
	float SV_m[6]={0};
	float OriginaAltitudeTemp=0,PA=0,ADCValue=0;
	double T_d=0;
	double P0[5],P1[5],P2[5],P3[5],P4[5],P5[5];
	double TEMP_Cor_VAL[6]={0};
	double Cor_result=0;
	double Liquid_D=0;
	double LD_V[4]={0};
		
	T_d=InputRegister.Temperature;  //��ȡ��ǰ�¶�
	
	/******************************************************************
	**����Һ���ܶ�
	******************************************************************/
	//�ж�Һ������
//  	if((KeepRegister.Liquid_Sec&0x0F00) == 0x0100)      {LD_V[0] =  0.00000001874 ; LD_V[1] = -0.000006231;  LD_V[2] =  0.0000262; LD_V[3] = 1     ;}       //����ˮ
//  	else if((KeepRegister.Liquid_Sec&0x0F00) == 0x0200) {LD_V[0] =  0.000000002806; LD_V[1] = -0.000002759;  LD_V[2] = -0.0003734; LD_V[3] = 1.075 ;}       //����FD-1�� -25�����Һ
//  	else if((KeepRegister.Liquid_Sec&0x0F00) == 0x0300) {LD_V[0] = -0.00000003493;  LD_V[1] =  0.000001245;  LD_V[2] = -0.0005355; LD_V[3] = 1.096; }       //����YF-2A�� -45�����Һ
//  	else if((KeepRegister.Liquid_Sec&0x0F00) == 0x0400) {LD_V[0] = -0.00000003079;  LD_V[1] =  0.0000006097; LD_V[2] = -0.0004351; LD_V[3] = 1.078; }       //����OAT -30�����Һ
//  	else if((KeepRegister.Liquid_Sec&0x0F00) == 0x0500) {LD_V[0] = -0.00000002093;  LD_V[1] = -0.0000001951; LD_V[2] = -0.0004336; LD_V[3] = 1.09;  }       //����OAT -45��
//  	
// 	Liquid_D=LD_V[0]*T_d*T_d*T_d + LD_V[1]*T_d*T_d + LD_V[2]*T_d + LD_V[3];  //����Һ���ܶ� 
	
	//�ж�Һ������
	if((KeepRegister.Liquid_Sec&0x0F00) == 0x0100)       Liquid_D = 1;       //����ˮ
	else if((KeepRegister.Liquid_Sec&0x0F00) == 0x0200)  Liquid_D = 1.075;       //����FD-1�� -25�����Һ
	else if((KeepRegister.Liquid_Sec&0x0F00) == 0x0300)  Liquid_D = 1.096;        //����YF-2A�� -45�����Һ
	else if((KeepRegister.Liquid_Sec&0x0F00) == 0x0400)  Liquid_D = 1.078;       //����OAT -30�����Һ
	else if((KeepRegister.Liquid_Sec&0x0F00) == 0x0500)  Liquid_D = 1.09;         //����OAT -45��
	
	/************************************************************************
	**�������У׼                                                           
	************************************************************************/
	for(i=0;i<6;i++) SV[i]= i*(float)KeepRegister.Sensor_Range*1000/5;       //�������У׼��׼��
	
	for(i=0;i<1;i++) ADCValue += ADCSmaple();	                             //����
	ADCValue=ADCValue/1;
	
	//���������У׼��ͬʱ�����źţ�mV��ת��Ϊѹ���źţ�Pa��
	if     (ADCValue < KeepRegister.MV[1]){PA=(ADCValue-KeepRegister.MV[0])*(SV[1]-SV[0])/(KeepRegister.MV[1]-KeepRegister.MV[0])+SV[0];}
	else if(ADCValue < KeepRegister.MV[2]){PA=(ADCValue-KeepRegister.MV[1])*(SV[2]-SV[1])/(KeepRegister.MV[2]-KeepRegister.MV[1])+SV[1];}
	else if(ADCValue < KeepRegister.MV[3]){PA=(ADCValue-KeepRegister.MV[2])*(SV[3]-SV[2])/(KeepRegister.MV[3]-KeepRegister.MV[2])+SV[2];}
	else if(ADCValue < KeepRegister.MV[4]){PA=(ADCValue-KeepRegister.MV[3])*(SV[4]-SV[3])/(KeepRegister.MV[4]-KeepRegister.MV[3])+SV[3];}
	else                                  {PA=(ADCValue-KeepRegister.MV[4])*(SV[5]-SV[4])/(KeepRegister.MV[5]-KeepRegister.MV[4])+SV[4];}
	
	OriginaAltitudeTemp= ( PA/(Liquid_D*1000*KeepRegister.LocalAccelerationOfGravity) );   //p=��gh+�˵صĴ���ѹ ��:ˮ��1.0X1000 Kg/m3    g��9.8N/kg     �߶�:m
	OriginaAltitudeTemp =OriginaAltitudeTemp*1000;//ת����mm
	
	/************************************************************************
	**�ฺ���¶�����                                                         
	************************************************************************/
	if( (KeepRegister.Liquid_Sec & 0x0F) == 0x01)
	{
		
		for(i=0;i<6;i++)SV_m[i]=SV[i]/(Liquid_D*KeepRegister.LocalAccelerationOfGravity);   //�����׼ѹ���µ�mmҺ����
		//6��У׼���ߣ��¶�-ѹǿ������
		//��1000����ȡ��ʵϵ��
		for(i=0;i<5;i++)P0[i]=((double)KeepRegister.LTC0[i])/1000;
		for(i=0;i<5;i++)P1[i]=((double)KeepRegister.LTC1[i])/1000;
		for(i=0;i<5;i++)P2[i]=((double)KeepRegister.LTC2[i])/1000;
		for(i=0;i<5;i++)P3[i]=((double)KeepRegister.LTC3[i])/1000;
		for(i=0;i<5;i++)P4[i]=((double)KeepRegister.LTC4[i])/1000;
		for(i=0;i<5;i++)P5[i]=((double)KeepRegister.LTC5[i])/1000;
		
		//6���¶�����
		//���ݵ�ǰ�¶ȣ���ȡ6�������ϵ�ѹǿ��У׼�㣬
		//�¶�������ʽ���Ľ�����
		TEMP_Cor_VAL[0] = P0[4]*pow(T_d,4) + P0[3]*pow(T_d,3) + P0[2]*T_d*T_d + P0[1]*T_d + P0[0];   //  �㸺�����¶�-����ֵ����
		TEMP_Cor_VAL[1] = P1[4]*pow(T_d,4) + P1[3]*pow(T_d,3) + P1[2]*T_d*T_d + P1[1]*T_d + P1[0];   //  1/5�������¶�-����ֵ����
		TEMP_Cor_VAL[2] = P2[4]*pow(T_d,4) + P2[3]*pow(T_d,3) + P2[2]*T_d*T_d + P2[1]*T_d + P2[0];   //  1/5�������¶�-����ֵ����
		TEMP_Cor_VAL[3] = P3[4]*pow(T_d,4) + P3[3]*pow(T_d,3) + P3[2]*T_d*T_d + P3[1]*T_d + P3[0];   //  1/5�������¶�-����ֵ����
		TEMP_Cor_VAL[4] = P4[4]*pow(T_d,4) + P4[3]*pow(T_d,3) + P4[2]*T_d*T_d + P4[1]*T_d + P4[0];   //  1/5�������¶�-����ֵ����
		TEMP_Cor_VAL[5] = P5[4]*pow(T_d,4) + P5[3]*pow(T_d,3) + P5[2]*T_d*T_d + P5[1]*T_d + P5[0];   //  ���������¶�-����ֵ����
	
		//�����¶�ѹǿ�����ϵ�����ѹǿ��У׼�㣬ѹǿ
		for(i=0;i<6;i++)TEMP_Cor_VAL[i]=TEMP_Cor_VAL[i]/(KeepRegister.LocalAccelerationOfGravity*Liquid_D);        //��ѹǿת��ΪmmҺ����ֵ
		
		if     (ADCValue < KeepRegister.MV[1]){Cor_result=(OriginaAltitudeTemp-SV_m[0])*(TEMP_Cor_VAL[1]-TEMP_Cor_VAL[0])/(SV_m[1]-SV_m[0])+TEMP_Cor_VAL[0];}
		else if(ADCValue < KeepRegister.MV[2]){Cor_result=(OriginaAltitudeTemp-SV_m[1])*(TEMP_Cor_VAL[2]-TEMP_Cor_VAL[1])/(SV_m[2]-SV_m[1])+TEMP_Cor_VAL[1];}
		else if(ADCValue < KeepRegister.MV[3]){Cor_result=(OriginaAltitudeTemp-SV_m[2])*(TEMP_Cor_VAL[3]-TEMP_Cor_VAL[2])/(SV_m[3]-SV_m[2])+TEMP_Cor_VAL[2];}
		else if(ADCValue < KeepRegister.MV[4]){Cor_result=(OriginaAltitudeTemp-SV_m[3])*(TEMP_Cor_VAL[4]-TEMP_Cor_VAL[3])/(SV_m[4]-SV_m[3])+TEMP_Cor_VAL[3];}
		else                                  {Cor_result=(OriginaAltitudeTemp-SV_m[4])*(TEMP_Cor_VAL[5]-TEMP_Cor_VAL[4])/(SV_m[5]-SV_m[4])+TEMP_Cor_VAL[4];}
	}
	else 
		Cor_result=0;
	
	OriginaAltitudeTemp=OriginaAltitudeTemp - Cor_result; 

	InputRegister.ADCOriginalValue = ADCValue;						//��ADCԭʼ����ֵ��mV����������Ĵ���
	InputRegister.LiquidAltitudeAbsoluteValue = OriginaAltitudeTemp;//���ɼ�����Һλ���Ը߶�ֵ��������Ĵ���
	InputRegister.AltitudeDifference= OriginaAltitudeTemp - KeepRegister.OriginaLiquidAltitude;//����߶Ȳ֮�����ݴ�������Ĵ���
	InputRegister.PA  =  PA;
	
	return OriginaAltitudeTemp;
}

/********************************
�趨��ֵ����ɺ�д���ּĴ���
���ݸ�ʽ:������ַ(1�ֽ�)+������(0x41)+������(4�ֽ�)+У��(2�ֽ�)
���ظ�ʽ:������ַ(1�ֽ�)+������(0x41)+���޸ļĴ����׵�ַ(2�ֽ�)+���޸ļĴ�������(2�ֽ�)+У��(2�ֽ�)
������:0x44,0x66,0x88,0xaa
*********************************/
char ModbusSetInitalValue(void)
{
	uint8_t err=0;
	uint8_t temp[10];
	uint16_t crc;
	float OriginaAltitudeTemp=0;
	//SenPow_ON;Delay_ms(200);
	if(ModbusDataPackage.DataLen !=8 )err = err_OE;
	if( strncmp(FactorySetValueWord,(unsigned char *)&ModbusDataPackage.dat[2],4) !=0 )err=err_OE;
	if(  (err != 0)  &&  (ModbusDataPackage.dat[0] != 0) )
	{
		ModbusReturnAckInfo(err);
		return ERROR;
	}
	
	OriginaAltitudeTemp=Level_height_conversion();
	KeepRegister.OriginaLiquidAltitude = OriginaAltitudeTemp;    //ͬʱ�����ݱ�����EEPROM
	EEWrite(KREEPROM_BASEADDR+4,(void *)&OriginaAltitudeTemp,4);
	
	temp[0] = KeepRegister.DeviceAddress;
	temp[1] = ModbusDataPackage.dat[1];
	temp[2] = 0x00;
	temp[3] = 0x02;
	temp[4] = 0x00;
	temp[5] = 0x02;
	
	crc = CRC16_Check(temp,6);
	
	temp[6] = crc &0xff;
	temp[7] = crc >> 8;
	if(ModbusDataPackage.dat[0]) U485SendData(temp,8);
	
	//SenPow_OFF;
	return SUCCESS;
}

/********************************
��ʼ������������ɺ���ֵд������Ĵ���
���ݸ�ʽ:������ַ(1�ֽ�)+������(0x42)+������(4�ֽ�)+У��(2�ֽ�)
���ظ�ʽ:������ַ(1�ֽ�)+������(0x42)+���޸ļĴ����׵�ַ(2�ֽ�)+���޸ļĴ�������(2�ֽ�)+У��(2�ֽ�)
������:0x44,0x66,0x88,0xaa
*********************************/
char ModbusStartSample(void)
{
	uint8_t err=0;
	uint8_t temp[10];
	uint16_t crc;
	//SenPow_ON;Delay_ms(1500);
	if(ModbusDataPackage.DataLen !=8 )err = err_OE;
	if( strncmp(FactorySetValueWord,(unsigned char *)&ModbusDataPackage.dat[2],4) !=0 )err=err_OE;
	
	if(  (err != 0)  && (ModbusDataPackage.dat[0] != 0) )//���ж�
	{
		ModbusReturnAckInfo(err);
		return ERROR;
	}
	
	(void)Level_height_conversion();    //ִ��һ�β���
	
	temp[0] = KeepRegister.DeviceAddress;
	temp[1] = ModbusDataPackage.dat[1];
	temp[2] = 0x00;
	temp[3] = 0x0c;
	temp[4] = 0x00;
	temp[5] = 0x02;
	
	crc = CRC16_Check(temp,6);
	temp[6] = crc &0xff;
	temp[7] = crc >> 8;
	if(ModbusDataPackage.dat[0]) U485SendData(temp,8);
	
	//SenPow_OFF;
	return SUCCESS;
}


/********************************
������У׼��У׼��ɺ�д���ּĴ���
���ݸ�ʽ:������ַ(1�ֽ�)+������(0x43)+������(4�ֽ�)+У��(2�ֽ�)
���ظ�ʽ:������ַ(1�ֽ�)+������(0x43)+���޸ļĴ����׵�ַ(2�ֽ�)+���޸ļĴ�������(2�ֽ�)+У��(2�ֽ�)
�����룺����������Ϊ��0x43��0x44��0x45��0x46��0x47��0x48
������:0x44,0x66,0x88,0xaa:�û�У׼
������:0x40,0x66,0x88,0xaa:����У׼
*********************************/
char FactoryCalibration(void)
{
	uint8_t i,err=0;
	uint8_t temp[10];
	uint8_t function_code=0x00;
	uint16_t crc;
	float ADCValue=0;
	//SenPow_ON;Delay_ms(500);
	if(ModbusDataPackage.DataLen !=8 )err = err_OE;
	if( (strncmp(FactorySetValueWord,(unsigned char *)&ModbusDataPackage.dat[2],4) !=0) && (strncmp(FactorySetValueWord2,(unsigned char *)&ModbusDataPackage.dat[2],4) !=0)  )err=err_OE;
	if(SuperMode_Flag==0)err=err_mode;//�������ͨģʽ���򷵻ش���
	if(  err != 0 )
	{
		ModbusReturnAckInfo(err);
		return ERROR;
	}
	
	function_code=ModbusDataPackage.dat[1]; //��ȡ������
	
	for(i=0;i<1;i++) ADCValue = ADCSmaple();//ȡn��ƽ����mV
	
	switch (function_code)   //���ݹ��������Ӧ��У׼�����У׼
	{
		case FactoryCalibration0: KeepRegister.MV[0]=ADCValue;
								  EEWrite(KREEPROM_BASEADDR+24,(void *)&KeepRegister.MV[0],4);
								  KeepRegister.Temp_T0=InputRegister.Temperature;
								  EEWrite(KREEPROM_BASEADDR+48,(void *)&KeepRegister.Temp_T0,4);
								  break;//���У׼��0��ͬʱ�����ݱ�����EEPROM
		case FactoryCalibration1: KeepRegister.MV[1]=ADCValue; EEWrite(KREEPROM_BASEADDR+28,(void *)&KeepRegister.MV[1],4);break;//���У׼��1��ͬʱ�����ݱ�����EEPROM
		case FactoryCalibration2: KeepRegister.MV[2]=ADCValue; EEWrite(KREEPROM_BASEADDR+32,(void *)&KeepRegister.MV[2],4);break;//���У׼��2��ͬʱ�����ݱ�����EEPROM
		case FactoryCalibration3: KeepRegister.MV[3]=ADCValue; EEWrite(KREEPROM_BASEADDR+36,(void *)&KeepRegister.MV[3],4);break;//���У׼��3��ͬʱ�����ݱ�����EEPROM
		case FactoryCalibration4: KeepRegister.MV[4]=ADCValue; EEWrite(KREEPROM_BASEADDR+40,(void *)&KeepRegister.MV[4],4);break;//���У׼��4��ͬʱ�����ݱ�����EEPROM
		case FactoryCalibration5: KeepRegister.MV[5]=ADCValue; EEWrite(KREEPROM_BASEADDR+44,(void *)&KeepRegister.MV[5],4);break;//���У׼��5��ͬʱ�����ݱ�����EEPROM
		default: break;
	}
	
	
	temp[0] = KeepRegister.DeviceAddress;
	temp[1] = ModbusDataPackage.dat[1];
	temp[2] = 0x00;
	temp[3] = 0x0a;
	temp[4] = 0x00;
	temp[5] = 0x02;
	
	crc = CRC16_Check(temp,6);
	
	temp[6] = crc &0xff;
	temp[7] = crc >> 8;
	if(ModbusDataPackage.dat[0]) U485SendData(temp,8);
	
	//SenPow_OFF;
	return SUCCESS;
}

/***********************************************************************
��ȡSN��Ϣ
���ݸ�ʽ:������ַ(1�ֽ�)+������(0x40)+������(4�ֽ�)+У��(2�ֽ�)
���ظ�ʽ:������ַ(1�ֽ�)+������(0x40)+��������(1�ֽ�)+SN��Ϣ(15�ֽ�)+У��(2�ֽ�)
************************************************************************/
uint16_t TOS_NODE_ID=111;
char Get_SNInfo_Fun(void)
{
	unsigned char i;
	unsigned char err=0;
	uint16_t crc;
	unsigned char SN_Info[15]={0};
	unsigned char temp[25]={0};
	volatile unsigned char reservalData = TOS_NODE_ID;
	
	if(ModbusDataPackage.DataLen !=8 )err = err_OE;
	if( strncmp(FactorySetValueWord,(unsigned char *)&ModbusDataPackage.dat[2],4) !=0 )err=err_OE;
	
	if(  (err != 0)  && (ModbusDataPackage.dat[0] != 0) )//���ж�
	{
		ModbusReturnAckInfo(err);
		return ERROR;
	}
	
	//const uint8_t __progmem_smartbow_start[64] PROGMEM = {"&&SUPERHHH111401&&2"};
	for(i=0; i<14; i++)
	{
		SN_Info[i] = pgm_read_byte(&__progmem_smartbow_start[i+2]);
	}
	SN_Info[14] = pgm_read_byte(&__progmem_smartbow_start[18]);
	
	//U485TX;Delay_ms(10);
	//printf("SN_Info:%s\r\n",&SN_Info);
	//U485RX;Delay_ms(100);
	
	temp[0] = KeepRegister.DeviceAddress;
	temp[1] = ModbusDataPackage.dat[1];
	temp[2] = 0x0F;
	
	for(i=0;i<15;i++)
	{
		temp[i+3]=SN_Info[i];
	}
	
	crc = CRC16_Check(temp,18);
	
	temp[18] = crc &0xff;
	temp[19] = crc >> 8;
	if(ModbusDataPackage.dat[0]) U485SendData(temp,20);
	
	return SUCCESS;
}

/********************************
ģʽ�л�
���ݸ�ʽ:������ַ(1�ֽ�)+������(0x66)+������(4�ֽ�)+У��(2�ֽ�)

������:0x44,0x66,0x88,0xaa�л�Ϊ��ͨģʽ    ���ظ�ʽ:������ַ(1�ֽ�)+������(0x66)+0x44+0x66+0x88+0xaa+У��(2�ֽ�)
������:0x40,0x60,0x80,0xa0�л�Ϊ����ģʽ    ���ظ�ʽ:������ַ(1�ֽ�)+������(0x66)+0x40+0x60+0x80+0xa0+У��(2�ֽ�)
*********************************/
char ModbusSwitchMode(void)
{
	uint8_t err=0;
	uint8_t temp[10];
	uint16_t crc;
	uint8_t temp2=3;
	
	if(ModbusDataPackage.DataLen !=8 )err = err_OE;
	if( strncmp(SuperModeValueWord2,(unsigned char *)&ModbusDataPackage.dat[2],4) ==0 )//�л�Ϊ����ģʽ
	{
		temp2=1;		
	}
	else if( strncmp(FactorySetValueWord,(unsigned char *)&ModbusDataPackage.dat[2],4) ==0 )//�л�Ϊ��ͨģʽ
	{
		temp2=2;
	}
	else err=err_OE;
	
	if(  (err != 0)  &&  (ModbusDataPackage.dat[0] != 0) )
	{
		ModbusReturnAckInfo(err);
		return ERROR;
	}
	
		
	if( temp2==1 )//�л�Ϊ����ģʽ
	{
		SuperMode_Flag=1;
		temp[0] = 0x34;
		temp[1] = 0x12;
		EEWrite(SUPERMODE_FLAG_BASEADDR,temp,2);
		InputRegister.SystemWorkStatus=(InputRegister.SystemWorkStatus & 0x00ff)|0x0100;
		
		temp[0] = KeepRegister.DeviceAddress;
		temp[1] = ModbusDataPackage.dat[1];
		temp[2] = 0x40;
		temp[3] = 0x60;
		temp[4] = 0x80;
		temp[5] = 0xa0;
	}
	
	else if( temp2==2 )//�л�Ϊ��ͨģʽ
	{
		SuperMode_Flag=0;
		temp[0] = 0x00;
		temp[1] = 0x00;
		EEWrite(SUPERMODE_FLAG_BASEADDR,temp,2);
		InputRegister.SystemWorkStatus=(InputRegister.SystemWorkStatus & 0x00ff)|0x0200;
		
		temp[0] = KeepRegister.DeviceAddress;
		temp[1] = ModbusDataPackage.dat[1];
		temp[2] = 0x44;
		temp[3] = 0x66;
		temp[4] = 0x88;
		temp[5] = 0xaa;
	}
	
	crc = CRC16_Check(temp,6);
	
	temp[6] = crc &0xff;
	temp[7] = crc >> 8;
	if(ModbusDataPackage.dat[0]) U485SendData(temp,8);
	
	return SUCCESS;
}

/********************************
����¶�����ϵ�������¶�����ϵ��ȫ������
���ݸ�ʽ:������ַ(1�ֽ�)+������(0x49)+������(4�ֽ�)+У��(2�ֽ�)
���ظ�ʽ:������ַ(1�ֽ�)+������(0x49)+���޸ļĴ����׵�ַ(2�ֽ�)+���޸ļĴ�������(2�ֽ�)+У��(2�ֽ�)
������:0x44,0x66,0x88,0xaa
*********************************/
char CTC(void)
{
	uint8_t err=0;
	uint8_t i=0;
	uint8_t temp[10];
	uint16_t crc;

	if(ModbusDataPackage.DataLen !=8 )err = err_OE;
	if( strncmp(FactorySetValueWord,(unsigned char *)&ModbusDataPackage.dat[2],4) !=0 )err=err_OE;
	if(SuperMode_Flag==0)err=err_mode;
	
	
	if(  (err != 0)  && (ModbusDataPackage.dat[0] != 0) )//���ж�
	{
		ModbusReturnAckInfo(err);
		return ERROR;
	}
	
	//��ϵ��ȫ������
	for(i=0;i<5;i++) KeepRegister.LTC0[i]=0;
	for(i=0;i<5;i++) KeepRegister.LTC1[i]=0;
	for(i=0;i<5;i++) KeepRegister.LTC2[i]=0;
	for(i=0;i<5;i++) KeepRegister.LTC3[i]=0;
	for(i=0;i<5;i++) KeepRegister.LTC4[i]=0;
	for(i=0;i<5;i++) KeepRegister.LTC5[i]=0;
	
	EEErase(KREEPROM_BASEADDR+52,120);    //���浽EEPROM
	
	temp[0] = KeepRegister.DeviceAddress;
	temp[1] = ModbusDataPackage.dat[1];
	temp[2] = 0x00;
	temp[3] = 0x0c;
	temp[4] = 0x00;
	temp[5] = 0x02;
	
	crc = CRC16_Check(temp,6);
	temp[6] = crc &0xff;
	temp[7] = crc >> 8;
	if(ModbusDataPackage.dat[0]) U485SendData(temp,8);
	
	return SUCCESS;
}

/********************************
�ָ���������
���ݸ�ʽ:������ַ(0)+������(0x68)+������(4�ֽ�)+У��(2�ֽ�)
���ظ�ʽ:������ַ(1�ֽ�)+������(0x68)+�Ĵ�����ַ(2�ֽ�)+�Ĵ�������(2�ֽ�)+�ֽ���(1�ֽ�)+���б��ּĴ�������(N�ֽ�)+У��(2�ֽ�)
������:0x33,0x55,0x77,0x99
*********************************/
char ModbusFactoryParameterReset(void)
{
	uint8_t err=0;
	uint8_t temp[255];
	uint16_t crc;
	if(ModbusDataPackage.DataLen != 8)err = err_OE;//��Ч���������쳣
	if(strncmp(FactoryResetWord,(char *)&ModbusDataPackage.dat[2],4) != 0)err = err_OE;//�Ĵ���ֵ������Χ
	if(SuperMode_Flag==0)err=err_mode;
	if(  (err != 0)  && (ModbusDataPackage.dat[0] != 0) )	  //�����쳣����Ϣ
	{
		ModbusReturnAckInfo(err);			  //��485�����쳣����Ϣ
		return ERROR;
	}
	
	//KeepRegister_Num=0;
	//KeepRegister_Byte_Num=0;
	
	EEErase(KREEPROM_BASEADDR,12);           //����EEPROMǰ12���ֽڣ�������ַ����š���ʼ�߶ȡ��ܶȡ��������ٶ�
	EEErase(KREEPROM_BASEADDR+16,4);         //�����������ٶȡ�
	
	EEErase(FPOWERONFLAG_BASEADDR,2);        //�����״��ϵ��־λ
	
	KeepRegistorDataHton();				 //��С�����ݴ���,���ڻ�����
										 
	temp[0] = KeepRegister.DeviceAddress;//��ֵ�豸��ַ
	temp[1] = ModbusDataPackage.dat[1];	 //��ֵ������
	temp[2] = 0x00;
	temp[3] = 0x00;
	temp[4] = 0x00;
	temp[5] = KeepRegister_Num;          
	temp[6] = KeepRegister_Byte_Num;
	
	memcpy(&temp[7],(uint8_t *)&KeepRegisterTemp,KeepRegister_Byte_Num);//��ȡ����
	
	//У��
	crc = CRC16_Check(temp,KeepRegister_Byte_Num+7); //crcУ��
	temp[KeepRegister_Byte_Num+7] = crc & 0xff;		//crc��λ��ǰ
	temp[KeepRegister_Byte_Num+8] = crc >> 8;		//��λ�ں�
	
	
	if(ModbusDataPackage.dat[0]) U485SendData(temp,KeepRegister_Byte_Num+9);//��������,����=��ȡ�ֽ���+ǰ�������ֽ�+����У�飻����ǹ㲥�����򲻻ظ���
	while(1);//�ȴ����Ź���λ��������ɻָ���������
	return SUCCESS;
}

/*******************************************************************************
* Function Name  : ProcessTask
* Description    : �������̴�����
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
static void ProcessTask()
{
	unsigned char ix;
	unsigned short crc;
	U485RX;
	if(ModbusDataPackage.DataFlag)		  //���ݰ��ѽ������
	{ 	
		LED2_COM;
		for(ix=0;ix<ModbusDataPackage.DataLen;ix++)
		{
			ModbusDataPackage.dat[ix] =USART0_GetChar();//���������ݷŵ�ָ��buf
		}
#ifdef  DUBEG_TEST_ORDER		
		U485TX;Delay_ms(10);
		for(ix=0;ix<ModbusDataPackage.DataLen;ix++)
		{
			printf(":0x%x\r\n",ModbusDataPackage.dat[ix]);
		}
		printf("rxlen:%d\r\n",ModbusDataPackage.DataLen);
		Delay_ms(100);U485RX;
#endif		
		crc = CRC16_Check((uint8_t *)ModbusDataPackage.dat,ModbusDataPackage.DataLen-2 );
		if( (( crc == ( (ModbusDataPackage.dat[ModbusDataPackage.DataLen - 1] << 8) |    ModbusDataPackage.dat[ModbusDataPackage.DataLen - 2])        )) \
		||  ( ( (ModbusDataPackage.dat[ModbusDataPackage.DataLen - 1]) == 0xff    ) && ((ModbusDataPackage.dat[ModbusDataPackage.DataLen - 2]) == 0xff))  )				
		{
			WorkTime = 0;//�����ʱʱ��
			if(ModbusDataPackage.dat[0] == KeepRegister.DeviceAddress)//�Ǳ���ָ��
			{
				switch(ModbusDataPackage.dat[1])
				{
					case ReadKeepRegistor:			//�����ּĴ���
								ModbusReadKeepRegistor();
								//InputRegister.SystemWorkStatus=0x01;
								break;
					case ReadInputRegistor:			//������Ĵ���
								ModbusReadInputRegistor();
								InputRegister.SystemWorkStatus=(InputRegister.SystemWorkStatus & 0xff00)|0x02;
								break;
					case WriteSingleRegistor:		//д�������ּĴ���
								ModbusWriteSingleRegistor();
								InputRegister.SystemWorkStatus=(InputRegister.SystemWorkStatus & 0xff00)|0x03;
								break;
					case WriteSomeRegistor:			//д������ּĴ���
								ModbusWriteSomeRegistor();
								InputRegister.SystemWorkStatus=(InputRegister.SystemWorkStatus & 0xff00)|0x04;
								break;
					case Get_SNInfo:                //��ȡSN��Ϣ
								Get_SNInfo_Fun();
							break;
					case AutoSetInitalValue:		//�Զ����ó�ֵ
								ModbusSetInitalValue();
								InputRegister.SystemWorkStatus=(InputRegister.SystemWorkStatus & 0xff00)|0x05;
								break;
					case StartSample:				//��ʼ��������
								ModbusStartSample();
								InputRegister.SystemWorkStatus=(InputRegister.SystemWorkStatus & 0xff00)|0x06;
								break;
					case FactoryCalibration0:
					case FactoryCalibration1:
					case FactoryCalibration2:
					case FactoryCalibration3:
					case FactoryCalibration4:
					case FactoryCalibration5:		
								FactoryCalibration();			
								InputRegister.SystemWorkStatus=(InputRegister.SystemWorkStatus & 0xff00)|0x07;
								break;//����У׼
					case Clear_temp_corr:                //���У׼ϵ��
								CTC();
								break;
					case ModeSwitch:
								ModbusSwitchMode();		//ģʽ�л�
								break;
								
					case FactoryParameterReset:		//�ָ���������
								ModbusFactoryParameterReset();
								InputRegister.SystemWorkStatus=(InputRegister.SystemWorkStatus & 0xff00)|0x09;
								break;
					default:
								ModbusReturnAckInfo(err_Fu);//��485�����쳣����Ϣ,���ܺ���Ч
								break;
				}
			}
			/********************************
			�㲥ָ����ظ�
			���ݸ�ʽ:������ַ(1�ֽ�)+������(0x)+������(4�ֽ�)+У��(2�ֽ�) 
			�ֶ�:0x33,0x55,0x77,0x99
			*********************************/
			else if (ModbusDataPackage.dat[0] == 0x00)//�ǹ㲥ָ��
			{
				switch(ModbusDataPackage.dat[1])
				{
					case AutoSetInitalValue:
								ModbusSetInitalValue();
								InputRegister.SystemWorkStatus=(InputRegister.SystemWorkStatus & 0xff00)|0x11;
								break;
					case StartSample:
								ModbusStartSample();
								InputRegister.SystemWorkStatus=(InputRegister.SystemWorkStatus & 0xff00)|0x12;
								break;
					case FactoryCalibration0:
					case FactoryCalibration1:
					case FactoryCalibration2:
					case FactoryCalibration3:
					case FactoryCalibration4:
					case FactoryCalibration5:
								FactoryCalibration();
								InputRegister.SystemWorkStatus=(InputRegister.SystemWorkStatus & 0xff00)|0x07;
								break;//����У׼	
								
					case ModeSwitch:
								ModbusSwitchMode();		//ģʽ�л�
								break;	
					case FactoryParameterReset:	//�ָ���������
								ModbusFactoryParameterReset();
								InputRegister.SystemWorkStatus=(InputRegister.SystemWorkStatus & 0xff00)|0x13;
								break;
					default:
								break;
				} 
			} 
		}
		USART0_ClearBuf();				 //��մ��ڽ��ջ���
		ModbusDataPackage.DataLen = 0;  //����ճ��ȣ�ע�����˳��
		ModbusDataPackage.DataFlag = 0; //��ձ��λ
		LED2_COM;
	}
}

/*******************************************************************************
* Function Name  : ClC_WatchDogTask
* Description    : ���Ź�����
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void ClC_WatchDogTask(void)
{
	if(!WorkTimeOutFlag) 
	{WDI_COM;__asm("nop");__asm("nop");__asm("nop");__asm("nop");__asm("nop");_WDR();}   //ι���������������913����δ�յ��κ�ָ�ִ��ϵͳ����������ι����
#ifdef DUBEG_TEST
	WDI_COM;__asm("nop");__asm("nop");__asm("nop");__asm("nop");__asm("nop");_WDR();//����Ϊ���Դ��룬������ϵͳ
#endif
}

void Temp_ReadTASK(void)
{
	if(Temp_Flag)
	{
		Temp_Flag=0;
		InputRegister.SystemWorkStatus=(InputRegister.SystemWorkStatus & 0xff00)|0x21;
		InputRegister.Temperature=Temp_Read();
	}
}

/*******************************************************************************
* Function Name  : main
* Description    : ������
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
int main(void)
{
#ifdef DUBEG_TEST
	float AltitudeTemp1,PA1,ADCValue1;
#endif
	
	stdout = &mystdout;
    Init_Devices();
	Init_Parameter();

	InputRegister.Temperature=Temp_Read();   //�ϵ��ʼ����ִ��һ���¶Ȳ�������ֹ�¶������г������䡣
	//SenPow_OFF;
	
	while(1)
    {
		ProcessTask();		//��������
		Temp_ReadTASK();	//�¶Ȳ�������ÿ��5s��һ��
		ClC_WatchDogTask();	//ι������
		
		
		
		//����Ϊ���Դ���
#ifdef DUBEG_TEST
		U485TX;Delay_ms(10);
		ADCValue1 = ADCSmaple();
		printf("ADCValue1:%fmV\r\n",ADCValue1);		
		PA1=(ADCValue1*20000)/KeepRegister.Sensor_FS_Val;//�ڴ˴�����ѹ�źŻ����ѹ��ֵ����λPa
		printf("PA1:%f\r\n",PA1);
		printf("KeepRegister.LiquidDensity:%f\r\n",KeepRegister.LiquidDensity);
		printf("KeepRegister.LocalAccelerationOfGravity:%f\r\n",KeepRegister.LocalAccelerationOfGravity);
		AltitudeTemp1= ( PA1/(KeepRegister.LiquidDensity*1000*KeepRegister.LocalAccelerationOfGravity) );//p=��gh+�˵صĴ���ѹ ��:ˮ��1.0X1000 Kg/m3 g��9.8N/kg �߶�:mm
		printf("AltitudeTemp1:%f\r\n",AltitudeTemp1);
		AltitudeTemp1 =AltitudeTemp1*1000;   //ת����mm
		printf("Altitude %f    Temp:%f\r\n",AltitudeTemp1,InputRegister.Temperature);
		printf("Sensor_FS_Val:%fmv\r\n\r\n",KeepRegister.Sensor_FS_Val);
		Delay_ms(1850);
		U485RX;Delay_ms(100);
#endif
    }
}


