/*
 * DIS100.h
 *
 * Created: 2016/1/28 16:32:54
 *  Author: haiqing
 */ 
#ifndef DIS100_H_
#define DIS100_H_
#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdio.h>
#include <avr/pgmspace.h>
#include <math.h>
#include "Usart.h"
#include "ADS8325.h"
#include "I2C.h"
#include "Timer.h"
#include "Temp.h"

//#define DUBEG_TEST           //Ԥ���룬printfѭ����ӡ����߶ȡ��¶�ֵ
//#define DUBEG_TEST_ORIGINA   //Ԥ���룬ÿ�β�����ӡ��ADCԭʼ����ֵ��������100�����ݣ�
//#define DUBEG_TEST_ORDER     //Ԥ���룬�鿴���յ�������

#define debug
#ifdef debug
//#define log_info(...)    printf(__VA_ARGS__);
#define log_info(...)     U485TX;Delay_ms(10);printf(__VA_ARGS__);U485RX;Delay_ms(100)
#else
#define log_info(...)
#endif

#define _WDR()	__asm("wdr");

//��С��ת��
#define htons(n) ((((n) & 0x00ff) << 8) | (((n) & 0xff00) >> 8))
#define ntohs(n) htons(n)
#define htonl(n) ((((n) & 0x000000ff) << 24)|  \
				 (((n) & 0x0000ff00) << 8)  |  \
				 (((n) & 0x00ff0000) >> 8)  |  \
				 (((n) & 0xff000000) >> 24))
#define ntohl(n) htonl(n)
#define htond(n) (htonl(n & 0xffffffff) << 32) | htonl(n >> 32)
#define ntohd(n) htond(n)

typedef union{									//float�����ݴ�С�˸�ʽת��
	float f;
	char c[4];
}FLOAT_CONV;


#define DEVICETYPE					  700	   //��������
#define SOFTWAREVERSION			      0x0415   //����汾�Ŷ�����򣺰汾�Ź���λ��ʾ������1.2.3����ʾ����Ϊ���߰�λ��ʾ1���Ͱ�λ��ʾ23,���еͰ�λ����ʾ99 V4.1.4

#define DefaultDeviceADDR			  0x01	   //Ĭ��������ַ
#define DefaultLiquidDensity		  1.0	   //Ĭ��Һ���ܶ�
#define DefaultAltitude				  0.0	   //Ĭ�ϳ�ʼ�߶�
#define DefaultAccelerationOfGravity  9.8015   //Ĭ�ϵ����������ٶȣ�������
#define DefaultSensor_Range			  20       //Ĭ�ϴ���������Ϊ20KPa

#define FPOWERONFLAG_BASEADDR		  32	   //�״ο�����־λ��ŵĵ�ַ
#define KREEPROM_BASEADDR			  64	   //���ּĴ����洢����ַ
#define SUPERMODE_FLAG_BASEADDR       48	   //�߼�ģʽ��־λ��ŵ�ַ

#define ERROR			              0		   //ʧ��
#define SUCCESS			              1		   //�ɹ�

#define CPU_SPEED		              7372800  //ʱ��Ƶ��
#define BAUDRATE		              9600	   //���ڲ�����

#define WDI_H		PORTB |= (1<<2)
#define WDI_L		PORTB &= ~(1<<2)
#define WDI_COM		PORTB ^= (1 << 2)

#define LED1_OFF		PORTB |= (1<<0)
#define LED1_ON			PORTB &= ~(1<<0)
#define LED1_COM		PORTB ^= (1 << 0)

#define LED2_OFF		PORTB |= (1<<1)
#define LED2_ON			PORTB &= ~(1<<1)
#define LED2_COM		PORTB ^= (1 << 1)

#define SenPow_ON		PORTD |= (1<<5)
#define SenPow_OFF		PORTD &= ~(1<<5)

#define TEMPCS_H		PORTC |=(1<<3)
#define TEMPCS_L		PORTC &=~(1<<3)

#define ADS8325_CS_H	PORTC |= (1<<2)
#define ADS8325_CS_L	PORTC &= ~(1<<2)
#define ADS8325_SCK_H	PORTC |= (1<<1)
#define ADS8325_SCK_L	PORTC &= ~(1<<1)
#define ReadADS8325		(PINC & (1<<0))

#define U485TX			PORTD |= (1<<2)				
#define U485RX			PORTD &= ~(1<<2)

#define KeepRegister_Num	    (sizeof(KeepRegister)/2)
#define KeepRegister_Byte_Num    sizeof(KeepRegister)


//modbus ������
enum ModbusFunction_t
{
	ReadKeepRegistor=0x03,        //��ȡ���ּĴ���
	ReadInputRegistor=0x04,       //��ȡ����Ĵ���
	WriteSingleRegistor=0x06,     //Ԥ�õ��Ĵ���
	WriteSomeRegistor=0x10,       //Ԥ�ö�Ĵ���
	Get_SNInfo=0x40,              //��ȡSN��Ϣ
	AutoSetInitalValue=0x41,	  //�Զ����ó�ֵ
	StartSample	=0x42,			  //��������
	FactoryCalibration0=0x43,	  //����У׼��0
	FactoryCalibration1=0x44,	  //����У׼��1
	FactoryCalibration2=0x45,	  //����У׼��2
	FactoryCalibration3=0x46,	  //����У׼��3
	FactoryCalibration4=0x47,	  //����У׼��4
	FactoryCalibration5=0x48,	  //����У׼��5
	Clear_temp_corr    =0x49,     //����¶�����ϵ��
	ModeSwitch		   =0x66,     //ģʽ�л�
	FactoryParameterReset=0x68,   //�ָ���������
};

enum err_num  {
	err_Fu=1,           //��֧�ֵĹ��ܣ��������쳣��
	err_add,            //�Ĵ�����ַ����ȷ
	err_Re_VOR,         //�Ĵ���ֵ������Χ
	err_OE,             //��Ч���������쳣
	err_mode,
	//err_CON,            //ȷ��
	//err_Fu_NUM,         //���ܺ���Ч
	//err_busy            //�豸æ
};


//���ּĴ����ṹ��
typedef struct __attribute__ ((__packed__))   //�����ṹ��������ṹ����뷽ʽΪ�ֽڶ���
{
	uint16_t DeviceAddress;					//�豸��ַ
	uint16_t DeviceGroupNum;				//�豸���
	float	 OriginaLiquidAltitude;			//Һλ��ʼ�߶ȸ�λ
	float	 Reserve1and2;					//Һ���ܶ�
	uint16_t Sensor_Range;					//���������̣�KPa��
	uint16_t Liquid_Sec;						//����3
	float    LocalAccelerationOfGravity;	//�����������ٶ�ֵ����λ��
	float    Sensor_FS_Val;                 //�ڲ�OEMѹ�����������������ֵ���ο�ֵ��
	float    MV[6];                         //���У׼�����6����
	float    Temp_T0;                       //��ʼ�¶�ֵ
	float    LTC0[5];           // 0����������ֵ-�¶�����ϵ����4��     Load_Temp_Curve
	float    LTC1[5];           // 1/5����������ֵ-�¶�����ϵ����4��
	float    LTC2[5];           // 2/5����������ֵ-�¶�����ϵ����4��
	float    LTC3[5];           // 3/5����������ֵ-�¶�����ϵ����4��
	float    LTC4[5];           // 4/5����������ֵ-�¶�����ϵ����4��
	float    LTC5[5];           // ������������ֵ-�¶�����ϵ����4��
	
	
}KeepRegister_t;

//����Ĵ����ṹ��
typedef struct __attribute__ ((__packed__))
{
	uint16_t DeviceType;					//�豸����
	uint16_t SoftwareVersion;				//����汾
	uint32_t SystemWorkTime;				//ϵͳ����ʱ��
	uint16_t SystemWorkStatus;				//ϵͳ����״̬
	uint16_t Reserve1;						//����1
	float    AltitudeDifference;			//����ֵҺλ�߶Ȳ�
	float    Temperature;					//�¶�ֵ
	float    LiquidAltitudeAbsoluteValue;	//Һλ����ֵ�߶�ֵ
	float    ADCOriginalValue;				//ADCԭʼ��������
	float    PA;
}InputRegister_t;

//modbus ��Ϣ���սṹ��
typedef struct __attribute__ ((__packed__))
{
	uint8_t DataFlag;					//���ݱ��,�Ƿ�������
	uint8_t DataLen;					//���ݳ���
	uint8_t dat[UART0_RBUF_SIZE];	    //���ݻ���
}ModbusDataPackage_t;


extern volatile KeepRegister_t		KeepRegister;		//���屣�ּĴ���
extern volatile InputRegister_t		InputRegister;		//��������Ĵ���
extern volatile ModbusDataPackage_t ModbusDataPackage;	//����modbus���ջ���
extern volatile unsigned int   WorkTime;
extern volatile unsigned char  WorkTimeOutFlag;
extern volatile unsigned char  Temp_Flag;            
extern volatile unsigned int   Temp_Count;		
	

void Delay_ms(unsigned int ms);

#endif /* DIS100_H_ */