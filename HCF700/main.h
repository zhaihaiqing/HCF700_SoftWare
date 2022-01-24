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

//#define DUBEG_TEST           //预编译，printf循环打印输出高度、温度值
//#define DUBEG_TEST_ORIGINA   //预编译，每次采样打印出ADC原始采样值（排序后的100个数据）
//#define DUBEG_TEST_ORDER     //预编译，查看接收到的命令

#define debug
#ifdef debug
//#define log_info(...)    printf(__VA_ARGS__);
#define log_info(...)     U485TX;Delay_ms(10);printf(__VA_ARGS__);U485RX;Delay_ms(100)
#else
#define log_info(...)
#endif

#define _WDR()	__asm("wdr");

//大小端转换
#define htons(n) ((((n) & 0x00ff) << 8) | (((n) & 0xff00) >> 8))
#define ntohs(n) htons(n)
#define htonl(n) ((((n) & 0x000000ff) << 24)|  \
				 (((n) & 0x0000ff00) << 8)  |  \
				 (((n) & 0x00ff0000) >> 8)  |  \
				 (((n) & 0xff000000) >> 24))
#define ntohl(n) htonl(n)
#define htond(n) (htonl(n & 0xffffffff) << 32) | htonl(n >> 32)
#define ntohd(n) htond(n)

typedef union{									//float型数据大小端格式转换
	float f;
	char c[4];
}FLOAT_CONV;


#define DEVICETYPE					  700	   //器件类型
#define SOFTWAREVERSION			      0x0415   //软件版本号定义规则：版本号共三位表示，例如1.2.3，表示方法为：高八位表示1，低八位表示23,其中低八位最大表示99 V4.1.4

#define DefaultDeviceADDR			  0x01	   //默认器件地址
#define DefaultLiquidDensity		  1.0	   //默认液体密度
#define DefaultAltitude				  0.0	   //默认初始高度
#define DefaultAccelerationOfGravity  9.8015   //默认当地重力加速度（北京）
#define DefaultSensor_Range			  20       //默认传感器量程为20KPa

#define FPOWERONFLAG_BASEADDR		  32	   //首次开机标志位存放的地址
#define KREEPROM_BASEADDR			  64	   //保持寄存器存储基地址
#define SUPERMODE_FLAG_BASEADDR       48	   //高级模式标志位存放地址

#define ERROR			              0		   //失败
#define SUCCESS			              1		   //成功

#define CPU_SPEED		              7372800  //时钟频率
#define BAUDRATE		              9600	   //串口波特率

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


//modbus 功能码
enum ModbusFunction_t
{
	ReadKeepRegistor=0x03,        //读取保持寄存器
	ReadInputRegistor=0x04,       //读取输入寄存器
	WriteSingleRegistor=0x06,     //预置单寄存器
	WriteSomeRegistor=0x10,       //预置多寄存器
	Get_SNInfo=0x40,              //获取SN信息
	AutoSetInitalValue=0x41,	  //自动设置初值
	StartSample	=0x42,			  //采样命令
	FactoryCalibration0=0x43,	  //出厂校准点0
	FactoryCalibration1=0x44,	  //出厂校准点1
	FactoryCalibration2=0x45,	  //出厂校准点2
	FactoryCalibration3=0x46,	  //出厂校准点3
	FactoryCalibration4=0x47,	  //出厂校准点4
	FactoryCalibration5=0x48,	  //出厂校准点5
	Clear_temp_corr    =0x49,     //清除温度修正系数
	ModeSwitch		   =0x66,     //模式切换
	FactoryParameterReset=0x68,   //恢复出厂设置
};

enum err_num  {
	err_Fu=1,           //非支持的功能（功能码异常）
	err_add,            //寄存器地址不正确
	err_Re_VOR,         //寄存器值超出范围
	err_OE,             //有效操作发生异常
	err_mode,
	//err_CON,            //确认
	//err_Fu_NUM,         //功能号无效
	//err_busy            //设备忙
};


//保持寄存器结构体
typedef struct __attribute__ ((__packed__))   //声明结构体变量，结构体对齐方式为字节对齐
{
	uint16_t DeviceAddress;					//设备地址
	uint16_t DeviceGroupNum;				//设备组号
	float	 OriginaLiquidAltitude;			//液位初始高度高位
	float	 Reserve1and2;					//液体密度
	uint16_t Sensor_Range;					//传感器量程（KPa）
	uint16_t Liquid_Sec;						//保留3
	float    LocalAccelerationOfGravity;	//当地重力加速度值（高位）
	float    Sensor_FS_Val;                 //内部OEM压力传感器满量程输出值（参考值）
	float    MV[6];                         //五阶校准所需的6个点
	float    Temp_T0;                       //初始温度值
	float    LTC0[5];           // 0负载下修正值-温度曲线系数，4阶     Load_Temp_Curve
	float    LTC1[5];           // 1/5负载下修正值-温度曲线系数，4阶
	float    LTC2[5];           // 2/5负载下修正值-温度曲线系数，4阶
	float    LTC3[5];           // 3/5负载下修正值-温度曲线系数，4阶
	float    LTC4[5];           // 4/5负载下修正值-温度曲线系数，4阶
	float    LTC5[5];           // 满负载下修正值-温度曲线系数，4阶
	
	
}KeepRegister_t;

//输入寄存器结构体
typedef struct __attribute__ ((__packed__))
{
	uint16_t DeviceType;					//设备类型
	uint16_t SoftwareVersion;				//软件版本
	uint32_t SystemWorkTime;				//系统工作时间
	uint16_t SystemWorkStatus;				//系统工作状态
	uint16_t Reserve1;						//保留1
	float    AltitudeDifference;			//减初值液位高度差
	float    Temperature;					//温度值
	float    LiquidAltitudeAbsoluteValue;	//液位绝对值高度值
	float    ADCOriginalValue;				//ADC原始采样度数
	float    PA;
}InputRegister_t;

//modbus 消息接收结构体
typedef struct __attribute__ ((__packed__))
{
	uint8_t DataFlag;					//数据标记,是否有数据
	uint8_t DataLen;					//数据长度
	uint8_t dat[UART0_RBUF_SIZE];	    //数据缓存
}ModbusDataPackage_t;


extern volatile KeepRegister_t		KeepRegister;		//定义保持寄存器
extern volatile InputRegister_t		InputRegister;		//定义输入寄存器
extern volatile ModbusDataPackage_t ModbusDataPackage;	//定义modbus接收缓存
extern volatile unsigned int   WorkTime;
extern volatile unsigned char  WorkTimeOutFlag;
extern volatile unsigned char  Temp_Flag;            
extern volatile unsigned int   Temp_Count;		
	

void Delay_ms(unsigned int ms);

#endif /* DIS100_H_ */