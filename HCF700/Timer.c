/*
 * Timer.c
 *
 * Created: 2016/3/2 9:26:48
 *  Author: haiqing
 */ 

#include "main.h"

volatile unsigned int _MS=0;
volatile unsigned int _MSCount=0;
volatile unsigned int SysTick_Count=0;

//TIMER1 initialize - prescale:1
// WGM: 0) Normal, TOP=0xFFFF
// desired value: 1mSec
// actual value:  1.000mSec (0.0%)
void timer1_init(void)
{	
	TCNT1H = 0x00;
	TCNT1L = 0x00;
	TCCR1A = 0x00;
	TCCR1B = 0x01;//start Timer
	TCCR1C = 0x00;
	OCR1AH = 0x1C;
	OCR1AL = 0xCC;
	OCR1BH = 0x1C;
	OCR1BL = 0xCC;
	ICR1H  = 0x1C;
	ICR1L  = 0xCC;
	TIMSK1 = 0x01;
	TIFR1  = 0x00;
	GTCCR  = 0x00;
	TCNT1H = 0xE3;//setup
	TCNT1L = 0x34;
}
//#pragma interrupt_handler timer1_ovf_isr:iv_TIM1_OVF
SIGNAL(TIMER1_OVF_vect)
{
	TCNT1H = 0xE3; //reload counter high value
	TCNT1L = 0x34; //reload counter low value
	if(_MS) _MS--;
	_MSCount++;
	if(SysTick_Count++ == 1950) LED1_ON;
	else if(SysTick_Count >= 2000){SysTick_Count=0;LED1_OFF;};
	if(_MSCount>=1000)
	{
		_MSCount=0;
		InputRegister.SystemWorkTime++;//系统工作时间++
		WorkTime++;
		Temp_Count++;
		if(WorkTime >= 913)//15分钟内收到指令，清零WorkTime
		{
			WorkTimeOutFlag=1;
		}
		if(Temp_Count>=5)   //温度采样标志位，每5S采集一次
		{
			Temp_Count=0;
			Temp_Flag =1;
		}
	}
	if(ModbusIntervalTime) ModbusIntervalTime--;
	else
	{
		if(ModbusDataPackage.DataLen && !ModbusDataPackage.DataFlag)
		ModbusDataPackage.DataFlag = 1;
	}
}