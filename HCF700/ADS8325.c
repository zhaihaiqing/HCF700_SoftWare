/* Includes ------------------------------------------------------------------*/
#include "main.h"

void ADS8325_Start(void)
{
	ADS8325_SCK_L;
	ADS8325_CS_H;
}
void ADS8325_Stop(void)
{
	ADS8325_SCK_L;
	ADS8325_CS_L;
}
void ADS8325_Reset(void)
{
	ADS8325_SCK_L;
	ADS8325_CS_H;
}
uint16_t ADS8325_GetData(void)
{
		uint8_t i,j;
		uint32_t raw_data = 0;

			ADS8325_CS_L;
			// Clock 0~5 for sample, clock 6~21 for conversion and data output
			for(j=0;j<20;j++){ __asm("nop");}
			for (i = 0; i < 22; i++)
			{		
				ADS8325_SCK_H;
				for(j=0;j<20;j++){ __asm("nop");}
				raw_data <<=1;
				raw_data |= ReadADS8325;	// Read a byte from data out
				ADS8325_SCK_L;
				for(j=0;j<20;j++){ __asm("nop");}
			}
			ADS8325_CS_H;
			for(i = 0;i < 3;i++)
			{
				ADS8325_SCK_H;
				for(j=0;j<20;j++){ __asm("nop");}
				ADS8325_SCK_L;
				for(j=0;j<20;j++){ __asm("nop");}
			}
			
		return raw_data & 0xffff;
}



