/*
 * ADS8325.h
 *
 * Created: 2016/1/29 13:28:54
 *  Author: haiqing
 */ 


#ifndef ADS8325_H_
#define ADS8325_H_

void ADS8325_Start(void);
void ADS8325_Stop(void);
void ADS8325_Reset(void);
uint16_t ADS8325_GetData(void);



#endif /* ADS8325_H_ */