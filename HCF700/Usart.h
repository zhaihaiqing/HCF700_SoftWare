/*
 * Usart.h
 *
 * Created: 2016/1/29 10:37:12
 *  Author: haiqing
 */ 


#ifndef USART_H_
#define USART_H_

#define EMPTY 	0xFFFF
#define TIMEOUT 0xFFFF

#define UART0_RBUF_SIZE   256		//Ҫ��2 ��������
#if UART0_RBUF_SIZE < 2
#error UART0_RBUF_SIZE is too small.  It must be larger than 1.
#elif ((UART0_RBUF_SIZE & (UART0_RBUF_SIZE-1)) != 0)
#error UART0_RBUF_SIZE must be a power of 2.
#endif

#define UART_FLAG_TimeOut  0x50000   //��ʱ����
#define MODBUS_INTERVAL_TIME 5  //��������ʱ������ֵ,5ms

#define U485SendData(dat,len)   {U485TX;Delay_ms(1);USART0_PutData(dat,len); Delay_ms(10); U485RX;}//485���ݷ���

extern volatile uint8_t ModbusIntervalTime;//modbus���������ֵ





typedef struct uart0_rbuf_st
{
	unsigned int in;							//����
	unsigned int out;							//���
	unsigned char  buf [UART0_RBUF_SIZE];		//�������ռ�
}UART0_RBUF_ST;



extern UART0_RBUF_ST	uart0_rbuf;

void USART0_PutData(unsigned char *dat,unsigned short int len);
void uart0_init(void);
uint16_t USART0_ClearBuf(void);



#endif /* USART_H_ */