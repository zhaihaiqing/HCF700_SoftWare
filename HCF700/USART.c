/*
 * CFile1.c
 *
 * Created: 2016/1/28 15:13:13
 *  Author: haiqing
 */

//UART0 initialize
// desired baud rate: 115200
// actual: baud rate:115200 (0.0%)

#include "main.h"

unsigned char Uart0Flag;
UART0_RBUF_ST	uart0_rbuf	=	{ 0, 0, };
	
volatile uint8_t ModbusIntervalTime;//modbus���������ֵ


/*******************************************************************************
* Function Name  : uart0_init
* Description    :  ���ڳ�ʼ������
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void uart0_init(void)
{
	UCSR0B = 0x00; //disable while setting baud rate
	UCSR0A = 0x00;
	UCSR0C = 0x06;  //8,n,1
	UBRR0L = (CPU_SPEED/BAUDRATE/16-1)%256; //set baud rate lo  UBRRL= (CPU_SPEED/BAUDRATE/16-1)%256;
	UBRR0H = (CPU_SPEED/BAUDRATE/16-1)/256; //set baud rate hi  UBRRH= (CPU_SPEED/BAUDRATE/16-1)/256;
	UCSR0B = 0x98; //�����ж�������������,TX���Ź���Ϊ���ڷ���
}

/*******************************************************************************
* Function Name  : USART0_PutChar
* Description    :  ���ڷ���һ���ַ�
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void USART0_PutChar(unsigned char ch)
{
	while(    !(UCSR0A & (1<<UDRE0))      );
	UDR0 = ch;
}

/*******************************************************************************
* Function Name  : USART0_PutData
* Description    :  ���ڷ������ɸ��ַ�
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void USART0_PutData(unsigned char *dat,unsigned short int len)
{
	unsigned short int i;
	for(i = 0;i < len;i++)USART0_PutChar((uint8_t)* (dat++));
}

/*******************************************************************************
* Function Name  : USART0_PutS
* Description    :  ���ڷ����ַ���
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void USART0_PutS(unsigned char *s)
{
	while(*s != '\0')USART0_PutChar(*(s++));
}

/*******************************************************************************
* Function Name  : USART0_GetCharBlock
* Description    :  ����0�����ַ�����������ģʽ�����ջ���������ȡ��
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
uint16_t USART0_GetCharBlock(uint16_t timeout)
{
	UART0_RBUF_ST *p = &uart0_rbuf;
	uint16_t to = timeout;
	while(((p->out - p->in)& (UART0_RBUF_SIZE - 1)) == 0)if(!(--to))return TIMEOUT;
	return (p->buf [(p->out++) & (UART0_RBUF_SIZE - 1)]);
}

/*******************************************************************************
* Function Name  : USART0_GetChar
* Description    : ����2�����ַ�������������ģʽ�����ջ���������ȡ��
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
uint16_t USART0_GetChar(void)
{
	UART0_RBUF_ST *p = &uart0_rbuf;
	if(((p->out - p->in) & (UART0_RBUF_SIZE - 1)) == 0) //������������
	return EMPTY;
	return USART0_GetCharBlock(1000);
}

/*******************************************************************************
* Function Name  : USART0_ClearBuf
* Description    : ��մ��ڻ���
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
uint16_t USART0_ClearBuf(void)
{
	UART0_RBUF_ST *p = &uart0_rbuf;
	p->out = 0;
	p->in = 0;
	return 1;
}

/*******************************************************************************
* Function Name  : usart_putchar_printf
* Description    : printf����ʵ�ֺ�����
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
int usart_putchar_printf(char var, FILE *stream) {
	if (var == '\n')USART0_PutChar('\r');
	USART0_PutChar(var);
	return 0;
}

/*******************************************************************************
* Function Name  : ISR(USART_RXC_vect)
* Description    : ���ڽ����жϷ�����
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
ISR(USART_RX_vect)
{
	unsigned char Uart_Get_Data;		//���ڽ��յ�����
	UART0_RBUF_ST *p = &uart0_rbuf;		//����*p��ʹ��ָ�뷵�ض������
	
	Uart_Get_Data = UDR0;			//�����жϣ����UDR�Ĵ����н��յ�����ֵ
	//LED2_ON;
	if(!ModbusDataPackage.DataFlag) //�ж��Ƿ��Ѿ����������İ�����,���û�������������
	{
		//�������Ϊ��,��ʾ��һ������,����δ�ﵽ����ʱ�������ж�Ϊ��������
		if((((p->out - p->in) & (UART0_RBUF_SIZE - 1)) == 0) || ModbusIntervalTime)
		{
			ModbusIntervalTime = MODBUS_INTERVAL_TIME;//��ֵ��������	
			if((p->in - p->out)<UART0_RBUF_SIZE)//�����ݷ��뻺��
			{
				p->buf[p->in & (UART0_RBUF_SIZE-1)] = Uart_Get_Data;//�����ݷ��뵽buf��
				p->in++;
			}
			ModbusDataPackage.DataLen  = (p->in - p->out) & (UART0_RBUF_SIZE - 1);//��ȡ���ݳ���
		}
	}	
}








 
