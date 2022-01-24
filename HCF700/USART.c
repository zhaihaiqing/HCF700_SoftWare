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
	
volatile uint8_t ModbusIntervalTime;//modbus包间隔计数值


/*******************************************************************************
* Function Name  : uart0_init
* Description    :  串口初始换函数
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
	UCSR0B = 0x98; //接收中断允许，接收允许,TX引脚功能为串口发送
}

/*******************************************************************************
* Function Name  : USART0_PutChar
* Description    :  串口发送一个字符
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
* Description    :  串口发送若干个字符
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
* Description    :  串口发送字符串
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
* Description    :  串口0接收字符函数，阻塞模式（接收缓冲区中提取）
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
* Description    : 串口2接收字符函数，非阻塞模式（接收缓冲区中提取）
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
uint16_t USART0_GetChar(void)
{
	UART0_RBUF_ST *p = &uart0_rbuf;
	if(((p->out - p->in) & (UART0_RBUF_SIZE - 1)) == 0) //缓冲区空条件
	return EMPTY;
	return USART0_GetCharBlock(1000);
}

/*******************************************************************************
* Function Name  : USART0_ClearBuf
* Description    : 清空串口缓存
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
* Description    : printf功能实现函数，
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
* Description    : 串口接收中断服务函数
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
ISR(USART_RX_vect)
{
	unsigned char Uart_Get_Data;		//串口接收的数据
	UART0_RBUF_ST *p = &uart0_rbuf;		//定义*p，使用指针返回多个数据
	
	Uart_Get_Data = UDR0;			//进入中断，获得UDR寄存器中接收到的数值
	//LED2_ON;
	if(!ModbusDataPackage.DataFlag) //判断是否已经存在完整的包数据,如果没有则接收新数据
	{
		//如果缓存为空,表示第一个数据,或者未达到接收时间间隔，判定为继续接收
		if((((p->out - p->in) & (UART0_RBUF_SIZE - 1)) == 0) || ModbusIntervalTime)
		{
			ModbusIntervalTime = MODBUS_INTERVAL_TIME;//赋值计数变量	
			if((p->in - p->out)<UART0_RBUF_SIZE)//将数据放入缓存
			{
				p->buf[p->in & (UART0_RBUF_SIZE-1)] = Uart_Get_Data;//将数据放入到buf中
				p->in++;
			}
			ModbusDataPackage.DataLen  = (p->in - p->out) & (UART0_RBUF_SIZE - 1);//获取数据长度
		}
	}	
}








 
