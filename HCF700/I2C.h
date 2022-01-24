/*
 * I2C.h
 *
 * Created: 2016/1/29 14:16:55
 *  Author: haiqing
 */ 

#ifndef I2C_H_
#define I2C_H_

#define I2C_Speed			50000			//I2C数据速度
#define TWBR_INIT			(((CPU_SPEED/I2C_Speed)-16)/2)
#define EEPROM_Addr			0xa0			//EEPROM地址
#define EEDEVADR		    0xa0
#define I2C1_SLAVE_ADDRESS7 0x30			//自身设备地址
#define I2C_PAGESIZE		32				//AT24C128C每页64byte
#define I2C_BUFFSIZE		35
#define I2C_FLAG_TimeOut  	0xffff			//超时常量 0x5000
#define I2C_LONG_TimeOut  	(10 * I2C_FLAG_TimeOut)

// 主机模式启动状态码
// #define    TW_START                 0X08
// #define    TW_RESTART               0X10
// 主机发送模式状态码
// #define    TW_MT_SLA_ACK            0X18   //SLA+W 已发送,接收到ACK
// #define    TW_MT_SLA_NACK           0X20   //SLA+W 已发送,接收到NOT ACK
// #define    TW_MT_DATA_ACK           0X28   //数据已发送,接收到ACK
// #define    TW_MT_DATA_NACK          0X30   //数据已发送,接收到NOT ACK
// #define    TW_MT_ARB_LOST           0X38   //SLA+W 或数据的仲裁失败
// 主机接收模式状态码
// #define    TW_MR_ARB_LOST           0X38   //SLA+R 或数据的仲裁失败
// #define    TW_MR_SLA_ACK            0X40   //SLA+R 已发送,接收到ACK
// #define    TW_MR_SLA_NACK           0X48   //SLA+R 已发送,接收到NOT ACK
// #define    TW_MR_DATA_ACK           0X50   //数据已接收,接收到ACK
// #define    TW_MR_DATA_NACK          0X58   //数据已接收,接收到NOT ACK
// 从机接收模式状态码
// #define    TW_SR_SLA_ACK            0X60   //自己的SLA+W 已经被接收,ACK 已返回
// #define    TW_SR_ARB_LOST_SLA_ACK   0X68   //SLA+R/W 作为主机的仲裁失败；自己的SLA+W 已经被接收,ACK 已返回
// #define    TW_SR_GCALL_ACK          0X70   //接收到广播地址,ACK 已返回
// #define    TW_SR_ARB_LOST_GCALL_ACK 0X78   //SLA+R/W 作为主机的仲裁失败；接收到广播地址ACK 已返回
// #define    TW_SR_DATA_ACK           0X80   //以前以自己的 SLA+W 被寻址；数据已经被接收ACK 已返回
// #define    TW_SR_DATA_NACK          0X88   //以前以自己的 SLA+W 被寻址；数据已经被接收NOT ACK 已返回
// #define    TW_SR_GCALL_DATA_ACK     0X90   //以前以广播方式被寻址；数据已经被接收ACK 已返回
// #define    TW_SR_GCALL_DATA_NACK    0X98   //以前以广播方式被寻址；数据已经被接收NOT ACK 已返回
// #define    TW_SR_STOP               0XA0   //在以从机工作时接收到STOP或重复START
// 从机发送模式状态码
// #define    TW_ST_SLA_ACK            0XA8   //自己的SLA+R 已经被接收ACK 已返回
// #define    TW_ST_ARB_LOST_SLA_ACK   0XB0   //SLA+R/W 作为主机的仲裁失败；自己的SLA+R 已经被接收ACK 已返回
// #define    TW_ST_DATA_ACK           0XB8   //TWDR 里数据已经发送,接收到ACK
// #define    TW_ST_DATA_NACK          0XC0   //TWDR 里数据已经发送,接收到NOT ACK
// #define    TW_ST_LAST_DATA          0XC8   //TWDR 的一字节数据已经发送(TWAE = “0”);接收到ACK
// 其他状态
// #define    TW_NO_INFO       0xF8   //没有相关的状态信息，TWINT='0'
// #define    TW_BUS_ERROR      0x00   //由于非法的START或STOP引起的总线错误

void TWI_init(void);
uint8_t EEWriteByte(uint16_t u16addr, uint8_t u8data);
uint8_t EEReadByte(uint16_t u16addr, uint8_t *u8data);
uint8_t EEWrite(unsigned short addr, unsigned char *pbuff,unsigned short length);
uint8_t EEReadPage(uint8_t page, uint8_t *u8data);
unsigned char  EEWriteOnePageNByte(unsigned char addr,unsigned char *pdata,unsigned char length);
unsigned char  EEReadOnePageNByte(unsigned char addr,unsigned char *pdata,unsigned char length);
uint8_t EERead(unsigned short addr, unsigned char *pbuff,unsigned short length);
uint8_t EEWrite(unsigned short addr, unsigned char *pbuff,unsigned short length);
void EEErase(unsigned short addr, unsigned short length);



#endif /* I2C_H_ */