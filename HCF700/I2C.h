/*
 * I2C.h
 *
 * Created: 2016/1/29 14:16:55
 *  Author: haiqing
 */ 

#ifndef I2C_H_
#define I2C_H_

#define I2C_Speed			50000			//I2C�����ٶ�
#define TWBR_INIT			(((CPU_SPEED/I2C_Speed)-16)/2)
#define EEPROM_Addr			0xa0			//EEPROM��ַ
#define EEDEVADR		    0xa0
#define I2C1_SLAVE_ADDRESS7 0x30			//�����豸��ַ
#define I2C_PAGESIZE		32				//AT24C128Cÿҳ64byte
#define I2C_BUFFSIZE		35
#define I2C_FLAG_TimeOut  	0xffff			//��ʱ���� 0x5000
#define I2C_LONG_TimeOut  	(10 * I2C_FLAG_TimeOut)

// ����ģʽ����״̬��
// #define    TW_START                 0X08
// #define    TW_RESTART               0X10
// ��������ģʽ״̬��
// #define    TW_MT_SLA_ACK            0X18   //SLA+W �ѷ���,���յ�ACK
// #define    TW_MT_SLA_NACK           0X20   //SLA+W �ѷ���,���յ�NOT ACK
// #define    TW_MT_DATA_ACK           0X28   //�����ѷ���,���յ�ACK
// #define    TW_MT_DATA_NACK          0X30   //�����ѷ���,���յ�NOT ACK
// #define    TW_MT_ARB_LOST           0X38   //SLA+W �����ݵ��ٲ�ʧ��
// ��������ģʽ״̬��
// #define    TW_MR_ARB_LOST           0X38   //SLA+R �����ݵ��ٲ�ʧ��
// #define    TW_MR_SLA_ACK            0X40   //SLA+R �ѷ���,���յ�ACK
// #define    TW_MR_SLA_NACK           0X48   //SLA+R �ѷ���,���յ�NOT ACK
// #define    TW_MR_DATA_ACK           0X50   //�����ѽ���,���յ�ACK
// #define    TW_MR_DATA_NACK          0X58   //�����ѽ���,���յ�NOT ACK
// �ӻ�����ģʽ״̬��
// #define    TW_SR_SLA_ACK            0X60   //�Լ���SLA+W �Ѿ�������,ACK �ѷ���
// #define    TW_SR_ARB_LOST_SLA_ACK   0X68   //SLA+R/W ��Ϊ�������ٲ�ʧ�ܣ��Լ���SLA+W �Ѿ�������,ACK �ѷ���
// #define    TW_SR_GCALL_ACK          0X70   //���յ��㲥��ַ,ACK �ѷ���
// #define    TW_SR_ARB_LOST_GCALL_ACK 0X78   //SLA+R/W ��Ϊ�������ٲ�ʧ�ܣ����յ��㲥��ַACK �ѷ���
// #define    TW_SR_DATA_ACK           0X80   //��ǰ���Լ��� SLA+W ��Ѱַ�������Ѿ�������ACK �ѷ���
// #define    TW_SR_DATA_NACK          0X88   //��ǰ���Լ��� SLA+W ��Ѱַ�������Ѿ�������NOT ACK �ѷ���
// #define    TW_SR_GCALL_DATA_ACK     0X90   //��ǰ�Թ㲥��ʽ��Ѱַ�������Ѿ�������ACK �ѷ���
// #define    TW_SR_GCALL_DATA_NACK    0X98   //��ǰ�Թ㲥��ʽ��Ѱַ�������Ѿ�������NOT ACK �ѷ���
// #define    TW_SR_STOP               0XA0   //���Դӻ�����ʱ���յ�STOP���ظ�START
// �ӻ�����ģʽ״̬��
// #define    TW_ST_SLA_ACK            0XA8   //�Լ���SLA+R �Ѿ�������ACK �ѷ���
// #define    TW_ST_ARB_LOST_SLA_ACK   0XB0   //SLA+R/W ��Ϊ�������ٲ�ʧ�ܣ��Լ���SLA+R �Ѿ�������ACK �ѷ���
// #define    TW_ST_DATA_ACK           0XB8   //TWDR �������Ѿ�����,���յ�ACK
// #define    TW_ST_DATA_NACK          0XC0   //TWDR �������Ѿ�����,���յ�NOT ACK
// #define    TW_ST_LAST_DATA          0XC8   //TWDR ��һ�ֽ������Ѿ�����(TWAE = ��0��);���յ�ACK
// ����״̬
// #define    TW_NO_INFO       0xF8   //û����ص�״̬��Ϣ��TWINT='0'
// #define    TW_BUS_ERROR      0x00   //���ڷǷ���START��STOP��������ߴ���

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