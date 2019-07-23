/************************ (C) COPYRIGHT SWS *******************************
* File Name          : swsbus.h
* Author             : ͯ��
* Version            : V2.0
* Date               : 2012.11.30
* Description        : SWSBUSЭ�����ͷ�ļ�
********************************************************************************/
#ifndef __SWSBUS_H
#define __SWSBUS_H

u8  recive_status = 0;



//�û������ã��ɸ�����Ҫ����
#define   RS485_TIM       TIM1                 //RS485ʹ��TIM
#define   RS485_USART     USART3               //RS485ʹ�ô���
#define   GPIO_USART      GPIOB                //RS485ʹ��GPIO(Ҫ��RD,TX,RXʹ��ͬһ��GPIO)
#define   USART_RD        GPIO_Pin_9           //���ͽ���ʹ�ܶ˿�
#define   USART_TX        GPIO_Pin_10          //���ڷ����źŶ˿�
#define   USART_RX        GPIO_Pin_11          //���ڽ����źŶ˿�
#define   RS485_RE        GPIO_ResetBits(GPIO_USART,USART_RD)//485����ʹ��
#define   RS485_SE        GPIO_SetBits(GPIO_USART,USART_RD)  //485����ʹ��
#define   BaudRate        115200                             //���ò�����

#define   LocalAddress      0x06        // ������ַ
#define   BroadcastAddress  0xAA        // �㲥��ַ

#define   TxBufferSize      20          //���ͻ��������鳤��
#define   RxBufferSize      20          //���ջ��������鳤��
#define   WR0_BufferSize    6           // д����0�Ľӿ����鳤��
#define   WR1_BufferSize    6           // д����1�Ľӿ����鳤��
#define   WR2_BufferSize    5           // д����2�Ľӿ����鳤��
#define   WR3_BufferSize    2           // д����3�Ľӿ����鳤��
#define   WR4_BufferSize    16          // д����4�Ľӿ����鳤��
#define   WR5_BufferSize    16          // д����5�Ľӿ����鳤��
#define   WR6_BufferSize    16          // д����6�Ľӿ����鳤��
#define   WR7_BufferSize    16          // д����7�Ľӿ����鳤��

#define   RD0_BufferSize    12          // ������0�Ľӿ����鳤��
#define   RD1_BufferSize    14          // ������1�Ľӿ����鳤��
#define   RD2_BufferSize    4           // ������2�Ľӿ����鳤��
#define   RD3_BufferSize    6           // ������3�Ľӿ����鳤��
#define   RD4_BufferSize    6           // ������4�Ľӿ����鳤��
#define   RD5_BufferSize    16          // ������5�Ľӿ����鳤��
#define   RD6_BufferSize    16          // ������6�Ľӿ����鳤��
#define   RD7_BufferSize    6           // ������7�Ľӿ����鳤��
#define   MIN_BufferSize    0x04        // ���ж���д����Ľӿ��������С����


//�����붨�壬�û����ɸ���
#define   BUS_INI     0x20     // ͨ�ų�ʼ��״̬
#define   BUS_ASK     0x21     // ��ӻ�����������Ϣ
#define   CRC_ERR     0x22     // CRCУ�����

#define   BUS_WR0     0x30     // д����0
#define   BUS_WR1     0x31     // д����1
#define   BUS_WR2     0x32     // д����2
#define   BUS_WR3     0x33     // д����3
#define   BUS_WR4     0x34     // д����4
#define   BUS_WR5     0x35     // д����5
#define   BUS_WR6     0x36     // д����6
#define   BUS_WR7     0x37     // д����7

#define   BUS_RD0     0x40     // ������0
#define   BUS_RD1     0x41     // ������1
#define   BUS_RD2     0x42     // ������2
#define   BUS_RD3     0x43     // ������3
#define   BUS_RD4     0x44     // ������4
#define   BUS_RD5     0x45     // ������5
#define   BUS_RD6     0x46     // ������6
#define   BUS_RD7     0x47     // ������7

extern u8     TxBuffer[TxBufferSize]; //�������ݻ���
extern u8     RxBuffer[RxBufferSize]; //�������ݻ���
extern u8     TxBufferLength;         //�������ݳ���
extern u8     RxBufferLength;         //�������ݳ���
extern struct COM_Array BUS_Buffer;

struct COM_Array
{
  u8 WR0[WR0_BufferSize];    // д����0�Ľӿ�����
  u8 WR1[WR1_BufferSize];    // д����1�Ľӿ�����
  u8 WR2[WR2_BufferSize];    // д����2�Ľӿ�����
  u8 WR3[WR3_BufferSize];    // д����3�Ľӿ�����
  u8 WR4[WR4_BufferSize];    // д����4�Ľӿ�����
  u8 WR5[WR5_BufferSize];    // д����5�Ľӿ�����
  u8 WR6[WR6_BufferSize];    // д����6�Ľӿ�����
  u8 WR7[WR7_BufferSize];    // д����7�Ľӿ�����

  u8 RD0[RD0_BufferSize];    // ������0�Ľӿ�����
  u8 RD1[RD1_BufferSize];    // ������1�Ľӿ�����
  u8 RD2[RD2_BufferSize];    // ������2�Ľӿ�����
  u8 RD3[RD3_BufferSize];    // ������3�Ľӿ�����
  u8 RD4[RD4_BufferSize];    // ������4�Ľӿ�����
  u8 RD5[RD5_BufferSize];    // ������5�Ľӿ�����
  u8 RD6[RD6_BufferSize];    // ������6�Ľӿ�����
  u8 RD7[RD7_BufferSize];    // ������7�Ľӿ�����
};

void RS485_Init(void);
void BUS_DataResponse(void);             //�������ݽ���
u16  CRC16(u8 *Frame,u8 Length);         //CRCУ��
void RS485_SendData(u8 SendDataLength);  //��������
void TIM_IRQ(void);
void USART_IRQ(void);
void Delay_Bus(u32 dly);                 //��ʱ



#endif
/******************* (C) COPYRIGHT 2008 SWS *****END OF FILE*******************/
