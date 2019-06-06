/************************ (C) COPYRIGHT SWS *******************************
* File Name          : swsbus.h
* Author             : ͯ��
* Version            : V2.2
* Date               : 2015.10.24
* Description        : SWSBUSЭ�����ͷ�ļ�
********************************************************************************/
#ifndef __SWSBUS_H
#define __SWSBUS_H

//�û������ã��ɸ�����Ҫ����
/***************************IAP����ʹ��****************************************/
#define   IAP_RESET_EN                         //ʹ��IAP������Ч
/***************************ͨ��ָʾ����˸ʹ��****************************************/
//#define   RS485_LED_EN                         //��ͨ��ָʾ����Ч
#define   RS485_LED_GPIO  GPIOE                 //ͨ��ָʾ��GPIO
#define   RS485_LED_PIN   GPIO_Pin_5
#define   RS485_LED_STATE GPIO_ReadInputDataBit(RS485_LED_GPIO,RS485_LED_PIN)  
#define   RS485_LED_ON    GPIO_ResetBits(RS485_LED_GPIO,RS485_LED_PIN)
#define   RS485_LED_OFF   GPIO_SetBits(RS485_LED_GPIO,RS485_LED_PIN)  
/***************************ͨ�Ŷ�ʱ�����˿�����*******************************/
#define   RS485_TIM       TIM3                 //RS485ʹ��TIM
#define   RS485_USART     USART2               //RS485ʹ�ô���
#define   GPIO_USART      GPIOA                //RS485 Rx Txʹ��GPIO
#define   GPIO_RD         GPIOA                //RS485 RD ʹ��GPIO
#define   USART_RD        GPIO_Pin_4           //���ͽ���ʹ�ܶ˿�
#define   USART_TX        GPIO_Pin_2           //���ڷ����źŶ˿�
#define   USART_RX        GPIO_Pin_3           //���ڽ����źŶ˿�
#define   RS485_RE        GPIO_ResetBits(GPIO_RD,USART_RD)//485����ʹ��
#define   RS485_DE        GPIO_SetBits(GPIO_RD,USART_RD)  //485����ʹ��
#define   BaudRate        115200                             //���ò�����        
/***************************ͨ�ŵ�ַ����****************************************/
#define   LocalAddress      0x28        // ������ַ
#define   BroadcastAddress  0xAA        // �㲥��ַ
/***************************�ӿ���������****************************************/
#define   TxBufferSize      40          //���ͻ��������鳤��
#define   RxBufferSize      40          //���ջ��������鳤��
#define   WR0_BufferSize    7           // д����0�Ľӿ����鳤�ȣ�Э�鳤������+1��
#define   WR1_BufferSize    1           // д����1�Ľӿ����鳤�ȣ�Э�鳤������+1��
#define   WR2_BufferSize    1           // д����2�Ľӿ����鳤�ȣ�Э�鳤������+1��
#define   WR3_BufferSize    1           // д����3�Ľӿ����鳤�ȣ�Э�鳤������+1��
#define   WR4_BufferSize    1          // д����4�Ľӿ����鳤�ȣ�Э�鳤������+1��
#define   WR5_BufferSize    1          // д����5�Ľӿ����鳤�ȣ�Э�鳤������+1��
#define   WR6_BufferSize    1          // д����6�Ľӿ����鳤�ȣ�Э�鳤������+1��
#define   WR7_BufferSize    7          // д����7�Ľӿ����鳤�ȣ�Э�鳤������+1��
#define   WR8_BufferSize    1           // д����7�Ľӿ����鳤�ȣ�Э�鳤������+1��

#define   RD0_BufferSize    18          // ������0�Ľӿ����鳤��
#define   RD1_BufferSize    1          // ������1�Ľӿ����鳤��
#define   RD2_BufferSize    1           // ������2�Ľӿ����鳤��
#define   RD3_BufferSize    1           // ������3�Ľӿ����鳤��
#define   RD4_BufferSize    1           // ������4�Ľӿ����鳤��
#define   RD5_BufferSize    1           // ������5�Ľӿ����鳤��
#define   RD6_BufferSize    1           // ������6�Ľӿ����鳤��
#define   RD7_BufferSize    6           // ������7�Ľӿ����鳤��
#define   MIN_BufferSize    4           // ���ж���д����Ľӿ��������С����

/*******************************************************************************
*******************************************************************************/
//�����붨�壬�û����ɸ���
#define   BUS_INI     0x20     // ͨ�ų�ʼ��״̬
#define   BUS_ASK     0x21     // ��ӻ�����������Ϣ

#define   BUS_WR0     0x30     // д����0
#define   BUS_WR1     0x31     // д����1
#define   BUS_WR2     0x32     // д����2
#define   BUS_WR3     0x33     // д����3
#define   BUS_WR4     0x34     // д����4
#define   BUS_WR5     0x35     // д����5
#define   BUS_WR6     0x36     // д����6
#define   BUS_WR7     0x37     // д����7
#define   BUS_WR8     0x38     // д����8

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
extern u8     BUS_ERR;                //�����쳣������5S������û�����ݣ���λ��1
extern u32    LCD_CON;                //������������
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
  u8 WR8[WR8_BufferSize];    // д����8�Ľӿ�����
  
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
void RS485_SendData(u16 SendDataLength);  //��������
void TIM_IRQ(void);
void USART_IRQ(void);
void Delay_Bus(u32 dly);                 //��ʱ



#endif
/******************* (C) COPYRIGHT 2008 SWS *****END OF FILE*******************/
