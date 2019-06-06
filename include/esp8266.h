#ifndef __ESP8266_H
#define __ESP8266_H

#define   ESP8266_TIM         TIM3                 //RS485ʹ��TIM
#define   ESP8266_USART       USART1               //RS485ʹ�ô���
#define   ESP8266_GPIO_USART  GPIOA                //RS485 Rx Txʹ��GPIO
#define   ESP8266_USART_TX    GPIO_Pin_9           //���ڷ����źŶ˿�
#define   ESP8266_USART_RX    GPIO_Pin_10          //���ڽ����źŶ˿�
#define   ESP8266_BaudRate    9600//115200                             //���ò�����        

#define   ESP8266_BufferSize    255

extern u8 ESP8266_TxBuffer[],ESP8266_RxBuffer[],ESP8266_RxBufferLength,ESP8266_Received;

void esp8266_Init(void);
void esp8266_SendData(u16);
void ESP8266_TIM_IRQ(void);
void ESP8266_USART_IRQ(void);
void Esp8266_CMD(char *);
char Esp8266_Config(void);

#endif