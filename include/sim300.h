#ifndef __SIM300_H
#define __SIM300_H

#define   SIM300_TIM         TIM3                 //RS485使用TIM
#define   SIM300_USART       USART1               //RS485使用串口
#define   SIM300_GPIO_USART  GPIOA                //RS485 Rx Tx使用GPIO
#define   SIM300_RST         GPIO_Pin_3           //SIM300复位
#define   SIM300_PWR         GPIO_Pin_2          //SIM300电源
#define   SIM300_USART_TX    GPIO_Pin_9           //串口发送信号端口
#define   SIM300_USART_RX    GPIO_Pin_10          //串口接收信号端口
#define   SIM300_BaudRate    115200                             //设置波特率        

#define   SIM300_RST_H       GPIO_SetBits(SIM300_GPIO_USART,SIM300_RST)
#define   SIM300_RST_L       GPIO_ResetBits(SIM300_GPIO_USART,SIM300_RST)



#define   SIM300_BufferSize    255

extern u8 SIM300_TxBuffer[],SIM300_RxBuffer[],SIM300_RxBufferLength,SIM300_Received;

void sim300_Init(void);
void sim300_SendData(u16);
void SIM300_TIM_IRQ(void);
void SIM300_USART_IRQ(void);
void SIM300_CMD(char *);
char SIM300_Config(void);

#endif