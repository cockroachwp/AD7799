#include "stm32f10x.h"
#include "string.h"
#include "esp8266.h"
#include "main.h"

u8 ESP8266_TxBuffer[ESP8266_BufferSize]={0},ESP8266_RxBuffer[ESP8266_BufferSize]={0},ESP8266_RxBufferLength=0,ESP8266_Received=0;

void esp8266_Init(void)
{
  USART_InitTypeDef USART_InitStructure;
  GPIO_InitTypeDef GPIO_InitStructure;
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
  TIM_OCInitTypeDef  TIM_OCInitStructure;
  
  GPIO_InitStructure.GPIO_Pin = ESP8266_USART_TX;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_Init(ESP8266_GPIO_USART,&GPIO_InitStructure);
  
  GPIO_InitStructure.GPIO_Pin = ESP8266_USART_RX;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(ESP8266_GPIO_USART,&GPIO_InitStructure);
  
  USART_InitStructure.USART_BaudRate = ESP8266_BaudRate;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
  USART_Init(ESP8266_USART, &USART_InitStructure);
  USART_ITConfig(ESP8266_USART, USART_IT_RXNE, ENABLE);
  USART_Cmd(ESP8266_USART, ENABLE);
  
  TIM_TimeBaseStructure.TIM_Period = 65535;
  TIM_TimeBaseStructure.TIM_Prescaler = 0;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInit(ESP8266_TIM,&TIM_TimeBaseStructure);
  
  TIM_InternalClockConfig(ESP8266_TIM);
  TIM_PrescalerConfig(ESP8266_TIM, 7199, TIM_PSCReloadMode_Immediate);
  
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_Inactive;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = 60;           //��ʱ6ms
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
  
  TIM_OC1Init(ESP8266_TIM, &TIM_OCInitStructure);
  TIM_OC1PreloadConfig(ESP8266_TIM, TIM_OCPreload_Disable);
  
  TIM_ClearITPendingBit(ESP8266_TIM, TIM_IT_CC1); //��ʱ�жϱ�־����
  TIM_ITConfig(ESP8266_TIM, TIM_IT_CC1, DISABLE);  //�ж�ʹ��
  
  TIM_Cmd(ESP8266_TIM, ENABLE);                   //��ʱ��ʹ��
}

char Esp8266_Config(void)
{
  Esp8266_CMD("AT\r\n");
  Delay_10ms(20);
  if(ESP8266_Received)
  {
    ESP8266_Received=0;
    ESP8266_RxBufferLength=0;
    memset((char *)ESP8266_RxBuffer, 0, ESP8266_BufferSize);//�建������
  }
  Esp8266_CMD("AT+RST\r\n");                                                    //��λ
  Delay_10ms(1500);
  
  if(ESP8266_Received)
  {
    ESP8266_Received=0;
    ESP8266_RxBufferLength=0;
    memset((char *)ESP8266_RxBuffer, 0, ESP8266_BufferSize);//�建������
  } 
  
  Esp8266_CMD("AT+UART_DEF=9600,8,1,0,0\r\n");                                  //������
  Delay_10ms(20);
  if(ESP8266_Received)
  {
    ESP8266_Received=0;
    ESP8266_RxBufferLength=0;
    memset((char *)ESP8266_RxBuffer, 0, ESP8266_BufferSize);//�建������
  } 
  Esp8266_CMD("AT+CWMODE_DEF=1\r\n");                                           //����ģʽ station
  Delay_10ms(20);
  if(ESP8266_Received)
  {
    ESP8266_Received=0;
    ESP8266_RxBufferLength=0;
    memset((char *)ESP8266_RxBuffer, 0, ESP8266_BufferSize);//�建������
  } 
  //Esp8266_CMD("AT+CWJAP_DEF=\"XiaomiDEEP\",\"11111111\"\r\n");                      //����AP
  Esp8266_CMD("AT+CWJAP_DEF=\"SWSTX\",\"sws666666\"\r\n");                      //����AP
  Delay_10ms(1200);
  if(ESP8266_Received)
  {
    ESP8266_Received=0;
    ESP8266_RxBufferLength=0;
    memset((char *)ESP8266_RxBuffer, 0, ESP8266_BufferSize);//�建������
  } 
  Esp8266_CMD("AT\r\n");
  Delay_10ms(20);
  
  ESP8266_Received=0;
  ESP8266_RxBufferLength=0;
  memset((char *)ESP8266_RxBuffer, 0, ESP8266_BufferSize);//�建������
  
  //Esp8266_CMD("AT+CWAUTOCONN\r\n");                                           //�ϵ�ʱ�Զ�����AP,station ��Ĭ���Զ�����AP
  //Delay_10ms(50);
  //Esp8266_CMD("AT+CIFSR\r\n");                                                //�鿴ip
  //Delay_10ms(50);
  Esp8266_CMD("AT+CIPSTART=\"TCP\",\"104.224.153.207\",8080\r\n");              //TCP����
  Delay_10ms(800);
  //Esp8266_CMD("AT+CIPSEND=5\r\n");                                            //Ҫ���Ͷ����ֽ�����
  //Delay_10ms(50);
  //Esp8266_CMD("DATAS");                                                       //���ݣ�no cr
  //Delay_10ms(50);
  //Esp8266_CMD("AT+CIPCLOSE\r\n");                                             //�Ͽ�����
  //Delay_10ms(50);  
  char *p=strstr((char *)ESP8266_RxBuffer,"OK");
  char pp=*p;
  ESP8266_Received=0;
  ESP8266_RxBufferLength=0;
  memset((char *)ESP8266_RxBuffer, 0, ESP8266_BufferSize);//�建������
  return pp;  
}

void esp8266_SendData(u16 SendDataLength)
{
  u16 i;
  for(i=0;i<SendDataLength;i++)
  {
    USART_SendData(ESP8266_USART, ESP8266_TxBuffer[i]);//������Ӧ
    while(USART_GetFlagStatus(ESP8266_USART, USART_FLAG_TC) == RESET);
  }
}

void Esp8266_CMD(char * esp8266_string)
{
  memset((char *)ESP8266_TxBuffer, 0, ESP8266_BufferSize);//�建������
  strcpy((char *)ESP8266_TxBuffer,esp8266_string);   
  esp8266_SendData(strlen(esp8266_string));
}

void ESP8266_TIM_IRQ(void)
{
  if(TIM_GetITStatus(ESP8266_TIM, TIM_IT_CC1) != RESET)
  {
    TIM_ClearITPendingBit(ESP8266_TIM, TIM_IT_CC1);//��ʱ�жϱ�־����
    TIM_ITConfig(ESP8266_TIM, TIM_IT_CC1, DISABLE);//�жϹر�
    ESP8266_Received=1;
  }
}

void ESP8266_USART_IRQ(void)
{  
  u16 usartsr;
  if(USART_GetITStatus(ESP8266_USART, USART_IT_RXNE) != RESET)
  {  
    ESP8266_RxBuffer[ESP8266_RxBufferLength++] = USART_ReceiveData(ESP8266_USART);
    if(ESP8266_RxBufferLength >= ESP8266_BufferSize) 
      ESP8266_RxBufferLength = ESP8266_BufferSize-1;
    USART_ClearITPendingBit(ESP8266_USART, USART_IT_RXNE);
    
    /*ÿ������һ���ֽ������ж�һ�μ��ʱ��*/
    TIM_ClearITPendingBit(ESP8266_TIM, TIM_IT_CC1); //��ʱ�жϱ�־����
    TIM_SetCounter(ESP8266_TIM,0);                  //��ʱ������ 
    TIM_ITConfig(ESP8266_TIM, TIM_IT_CC1, ENABLE);  //�ж�ʹ�� 
  }
  else
  {   //���ORE(����ж�)��־
    usartsr = ESP8266_USART->SR;
    usartsr += USART_ReceiveData(ESP8266_USART);
  }
}