#include "stm32f10x.h"
#include "string.h"
#include "sim300.h"
#include "main.h"

u8 SIM300_TxBuffer[SIM300_BufferSize]={0},SIM300_RxBuffer[SIM300_BufferSize]={0},SIM300_RxBufferLength=0,SIM300_Received=0;

void sim300_Init(void)
{
  USART_InitTypeDef USART_InitStructure;
  GPIO_InitTypeDef GPIO_InitStructure;
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
  TIM_OCInitTypeDef  TIM_OCInitStructure;
  
  GPIO_InitStructure.GPIO_Pin = SIM300_RST|SIM300_PWR;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_ResetBits(SIM300_GPIO_USART,SIM300_PWR);
  GPIO_ResetBits(SIM300_GPIO_USART,SIM300_RST);
  GPIO_Init(SIM300_GPIO_USART,&GPIO_InitStructure);
  
  GPIO_InitStructure.GPIO_Pin = SIM300_USART_TX;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_Init(SIM300_GPIO_USART,&GPIO_InitStructure);
  
  GPIO_InitStructure.GPIO_Pin = SIM300_USART_RX;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(SIM300_GPIO_USART,&GPIO_InitStructure);
  
  USART_InitStructure.USART_BaudRate = SIM300_BaudRate;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
  USART_Init(SIM300_USART, &USART_InitStructure);
  USART_ITConfig(SIM300_USART, USART_IT_RXNE, ENABLE);
  USART_Cmd(SIM300_USART, ENABLE);
  
  TIM_TimeBaseStructure.TIM_Period = 65535;
  TIM_TimeBaseStructure.TIM_Prescaler = 0;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInit(SIM300_TIM,&TIM_TimeBaseStructure);
  
  TIM_InternalClockConfig(SIM300_TIM);
  TIM_PrescalerConfig(SIM300_TIM, 7199, TIM_PSCReloadMode_Immediate);
  
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_Inactive;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = 60;           //定时6ms
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
  
  TIM_OC1Init(SIM300_TIM, &TIM_OCInitStructure);
  TIM_OC1PreloadConfig(SIM300_TIM, TIM_OCPreload_Disable);
  
  TIM_ClearITPendingBit(SIM300_TIM, TIM_IT_CC1); //定时中断标志清零
  TIM_ITConfig(SIM300_TIM, TIM_IT_CC1, DISABLE);  //中断使能
  
  TIM_Cmd(SIM300_TIM, ENABLE);                   //定时器使能
}

char SIM300_Config(void)
{
  u8  findex;
  SIM300_RST_H;  
  Delay_10ms(50);
  SIM300_RST_L;
  Delay_10ms(50);
  SIM300_Received=0;
  SIM300_RxBufferLength=0;
  memset((char *)SIM300_RxBuffer, 0, SIM300_BufferSize);//清缓存数据
  GPIO_SetBits(SIM300_GPIO_USART,SIM300_PWR);
  Delay_10ms(200);
  GPIO_ResetBits(SIM300_GPIO_USART,SIM300_PWR);  
  Delay_10ms(1500);
  SIM300_Received=0;
  SIM300_RxBufferLength=0;
  memset((char *)SIM300_RxBuffer, 0, SIM300_BufferSize);//清缓存数据
  SIM300_CMD("AT+CCID\r\n");                            //
  Delay_10ms(600);   
  
  SIM300_Received=0;
  SIM300_RxBufferLength=0;
  memset((char *)SIM300_RxBuffer, 0, SIM300_BufferSize);//清缓存数据
  SIM300_CMD("AT+CREG?\r\n");                            //
  Delay_10ms(300);   
    
  SIM300_Received=0;
  SIM300_RxBufferLength=0;
  memset((char *)SIM300_RxBuffer, 0, SIM300_BufferSize);//清缓存数据
  SIM300_CMD("AT+CGATT=1\r\n");                            //
  Delay_10ms(600);
  
  SIM300_Received=0;
  SIM300_RxBufferLength=0;
  memset((char *)SIM300_RxBuffer, 0, SIM300_BufferSize);//清缓存数据
  SIM300_CMD("AT+CGDCONT=1,\"IP\",\"CMNET\"\r\n");                            //
  Delay_10ms(300);
  
  SIM300_Received=0;
  SIM300_RxBufferLength=0;
  memset((char *)SIM300_RxBuffer, 0, SIM300_BufferSize);//清缓存数据
  SIM300_CMD("AT+CGACT=1,1\r\n");                            //
  Delay_10ms(300);
  
  SIM300_Received=0;
  SIM300_RxBufferLength=0;
  memset((char *)SIM300_RxBuffer, 0, SIM300_BufferSize);//清缓存数据  
  SIM300_CMD("AT+CIPSTART=\"TCP\",\"flwjh.tpddns.cn\",8888\r\n");              //TCP连接
  Delay_10ms(1100);
 
  findex=0;
  while(SIM300_RxBuffer[findex++]==0)
  {
    SIM300_RxBuffer[findex-1]=0x01;
    if(findex>=SIM300_BufferSize-1)
      break;
  }
  char *p=strstr((char *)SIM300_RxBuffer,"CONNECT OK");
  char pp=*p;
  SIM300_Received=0;
  SIM300_RxBufferLength=0;
  memset((char *)SIM300_RxBuffer, 0, SIM300_BufferSize);//清缓存数据
  return pp;  
}

void sim300_SendData(u16 SendDataLength)
{
  u16 i;
  for(i=0;i<SendDataLength;i++)
  {
    USART_SendData(SIM300_USART, SIM300_TxBuffer[i]);//发送响应
    while(USART_GetFlagStatus(SIM300_USART, USART_FLAG_TC) == RESET);
  }
}

void SIM300_CMD(char * sim300_string)
{
  memset((char *)SIM300_TxBuffer, 0, SIM300_BufferSize);//清缓存数据
  strcpy((char *)SIM300_TxBuffer,sim300_string);   
  sim300_SendData(strlen(sim300_string));
}

void SIM300_TIM_IRQ(void)
{
  if(TIM_GetITStatus(SIM300_TIM, TIM_IT_CC1) != RESET)
  {
    TIM_ClearITPendingBit(SIM300_TIM, TIM_IT_CC1);//定时中断标志清零
    TIM_ITConfig(SIM300_TIM, TIM_IT_CC1, DISABLE);//中断关闭
    SIM300_Received=1;
  }
}

void SIM300_USART_IRQ(void)
{  
  u16 usartsr;
  if(USART_GetITStatus(SIM300_USART, USART_IT_RXNE) != RESET)
  {  
    SIM300_RxBuffer[SIM300_RxBufferLength++] = USART_ReceiveData(SIM300_USART);
    if(SIM300_RxBufferLength >= SIM300_BufferSize) 
      SIM300_RxBufferLength = SIM300_BufferSize-1;
    USART_ClearITPendingBit(SIM300_USART, USART_IT_RXNE);
    
    /*每接收完一个字节数据判断一次间隔时间*/
    TIM_ClearITPendingBit(SIM300_TIM, TIM_IT_CC1); //定时中断标志清零
    TIM_SetCounter(SIM300_TIM,0);                  //定时器清零 
    TIM_ITConfig(SIM300_TIM, TIM_IT_CC1, ENABLE);  //中断使能 
  }
  else
  {   //清除ORE(溢出中断)标志
    usartsr = SIM300_USART->SR;
    usartsr += USART_ReceiveData(SIM300_USART);
  }
}