
#include "stm32f10x.h"
#include "sws_ad7799.h"

#define uchar unsigned char

//��ʱ����*

void Delay2cp(unsigned char i);

#define WRDYN 45 //д��ʱ

#define RDDYN 43 //����ʱ

//������дһ���ֽ�

static  sws_gpio_info_t  uart_tx = {
      GPIOA,
      GPIO_Pin_9
};

static  sws_gpio_info_t  uart_rx = {
      GPIOA,
      GPIO_Pin_10
};


void uart_gpio_init()
{
  
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA| RCC_APB2Periph_AFIO, ENABLE);
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Pin = uart_tx.PIN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;    //  �������
    GPIO_Init(uart_tx.POR, &GPIO_InitStructure); 
    GPIO_SetBits(uart_tx.POR,uart_tx.PIN);
  
//    GPIO_InitStructure.GPIO_Pin = uart_rx.PIN;
//    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;    
//    GPIO_Init(ad7799_out.POR, &GPIO_InitStructure);  
//    GPIO_SetBits(uart_tx.POR,uart_tx.PIN);

}


void WByte(uchar input)

{

     uchar i=8;

     GPIO_ResetBits(uart_tx.POR,uart_tx.PIN); //TXD=(bit)0;                      //������ʼλ

     Delay2cp(39);

     //����8λ����λ

     while(i--)

     {

         //TXD=(bit)(input&0x01);      //�ȴ���λ
         if (input&0x01) {
             GPIO_SetBits(uart_tx.POR,uart_tx.PIN);  
         } else {
             GPIO_ResetBits(uart_tx.POR,uart_tx.PIN);
         }

         Delay2cp(36);

         input=input>>1;

     }

     //����У��λ(��)

     GPIO_SetBits(uart_tx.POR,uart_tx.PIN);//TXD=(bit)1;                      //���ͽ���λ

     Delay2cp(46);

}
//
//void printf_u32(u32 p_u32)
//{
//    WByte((uchar)((p_u32)>>24)&0xff + '0');
//    WByte((uchar)((p_u32)>>16)&0xff + '0');
//    WByte((uchar)((p_u32)>>8)&0xff + '0');
//    WByte((uchar)((p_u32)&0xff) + '0');
//}

void printf_char(char *p_char)
{
  while(*p_char != '\0') {
        WByte(*p_char++);
  }
}





//��ʱ����*

void Delay2cp(unsigned char i)

{

     while(--i);                      //�պ�����ָ�����ڡ�

}