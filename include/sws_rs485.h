/************************ (C) COPYRIGHT SWS *******************************
* File Name          : swsbus.h
* Author             : 童锦
* Version            : V2.0
* Date               : 2012.11.30
* Description        : SWSBUS协议程序头文件
********************************************************************************/
#ifndef __SWSBUS_H
#define __SWSBUS_H

u8  recive_status = 0;



//用户可配置，可根据需要更改
#define   RS485_TIM       TIM1                 //RS485使用TIM
#define   RS485_USART     USART3               //RS485使用串口
#define   GPIO_USART      GPIOB                //RS485使用GPIO(要求RD,TX,RX使用同一个GPIO)
#define   USART_RD        GPIO_Pin_9           //发送接收使能端口
#define   USART_TX        GPIO_Pin_10          //串口发送信号端口
#define   USART_RX        GPIO_Pin_11          //串口接收信号端口
#define   RS485_RE        GPIO_ResetBits(GPIO_USART,USART_RD)//485接收使能
#define   RS485_SE        GPIO_SetBits(GPIO_USART,USART_RD)  //485发送使能
#define   BaudRate        115200                             //设置波特率

#define   LocalAddress      0x06        // 本机地址
#define   BroadcastAddress  0xAA        // 广播地址

#define   TxBufferSize      20          //发送缓冲区数组长度
#define   RxBufferSize      20          //接收缓冲区数组长度
#define   WR0_BufferSize    6           // 写命令0的接口数组长度
#define   WR1_BufferSize    6           // 写命令1的接口数组长度
#define   WR2_BufferSize    5           // 写命令2的接口数组长度
#define   WR3_BufferSize    2           // 写命令3的接口数组长度
#define   WR4_BufferSize    16          // 写命令4的接口数组长度
#define   WR5_BufferSize    16          // 写命令5的接口数组长度
#define   WR6_BufferSize    16          // 写命令6的接口数组长度
#define   WR7_BufferSize    16          // 写命令7的接口数组长度

#define   RD0_BufferSize    12          // 读命令0的接口数组长度
#define   RD1_BufferSize    14          // 读命令1的接口数组长度
#define   RD2_BufferSize    4           // 读命令2的接口数组长度
#define   RD3_BufferSize    6           // 读命令3的接口数组长度
#define   RD4_BufferSize    6           // 读命令4的接口数组长度
#define   RD5_BufferSize    16          // 读命令5的接口数组长度
#define   RD6_BufferSize    16          // 读命令6的接口数组长度
#define   RD7_BufferSize    6           // 读命令7的接口数组长度
#define   MIN_BufferSize    0x04        // 所有读、写命令的接口数组的最小长度


//功能码定义，用户不可更改
#define   BUS_INI     0x20     // 通信初始化状态
#define   BUS_ASK     0x21     // 向从机发出握手信息
#define   CRC_ERR     0x22     // CRC校验错误

#define   BUS_WR0     0x30     // 写命令0
#define   BUS_WR1     0x31     // 写命令1
#define   BUS_WR2     0x32     // 写命令2
#define   BUS_WR3     0x33     // 写命令3
#define   BUS_WR4     0x34     // 写命令4
#define   BUS_WR5     0x35     // 写命令5
#define   BUS_WR6     0x36     // 写命令6
#define   BUS_WR7     0x37     // 写命令7

#define   BUS_RD0     0x40     // 读命令0
#define   BUS_RD1     0x41     // 读命令1
#define   BUS_RD2     0x42     // 读命令2
#define   BUS_RD3     0x43     // 读命令3
#define   BUS_RD4     0x44     // 读命令4
#define   BUS_RD5     0x45     // 读命令5
#define   BUS_RD6     0x46     // 读命令6
#define   BUS_RD7     0x47     // 读命令7

extern u8     TxBuffer[TxBufferSize]; //发送数据缓存
extern u8     RxBuffer[RxBufferSize]; //接受数据缓存
extern u8     TxBufferLength;         //发送数据长度
extern u8     RxBufferLength;         //接受数据长度
extern struct COM_Array BUS_Buffer;

struct COM_Array
{
  u8 WR0[WR0_BufferSize];    // 写命令0的接口数组
  u8 WR1[WR1_BufferSize];    // 写命令1的接口数组
  u8 WR2[WR2_BufferSize];    // 写命令2的接口数组
  u8 WR3[WR3_BufferSize];    // 写命令3的接口数组
  u8 WR4[WR4_BufferSize];    // 写命令4的接口数组
  u8 WR5[WR5_BufferSize];    // 写命令5的接口数组
  u8 WR6[WR6_BufferSize];    // 写命令6的接口数组
  u8 WR7[WR7_BufferSize];    // 写命令7的接口数组

  u8 RD0[RD0_BufferSize];    // 读命令0的接口数组
  u8 RD1[RD1_BufferSize];    // 读命令1的接口数组
  u8 RD2[RD2_BufferSize];    // 读命令2的接口数组
  u8 RD3[RD3_BufferSize];    // 读命令3的接口数组
  u8 RD4[RD4_BufferSize];    // 读命令4的接口数组
  u8 RD5[RD5_BufferSize];    // 读命令5的接口数组
  u8 RD6[RD6_BufferSize];    // 读命令6的接口数组
  u8 RD7[RD7_BufferSize];    // 读命令7的接口数组
};

void RS485_Init(void);
void BUS_DataResponse(void);             //总线数据解析
u16  CRC16(u8 *Frame,u8 Length);         //CRC校验
void RS485_SendData(u8 SendDataLength);  //发送数据
void TIM_IRQ(void);
void USART_IRQ(void);
void Delay_Bus(u32 dly);                 //延时



#endif
/******************* (C) COPYRIGHT 2008 SWS *****END OF FILE*******************/
