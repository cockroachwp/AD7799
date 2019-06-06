#ifndef __SMBUS_CM_H
#define __SMBUS_CM_H

#include <stm32f10x.h>

#define SCL_Set() GPIO_SetBits(GPIOB,GPIO_Pin_7)  //配置b口管脚1为时钟线
#define SCL_Reset() GPIO_ResetBits(GPIOB,GPIO_Pin_7)
#define SDA_Set() GPIO_SetBits(GPIOB,GPIO_Pin_6)  //配置b口管脚2为数据线
#define SDA_Reset() GPIO_ResetBits(GPIOB,GPIO_Pin_6)
#define SDA_InputData() GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_6)

#define SCL_Set2() GPIO_SetBits(GPIOB,GPIO_Pin_2)  //配置b口管脚1为时钟线
#define SCL_Reset2() GPIO_ResetBits(GPIOB,GPIO_Pin_2)
#define SDA_Set2() GPIO_SetBits(GPIOB,GPIO_Pin_1)  //配置b口管脚2为数据线
#define SDA_Reset2() GPIO_ResetBits(GPIOB,GPIO_Pin_1)
#define SDA_InputData2() GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_1)


void start_bit(u8 id);
void restart_bit(u8 id);
void stop_bit(u8 id);
void send_bit(unsigned char bit_out,u8 id);
unsigned char receive_bit(u8 id);
unsigned char slave_ack(u8 id);
void TX_byte(unsigned char TX_buffer,u8 id);
unsigned char RX_byte(unsigned char ack_nack,u8 id);

#endif