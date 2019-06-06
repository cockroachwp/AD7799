#ifndef __MLX90615_H
#define __MLX90615_H

#include"stm32f10x_conf.h"

#define BROADCAST_ADDRESS 0x00//广播地址
#define Slave_Address 0x5B//MLX90615 SMBus出厂默认地址

#define ACCESS_SA 0x10//读写SMBus地址命令，操作码0001 0000，0000为EEPROM中的00h的低四位
#define ACCESS_Ta 0x26//读环境温度命令，操作码0010 0110，0110为RAM中的06h的低四位
#define ACCESS_To 0x27//读物体温度命令，操作码0010 0111，0111为RAM中的07h的低四位
#define ACCESS_E 0x13//读写发射率命令，操作码0001 0011，0011为EEPROM中的03h的低四位



void MLX90615_Init(void);
float Read_Temperature(u8 slave_addR,u8 cmdR,u8 id);
void Set_Emissivity(u8 slave_addR,float Emissivity,u8 id);
float Read_Emissivity(u8 slave_addR,u8 id);

#endif