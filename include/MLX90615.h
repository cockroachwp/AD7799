#ifndef __MLX90615_H
#define __MLX90615_H

#include"stm32f10x_conf.h"

#define BROADCAST_ADDRESS 0x00//�㲥��ַ
#define Slave_Address 0x5B//MLX90615 SMBus����Ĭ�ϵ�ַ

#define ACCESS_SA 0x10//��дSMBus��ַ���������0001 0000��0000ΪEEPROM�е�00h�ĵ���λ
#define ACCESS_Ta 0x26//�������¶����������0010 0110��0110ΪRAM�е�06h�ĵ���λ
#define ACCESS_To 0x27//�������¶����������0010 0111��0111ΪRAM�е�07h�ĵ���λ
#define ACCESS_E 0x13//��д���������������0001 0011��0011ΪEEPROM�е�03h�ĵ���λ



void MLX90615_Init(void);
float Read_Temperature(u8 slave_addR,u8 cmdR,u8 id);
void Set_Emissivity(u8 slave_addR,float Emissivity,u8 id);
float Read_Emissivity(u8 slave_addR,u8 id);

#endif