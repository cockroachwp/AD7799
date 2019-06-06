/****************************************************************************
Copyright (c) 2012,����ɽ��ɽ�Ƽ����޹�˾��������
All rights reserved.

�ļ����ƣ�	main.h
ժ 	 Ҫ ��	������ͷ�ļ�
��ǰ�汾��  V4.80
��   �� ��	ͯ��
�޸����ݣ�	
������ڣ�	2013-07-25
*****************************************************************************/
#ifndef __MAIN_H
#define __MAIN_H

#define PWM_Period        8000
#define UV_ON         TIM4->CCR3 =800 //10%ռ�ձ� �����0.9*3.3V������Ϊ 0.9*3.3/3/51=19.4mA
#define UV_OFF        TIM4->CCR3=PWM_Period

#define ADD_HV         0        //Ӳ���汾��ַ
#define ADD_CODE       1        //���������ַ

void Delay_10ms(u32);
unsigned char Delay_Pass(u32);
void Hardware_Init(void);
void RCC_Configuration(void);
void NVIC_Configuration(void);
void SysTick_Configuration(void);
void PWM_Init(void);
void GPIO_Config(void);
void Down_flag(void);              
void Up_flag(void); 
void CMD_Parse(void);
void Version_Load(void);
void H_VERSION_Write(void);
void UREA_Run(void);
void UR_AD_Conversion(u16 Con_Tim);

typedef struct       
{
    u8  Enable;       //��������/ͣ��� DISABLE=�رմ�������ENABLE=����������
    u8  Err;          //�������Լ��־λ��1=�������쳣��0=����������
    u8  Calibration_Enable; //�������Լ���/ͣ��� DISABLE=�رմ������Լ죻ENABLE=�����������Լ�   
    u8  Calibration;  //У׼��־λ��ERROR=������δУ׼��SUCCESS=��������У׼
    float ABS;        //�����
    u32 I_AD;         //������1 AD��������ο���AD���
    u32 I0_AD;        //������2 AD�������ȡ����AD���
}UREA_WorkInfo_Def;  //�������нṹ���װ


extern u32 time_stamp;
#endif
/******************* (C) COPYRIGHT 2008 SWS *****END OF FILE*******************/
