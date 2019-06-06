/****************************************************************************
Copyright (c) 2012,重庆山外山科技有限公司技术中心
All rights reserved.

文件名称：	main.h
摘 	 要 ：	主程序头文件
当前版本：  V4.80
作   者 ：	童锦
修改内容：	
完成日期：	2013-07-25
*****************************************************************************/
#ifndef __MAIN_H
#define __MAIN_H

#define PWM_Period        8000
#define UV_ON         TIM4->CCR3 =800 //10%占空比 即输出0.9*3.3V，电流为 0.9*3.3/3/51=19.4mA
#define UV_OFF        TIM4->CCR3=PWM_Period

#define ADD_HV         0        //硬件版本地址
#define ADD_CODE       1        //部件代码地址

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
    u8  Enable;       //传感器启/停命令： DISABLE=关闭传感器；ENABLE=开启传感器
    u8  Err;          //传感器自检标志位，1=传感器异常；0=传感器正常
    u8  Calibration_Enable; //传感器自检启/停命令： DISABLE=关闭传感器自检；ENABLE=开启传感器自检   
    u8  Calibration;  //校准标志位，ERROR=传感器未校准；SUCCESS=传感器已校准
    float ABS;        //吸光度
    u32 I_AD;         //传感器1 AD输出，即参考端AD输出
    u32 I0_AD;        //传感器2 AD输出，即取样端AD输出
}UREA_WorkInfo_Def;  //上行下行结构体封装


extern u32 time_stamp;
#endif
/******************* (C) COPYRIGHT 2008 SWS *****END OF FILE*******************/
