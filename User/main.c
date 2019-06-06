/****************************************************
Copyright (c) 2012,����ɽ��ɽ�Ƽ����޹�˾��������
All rights reserved.
�ļ����ƣ�	main.c
ժ 	 Ҫ ��	
��ǰ�汾��      V6.0
��   �� ��	ͯ��
�޸����ݣ�	�������ļ���ʵ�ֹ���
������ڣ�	2013-07-25
****************************************************/
/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"
#include "main.h"
#include "swsbus.h" 
#include "sim300.h"
#include "flash.h"
#include "AD7799.h"
#include "stm32f10x_tim.h"
#include "string.h"
#include "math.h"

#define S_VERSION      600      //����汾

//BUN�������  ABS=a*abs+b
#define a 1         //4.4957    //����Ũ��ϵ��
#define b 0         //ԭ����Ϊ0������0�������У׼���������������

#define AD_time_10 1000 //AD���������10sһ��  �Ŵ�100��
#define AD_time_60 6000 //AD���������60sһ��  �Ŵ�100��
#define AD_timeChange 72000 //12���Ӻ��л�����Ƶ��
#define ADERRDATA 16777000 //AD������������
#define ABS_Init_time 1   //�����0���������
#define AD_Init_time 1    //AD����0���������
#define ABS_GAIN  12000      //����Ũ�ȷŴ���

char *ps;
u8 lsbuff[16]={0};
u32 time_stamp=0,time_stamps=0;

u16 H_VERSION = 600;               // Ӳ���汾
u16 BORD_CODE;                     // ��������

UREA_WorkInfo_Def UREA_Data; 

u32 Enable_Date=0;//����ʱ�仺��
u32 AD1[5],AD2[5];//ADֵ�洢����
u32 ad10=0,ad20=0;//AD��ʼֵ��0�㣩�洢
float lga=0;      //����ȼ�������洢
float ABS=0;      //�����ABS=a*abs+b
float abs=0;      //�����abs=lga+lg((AD1-ad10)/(AD2-ad20))

/*******************************************************************************
* Function Name  : main
* Description    : Main program
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
u8 up_flag=0;
int main(void)
{    
  Hardware_Init();      //Ӳ����ʼ��    
  while(1)
  {  
    
    Down_flag();              //������־λ
 
    UREA_Run();
    
    Up_flag();
  } 
}
/*******************************************************************************
* Function Name  : Down_flag
* Description    : 
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void Down_flag(void)              //������־λ
{
  if(BUS_Buffer.WR0[0] == 1)
  {
    BUS_Buffer.WR0[0] = 0;
    CMD_Parse();    //�����
  } 
  /* ����Ӳ���汾��Ϣ */
  if(BUS_Buffer.WR7[0] == 1)
  {
    BUS_Buffer.WR7[0] = 0;
    H_VERSION_Write();
    Version_Load();
  }
}
/*******************************************************************************
* Function Name  : Up_flag
* Description    : 
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void Up_flag(void)                //���±�־λ
{
  u8 i=0,j=0;
  u32 tmp1_ad1=0,tmp2_ad1=0,tmp1_ad2=0,tmp2_ad2=0;//�˲�����ֵ
  u32 AD1_Filter[5],AD2_Filter[5];  //AD���򻺴�
  u32 AD1_Median=0,AD2_Median=0;    //��ֵ

  BUS_Buffer.RD0[0] = (UREA_Data.Calibration << 2) | (UREA_Data.Err << 1) | UREA_Data.Enable;

  /***************ABS****************/
  if(up_flag == 1 && UREA_Data.Enable ==  ENABLE)
  {
    //��AD���򣬽��������洢��AD_Filter
    up_flag = 0;
    for(i=0;i<5;i++)
    {
      tmp1_ad1 = AD1[i];
      tmp1_ad2 = AD2[i];
      
      for(j=0;j<i;j++)
      {
        
        if(AD1_Filter[j] > tmp1_ad1)
        {
          tmp2_ad1 = AD1_Filter[j];
          AD1_Filter[j] = tmp1_ad1;
          tmp1_ad1 = tmp2_ad1;
        }
        
        if(AD2_Filter[j] > tmp1_ad2)
        {
          tmp2_ad2 = AD2_Filter[j];
          AD2_Filter[j] = tmp1_ad2;
          tmp1_ad2 = tmp2_ad2;
        }
      }
      AD1_Filter[i] = tmp1_ad1;
      AD2_Filter[i] = tmp1_ad2;
    }
    
    for(i=0;i<5;i++)
    {
      if(AD1_Filter[i] != 0)
      {
        AD1_Median=AD1_Filter[(i+4)/2];
        i=5;
      }
    }
    
    for(i=0;i<5;i++)
    {
      if(AD2_Filter[i] != 0)
      {
        AD2_Median=AD2_Filter[(i+4)/2];
        i=5;
      }
    }
    
    if((AD2_Median- ad20) != 0)
    {
      abs = lga + log10(((float)AD1_Median - (float)ad10)/((float)AD2_Median - (float)ad20));
      ABS = a*abs + b;
    }
    
    if(ABS<0)
    {
      ABS=-ABS;   //ABSֻ����ֵ
      BUS_Buffer.RD0[2] = (u8)(ABS * ABS_GAIN);   //�Ŵ�ABS_GAIN��
      BUS_Buffer.RD0[3] = (u8)((u16)(ABS * ABS_GAIN)>>8);//|0X80;    
    }
    else
    {
      BUS_Buffer.RD0[2] = (u8)(ABS * ABS_GAIN);
      BUS_Buffer.RD0[3] = (u8)((u16)(ABS * ABS_GAIN)>>8);
    }
  }
  else if(UREA_Data.Enable ==  DISABLE)
  {
    BUS_Buffer.RD0[2] = 0;
    BUS_Buffer.RD0[3] = 0;
  }
  
  /***************AD1****************/
  BUS_Buffer.RD0[6] = (u8)AD_Value[0].Value;
  BUS_Buffer.RD0[7] = (u8)((u32)AD_Value[0].Value>>8);
  BUS_Buffer.RD0[8] = (u8)((u32)AD_Value[0].Value>>16);
  BUS_Buffer.RD0[9] = (u8)((u32)AD_Value[0].Value>>24);
  
  /***************AD2****************/
  BUS_Buffer.RD0[10] = (u8)AD_Value[1].Value;
  BUS_Buffer.RD0[11] = (u8)((u32)AD_Value[1].Value>>8);
  BUS_Buffer.RD0[12] = (u8)((u32)AD_Value[1].Value>>16);
  BUS_Buffer.RD0[13] = (u8)((u32)AD_Value[1].Value>>24);
}

/*******************************************************************************
* Function Name  : UREA_Run
* Description    : 
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/

void UREA_Run(void)
{
  u8 i=0;
  u8 err_flag=0;
  u32 ad1_data=0,ad2_data=0;
  static u8 Enable_flag = 0;
  
  
  /*****************UREAУ׼*****************/
  if(UREA_Data.Calibration_Enable == ENABLE)
  {
    ad10 = 0;
    ad20 = 0;
    lga = 0;
    UREA_Data.Calibration = ERROR; 
    UREA_Data.Err = 0;
    
    /***********��ȡ�����������0��************/
    UV_ON; //�򿪷���LED��
    Delay_10ms(100);
    for(i=0;i<ABS_Init_time;i++)
    {
      UR_AD_Conversion(400);  //AD���� 810ms����һ������  LEDÿ�ο���2S  200=4S
      if(AD_Value[0].Value==0 || AD_Value[0].Value > 16777000 || AD_Value[1].Value==0 || AD_Value[1].Value > 16777000)
      {
        err_flag++;   //  ��������������
      }
      else 
      {
        ad1_data+=AD_Value[0].Value;
        ad2_data+=AD_Value[1].Value;   
      }
    }
    UV_OFF; //�رշ���LED��
    
    /***********��ȡ������AD����0��ֵ************/
    for(i=0;i<AD_Init_time;i++)
    {
      UR_AD_Conversion(200);  //AD���� 810ms����һ������  200=2S

      if(AD_Value[0].Value==0 || AD_Value[0].Value > 16777000 || AD_Value[1].Value==0 || AD_Value[1].Value > 16777000)
      {
        err_flag++;
      }  
      else 
      {
        ad10+=AD_Value[0].Value;
        ad20+=AD_Value[1].Value;   
      }
    }
    
    if(err_flag > 0)
    {
      UREA_Data.Calibration = ERROR;           //��λ������У׼��־ δУ׼
      UREA_Data.Err = 1;                       //�������쳣
      ad10 = 0;
      ad20 = 0;
      ad1_data = 0;
      ad2_data = 0;
      err_flag = 0;
      lga = 0;
    }
    else
    {
      UREA_Data.Calibration = SUCCESS;        //������У׼�ɹ�
      UREA_Data.Err = 0;                      //����������
      ad10 /= AD_Init_time;
      ad20 /= AD_Init_time; 
      ad1_data /= ABS_Init_time;
      ad2_data /= ABS_Init_time;
      lga = -log10(((float)ad1_data-(float)ad10)/((float)ad2_data-(float)ad20));
    }
    
    UREA_Data.Calibration_Enable = DISABLE; //��λУ׼������־
  }
  
  /*****************UREA���*****************/
  if(UREA_Data.Enable == ENABLE)
  { 
    Enable_flag = ENABLE;
    if((((time_stamp-Enable_Date) < AD_timeChange) & ((time_stamp-Enable_Date)%AD_time_10 == 0)) || (((time_stamp-Enable_Date) > AD_timeChange) & ((time_stamp-Enable_Date)%AD_time_60 == 0)))
    {
      up_flag = 1;//��λ�������ݱ�־λ
      
      UV_ON;//6520; //�򿪷���LED�� 
      Delay_10ms(100);
      UR_AD_Conversion(400);  //AD���� 810ms����һ������  LEDÿ�ο���4S  400=4S
      UV_OFF; //�رշ���LED��
      
      if(AD_Value[0].Value==0 || AD_Value[0].Value > 16777000 || AD_Value[1].Value==0 || AD_Value[1].Value > 16777000)
      {
        err_flag++;   //  ��������������
      }
      else 
      {
        for(i=0;i<4;i++)
        {
          AD1[i] = AD1[i+1];
          AD2[i] = AD2[i+1];
        }
        AD1[4]=(u32)(AD_Value[0].Value);
        AD2[4]=(u32)(AD_Value[1].Value); 
      }      
      if(err_flag > 10)
      {
        UREA_Data.Err = 1;                       //�������쳣
        err_flag = 0;
      }   
    }
  }  
  else //��������״̬�򿪣����Ϊ�رգ������������
  {
    UV_OFF; //�رշ���LED��
    if(Enable_flag != DISABLE)
    {
      Enable_flag = DISABLE;
      ad_update();//AD7799��λ
      abs = 0;
      ABS = 0;
      UREA_Data.Calibration = ERROR;
      for(i=0;i<5;i++)
      {
        AD1[i]=0;
        AD2[i]=0;
      }
    }
  }
}


/*******************************************************************************
* Function Name  : UR_AD_Conversion
* Description    : 
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void UR_AD_Conversion(u16 Con_Tim)
{
  u8 ADerr_flag = 0; 
  u16 findex=0;
  ad_update();
  for(findex=0;findex<Con_Tim;findex++)//ÿ810ms����һ������  LEDÿ�ο���4S
  {
     Delay_10ms(1);
     ADC_Auto_Conversion();  
     if(AD_Value[0].Value>=ADERRDATA || AD_Value[1].Value>=ADERRDATA)
     {
       if(ADerr_flag < 10) findex=0;//����������¿�ʼ���ԣ�������5�Σ�����ѭ��
       ad_update();
       ADerr_flag++;
     }
  }
}
/*******************************************************************************
* Function Name  : CMD_Parse
* Description    : 
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void CMD_Parse(void)
{
  UREA_Data.Enable = BUS_Buffer.WR0[1] & 0x01;
  if(UREA_Data.Enable == ENABLE)
  {
    Enable_Date = time_stamp;//���濪ʼʱ��
  }
  UREA_Data.Calibration_Enable = (BUS_Buffer.WR0[1] >> 1) & 0x01;
}

/*****************************************************************************
** �������ƣ� H_VERSION_Write
** �������ܣ�дӲ���汾��Ϣ
** ��ڲ�������
** ���ڲ�������
*****************************************************************************/
void H_VERSION_Write(void)
{
  u8 res;
 
  /* ���沿������ */
  res = Flash_Save_Param(ADD_CODE,(u8 *)&BUS_Buffer.WR7[1],2);
  if(res)
  {
    Delay_10ms(1);
    res = Flash_Save_Param(ADD_CODE,(u8 *)&BUS_Buffer.WR7[1],2);
  }
  /* ����Ӳ���汾 */
  res = Flash_Save_Param(ADD_HV,(u8 *)&BUS_Buffer.WR7[3],2);
  if(res)
  {
    Delay_10ms(1);
    res = Flash_Save_Param(ADD_HV,(u8 *)&BUS_Buffer.WR7[3],2);
  }   
}
/**************************************************************************
** �������ƣ�Version_Load
** �������ܣ����ذ汾��Ϣ
** ��ڲ�������
** ���ڲ�������
***************************************************************************/
void Version_Load(void)
{
  u8 res;
  u8 temp[2];
  
  /* ���������� */
  res = Flash_Read_Param(ADD_CODE,temp,2);
  /* �����ȡʧ�� */
  if(res)
  {
    
  }
  BORD_CODE = temp[1]*100 + temp[0];
  /* ��Ӳ���汾 */
  res = Flash_Read_Param(ADD_HV,temp,2);
  /* �����ȡʧ�� */
  if(res)
  {
    
  }
  H_VERSION = temp[1]*100 + temp[0];

  /* ���ֽ���ǰ */
  BUS_Buffer.RD7[0] = (u8)(BORD_CODE % 100); 
  BUS_Buffer.RD7[1] = (u8)(BORD_CODE / 100);
  BUS_Buffer.RD7[2] = (u8)(H_VERSION % 100);
  BUS_Buffer.RD7[3] = (u8)(H_VERSION / 100);
  BUS_Buffer.RD7[4] = (u8)(S_VERSION % 100);
  BUS_Buffer.RD7[5] = (u8)(S_VERSION / 100);
}

/*******************************************************************************
* Function Name  : Hardware_Init
* Description    : ������Ҫ���ò�ͬ��Ӳ��
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void Hardware_Init(void)
{
  RCC_Configuration();
  NVIC_Configuration();
  SysTick_Configuration();
  RS485_Init();
  ADC7799_Init();
  GPIO_Config();
  PWM_Init();   
  Version_Load();  
}
/*******************************************************************************
* Function Name  : RCC_Configuration
* Description    : ����ϵͳʱ��
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void RCC_Configuration(void)
{    
  SystemInit(); 
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3 | RCC_APB1Periph_TIM4 | RCC_APB1Periph_USART2 ,ENABLE); 
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE); 
  NVIC_SetVectorTable(NVIC_VectTab_FLASH,0x4000);
}
/*******************************************************************************
* Function Name  : NVIC_Configuration
* Description    :����Ƕ���ж��������û��ɸ�����Ҫ����Լ����ж�
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void NVIC_Configuration(void)
{
  NVIC_InitTypeDef NVIC_InitStructure;
  
  //NVIC_SetVectorTable(NVIC_VectTab_FLASH,0x4000);
  /* Configure one bit for preemption priority */
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
  
  /* Enable the USART global Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  
  /* Enable the TIM2 for 485 global Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}
/*******************************************************************************
* Function Name  : Systick_Configuration
* Description    : ʱ�ӵδ�
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void SysTick_Configuration(void)
{
  /* Setup SysTick Timer for 10ms interrupts  */
  if (SysTick_Config(SystemCoreClock /100))   //100:10ms  10:100ms
  { 
    /* Capture error */ 
    while (1);
  }
}
/********************************************************************************
** �������� ��Delay()
** �������� ��
** ��ڲ��� ��
** ���ڲ��� ��
********************************************************************************/
void Delay_10ms(u32 dly)
{
  u32 i=time_stamp;
  while((dly+i)>time_stamp);
}
/********************************************************************************
** �������� ��Delay_Pass()
** �������� ��
** ��ڲ��� ��
** ���ڲ��� ��
********************************************************************************/
unsigned char Delay_Pass(u32 dly)
{
  if((time_stamp-time_stamps)>dly)
  {
    time_stamps=time_stamp;
    return 1;
  }
  else
    return 0;
}
/*******************************************************************************
** �������� ��PWM_Init
** �������� �������PWM��ʼ��
** ��ڲ��� ����
** ���ڲ��� ����
*******************************************************************************/
void PWM_Init()
{  
  TIM_TimeBaseInitTypeDef   TIM_TimeBaseStructure;
  TIM_OCInitTypeDef         TIM_OCInitStructure;
  
  /* Time base configuration */
  TIM_TimeBaseStructure.TIM_Period = PWM_Period-1;                              //30000-1; //��������һ�������¼�װ�����Զ���װ�ؼĴ������ڵ�ֵ 80K
  //._Period�����������TIMx_ARR��ֵ������������pwm��Ƶ��)
  
  TIM_TimeBaseStructure.TIM_Prescaler =2;                                       //����������ΪTIMxʱ��Ƶ�ʳ�����Ԥ��Ƶֵ
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;                                  //����ʱ�ӷָ�:TDTS = Tck_tim
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;                   //TIM���ϼ���ģʽ
  TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);                               //����TIM_TimeBaseInitStruct��ָ���Ĳ�����ʼ��TIMx��ʱ�������λ
  
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2;                             //ѡ��ʱ��ģʽ:TIM�����ȵ���ģʽ2
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;                 //�Ƚ����ʹ��
  TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;
  TIM_OCInitStructure.TIM_Pulse = PWM_Period/2;                                 //���ô�װ�벶��ȽϼĴ���������ֵ������ռ�ձ�
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;                     //�������:TIM����Ƚϼ��Ը�
  TIM_OC3Init(TIM4, &TIM_OCInitStructure);                                      //����TIM_OCInitStruct��ָ���Ĳ�����ʼ������TIMx
  TIM_OC3PreloadConfig(TIM4, TIM_OCPreload_Enable);                             //ʹ��TIMx��CCR3�ϵ�Ԥװ�ؼĴ���
  
  TIM_ARRPreloadConfig(TIM4, ENABLE);                                           //ʹ��TIMx��ARR�ϵ�Ԥװ�ؼĴ���
  TIM_ITConfig(TIM4,TIM_IT_Update,ENABLE);                                      // �ж�ʹ��
  TIM_ClearFlag(TIM4,TIM_FLAG_Update);		                                // �������жϱ�־
  //TIM_Cmd(TIM4, DISABLE); 
  TIM_CtrlPWMOutputs(TIM4,ENABLE);
  TIM_Cmd(TIM4, ENABLE); //ʹ��TIMx����
  TIM4->CCR3 = PWM_Period-2;//6520;
}
/**************************************************************************
����ԭ��:void GPIO_Config(void)
����:	 �˿�����ģʽ����
��ڲ���: ��
���ڲ���: ��
**************************************************************************/
void GPIO_Config(void)
{                                         
  GPIO_InitTypeDef GPIO_InitStructure;
  
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;                                     //PWM���Ʒ����
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOB, &GPIO_InitStructure); 
}
/******************* (C) COPYRIGHT 2007 STMicroelectronics *****END OF FILE****/