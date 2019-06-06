/****************************************************
Copyright (c) 2012,重庆山外山科技有限公司技术中心
All rights reserved.
文件名称：	main.c
摘 	 要 ：	
当前版本：      V6.0
作   者 ：	童锦
修改内容：	创建该文件并实现功能
完成日期：	2013-07-25
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

#define S_VERSION      600      //软件版本

//BUN计算参数  ABS=a*abs+b
#define a 1         //4.4957    //尿素浓度系数
#define b 0         //原则上为0，基于0点吸光度校准，不建议进行修正

#define AD_time_10 1000 //AD采样间隔，10s一次  放大100倍
#define AD_time_60 6000 //AD采样间隔，60s一次  放大100倍
#define AD_timeChange 72000 //12分钟后切换采样频率
#define ADERRDATA 16777000 //AD采样数据上限
#define ABS_Init_time 1   //吸光度0点采样次数
#define AD_Init_time 1    //AD采样0点采样次数
#define ABS_GAIN  12000      //尿素浓度放大倍数

char *ps;
u8 lsbuff[16]={0};
u32 time_stamp=0,time_stamps=0;

u16 H_VERSION = 600;               // 硬件版本
u16 BORD_CODE;                     // 部件代码

UREA_WorkInfo_Def UREA_Data; 

u32 Enable_Date=0;//启动时间缓存
u32 AD1[5],AD2[5];//AD值存储数组
u32 ad10=0,ad20=0;//AD初始值（0点）存储
float lga=0;      //吸光度计算参数存储
float ABS=0;      //吸光度ABS=a*abs+b
float abs=0;      //吸光度abs=lga+lg((AD1-ad10)/(AD2-ad20))

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
  Hardware_Init();      //硬件初始化    
  while(1)
  {  
    
    Down_flag();              //解析标志位
 
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
void Down_flag(void)              //解析标志位
{
  if(BUS_Buffer.WR0[0] == 1)
  {
    BUS_Buffer.WR0[0] = 0;
    CMD_Parse();    //命令处理
  } 
  /* 设置硬件版本信息 */
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
void Up_flag(void)                //更新标志位
{
  u8 i=0,j=0;
  u32 tmp1_ad1=0,tmp2_ad1=0,tmp1_ad2=0,tmp2_ad2=0;//滤波缓存值
  u32 AD1_Filter[5],AD2_Filter[5];  //AD排序缓存
  u32 AD1_Median=0,AD2_Median=0;    //中值

  BUS_Buffer.RD0[0] = (UREA_Data.Calibration << 2) | (UREA_Data.Err << 1) | UREA_Data.Enable;

  /***************ABS****************/
  if(up_flag == 1 && UREA_Data.Enable ==  ENABLE)
  {
    //将AD排序，将排序结果存储到AD_Filter
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
      ABS=-ABS;   //ABS只传正值
      BUS_Buffer.RD0[2] = (u8)(ABS * ABS_GAIN);   //放大ABS_GAIN倍
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
  
  
  /*****************UREA校准*****************/
  if(UREA_Data.Calibration_Enable == ENABLE)
  {
    ad10 = 0;
    ad20 = 0;
    lga = 0;
    UREA_Data.Calibration = ERROR; 
    UREA_Data.Err = 0;
    
    /***********获取传感器吸光度0点************/
    UV_ON; //打开发射LED灯
    Delay_10ms(100);
    for(i=0;i<ABS_Init_time;i++)
    {
      UR_AD_Conversion(400);  //AD采样 810ms更新一组数据  LED每次开启2S  200=4S
      if(AD_Value[0].Value==0 || AD_Value[0].Value > 16777000 || AD_Value[1].Value==0 || AD_Value[1].Value > 16777000)
      {
        err_flag++;   //  传感器读数错误
      }
      else 
      {
        ad1_data+=AD_Value[0].Value;
        ad2_data+=AD_Value[1].Value;   
      }
    }
    UV_OFF; //关闭发射LED灯
    
    /***********获取传感器AD采样0点值************/
    for(i=0;i<AD_Init_time;i++)
    {
      UR_AD_Conversion(200);  //AD采样 810ms更新一组数据  200=2S

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
      UREA_Data.Calibration = ERROR;           //复位传感器校准标志 未校准
      UREA_Data.Err = 1;                       //传感器异常
      ad10 = 0;
      ad20 = 0;
      ad1_data = 0;
      ad2_data = 0;
      err_flag = 0;
      lga = 0;
    }
    else
    {
      UREA_Data.Calibration = SUCCESS;        //传感器校准成功
      UREA_Data.Err = 0;                      //传感器正常
      ad10 /= AD_Init_time;
      ad20 /= AD_Init_time; 
      ad1_data /= ABS_Init_time;
      ad2_data /= ABS_Init_time;
      lga = -log10(((float)ad1_data-(float)ad10)/((float)ad2_data-(float)ad20));
    }
    
    UREA_Data.Calibration_Enable = DISABLE; //复位校准启动标志
  }
  
  /*****************UREA检测*****************/
  if(UREA_Data.Enable == ENABLE)
  { 
    Enable_flag = ENABLE;
    if((((time_stamp-Enable_Date) < AD_timeChange) & ((time_stamp-Enable_Date)%AD_time_10 == 0)) || (((time_stamp-Enable_Date) > AD_timeChange) & ((time_stamp-Enable_Date)%AD_time_60 == 0)))
    {
      up_flag = 1;//置位更新数据标志位
      
      UV_ON;//6520; //打开发射LED灯 
      Delay_10ms(100);
      UR_AD_Conversion(400);  //AD采样 810ms更新一组数据  LED每次开启4S  400=4S
      UV_OFF; //关闭发射LED灯
      
      if(AD_Value[0].Value==0 || AD_Value[0].Value > 16777000 || AD_Value[1].Value==0 || AD_Value[1].Value > 16777000)
      {
        err_flag++;   //  传感器读数错误
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
        UREA_Data.Err = 1;                       //传感器异常
        err_flag = 0;
      }   
    }
  }  
  else //传感器由状态打开，变更为关闭，清除所有数据
  {
    UV_OFF; //关闭发射LED灯
    if(Enable_flag != DISABLE)
    {
      Enable_flag = DISABLE;
      ad_update();//AD7799复位
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
  for(findex=0;findex<Con_Tim;findex++)//每810ms更新一组数据  LED每次开启4S
  {
     Delay_10ms(1);
     ADC_Auto_Conversion();  
     if(AD_Value[0].Value>=ADERRDATA || AD_Value[1].Value>=ADERRDATA)
     {
       if(ADerr_flag < 10) findex=0;//如果出错重新开始测试，出错超过5次，跳出循环
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
    Enable_Date = time_stamp;//缓存开始时间
  }
  UREA_Data.Calibration_Enable = (BUS_Buffer.WR0[1] >> 1) & 0x01;
}

/*****************************************************************************
** 函数名称： H_VERSION_Write
** 函数功能：写硬件版本信息
** 入口参数：无
** 出口参数：无
*****************************************************************************/
void H_VERSION_Write(void)
{
  u8 res;
 
  /* 保存部件代码 */
  res = Flash_Save_Param(ADD_CODE,(u8 *)&BUS_Buffer.WR7[1],2);
  if(res)
  {
    Delay_10ms(1);
    res = Flash_Save_Param(ADD_CODE,(u8 *)&BUS_Buffer.WR7[1],2);
  }
  /* 保存硬件版本 */
  res = Flash_Save_Param(ADD_HV,(u8 *)&BUS_Buffer.WR7[3],2);
  if(res)
  {
    Delay_10ms(1);
    res = Flash_Save_Param(ADD_HV,(u8 *)&BUS_Buffer.WR7[3],2);
  }   
}
/**************************************************************************
** 函数名称：Version_Load
** 函数功能：加载版本信息
** 入口参数：无
** 出口参数：无
***************************************************************************/
void Version_Load(void)
{
  u8 res;
  u8 temp[2];
  
  /* 读部件代码 */
  res = Flash_Read_Param(ADD_CODE,temp,2);
  /* 如果读取失败 */
  if(res)
  {
    
  }
  BORD_CODE = temp[1]*100 + temp[0];
  /* 读硬件版本 */
  res = Flash_Read_Param(ADD_HV,temp,2);
  /* 如果读取失败 */
  if(res)
  {
    
  }
  H_VERSION = temp[1]*100 + temp[0];

  /* 低字节在前 */
  BUS_Buffer.RD7[0] = (u8)(BORD_CODE % 100); 
  BUS_Buffer.RD7[1] = (u8)(BORD_CODE / 100);
  BUS_Buffer.RD7[2] = (u8)(H_VERSION % 100);
  BUS_Buffer.RD7[3] = (u8)(H_VERSION / 100);
  BUS_Buffer.RD7[4] = (u8)(S_VERSION % 100);
  BUS_Buffer.RD7[5] = (u8)(S_VERSION / 100);
}

/*******************************************************************************
* Function Name  : Hardware_Init
* Description    : 根据需要配置不同的硬件
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
* Description    : 配置系统时钟
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
* Description    :配置嵌套中断向量，用户可根据需要添加自己的中断
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
* Description    : 时钟滴答
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
** 函数名称 ：Delay()
** 函数功能 ：
** 入口参数 ：
** 出口参数 ：
********************************************************************************/
void Delay_10ms(u32 dly)
{
  u32 i=time_stamp;
  while((dly+i)>time_stamp);
}
/********************************************************************************
** 函数名称 ：Delay_Pass()
** 函数功能 ：
** 入口参数 ：
** 出口参数 ：
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
** 函数名称 ：PWM_Init
** 函数功能 ：发射灯PWM初始化
** 入口参数 ：无
** 出口参数 ：无
*******************************************************************************/
void PWM_Init()
{  
  TIM_TimeBaseInitTypeDef   TIM_TimeBaseStructure;
  TIM_OCInitTypeDef         TIM_OCInitStructure;
  
  /* Time base configuration */
  TIM_TimeBaseStructure.TIM_Period = PWM_Period-1;                              //30000-1; //设置在下一个更新事件装入活动的自动重装载寄存器周期的值 80K
  //._Period这个即是设置TIMx_ARR的值，即用来设置pwm的频率)
  
  TIM_TimeBaseStructure.TIM_Prescaler =2;                                       //设置用来作为TIMx时钟频率除数的预分频值
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;                                  //设置时钟分割:TDTS = Tck_tim
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;                   //TIM向上计数模式
  TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);                               //根据TIM_TimeBaseInitStruct中指定的参数初始化TIMx的时间基数单位
  
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2;                             //选择定时器模式:TIM脉冲宽度调制模式2
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;                 //比较输出使能
  TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;
  TIM_OCInitStructure.TIM_Pulse = PWM_Period/2;                                 //设置待装入捕获比较寄存器的脉冲值，设置占空比
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;                     //输出极性:TIM输出比较极性高
  TIM_OC3Init(TIM4, &TIM_OCInitStructure);                                      //根据TIM_OCInitStruct中指定的参数初始化外设TIMx
  TIM_OC3PreloadConfig(TIM4, TIM_OCPreload_Enable);                             //使能TIMx在CCR3上的预装载寄存器
  
  TIM_ARRPreloadConfig(TIM4, ENABLE);                                           //使能TIMx在ARR上的预装载寄存器
  TIM_ITConfig(TIM4,TIM_IT_Update,ENABLE);                                      // 中断使能
  TIM_ClearFlag(TIM4,TIM_FLAG_Update);		                                // 清除溢出中断标志
  //TIM_Cmd(TIM4, DISABLE); 
  TIM_CtrlPWMOutputs(TIM4,ENABLE);
  TIM_Cmd(TIM4, ENABLE); //使能TIMx外设
  TIM4->CCR3 = PWM_Period-2;//6520;
}
/**************************************************************************
函数原形:void GPIO_Config(void)
功能:	 端口引脚模式配置
入口参数: 无
出口参数: 无
**************************************************************************/
void GPIO_Config(void)
{                                         
  GPIO_InitTypeDef GPIO_InitStructure;
  
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;                                     //PWM控制发射灯
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOB, &GPIO_InitStructure); 
}
/******************* (C) COPYRIGHT 2007 STMicroelectronics *****END OF FILE****/