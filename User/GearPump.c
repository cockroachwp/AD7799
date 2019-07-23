/******************************************************************************
Copyright (c) 2012,重庆山外山科技有限公司技术中心
All rights reserved.

文件名称:  GearPump.c
摘    要:  齿轮泵控制相关函数
当前版本:  V4.60
作    者:  童锦
修改内容:  --
完成日期:  2012年4月17日
******************************************************************************/
/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"
#include "main.h"
#include "GearPump.h"
#include "flash.h"

//u8  DP_PulseNum = 0;         //齿轮泵DP测速脉冲个数
//u8  YP_PulseNum = 0;         //齿轮泵YP测速脉冲个数
//u8  QP_PulseNum = 0;         //齿轮泵QP测速脉冲个数
//u8  DP_SpeedErr = 0;         //齿轮泵DP异常:进液泵损坏或测速信号异常
//u8  YP_SpeedErr = 0;         //齿轮泵YP异常:废液泵损坏或测速信号异常
//u8  QP_SpeedErr = 0;         //齿轮泵QP异常:除气泵损坏或测速信号异常
//u8  DP_ErrNum = 0;           //齿轮泵DP异常次数
//u8  YP_ErrNum = 0;           //齿轮泵YP异常次数
//u8  QP_ErrNum = 0;           //废液泵异常次数
////u8  DP_Direction = foreward; //齿轮泵DP转动方向
//u8  YP_Direction = foreward; //齿轮泵YP转动方向
//u8  QP_Direction = foreward; //齿轮泵QP转动方向
//u16 DP_FirstPulseTime = 0;   //齿轮泵DP第一个速度信号计时点时间
//u16 YP_FirstPulseTime = 0;   //齿轮泵YP第一个速度信号计时点时间
//u16 QP_FirstPulseTime = 0;   //齿轮泵QP第一个速度信号计时点时间
//u16 DP_ElevenPulseTime = 0;  //齿轮泵DP第二个速度信号计时点时间
//u16 YP_ElevenPulseTime = 0;  //齿轮泵YP第二个速度信号计时点时间
//u16 QP_ElevenPulseTime = 0;  //齿轮泵QP第二个速度信号计时点时间
//u16 DP_AimSpeed = 0;         //齿轮泵DP目标速度
//u16 YP_AimSpeed = 0;         //齿轮泵YP目标速度
//u16 QP_AimSpeed = 0;         //齿轮泵QP目标速度
//u16 DP_SpeedTestTime = 0;    //齿轮泵DP无速度信号时间
//u16 YP_SpeedTestTime = 0;    //齿轮泵YP无速度信号时间
//u16 QP_SpeedTestTime = 0;    //齿轮泵QP无速度信号时间
//u16 DP_Ratio = 0;               //齿轮泵DP每转流量（放大1000倍）
//u16 YP_Ratio = 0;               //齿轮泵YP每转流量（放大1000倍）
//u8 GP_Ratio[5];
//INTEGER DPV;           //齿轮泵DP调节电压PWM值
//INTEGER YPV;           //齿轮泵YP调节电压PWM值
//INTEGER QPV;           //齿轮泵QP调节电压PWM值
//INTEGER DPREV;         //齿轮泵DP转速
//INTEGER YPREV;         //齿轮泵YP转速
//INTEGER QPREV;         //齿轮泵QP转速


/*******************************************************************************
* Function Name  : GearPump_Init
* Description    : 齿轮泵相关控制端口初始化
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void GearPump_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  TIM_ICInitTypeDef TIM_ICInitStructure;
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;

   RCC_APB1PeriphClockCmd( RCC_APB1Periph_TIM4 , ENABLE);

  /* GPIOA,GPIOB,GPIOC,AFIO,AFIO clock enable */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC |
                         RCC_APB2Periph_ADC1 | RCC_APB2Periph_AFIO | RCC_APB2Periph_SPI1 | RCC_APB2Periph_TIM1, ENABLE);


  /*******************齿轮泵正反转控制端口配置*********************************/
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOC, &GPIO_InitStructure);

  /***************齿轮泵测速端口GPIO及定时器配置*******************************/
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;
  TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
  TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
  TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
  TIM_ICInitStructure.TIM_ICFilter = 0x0;
  TIM_ICInit(TIM2, &TIM_ICInitStructure);

  TIM_ICInitStructure.TIM_Channel = TIM_Channel_2;
  TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
  TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
  TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
  TIM_ICInitStructure.TIM_ICFilter = 0x0;
  TIM_ICInit(TIM2, &TIM_ICInitStructure);

  TIM_ICInitStructure.TIM_Channel = TIM_Channel_3;
  TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
  TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
  TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
  TIM_ICInitStructure.TIM_ICFilter = 0x0;
  TIM_ICInit(TIM2, &TIM_ICInitStructure);

  TIM_PrescalerConfig(TIM2, 0x0E0F, TIM_PSCReloadMode_Immediate);
  TIM_ITConfig(TIM2, TIM_IT_CC1, ENABLE); //齿轮泵DP测速捕获中断使能
  TIM_ITConfig(TIM2, TIM_IT_CC2, ENABLE); //齿轮泵YP测速捕获中断使能
  TIM_ITConfig(TIM2, TIM_IT_CC3, ENABLE); //齿轮泵QP测速捕获中断使能
  TIM_Cmd(TIM2, ENABLE);

  /***************齿轮泵速度控制端口GPIO及定时器配置***************************/
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

  TIM_TimeBaseStructure.TIM_Period = 3599; //频率10KHz
  TIM_TimeBaseStructure.TIM_Prescaler = 0;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);
  TIM_Cmd(TIM4, ENABLE);    //使能TIM4

  GearPumpControlPara(DP,0);
  GearPumpControlPara(YP,0);
  GearPumpControlPara(QP,0);

  DP_FOREWARD;
  YP_FOREWARD;
  QP_FOREWARD;
//  Flash_Read_Param(FLASH_GP_ADDR,GP_Ratio,5);  //读齿轮泵系数
//  if(GP_Ratio[4] == GP_Ratio[0]+GP_Ratio[1]+GP_Ratio[2]+GP_Ratio[3])
//  {
//    DP_Ratio = GP_Ratio[0] + GP_Ratio[1]*256;
//    YP_Ratio = GP_Ratio[2] + GP_Ratio[3]*256;
//  }
//  else
//  {
  //DP_Ratio = 300;
  //YP_Ratio = 300;
//  }
}

/*******************************************************************************
* Function Name  : GearPumpControlPara
* Description    : 齿轮泵控制参数
* Input          : PumpNumber--齿轮泵编号；PumpPara- 齿轮泵控制参数
* Output         : None
* Return         : None
*******************************************************************************/
void GearPumpControlPara(u8 PumpNumber,u16 PumpPara)
{
  TIM_OCInitTypeDef  TIM_OCInitStructure;

  if(PumpNumber==DP)
  {
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = PumpPara;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;//TIM_OCPolarity_High;

    TIM_OC1Init(TIM4, &TIM_OCInitStructure);
    TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Disable);
  }
  else if(PumpNumber==YP)
  {
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = PumpPara;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;//TIM_OCPolarity_High;

    TIM_OC2Init(TIM4, &TIM_OCInitStructure);
    TIM_OC2PreloadConfig(TIM4, TIM_OCPreload_Disable);
  }
  else if(PumpNumber==QP)
  {
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = PumpPara;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;//TIM_OCPolarity_High;

    TIM_OC3Init(TIM4, &TIM_OCInitStructure);
    TIM_OC3PreloadConfig(TIM4, TIM_OCPreload_Disable);
  }

  TIM_Cmd(TIM4, ENABLE);    //使能TIM4
}
//
///*******************************************************************************
//* Function Name  : GearPumpSpeedAdjust
//* Description    : 齿轮泵速度调节
//* Input          : PumpNumber--齿轮泵编号；AimSpeed- 齿轮泵目标速度
//* Output         : None
//* Return         : None
//*******************************************************************************/
//void GearPumpSpeedAdjust(u8 PumpNumber,u16 AimSpeed)
//{
//  u8  k = 5;          //调节系数
//  s16 AdjustPara = 0;// 调节变化量
//  if(PumpNumber==DP)
//  {
//    AdjustPara = (AimSpeed-DP_REV)/k;
//    if(DP_V+AdjustPara <= 800)
//      DP_V= 800;
//    else if((DP_V+AdjustPara>800) && (DP_V+AdjustPara<3500))
//      DP_V += AdjustPara;
//    else
//      DP_V = 3500;
//    GearPumpControlPara(DP,DP_V);
//  }
//
//  else if(PumpNumber==YP)
//  {
//    AdjustPara = (AimSpeed-YP_REV)/k;
//    if(YP_V+AdjustPara <= 800)
//      YP_V = 800;
//    else if((YP_V+AdjustPara>800) && (YP_V+AdjustPara<3500))
//      YP_V += AdjustPara;
//    else
//      YP_V = 3500;
//    GearPumpControlPara(YP,YP_V);
//  }
//
//  else if(PumpNumber==QP)
//  {
//    AdjustPara = (AimSpeed-QP_REV)/k;
//    if(QP_V+AdjustPara <= 800)
//      QP_V = 800;
//    else if((QP_V+AdjustPara>800) && (QP_V+AdjustPara<3500))
//      QP_V += AdjustPara;
//    else
//      QP_V = 3500;
//    GearPumpControlPara(QP,QP_V);
//  }
//}
///*******************************************************************************
//* Function Name  : GearPumpSpeedTest
//* Description    : 齿轮泵速度测试
//* Input          : PumpNumber--齿轮泵编号
//* Output         : None
//* Return         : None
//*******************************************************************************/
//void GearPumpSpeedTest(u8 PumpNumber)
//{
//  if(PumpNumber == DP)
//  {
//    DP_PulseNum++;
//    DP_SpeedTestTime = 0;                  //齿轮泵DP测速时间清零
//    if(DP_PulseNum == 1)
//      DP_FirstPulseTime = TIM_GetCapture1(TIM2);    //更新脉冲周期
//    if(DP_PulseNum == 11)
//    {
//      DP_PulseNum = 0;
//      DP_ElevenPulseTime = TIM_GetCapture1(TIM2);
//      if(DP_ElevenPulseTime > DP_FirstPulseTime) //定时器未溢出
//        DP_REV = 6000000 / (DP_ElevenPulseTime - DP_FirstPulseTime);
//      else                                               //定时器溢出
//        DP_REV = 6000000 / (65535 + DP_ElevenPulseTime - DP_FirstPulseTime);
//      GearPumpSpeedAdjust(DP,DP_AimSpeed);//齿轮泵DP速度速度调节
//    }
//  }
//  else if(PumpNumber == YP)
//  {
//    YP_PulseNum++;
//    YP_SpeedTestTime = 0;                  //齿轮泵YP测速时间清零
//    if(YP_PulseNum == 1)
//      YP_FirstPulseTime = TIM_GetCapture2(TIM2);    //更新脉冲周期
//    if(YP_PulseNum == 11)
//    {
//      YP_PulseNum = 0;
//      YP_ElevenPulseTime = TIM_GetCapture2(TIM2);
//      if(YP_ElevenPulseTime > YP_FirstPulseTime)
//        YP_REV = 6000000 / (YP_ElevenPulseTime - YP_FirstPulseTime);
//      else
//        YP_REV = 6000000 / (65535 + YP_ElevenPulseTime - YP_FirstPulseTime);
//      GearPumpSpeedAdjust(YP,YP_AimSpeed);
//    }
//  }
//  else if(PumpNumber==QP)
//  {
//    QP_PulseNum++;
//    QP_SpeedTestTime = 0;                  //齿轮泵QP测速时间清零
//    if(QP_PulseNum == 1)
//      QP_FirstPulseTime = TIM_GetCapture3(TIM2);   //更新脉冲周期
//    if(QP_PulseNum == 11)
//    {
//      QP_PulseNum = 0;
//      QP_ElevenPulseTime = TIM_GetCapture3(TIM2);
//      if(QP_ElevenPulseTime > QP_FirstPulseTime)
//        QP_REV = 6000000 / (QP_ElevenPulseTime - QP_FirstPulseTime);
//      else
//        QP_REV = 6000000 / (65535 + QP_ElevenPulseTime - QP_FirstPulseTime);
//      GearPumpSpeedAdjust(QP,QP_AimSpeed);
//    }
//  }
//}
//
//
///*******************************************************************************
//* Function Name  : GearPumpStart
//* Description    : 齿轮泵启动
//* Input          : PumpNumber--齿轮泵编号
//* Output         : None
//* Return         : None
//*******************************************************************************/
//void GearPumpStart(u8 PumpNumber)
//{
//  if(PumpNumber == DP)
//  {
//    DP_V = 2000;
//    DP_SpeedTestTime = 0;
//    GearPumpControlPara(DP,DP_V);
//  }
//  else if(PumpNumber == YP)
//  {
//    YP_V = 2000;
//    YP_SpeedTestTime = 0;
//    GearPumpControlPara(YP,YP_V);
//  }
//  else if(PumpNumber == QP)
//  {
//    QP_V = 3000;
//    QP_SpeedTestTime = 0;
//    GearPumpControlPara(QP,QP_V);
//  }
//}
///*******************************************************************************
//* Function Name  : GearPumpStop
//* Description    : 齿轮泵停止
//* Input          : PumpNumber--齿轮泵编号
//* Output         : None
//* Return         : None
//*******************************************************************************/
//void GearPumpStop(u8 PumpNumber)
//{
//  if(PumpNumber == DP)
//  {
//    DP_V = 0;
//    GearPumpControlPara(DP,DP_V);
//  }
//  else if(PumpNumber == YP)
//  {
//    YP_V = 0;
//    GearPumpControlPara(YP,YP_V);
//  }
//  else if(PumpNumber == QP)
//  {
//    QP_V = 0;
//    GearPumpControlPara(QP,QP_V);
//  }
//}
///*******************************************************************************
//* Function Name  : GearPumpSpeedErrCheck
//* Description    : 齿轮泵速度异常处理
//* Input          : None
//* Output         : None
//* Return         : None
//*******************************************************************************/
//void GearPumpSpeedErrCheck(void)
//{
//  /*************************齿轮泵DP异常检测*************************************/
//  if(DP_AimSpeed == 0)
//    GearPumpStop(DP);
//  else if(DP_V == 0)
//    GearPumpStart(DP);
//
//  if(DP_SpeedTestTime > 10)
//  {
//    DP_REV = 0;
//    if((DP_AimSpeed !=0)&&(DP_V != 0)&&(DP_SpeedTestTime > 200))
//    {
//      DP_SpeedErr = 1;
//      DP_AimSpeed = 0;
//    }
//  }
//  else
//    DP_SpeedErr = 0;
//  /*************************齿轮泵YP异常检测*************************************/
//  if(YP_AimSpeed == 0)
//    GearPumpStop(YP);
//  else if(YP_V == 0)
//    GearPumpStart(YP);
//
//  if(YP_SpeedTestTime > 10)
//  {
//    YP_REV = 0;
//    if((YP_AimSpeed !=0)&&(YP_V != 0)&&(YP_SpeedTestTime > 200))
//    {
//      YP_SpeedErr = 1;
//      YP_AimSpeed = 0;
//    }
//  }
//  else
//    YP_SpeedErr = 0;
//  /*************************齿轮泵QP异常检测*************************************/
//  if(QP_AimSpeed == 0)
//    GearPumpStop(QP);
//  else if(QP_V == 0)
//    GearPumpStart(QP);
//
//  if(QP_SpeedTestTime > 10)
//  {
//    QP_REV = 0;
//    if((QP_AimSpeed !=0)&&(QP_V != 0)&&(QP_SpeedTestTime > 200))
//    {
//      QP_SpeedErr = 1;
//      QP_AimSpeed = 0;
//    }
//  }
//  else
//    QP_SpeedErr = 0;
//}