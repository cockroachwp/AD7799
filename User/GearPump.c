/******************************************************************************
Copyright (c) 2012,����ɽ��ɽ�Ƽ����޹�˾��������
All rights reserved.

�ļ�����:  GearPump.c
ժ    Ҫ:  ���ֱÿ�����غ���
��ǰ�汾:  V4.60
��    ��:  ͯ��
�޸�����:  --
�������:  2012��4��17��
******************************************************************************/
/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"
#include "main.h"
#include "GearPump.h"
#include "flash.h"

//u8  DP_PulseNum = 0;         //���ֱ�DP�����������
//u8  YP_PulseNum = 0;         //���ֱ�YP�����������
//u8  QP_PulseNum = 0;         //���ֱ�QP�����������
//u8  DP_SpeedErr = 0;         //���ֱ�DP�쳣:��Һ���𻵻�����ź��쳣
//u8  YP_SpeedErr = 0;         //���ֱ�YP�쳣:��Һ���𻵻�����ź��쳣
//u8  QP_SpeedErr = 0;         //���ֱ�QP�쳣:�������𻵻�����ź��쳣
//u8  DP_ErrNum = 0;           //���ֱ�DP�쳣����
//u8  YP_ErrNum = 0;           //���ֱ�YP�쳣����
//u8  QP_ErrNum = 0;           //��Һ���쳣����
////u8  DP_Direction = foreward; //���ֱ�DPת������
//u8  YP_Direction = foreward; //���ֱ�YPת������
//u8  QP_Direction = foreward; //���ֱ�QPת������
//u16 DP_FirstPulseTime = 0;   //���ֱ�DP��һ���ٶ��źż�ʱ��ʱ��
//u16 YP_FirstPulseTime = 0;   //���ֱ�YP��һ���ٶ��źż�ʱ��ʱ��
//u16 QP_FirstPulseTime = 0;   //���ֱ�QP��һ���ٶ��źż�ʱ��ʱ��
//u16 DP_ElevenPulseTime = 0;  //���ֱ�DP�ڶ����ٶ��źż�ʱ��ʱ��
//u16 YP_ElevenPulseTime = 0;  //���ֱ�YP�ڶ����ٶ��źż�ʱ��ʱ��
//u16 QP_ElevenPulseTime = 0;  //���ֱ�QP�ڶ����ٶ��źż�ʱ��ʱ��
//u16 DP_AimSpeed = 0;         //���ֱ�DPĿ���ٶ�
//u16 YP_AimSpeed = 0;         //���ֱ�YPĿ���ٶ�
//u16 QP_AimSpeed = 0;         //���ֱ�QPĿ���ٶ�
//u16 DP_SpeedTestTime = 0;    //���ֱ�DP���ٶ��ź�ʱ��
//u16 YP_SpeedTestTime = 0;    //���ֱ�YP���ٶ��ź�ʱ��
//u16 QP_SpeedTestTime = 0;    //���ֱ�QP���ٶ��ź�ʱ��
//u16 DP_Ratio = 0;               //���ֱ�DPÿת�������Ŵ�1000����
//u16 YP_Ratio = 0;               //���ֱ�YPÿת�������Ŵ�1000����
//u8 GP_Ratio[5];
//INTEGER DPV;           //���ֱ�DP���ڵ�ѹPWMֵ
//INTEGER YPV;           //���ֱ�YP���ڵ�ѹPWMֵ
//INTEGER QPV;           //���ֱ�QP���ڵ�ѹPWMֵ
//INTEGER DPREV;         //���ֱ�DPת��
//INTEGER YPREV;         //���ֱ�YPת��
//INTEGER QPREV;         //���ֱ�QPת��


/*******************************************************************************
* Function Name  : GearPump_Init
* Description    : ���ֱ���ؿ��ƶ˿ڳ�ʼ��
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


  /*******************���ֱ�����ת���ƶ˿�����*********************************/
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOC, &GPIO_InitStructure);

  /***************���ֱò��ٶ˿�GPIO����ʱ������*******************************/
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
  TIM_ITConfig(TIM2, TIM_IT_CC1, ENABLE); //���ֱ�DP���ٲ����ж�ʹ��
  TIM_ITConfig(TIM2, TIM_IT_CC2, ENABLE); //���ֱ�YP���ٲ����ж�ʹ��
  TIM_ITConfig(TIM2, TIM_IT_CC3, ENABLE); //���ֱ�QP���ٲ����ж�ʹ��
  TIM_Cmd(TIM2, ENABLE);

  /***************���ֱ��ٶȿ��ƶ˿�GPIO����ʱ������***************************/
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

  TIM_TimeBaseStructure.TIM_Period = 3599; //Ƶ��10KHz
  TIM_TimeBaseStructure.TIM_Prescaler = 0;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);
  TIM_Cmd(TIM4, ENABLE);    //ʹ��TIM4

  GearPumpControlPara(DP,0);
  GearPumpControlPara(YP,0);
  GearPumpControlPara(QP,0);

  DP_FOREWARD;
  YP_FOREWARD;
  QP_FOREWARD;
//  Flash_Read_Param(FLASH_GP_ADDR,GP_Ratio,5);  //�����ֱ�ϵ��
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
* Description    : ���ֱÿ��Ʋ���
* Input          : PumpNumber--���ֱñ�ţ�PumpPara- ���ֱÿ��Ʋ���
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

  TIM_Cmd(TIM4, ENABLE);    //ʹ��TIM4
}
//
///*******************************************************************************
//* Function Name  : GearPumpSpeedAdjust
//* Description    : ���ֱ��ٶȵ���
//* Input          : PumpNumber--���ֱñ�ţ�AimSpeed- ���ֱ�Ŀ���ٶ�
//* Output         : None
//* Return         : None
//*******************************************************************************/
//void GearPumpSpeedAdjust(u8 PumpNumber,u16 AimSpeed)
//{
//  u8  k = 5;          //����ϵ��
//  s16 AdjustPara = 0;// ���ڱ仯��
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
//* Description    : ���ֱ��ٶȲ���
//* Input          : PumpNumber--���ֱñ��
//* Output         : None
//* Return         : None
//*******************************************************************************/
//void GearPumpSpeedTest(u8 PumpNumber)
//{
//  if(PumpNumber == DP)
//  {
//    DP_PulseNum++;
//    DP_SpeedTestTime = 0;                  //���ֱ�DP����ʱ������
//    if(DP_PulseNum == 1)
//      DP_FirstPulseTime = TIM_GetCapture1(TIM2);    //������������
//    if(DP_PulseNum == 11)
//    {
//      DP_PulseNum = 0;
//      DP_ElevenPulseTime = TIM_GetCapture1(TIM2);
//      if(DP_ElevenPulseTime > DP_FirstPulseTime) //��ʱ��δ���
//        DP_REV = 6000000 / (DP_ElevenPulseTime - DP_FirstPulseTime);
//      else                                               //��ʱ�����
//        DP_REV = 6000000 / (65535 + DP_ElevenPulseTime - DP_FirstPulseTime);
//      GearPumpSpeedAdjust(DP,DP_AimSpeed);//���ֱ�DP�ٶ��ٶȵ���
//    }
//  }
//  else if(PumpNumber == YP)
//  {
//    YP_PulseNum++;
//    YP_SpeedTestTime = 0;                  //���ֱ�YP����ʱ������
//    if(YP_PulseNum == 1)
//      YP_FirstPulseTime = TIM_GetCapture2(TIM2);    //������������
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
//    QP_SpeedTestTime = 0;                  //���ֱ�QP����ʱ������
//    if(QP_PulseNum == 1)
//      QP_FirstPulseTime = TIM_GetCapture3(TIM2);   //������������
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
//* Description    : ���ֱ�����
//* Input          : PumpNumber--���ֱñ��
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
//* Description    : ���ֱ�ֹͣ
//* Input          : PumpNumber--���ֱñ��
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
//* Description    : ���ֱ��ٶ��쳣����
//* Input          : None
//* Output         : None
//* Return         : None
//*******************************************************************************/
//void GearPumpSpeedErrCheck(void)
//{
//  /*************************���ֱ�DP�쳣���*************************************/
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
//  /*************************���ֱ�YP�쳣���*************************************/
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
//  /*************************���ֱ�QP�쳣���*************************************/
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