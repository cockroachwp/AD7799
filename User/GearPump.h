/**************************** (C) COPYRIGHT 2008 SWS ***************************
�ļ�����:  GearPumpControl.h
ժ    Ҫ:  ���ֱÿ�����غ���ͷ�ļ�
��ǰ�汾:  V4.60
��    ��:  ͯ��
�޸�����:  --
�������:  2012��8��15��
*******************************************************************************/
#ifndef __GEARPUMPCONTROL_H
#define __GEARPUMPCONTROL_H


#define DP   1     //���ֱ�DP���
#define YP   2     //���ֱ�YP���
#define QP   3     //���ֱ�QP���

#define DP_V        DPV.ID             //���ֱ�DP���ڵ�ѹ
#define DP_VH       DPV.CD[1]          //���ֱ�DP���ڵ�ѹ���ֽ�
#define DP_VL       DPV.CD[0]          //���ֱ�DP���ڵ�ѹ���ֽ�
#define YP_V        YPV.ID             //���ֱ�YP���ڵ�ѹ
#define YP_VH       YPV.CD[1]          //���ֱ�YP���ڵ�ѹ���ֽ�
#define YP_VL       YPV.CD[0]          //���ֱ�YP���ڵ�ѹ���ֽ�
#define QP_V        QPV.ID             //���ֱ�QP���ڵ�ѹ
#define QP_VH       QPV.CD[1]          //���ֱ�QP���ڵ�ѹ���ֽ�
#define QP_VL       QPV.CD[0]          //���ֱ�QP���ڵ�ѹ���ֽ�

#define DP_REV      DPREV.ID
#define DP_REVH     DPREV.CD[1]
#define DP_REVL     DPREV.CD[0]
#define YP_REV      YPREV.ID
#define YP_REVH     YPREV.CD[1]
#define YP_REVL     YPREV.CD[0]
#define QP_REV      QPREV.ID
#define QP_REVH     QPREV.CD[1]
#define QP_REVL     QPREV.CD[0]

#define DP_FOREWARD   GPIO_ResetBits(GPIOC , GPIO_Pin_13)  //���ֱ�DP��ת
#define YP_FOREWARD   GPIO_ResetBits(GPIOC , GPIO_Pin_14)  //���ֱ�YP��ת
#define QP_FOREWARD   GPIO_ResetBits(GPIOC , GPIO_Pin_15)  //���ֱ�QP��ת

#define DP_REVERSE   GPIO_SetBits(GPIOC , GPIO_Pin_13)//���ֱ�DP��ת
#define YP_REVERSE   GPIO_SetBits(GPIOC , GPIO_Pin_14)//���ֱ�YP��ת
#define QP_REVERSE   GPIO_SetBits(GPIOC , GPIO_Pin_15)//���ֱ�QP��ת

extern u8  DP_Direction;
extern u8  YP_Direction;
extern u8  QP_Direction;
extern u8  DP_SpeedErr;
extern u8  YP_SpeedErr;
extern u8  QP_SpeedErr;
extern u16 DP_AimSpeed;
extern u16 YP_AimSpeed;
extern u16 QP_AimSpeed;
extern u16 DP_SpeedTestTime;
extern u16 YP_SpeedTestTime;
extern u16 QP_SpeedTestTime;
extern u16 DP_Ratio;               //���ֱ�DPÿת�������Ŵ�1000����
extern u16 YP_Ratio;               //���ֱ�YPÿת�������Ŵ�1000����
//extern INTEGER DPV;           //DP�õ��ڵ�ѹPWMֵ
//extern INTEGER YPV;           //YP�õ��ڵ�ѹPWMֵ
//extern INTEGER QPV;           //QP�õ��ڵ�ѹPWMֵ
//extern INTEGER DPREV;         //���ֱ�DPת��
//extern INTEGER YPREV;         //���ֱ�YPת��
//extern INTEGER QPREV;         //���ֱ�QPת��
extern u8 GP_Ratio[5];

void GearPump_Init(void);
void GearPumpControlPara(u8 PumpNumber,u16 PumpPara);
void GearPumpSpeedAdjust(u8 PumpNumber,u16 AimSpeed);
void GearPumpSpeedTest(u8 PumpNumber);
void GearPumpStart(u8 PumpNumber);
void GearPumpStop(u8 PumpNumber);
void GearPumpSpeedErrCheck();

#endif