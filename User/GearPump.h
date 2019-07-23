/**************************** (C) COPYRIGHT 2008 SWS ***************************
文件名称:  GearPumpControl.h
摘    要:  齿轮泵控制相关函数头文件
当前版本:  V4.60
作    者:  童锦
修改内容:  --
完成日期:  2012年8月15日
*******************************************************************************/
#ifndef __GEARPUMPCONTROL_H
#define __GEARPUMPCONTROL_H


#define DP   1     //齿轮泵DP编号
#define YP   2     //齿轮泵YP编号
#define QP   3     //齿轮泵QP编号

#define DP_V        DPV.ID             //齿轮泵DP调节电压
#define DP_VH       DPV.CD[1]          //齿轮泵DP调节电压高字节
#define DP_VL       DPV.CD[0]          //齿轮泵DP调节电压低字节
#define YP_V        YPV.ID             //齿轮泵YP调节电压
#define YP_VH       YPV.CD[1]          //齿轮泵YP调节电压高字节
#define YP_VL       YPV.CD[0]          //齿轮泵YP调节电压低字节
#define QP_V        QPV.ID             //齿轮泵QP调节电压
#define QP_VH       QPV.CD[1]          //齿轮泵QP调节电压高字节
#define QP_VL       QPV.CD[0]          //齿轮泵QP调节电压低字节

#define DP_REV      DPREV.ID
#define DP_REVH     DPREV.CD[1]
#define DP_REVL     DPREV.CD[0]
#define YP_REV      YPREV.ID
#define YP_REVH     YPREV.CD[1]
#define YP_REVL     YPREV.CD[0]
#define QP_REV      QPREV.ID
#define QP_REVH     QPREV.CD[1]
#define QP_REVL     QPREV.CD[0]

#define DP_FOREWARD   GPIO_ResetBits(GPIOC , GPIO_Pin_13)  //齿轮泵DP正转
#define YP_FOREWARD   GPIO_ResetBits(GPIOC , GPIO_Pin_14)  //齿轮泵YP正转
#define QP_FOREWARD   GPIO_ResetBits(GPIOC , GPIO_Pin_15)  //齿轮泵QP正转

#define DP_REVERSE   GPIO_SetBits(GPIOC , GPIO_Pin_13)//齿轮泵DP反转
#define YP_REVERSE   GPIO_SetBits(GPIOC , GPIO_Pin_14)//齿轮泵YP反转
#define QP_REVERSE   GPIO_SetBits(GPIOC , GPIO_Pin_15)//齿轮泵QP反转

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
extern u16 DP_Ratio;               //齿轮泵DP每转流量（放大1000倍）
extern u16 YP_Ratio;               //齿轮泵YP每转流量（放大1000倍）
//extern INTEGER DPV;           //DP泵调节电压PWM值
//extern INTEGER YPV;           //YP泵调节电压PWM值
//extern INTEGER QPV;           //QP泵调节电压PWM值
//extern INTEGER DPREV;         //齿轮泵DP转速
//extern INTEGER YPREV;         //齿轮泵YP转速
//extern INTEGER QPREV;         //齿轮泵QP转速
extern u8 GP_Ratio[5];

void GearPump_Init(void);
void GearPumpControlPara(u8 PumpNumber,u16 PumpPara);
void GearPumpSpeedAdjust(u8 PumpNumber,u16 AimSpeed);
void GearPumpSpeedTest(u8 PumpNumber);
void GearPumpStart(u8 PumpNumber);
void GearPumpStop(u8 PumpNumber);
void GearPumpSpeedErrCheck();

#endif