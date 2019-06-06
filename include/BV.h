#ifndef __BV_H
#define __BV_H

#define MC_LED_ON GPIO_ResetBits(GPIOB,GPIO_Pin_0)            //监测盖状态指示灯开
#define MC_LED_OFF GPIO_SetBits(GPIOB,GPIO_Pin_0)             //监测盖状态指示灯关
#define IR_LED_ON GPIO_ResetBits(GPIOA,GPIO_Pin_7)            //近红外LED工作状态指示灯开
#define IR_LED_OFF GPIO_SetBits(GPIOA,GPIO_Pin_7)             //近红外LED工作状态指示灯关 
#define IR_Switch_ON GPIO_SetBits(GPIOB,GPIO_Pin_10)          //打开近红外LED
#define IR_Switch_OFF GPIO_ResetBits(GPIOB,GPIO_Pin_10)       //关闭近红外LED
#define Cover_Check GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_11)  //检测传感器板的盖子是否关闭，0为关闭，1为开启
#define REF_V 5000             //参考电压，mV
#define BV_SAMPLE_START 10     //盖子关闭且近红外LED灯打开多少秒后开始采集血容量数据，s
#define BV_SAMPLE_PERIOD 1000  //血容量数据采集周期，ms
#define BV_SAMPLE_SIZE 60      //血容量滤波样本个数，每BV_SAMPLE_PERIOD毫秒采集1个
#define RBV_MAX_CHANGE 0.006   //允许单位时间内相对血容量最大变化量，0.006表示0.6%
typedef struct{
  u8 Correction_Abnormal;  //校正异常，1表示异常
  float BV0;               //校正BV0的值
  float BV1;               //校正BV1的值
  float BV0_V;             //校正BV0时受光电压，mV
  float BV1_V;             //校正BV1时受光电压，mV
  float k;                 //校正后的斜率
  float b;                 //校正后的截距
}BV_Parameter;
typedef struct{
  float AD;                //受光电压的AD值
  float V;                 //受光电压，mV
  float BV;                //血容量
  float BV0;               //初始血容量
  float RBV;               //以BV0为基点的相对血容量，90.12表示90.12%
  u8 Initialized;          //0/1-未/已设定初始血容量
  u8 Mode;                 //0/1-模块/主控控制近红外LED
  u8 IR_LED;               //0/1-近红外LED关闭/打开
  u8 Abnormal;             //0/1-血容量传感器正常/异常
  u8 Blood;                //0/1-未检测到/检测到血液
  u8 Cover;                //0/1-监测盖关闭/打开
}BV_Data;

extern BV_Parameter BV_PARM;
extern BV_Data BV;

void BV_Process(void);
void Check_Cover(void);
float BV_Filtering(float data);

#endif