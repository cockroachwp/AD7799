#ifndef __BT_H
#define __BT_H

#define AT_LED_OFF   GPIO_SetBits(GPIOB,GPIO_Pin_8)//读动脉端温度指示灯
#define AT_LED_ON  GPIO_ResetBits(GPIOB,GPIO_Pin_8)
#define VT_LED_OFF   GPIO_SetBits(GPIOB,GPIO_Pin_9)//读静脉端温度指示灯
#define VT_LED_ON  GPIO_ResetBits(GPIOB,GPIO_Pin_9)
#define AT_LED_Check GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_8)  //检测传感器板的盖子是否关闭，0为关闭，1为开启
#define VT_LED_Check GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_9)  //检测传感器板的盖子是否关闭，0为关闭，1为开启

#define T_Ea_MAX 5             //温度误差上限，℃
#define DT 3.074285714                 //固定补偿值，由试验所得，℃
#define T_SAMPLE_SIZE 100      //采样样本个数

typedef struct{
  u8 AT_Correction_Abnormal;//动脉血温校正异常，1表示异常
  u8 VT_Correction_Abnormal;//静脉血温校正异常，1表示异常
  float AT_Ea;//动脉血温误差，测量值-真实值，℃
  float VT_Ea;//静脉血温误差，测量值-真实值，℃
}BT_Parameter;

typedef struct{
  float AT;//动脉血温，修正后的值，℃
  float VT;//静脉血温，修正后的值，℃
  float at;//动脉血温，传感器读出值，℃
  float vt;//静脉血温，传感器读出值，℃
  u8 AT_Abnormal;//动脉血温异常，1表示异常
  u8 VT_Abnormal;//静脉血温异常，1表示异常
}BT_Data;

extern BT_Parameter BT_PARM;
extern BT_Data BT;

void BT_Process(void);
float AT_Filtering(float data);
float VT_Filtering(float data);


#endif