#ifndef __BV_H
#define __BV_H

#define MC_LED_ON GPIO_ResetBits(GPIOB,GPIO_Pin_0)            //����״ָ̬ʾ�ƿ�
#define MC_LED_OFF GPIO_SetBits(GPIOB,GPIO_Pin_0)             //����״ָ̬ʾ�ƹ�
#define IR_LED_ON GPIO_ResetBits(GPIOA,GPIO_Pin_7)            //������LED����״ָ̬ʾ�ƿ�
#define IR_LED_OFF GPIO_SetBits(GPIOA,GPIO_Pin_7)             //������LED����״ָ̬ʾ�ƹ� 
#define IR_Switch_ON GPIO_SetBits(GPIOB,GPIO_Pin_10)          //�򿪽�����LED
#define IR_Switch_OFF GPIO_ResetBits(GPIOB,GPIO_Pin_10)       //�رս�����LED
#define Cover_Check GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_11)  //��⴫������ĸ����Ƿ�رգ�0Ϊ�رգ�1Ϊ����
#define REF_V 5000             //�ο���ѹ��mV
#define BV_SAMPLE_START 10     //���ӹر��ҽ�����LED�ƴ򿪶������ʼ�ɼ�Ѫ�������ݣ�s
#define BV_SAMPLE_PERIOD 1000  //Ѫ�������ݲɼ����ڣ�ms
#define BV_SAMPLE_SIZE 60      //Ѫ�����˲�����������ÿBV_SAMPLE_PERIOD����ɼ�1��
#define RBV_MAX_CHANGE 0.006   //����λʱ�������Ѫ�������仯����0.006��ʾ0.6%
typedef struct{
  u8 Correction_Abnormal;  //У���쳣��1��ʾ�쳣
  float BV0;               //У��BV0��ֵ
  float BV1;               //У��BV1��ֵ
  float BV0_V;             //У��BV0ʱ�ܹ��ѹ��mV
  float BV1_V;             //У��BV1ʱ�ܹ��ѹ��mV
  float k;                 //У�����б��
  float b;                 //У����Ľؾ�
}BV_Parameter;
typedef struct{
  float AD;                //�ܹ��ѹ��ADֵ
  float V;                 //�ܹ��ѹ��mV
  float BV;                //Ѫ����
  float BV0;               //��ʼѪ����
  float RBV;               //��BV0Ϊ��������Ѫ������90.12��ʾ90.12%
  u8 Initialized;          //0/1-δ/���趨��ʼѪ����
  u8 Mode;                 //0/1-ģ��/���ؿ��ƽ�����LED
  u8 IR_LED;               //0/1-������LED�ر�/��
  u8 Abnormal;             //0/1-Ѫ��������������/�쳣
  u8 Blood;                //0/1-δ��⵽/��⵽ѪҺ
  u8 Cover;                //0/1-���ǹر�/��
}BV_Data;

extern BV_Parameter BV_PARM;
extern BV_Data BV;

void BV_Process(void);
void Check_Cover(void);
float BV_Filtering(float data);

#endif