#ifndef __BT_H
#define __BT_H

#define AT_LED_OFF   GPIO_SetBits(GPIOB,GPIO_Pin_8)//���������¶�ָʾ��
#define AT_LED_ON  GPIO_ResetBits(GPIOB,GPIO_Pin_8)
#define VT_LED_OFF   GPIO_SetBits(GPIOB,GPIO_Pin_9)//���������¶�ָʾ��
#define VT_LED_ON  GPIO_ResetBits(GPIOB,GPIO_Pin_9)
#define AT_LED_Check GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_8)  //��⴫������ĸ����Ƿ�رգ�0Ϊ�رգ�1Ϊ����
#define VT_LED_Check GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_9)  //��⴫������ĸ����Ƿ�رգ�0Ϊ�رգ�1Ϊ����

#define T_Ea_MAX 5             //�¶�������ޣ���
#define DT 3.074285714                 //�̶�����ֵ�����������ã���
#define T_SAMPLE_SIZE 100      //������������

typedef struct{
  u8 AT_Correction_Abnormal;//����Ѫ��У���쳣��1��ʾ�쳣
  u8 VT_Correction_Abnormal;//����Ѫ��У���쳣��1��ʾ�쳣
  float AT_Ea;//����Ѫ��������ֵ-��ʵֵ����
  float VT_Ea;//����Ѫ��������ֵ-��ʵֵ����
}BT_Parameter;

typedef struct{
  float AT;//����Ѫ�£��������ֵ����
  float VT;//����Ѫ�£��������ֵ����
  float at;//����Ѫ�£�����������ֵ����
  float vt;//����Ѫ�£�����������ֵ����
  u8 AT_Abnormal;//����Ѫ���쳣��1��ʾ�쳣
  u8 VT_Abnormal;//����Ѫ���쳣��1��ʾ�쳣
}BT_Data;

extern BT_Parameter BT_PARM;
extern BT_Data BT;

void BT_Process(void);
float AT_Filtering(float data);
float VT_Filtering(float data);


#endif