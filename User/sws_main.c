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
#include "flash.h"
#include "AD7799.h"
#include "stm32f10x_tim.h"
#include "string.h"
#include "sws_adc.h"
#include "math.h"
#include "swsbus.h"
#include "delay.h"
#include "math.h"
#include "string.h"
 //#include "sws_rs485.h"



#define higt        0;
#define light       1;

#define PRESSURE    0
#define FLOW        1
#define CLOSE       2

typedef struct pressure_flow_cmd {
    char cmd;
    int  min_pressure;
    int  max_pressure;
    int  flow;
}pressure_flow_cmd_t;


//typedef struct flow_cmd {
//    char cmd[10];
//    int  min_pressure;
//}


/*******************************************************************************
* Function Name  : main
* Description    : Main program
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
uint8_t p_flag=0;
extern sws_adc_handle_t sws_ad7705_inst_init (void);

extern  u8 recive_status ;

extern int sws_adc_read (sws_adc_handle_t  handle,
                          int              chan,
                          void            *p_val,
                          uint32_t         length);



/*******************************************************************************
* Function Name  : Systick_Configuration
* Description    : None
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void SysTick_Configuration(void)
{
  /* Setup SysTick Timer for 100ms interrupts  */
  if (SysTick_Config(72000))
  {
    /* Capture error */
    while (1);
  }
}

uint32_t   SysTickTime = 0;

///**
//* @brief  This function handles SysTick Handler.
//* @param  None
//* @retval None
//*/
//
//void SysTick_Handler(void)
//{
//
//  SysTickTime++;
//
//}


 int main(void)
{
                                        /* 采样电压 */
                                                            /* 采样电压 */
 // uint32_t float_int = 0;
 // char     buf[30] = 0;

    static int  a = 0;

    float T;
    float value_to_write;
    const float pi =3.1416;
    float delta_t=0.01;
    float t;
    int value_to_write_bit;
    //byte buffer_serial[26];
    unsigned long currentMillis;
    unsigned long previousMillis;
    unsigned long interval;
    unsigned long currentMillis_2;
    unsigned long previousMillis_2;
    unsigned long interval_2;

    float baseline;
    float A_heart;
    float f_heart;
    float A_breathing;
    float f_breathing;
    float A1_heart;
    float P1_heart;
    float A2_heart;
    float P2_heart;
    float A3_heart;
    float P3_heart;
    float resolution_factor;


    unsigned long cet = 0;

    pressure_flow_cmd_t   pressure_flow_cmd = {1, 20, 150, 500};

    u8 status   = light;
    delay_init();
    //uart_gpio_init();
    GearPump_Init();
    RS485_Init();

    sws_adc_handle_t  adc_hand_t = sws_ad7705_inst_init();

    //u16 bb=1000;
    //TIM_SetCompare2(TIM4, 3500);

    sws_out_prompting_message();

    memset(&pressure_flow_cmd, 0,sizeof(pressure_flow_cmd));

    baseline=1;
    A_heart=0;
    f_heart=1;
    A_breathing=0;
    f_breathing=0;

	resolution_factor=0.0022;  // per 10 bit
    t=0.01;
    A1_heart=1;
    P1_heart=0;
    A2_heart=0;
    P2_heart=0;
    A3_heart=0;
    P3_heart=0;
    interval=1000;
    interval_2=10; /// should be = delta_t*1000

    int b = 0;

    float  min      =  1000;
    float  max      =  3500;


    while(1) {

       if ( recive_status == 1) {

         analyses_cmd(RxBuffer, &pressure_flow_cmd);
         memset(RxBuffer, 0, sizeof(RxBuffer));
         RxBufferLength = 0;
         recive_status  = 0;
       }

     // currentMillis = cet;  //cpu当前的运行时间

      //if(currentMillis - previousMillis_2 >= interval_2) {   //10 毫秒进入一次
         previousMillis_2 = currentMillis;

         T=1/f_heart;
         value_to_write = max*sin(2*pi*t*f_heart);  //+
                     // ((A2_heart/resolution_factor)*sin(2*pi*t*2*f_heart+P2_heart)) +
    				 // ((A3_heart/resolution_factor)*sin(2*pi*t*3*f_heart+P3_heart)) +
    				//  ( baseline/resolution_factor);

          if (value_to_write<min) {
               value_to_write=min;
          }

          if (value_to_write > max) {
               value_to_write = max;
          }

          if (t < (T)) {
             t = t+delta_t;
          } else {
             t=0.01;
          }

          value_to_write_bit=(int) value_to_write;



          TIM_SetCompare2(TIM4, value_to_write);
          sprintf(TxBuffer, "%f \r\n", value_to_write);

          RS485_SendData(strlen(TxBuffer));

          delay_ms(5);
          // delay_us(500);
          cet++;

}





  while(1)
  {

      if ( recive_status == 1) {

         analyses_cmd(RxBuffer, &pressure_flow_cmd);
         memset(RxBuffer, 0, sizeof(RxBuffer));
         RxBufferLength = 0;
         recive_status  = 0;
      }

     // memset(TxBuffer, 0,sizeof(TxBuffer));

      if (pressure_flow_cmd.cmd == CLOSE) {
          TIM_SetCompare2(TIM4, 0);

          GPIO_ResetBits(GPIOC,GPIO_Pin_9);
          continue;
      }


     a = get_pre(adc_hand_t);

     //memset(TxBuffer, 0, sizeof(TxBuffer));
     sprintf(TxBuffer, "%d \r\n", a);

     RS485_SendData(strlen(TxBuffer));

     if (a >= pressure_flow_cmd.max_pressure) {

         //TIM_Cmd(TIM4, DISABLE);
         TIM_SetCompare2(TIM4, 0);

         GPIO_ResetBits(GPIOC,GPIO_Pin_9);
         status = higt;
     }

      if (a <= pressure_flow_cmd.min_pressure ) {
          //TIM_Cmd(TIM4, ENABLE);
          TIM_SetCompare2(TIM4, 3500);
          GPIO_SetBits(GPIOC, GPIO_Pin_9);
          status = light
      }

  }





}


int get_pre(sws_adc_handle_t  adc_hand_t)
{
      uint8_t         i        = 0;
      int             adc_p    = 0;
      uint32_t adc_code = 0;                             /* 平均采样值 */
      uint32_t        temp     = 0;
      uint32_t  val_buf[2] = {0};

      sws_adc_read(adc_hand_t, 0, val_buf, 2);

      temp = 0;

      for (i = 0; i < (sizeof(val_buf) / sizeof(uint32_t)); i++) {
         temp += val_buf[i];
      }

      adc_code = temp / (sizeof(val_buf) / sizeof(uint32_t));

      //adc_code = adc_code & (~0x8000) ;

      /* 计算压力值*/
      adc_p = (adc_code * 500)  / 1813 - 8974;

      return adc_p;
}


static int sws_out_prompting_message(void)
{
     memset(TxBuffer, 0, sizeof(TxBuffer));
     sprintf(TxBuffer, "if you want to change different hight Pressure and low Pressure \r\n");
     RS485_SendData(strlen(TxBuffer));

     memset(TxBuffer, 0, sizeof(TxBuffer));
     sprintf(TxBuffer, "please input cmd and  Pressure \r\n");
     RS485_SendData(strlen(TxBuffer));

     memset(TxBuffer, 0, sizeof(TxBuffer));
     sprintf(TxBuffer, "example: Pressure <hight Pressure> <low Pressure> \r\n");
     RS485_SendData(strlen(TxBuffer));

}


static int sws_printf(char *p_data)
{
     int   i           = 0;
     char *p_data_head = p_data;
     while(*p_data_head != 0) {
         i++;
         p_data_head++;
     }
     memset(TxBuffer, 0, sizeof(TxBuffer));
     strcpy(TxBuffer, p_data);
     RS485_SendData(i);
}



static int analyses_cmd(char data[],pressure_flow_cmd_t *p_pressure_flow)
{
    char  cmd[10] = {0};
    char *p_cmd_head = cmd;
    int  temp1 = 0, temp2 = 0;

    u8 i = 0;

    for (; *data != ' ' && *data != '\r';) {
        if (data == NULL) {
           sws_printf("please input true command!\r\n");
           return -1;
        }
        *p_cmd_head++ = *data++;
    }

    p_cmd_head = cmd;

    if (strcmp(cmd, "pressure") == 0) {
         data++;
         memset(cmd, 0, sizeof(cmd));
        for (; *data != ' ';) {
            if (data == NULL) {
                sws_printf("please input true high pressure!\r\n");
                return -1;
            }
            *p_cmd_head++ = *data++;
        }
        temp1 = atoi(cmd);

        if (temp1 < 1 || temp1 > 200 ) {
            sws_printf("please input true high pressure!\r\n");
            return -1;
        }

        p_cmd_head = cmd;
        data++;
        memset(cmd, 0, sizeof(cmd));
        for (; *data != '\r';) {
            if (data == NULL) {
                sws_printf("please input true low pressure!\r\n");
                return -1;
            }
            *p_cmd_head++ = *data++;
        }
        temp2 = atoi(cmd);

        if (temp2 < 1 ||  temp2 > 200) {
            sws_printf("please input true low pressure!\r\n");
            return -1;
        }
        if ( temp1 <= temp2) {
            sws_printf("please input true high pressure more than low pressure!\r\n");
            return -1;
        }
        p_pressure_flow->cmd          = PRESSURE;
        p_pressure_flow->min_pressure =  temp2;
        p_pressure_flow->max_pressure =  temp1;
        sws_printf("pressure is OK\r\n") ;

    } else if (strcmp(cmd, "close") == 0) {
        p_pressure_flow->cmd      = CLOSE;
        sws_printf ("close OK\r\n");
    } else if ( strcmp(cmd, "flow") ==0) {
        sws_printf("set flow Completing in progress\r\n");
    } else if ( strcmp(cmd, "flow")) {
        sws_printf("please input true command!\r\n");
    }





    return 0;
}






