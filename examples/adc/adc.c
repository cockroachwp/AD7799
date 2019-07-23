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


static uint32_t val_buf[1] = {0};




int adc_demo( sws_adc_handle_t  adc_hand_t)
{
  float    adc_mv;                                           /* 采样电压 */
  uint32_t adc_code;                                         /* 平均采样值 */
  uint32_t temp    = 0;                                      /* 采样电压 */
  uint8_t  i       = 0;
  char     buf[30] = 0;
  uint8_t  ch      = 0;

  delay_init();
 // uart_gpio_init();
 // sws_adc_handle_t  adc_hand_t = sws_ad7799_inst_init();

  int adc_bits = sws_adc_bits_get(adc_hand_t , 0);        /* 获取ADC转换精度 */
  int adc_vref = sws_adc_vref_get(adc_hand_t , 0);

  while(1)
  {
      sws_adc_read(adc_hand_t, 0, val_buf, 1);

      temp = 0;

      for (i = 0; i < (sizeof(val_buf) / sizeof(uint32_t)); i++) {
         temp += val_buf[i];
      }

      adc_code = temp / (sizeof(val_buf) / sizeof(uint32_t));

      adc_code = adc_code - 0x800000;

      /* 计算电压值*/
      adc_mv  = (float)adc_vref * ((float)adc_code / ((1UL << (23 )) - 1));


      delay_ms(100);

  }

}