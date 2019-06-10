/*******************************************************************************
*                                 SWSetal
*                       ----------------------------
*                       innovating embedded platform
*
* Copyright (c) 2001-2018 Guangzhou ZHIYUAN Electronics Co., Ltd.
* All rights reserved.
*
* Contact information:
* web site:    http://www.zlg.cn/
*******************************************************************************/

/**
 * \file
 * \brief CS1237软件包实现
 *
 * \internal
 * \par Modification history
 * - 1.01 18-04-12  zcb, del sws_adc_val_t
 * - 1.00 17-09-15  lqy, first implementation.
 * \endinternal
 */

#include "stdlib.h"
#include "sws_adc.h"
#include "sws_ad7799.h"

/**
 * \brief CS1237读指令和写指令宏定义
 */
#define __SWS_CS1237_WRITE_CMD             (0x65 << 1)      /**< \brief 写指令 */
#define __SWS_CS1237_READ_CMD              (0x56 << 1)      /**< \brief 读指令 */

/*****************************************************************************
 * 静态声明
 ****************************************************************************/

// /** \brief CS1237 ADC读函数 */
//static int __sws_ad7799_dat_read(sws_ad7799_dev_t *p_dev, uint32_t *val);

 /** \brief CS1237开启中断 */
 static void __sws_ad7799_int_open(sws_ad7799_dev_t  *p_dev);

 /** \brief CS1237关闭中断 */
 static void __sws_ad7799_int_close(sws_ad7799_dev_t  *p_dev);

 /** \brief CS1237 adc时序 */
  void __sws_ad7799_ad_clk (sws_ad7799_dev_t  *p_dev);

 /** \brief CS1237 ADC中断处理 */
 static void __sws_ad7799_trigger_handle(void *p_arg);

 /** \brief ADC 配置寄存器写函数 */
 static int __sws_ad7799_config_reg_write (sws_ad7799_dev_t  *p_dev,
                                          uint8_t           config_reg);

 /** \brief 启动ADC转换 */
 static int __pfn_adc_start (void                   *p_drv,
                            int                     chan,
                            sws_adc_buf_desc_t      *p_desc,
                            uint32_t                desc_num,
                            uint32_t                count,
                            uint32_t                flags,
                            sws_adc_seq_cb_t         pfn_callback,
                            void                   *p_arg);

 /** \brief 停止转换 */
 static int __pfn_adc_stop (void *p_drv, int chan);

 /** \brief 获取ADC的采样率    */
 static int __pfn_adc_rate_get (void       *p_drv,
                               int         chan,
                               uint32_t   *p_rate);

 /** \brief 设置ADC的采样率，实际采样率可能存在差异 */
 static int __pfn_adc_rate_set (void     *p_drv,
                               int       chan,
                               uint32_t  rate);

 /** \brief 获取ADC转换精度 */
 static uint32_t __pfn_bits_get (void *p_drv, int chan);

 /** \brief 获取ADC参考电压 */
 static uint32_t __pfn_vref_get (void *p_drv, int chan);

 /**
  * \brief ADC服务函数
  */
 static const struct sws_adc_drv_funcs __g_adc_drvfuncs = {
     __pfn_adc_start,
     __pfn_adc_stop,
     __pfn_adc_rate_get,
     __pfn_adc_rate_set,
     __pfn_bits_get,
     __pfn_vref_get
 };

/*******************************************************************************
* Function Name  : Delay_7799
* Description    : 延时函数
* Input          : timecount - 延时参数(0~65535)
* Output         : None
* Return         : None
*******************************************************************************/
static void Delay_7799(u16 timecount)
{
  while(timecount>0)
    timecount--;
}
 
/**
 * \brief CS1237 SPI 时钟时序
 */
 void __sws_ad7799_ad_clk (sws_ad7799_dev_t  *p_dev)
{
//    int key;
//
//    if (p_dev == NULL) {
//        return ;
//    }
//
//    key = sws_int_cpu_lock();
//
//    sws_gpio_set(p_dev->p_devinfo->clk,1);
//
//    if(p_dev->p_devinfo->high_clk_delay){
//        sws_udelay(p_dev->p_devinfo->high_clk_delay);
//    }
//
//    sws_gpio_set(p_dev->p_devinfo->clk,0);
//
//    sws_int_cpu_unlock(key);
//
//    if(p_dev->p_devinfo->low_clk_delay){
//        sws_udelay(p_dev->p_devinfo->low_clk_delay);
//    }
}


/** \brief 启动ADC转换 */
static int __pfn_adc_start (void                   *p_drv,
                           int                     chan,
                           sws_adc_buf_desc_t      *p_desc,
                           uint32_t                desc_num,
                           uint32_t                count,
                           uint32_t                flags,
                           sws_adc_seq_cb_t         pfn_callback,
                           void                   *p_arg)
{
//    sws_ad7799_dev_t *p_dev;
//    
//    if (NULL == p_drv) {
//        return -SWS_EINVAL;
//    }
//
//    p_dev  = (sws_ad7799_dev_t *)p_drv;
//
//    p_dev->ch           = chan;
//    p_dev->p_desc       = p_desc;
//    p_dev->desc_num     = desc_num;
//    p_dev->count        = count;
//    p_dev->flags        = flags;
//    p_dev->pfn_callback = pfn_callback;
//    p_dev->p_arg        = p_arg;
//
//    sws_ad7799_ch_set(p_dev,p_dev->ch);
//    /* 开启外部中断 */
//    sws_ad7799_read_int_enable(p_dev);
//
//    return SWS_OK;
}

/** \brief 停止转换 */
static int __pfn_adc_stop (void *p_drv, int chan)
{
//    sws_ad7799_dev_t *p_dev;
//    
//    if (NULL == p_drv) {
//        return -SWS_EINVAL;
//    }
//
//    p_dev = (sws_ad7799_dev_t *)p_drv;
//
//    sws_ad7799_read_int_disable(p_dev);
//
//    return SWS_OK;
}

/** \brief 获取ADC的采样率    */
static int __pfn_adc_rate_get (void       *p_drv,
                              int         chan,
                              uint32_t   *p_rate)
{
//    sws_ad7799_dev_t *p_dev;
//    
//    if (NULL == p_drv) {
//        return -SWS_EINVAL;
//    }
//
//    p_dev = (sws_ad7799_dev_t *)p_drv;
//    
//    *p_rate = sws_ad7799_out_speed_get(p_dev);
//
//    return SWS_OK;
}

/** \brief 设置ADC的采样率，实际采样率可能存在差异 */
static int __pfn_adc_rate_set (void     *p_drv,
                              int       chan,
                              uint32_t  rate)
{
//    sws_ad7799_dev_t *p_dev;
//    
//    if (NULL == p_drv) {
//        return -SWS_EINVAL;
//    }
//
//    p_dev = (sws_ad7799_dev_t *)p_drv;
//
//    /* 10 40 640 1280 */
//    if ( rate <= 25 ) {
//        rate = SWS_CS1237_SPEED_10HZ;
//    } else if ( rate <= 340 ) {
//        rate = SWS_CS1237_SPEED_40HZ;
//    } else if ( rate <= 960){
//        rate = SWS_CS1237_SPEED_640HZ;
//    } else {
//        rate = SWS_CS1237_SPEED_1280HZ;
//    }
//    sws_ad7799_out_speed_set(p_dev,rate);
//
//    return SWS_OK;
}

/** \brief 获取ADC转换精度 */
static uint32_t __pfn_bits_get (void *p_drv, int chan)
{

//    if (NULL == p_drv) {
//        return (uint32_t)(-SWS_EINVAL);
//    }
//
//    return 24;
}

/** \brief 获取ADC参考电压 */
static uint32_t __pfn_vref_get (void *p_drv, int chan)
{
//    sws_ad7799_dev_t *p_dev;
//    
//    if (NULL == p_drv) {
//            return (uint32_t)(-SWS_EINVAL);
//        }
//
//    p_dev = (sws_ad7799_dev_t *)p_drv;
//
//    return p_dev->p_devinfo->vref;
}

/**
 * \brief ADC 配置寄存器读函数
 */
uint8_t sws_ad7799_config_reg_read(sws_ad7799_dev_t  *p_dev)
{


}

/*******************************************************************************
* Function Name  : Set_CS_7799
* Description    : 设置AD7799的CS为高电平(1)或低电平(0)
* Input          : chipn - AD7799芯片编号：0～(AD7799_PCS-1)，最大3
*                    val - CS的电平值: 0=低电平; 1=高电平
* Output         : None
* Return         : None
*******************************************************************************/
static void __set_CS_7799(sws_ad7799_dev_t  *p_dev,u8 val)  
{ 
  if(val == 0)
  {
    GPIO_ResetBits(p_dev->p_devinfo->cs->POR,p_dev->p_devinfo->cs->PIN); //AD7799_CS_L; 
  }
  else
  {
    GPIO_SetBits(p_dev->p_devinfo->cs->POR,p_dev->p_devinfo->cs->PIN);   //AD7799_CS_H;  
  }
}

/*******************************************************************************
* Function Name  : Wr1Byte7799
* Description    : 向AD7799写入1字节(MSB在前,即左移)上沿锁存
* Input          : chipn - AD7799芯片编号：0～(AD7799_PCS-1)，最大3
*                  data - 写入的8bit数据
* Output         : None
* Return         : None
*******************************************************************************/
static void __Wr1Byte7799(sws_ad7799_dev_t *p_dev, u8 data)  //模拟SPI
{ 
  u8 xi;
  for(xi = 0; xi < 8; xi++)
  {  GPIO_ResetBits(p_dev->p_devinfo->clk->POR,p_dev->p_devinfo->clk->PIN);     //AD7799_CLK_L;
  if((data & 0x80) == 0x80 ) 
  {
    GPIO_SetBits(p_dev->p_devinfo->in_pin->POR,p_dev->p_devinfo->in_pin->PIN);   //AD7799_DIN_H;  
  }  
  else 
  {
    GPIO_ResetBits(p_dev->p_devinfo->in_pin->POR,p_dev->p_devinfo->in_pin->PIN); //AD7799_DIN_L;  
  }  
  Delay_7799(p_dev->p_devinfo->low_clk_delay);
  data = data << 1;  //左移1位
  GPIO_SetBits(p_dev->p_devinfo->clk->POR,p_dev->p_devinfo->clk->PIN);    //AD7799_CLK_H;   
  Delay_7799(p_dev->p_devinfo->high_clk_delay);   
  }
}

/**
 * \brief 写寄存器
 */
static void __write_reg7799(sws_ad7799_dev_t *p_dev,u8 xcomm, u8 xlen, u8 *s)  //模拟SPI
{
  u8 xi;
  
  __set_CS_7799(p_dev, 0);      //AD7799_CS_L;  
  __Wr1Byte7799(p_dev,xcomm & 0xbf);	//bit6置为0 (0=写)
  for(xi = 0; xi < xlen; xi++)
  __Wr1Byte7799(p_dev,s[xi]);
  __set_CS_7799(p_dev, 1);      //AD7799_CS_H;  
}

/**
 * \brief 初始化函数
 */
sws_adc_handle_t sws_ad7799_init(sws_ad7799_dev_t                    *p_dev,
                                 const sws_ad7799_dev_info_t         *p_devinfo)
{
    u16 mode,config;
    u8  p_buf[2] = {0};
    
    /* 验证参数有效性 */
    if ((p_dev == NULL) || (p_devinfo == NULL)) {
        return NULL;
    }

    p_dev->ch  = SWS_AD7799_CHANNEL_0;
    p_dev->pga = SWS_AD7799_CONFIG_GAIN_1;
    p_dev->out_speed = SWS_AD7799_MODE_470HZ;

    p_dev->adc_serve.p_funcs = &__g_adc_drvfuncs;
    p_dev->adc_serve.p_drv   = p_dev;
    p_dev->pfn_callback      = NULL;
    p_dev->p_desc            = NULL;
    p_dev->p_arg             = NULL;
    p_dev->desc_num          = 0;
    p_dev->flags             = 0;
    p_dev->count             = 0;
    p_dev->seq_cnt           = 0;
    p_dev->desc_index        = 0;
    p_dev->conv_cnt          = 0;
    p_dev->p_devinfo         = p_devinfo;
//    p_dev->is_int            = 0;

    mode   = 0;
    config = 0;
    /*初始化config寄存器*/
    mode = mode                     | 
           SWS_AD7799_MODE_CONTINUE | 
           SWS_AD7799_MODE_470HZ;
    p_buf[0] = mode >> 8;   //H8
    p_buf[1] = mode;        //L8 
    __write_reg7799(p_dev,0x08,2,p_buf);	//写MODE寄存器: 
    
    config = config                    | 
             SWS_AD7799_CONFIG_BIPOLAR | 
             SWS_AD7799_CONFIG_BUF_EN  |
             SWS_AD7799_CHANNEL_0      | 
             SWS_AD7799_CONFIG_GAIN_1;
    
    //写config寄存器
    p_buf[0] = config >> 8;   //H8
    p_buf[1] = config;        //L8 
    __write_reg7799(p_dev,0x10,2,p_buf);	//写config寄存器
   
    return &p_dev->adc_serve;
}

/**
 * \brief 解初始化函数
 */
void sws_ad7799_deinit (sws_ad7799_adc_handle_t handle)
{
    handle->adc_serve.p_funcs = NULL;
    handle->adc_serve.p_drv   = NULL;
    handle->pfn_callback      = NULL;
    handle->p_desc            = NULL;
    handle->p_arg             = NULL;
    handle->p_devinfo = NULL;

    return ;
}

/**
 * \brief CS1237 配置寄存器pga写
 */
int sws_ad7799_pga_set(sws_ad7799_dev_t  *p_dev, uint32_t pga)
{
//    uint8_t config_reg = 0;
//
//    config_reg |= p_dev->ch;
//    config_reg |= pga << 2;
//    config_reg |= p_dev->out_speed << 4;
//    config_reg |= p_dev->p_devinfo->refo_off <<6;
//
//    if ( SWS_OK != __sws_ad7799_config_reg_write(p_dev, config_reg) ) {
//        return SWS_ERROR;
//    }
//    p_dev->pga = pga;
//
//    return SWS_OK;
}

/**
 * \brief CS1237 pga放大倍数读
 */
uint32_t sws_ad7799_pga_get(sws_ad7799_dev_t  *p_dev)
{
//    switch(p_dev->pga){
//
//    case SWS_CS1237_PGA_1:
//         return 1;
//
//    case SWS_CS1237_PGA_2:
//        return 2;
//
//    case SWS_CS1237_PGA_64:
//        return 64;
//
//    default :
//        return 128;
//    }
}

/**
 * \brief CS1237 配置寄存器ch写
 */
int sws_ad7799_ch_set(sws_ad7799_dev_t  *p_dev, uint32_t ch)
{
//    uint8_t config_reg = 0;
//
//    config_reg |= ch;
//    config_reg |= p_dev->pga << 2;
//    config_reg |= p_dev->out_speed << 4;
//    config_reg |= p_dev->p_devinfo->refo_off << 6;
//
//    if ( SWS_OK != __sws_ad7799_config_reg_write(p_dev, config_reg) ) {
//        return SWS_ERROR;
//    }
//
//    p_dev->ch = ch;
//
//    return SWS_OK;
}


/**
 * \brief CS1237 配置寄存器ch读
 */
uint8_t sws_ad7799_ch_get(sws_ad7799_dev_t  *p_dev)
{
//    uint8_t ch_reg = 0;
//
//    ch_reg = p_dev->ch & 0x03;

//    return ch_reg;
}
/**
 * \brief CS1237 配置寄存器speed写
 */
int sws_ad7799_out_speed_set(sws_ad7799_dev_t  *p_dev, uint32_t speed)
{
//    uint8_t config_reg = 0;
//
//
//    config_reg |= p_dev->ch;
//    config_reg |= p_dev->pga << 2;
//    config_reg |= speed << 4;
//    config_reg |= p_dev->p_devinfo->refo_off << 6;
//
//    if ( SWS_OK != __sws_ad7799_config_reg_write(p_dev, config_reg) ) {
//        return SWS_ERROR;
//    }
//
//    p_dev->out_speed = speed;
//
//    return SWS_OK;
}

/**
 * \brief CS1237 adc输出速率读
 */
uint32_t sws_ad7799_out_speed_get(sws_ad7799_dev_t  *p_dev)
{
//    switch(p_dev->out_speed){
//
//    case SWS_CS1237_SPEED_10HZ:
//        return 10;
//
//    case SWS_CS1237_SPEED_40HZ:
//        return 40;
//
//    case SWS_CS1237_SPEED_640HZ:
//        return 640;
//
//    default:
//        return 1280;
//    }
}

/**
 * \brief CS1237 允许读
 */
int sws_ad7799_read_int_enable(sws_ad7799_dev_t  *p_dev)
{
//    __sws_ad7799_int_open(p_dev);
//
//    return SWS_OK;
}

/**
 * \brief CS1237 禁止读
 *
 * \parsws[in] p_dev   : CS1237操作句柄
 *
 * \retval  SWS_OK     : 操作成功
 */
int sws_ad7799_read_int_disable(sws_ad7799_dev_t  *p_dev)
{
//    __sws_ad7799_int_close(p_dev);
//
//    return SWS_OK;
}


/**
 * \brief CS1237 轮循读adc采集值
 */
int sws_ad7799_read_polling(sws_ad7799_dev_t *p_dev, uint32_t *val)
{
//    return __sws_ad7799_dat_read(p_dev, val);
}

///**
// * \brief CS1237 低功耗模式
// *
// * \parsws[in] p_dev   : CS1237操作句柄
// *
// * \retval  无
// */
//void sws_ad7799_power_down_enter(sws_ad7799_dev_t  *p_dev)
//{
//    sws_gpio_set(p_dev->p_devinfo->clk,1);
//
//    if(p_dev->p_devinfo->powerdown_delay){
//        sws_udelay(p_dev->p_devinfo->powerdown_delay);
//    }
//
//    return ;
//}
//
///**
// * \brief CS1237 低功耗模式
// *
// * \parsws[in] p_dev   : CS1237操作句柄
// *
// * \retval  无
// */
//void sws_ad7799_power_down_out(sws_ad7799_dev_t  *p_dev)
//{
//    sws_gpio_set(p_dev->p_devinfo->clk,0);
//
//    return ;
//}
//
///**
// * \brief CS1237 连接中断连接函数
// */
//void sws_ad7799_int_connect(sws_ad7799_dev_t      *p_dev ,
//                           sws_ad7799_code_read_cb_t  p_fun,
//                           void                     *p_arg)
//{
//    p_dev->triginfo.pfn_callback = p_fun;
//    p_dev->triginfo.p_arg = p_arg;
//}
//
///**
// * \brief CS1237删除中断连接函数
// */
//void sws_ad7799_int_disconnect(sws_ad7799_dev_t      *p_dev ,
//                              sws_ad7799_code_read_cb_t  p_fun,
//                              void                     *p_arg)
//{
//    p_dev->triginfo.pfn_callback = NULL;
//    p_dev->triginfo.p_arg = NULL;
//}
//
//sws_adc_handle_t sws_ad7799_standard_adc_handle_get(sws_ad7799_dev_t *p_dev)
//{
//    return (sws_adc_handle_t)(&(p_dev->adc_serve));
//}
/* end of file */
