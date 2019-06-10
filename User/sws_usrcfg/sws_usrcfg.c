
/**
 * \file
 * \brief ZLG116 ADC 用户配置文件
 * \sa sws_hwconf_zlg116_adc.c
 *
 * \internal
 * \par Modification history
 * - 1.00 19-06-10  wangpeng, first implementation.
 * \endinternal
 */
#include "stm32f10x.h"
#include "sws_ad7799.h"

/**
 * \addtogroup sws_if_src_hwconf_zlg116_adc
 * \copydoc sws_hwconf_zlg116_adc.c
 * @{
 */

static  sws_gpio_info_t  ad7799_clk = {
      GPIOB,
      GPIO_Pin_12
};

static  sws_gpio_info_t  ad7799_out = {
      GPIOB,
      GPIO_Pin_14
};

static  sws_gpio_info_t  ad7799_in = {
      GPIOB,
      GPIO_Pin_15
};

static  sws_gpio_info_t  ad7799_cs = {
      GPIOB,
      GPIO_Pin_13
};

/** \brief ADC平台初始化 */
static void __plfm_ad7799_init (void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    
    GPIO_InitStructure.GPIO_Pin = ad7799_clk.PIN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;    //  推挽输出
    GPIO_Init(ad7799_clk.POR, &GPIO_InitStructure);  
  
    GPIO_InitStructure.GPIO_Pin = ad7799_out.PIN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;    //  推挽输出
    GPIO_Init(ad7799_out.POR, &GPIO_InitStructure);  
  
    GPIO_InitStructure.GPIO_Pin = ad7799_in.PIN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;    //  推挽输出
    GPIO_Init(ad7799_in.POR, &GPIO_InitStructure);  
  
    GPIO_InitStructure.GPIO_Pin = ad7799_in.PIN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;       //  上拉输入
    GPIO_Init(ad7799_in.POR, &GPIO_InitStructure);   
}

/** \brief 解除ADC平台初始化 */
static void __plfm_ad7799_deinit (void)
{

}




/** \brief ADC设备信息 */
static const sws_ad7799_dev_info_t __g_ad7799_devinfo = {
  
    /** \brief IO模拟SPI SCLK引脚选择 */
     &ad7799_clk,
 
    /**
     * \brief ad7799_out引脚选择
     * */
     &ad7799_out,

    /**
     * \brief ad7799_in引脚选择
     * */
     &ad7799_in,
    
    /**
     * \brief ad7799_cs引脚选择
     *
     * \note 如果硬件没有对DRDY/DOUT引脚分离，该引脚应该和out_pin相同
     * */
     &ad7799_cs,

    /**
     * \brief ADC参考电压，单位：mV
     *
     * \note 该参考电压由具体的电路决定
     *
     */
    3000,

    /** \brief CLK时序高电平延时时间 ，微妙级别*/
    20,

    /** \brief CLK时序低电平延时时间 ，微妙级别*/
    20,

    /** \brief 进入低功耗模式clk高电平持续时间 ，微妙级别，理论值100us*/
    20,

    __plfm_ad7799_init,              /**< \brief ADC的平台初始化 */
    __plfm_ad7799_deinit             /**< \brief ADC的平台去初始化 */
};

static sws_ad7799_dev_t  __g_ad7799_dev;   /**< \brief 定义ADC 设备 */

/** \brief ADC实例初始化，获得ADC标准服务句柄 */
sws_adc_handle_t sws_ad7799_inst_init (void)
{
    return sws_ad7799_init(&__g_ad7799_dev, &__g_ad7799_devinfo);
}

/** \brief ADC实例解初始化 */
void sws_ad7799_inst_deinit (sws_adc_handle_t handle)
{
  //  sws_adc_deinit(handle);
}

/**
 * @}
 */

/* end of file */
