
/**
 * \file
 * \brief ZLG116 ADC 用户配置文件
 * \sa am_hwconf_zlg116_adc.c
 *
 * \internal
 * \par Modification history
 * - 1.00 19-06-10  wangpeng, first implementation.
 * \endinternal
 */
#include "am_zlg116.h"
#include "am_gpio.h"
#include "am_zlg_adc.h"
#include "hw/amhw_zlg_adc.h"
#include "hw/amhw_zlg116_rcc.h"
#include "am_clk.h"

/**
 * \addtogroup am_if_src_hwconf_zlg116_adc
 * \copydoc am_hwconf_zlg116_adc.c
 * @{
 */

/** \brief ADC平台初始化 */
static void __zlg_plfm_adc_init (void)
{
    am_gpio_pin_cfg(PIOA_1, PIOA_1_ADC_IN1 | PIOA_1_AIN);
    am_gpio_pin_cfg(PIOB_0, PIOB_0_ADC_IN8 | PIOB_0_AIN);
    am_clk_enable(CLK_ADC1);
}

/** \brief 解除ADC平台初始化 */
static void __zlg_plfm_adc_deinit (void)
{
    am_gpio_pin_cfg(PIOA_1, PIOA_1_INPUT_FLOAT);
    am_gpio_pin_cfg(PIOB_0, PIOB_0_INPUT_FLOAT);
    am_clk_disable (CLK_ADC1);
}

sws_gpio_info_t 



/** \brief ADC设备信息 */
static const sws_ad7799_adc_devinfo_t __g_ad7799_devinfo = {
  
    /** \brief IO模拟SPI SCLK引脚选择 */
    {
      GPIOB,
      GPIO_Pin_12
    }
 
    /**
     * \brief ad7799_out引脚选择
     * */
    {
      GPIOB,
      GPIO_Pin_14
    }

    /**
     * \brief ad7799_in引脚选择
     * */
    {
      GPIOB,
      GPIO_Pin_15
    }
    
    /**
     * \brief ad7799_cs引脚选择
     *
     * \note 如果硬件没有对DRDY/DOUT引脚分离，该引脚应该和out_pin相同
     * */
    {
      GPIOB,
      GPIO_Pin_13
    }

    /**
     * \brief ADC参考电压，单位：mV
     *
     * \note 该参考电压由具体的电路决定
     *
     */
    uint32_t     vref;

    /** \brief CLK时序高电平延时时间 ，微妙级别*/
    uint8_t      high_clk_delay;

    /** \brief CLK时序低电平延时时间 ，微妙级别*/
    uint8_t      low_clk_delay;

    /** \brief 进入低功耗模式clk高电平持续时间 ，微妙级别，理论值100us*/
    uint8_t      powerdown_delay;

    __zlg_plfm_adc_init,              /**< \brief ADC的平台初始化 */
    __zlg_plfm_adc_deinit,            /**< \brief ADC的平台去初始化 */

};

static am_zlg_adc_dev_t  __g_adc_dev;   /**< \brief 定义ADC 设备 */

/** \brief ADC实例初始化，获得ADC标准服务句柄 */
am_adc_handle_t am_zlg116_adc_inst_init (void)
{
    return am_zlg_adc_init(&__g_adc_dev, &__g_adc_devinfo);
}

/** \brief ADC实例解初始化 */
void am_zlg116_adc_inst_deinit (am_adc_handle_t handle)
{
    am_zlg_adc_deinit(handle);
}

/**
 * @}
 */

/* end of file */
