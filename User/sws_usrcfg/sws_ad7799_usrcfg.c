
/**
 * \file
 * \brief ZLG116 ADC �û������ļ�
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

/** \brief ADCƽ̨��ʼ�� */
static void __zlg_plfm_adc_init (void)
{
    am_gpio_pin_cfg(PIOA_1, PIOA_1_ADC_IN1 | PIOA_1_AIN);
    am_gpio_pin_cfg(PIOB_0, PIOB_0_ADC_IN8 | PIOB_0_AIN);
    am_clk_enable(CLK_ADC1);
}

/** \brief ���ADCƽ̨��ʼ�� */
static void __zlg_plfm_adc_deinit (void)
{
    am_gpio_pin_cfg(PIOA_1, PIOA_1_INPUT_FLOAT);
    am_gpio_pin_cfg(PIOB_0, PIOB_0_INPUT_FLOAT);
    am_clk_disable (CLK_ADC1);
}

sws_gpio_info_t 



/** \brief ADC�豸��Ϣ */
static const sws_ad7799_adc_devinfo_t __g_ad7799_devinfo = {
  
    /** \brief IOģ��SPI SCLK����ѡ�� */
    {
      GPIOB,
      GPIO_Pin_12
    }
 
    /**
     * \brief ad7799_out����ѡ��
     * */
    {
      GPIOB,
      GPIO_Pin_14
    }

    /**
     * \brief ad7799_in����ѡ��
     * */
    {
      GPIOB,
      GPIO_Pin_15
    }
    
    /**
     * \brief ad7799_cs����ѡ��
     *
     * \note ���Ӳ��û�ж�DRDY/DOUT���ŷ��룬������Ӧ�ú�out_pin��ͬ
     * */
    {
      GPIOB,
      GPIO_Pin_13
    }

    /**
     * \brief ADC�ο���ѹ����λ��mV
     *
     * \note �òο���ѹ�ɾ���ĵ�·����
     *
     */
    uint32_t     vref;

    /** \brief CLKʱ��ߵ�ƽ��ʱʱ�� ��΢���*/
    uint8_t      high_clk_delay;

    /** \brief CLKʱ��͵�ƽ��ʱʱ�� ��΢���*/
    uint8_t      low_clk_delay;

    /** \brief ����͹���ģʽclk�ߵ�ƽ����ʱ�� ��΢�������ֵ100us*/
    uint8_t      powerdown_delay;

    __zlg_plfm_adc_init,              /**< \brief ADC��ƽ̨��ʼ�� */
    __zlg_plfm_adc_deinit,            /**< \brief ADC��ƽ̨ȥ��ʼ�� */

};

static am_zlg_adc_dev_t  __g_adc_dev;   /**< \brief ����ADC �豸 */

/** \brief ADCʵ����ʼ�������ADC��׼������ */
am_adc_handle_t am_zlg116_adc_inst_init (void)
{
    return am_zlg_adc_init(&__g_adc_dev, &__g_adc_devinfo);
}

/** \brief ADCʵ�����ʼ�� */
void am_zlg116_adc_inst_deinit (am_adc_handle_t handle)
{
    am_zlg_adc_deinit(handle);
}

/**
 * @}
 */

/* end of file */
