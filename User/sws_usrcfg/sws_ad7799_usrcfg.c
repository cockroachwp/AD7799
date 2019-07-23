
/**
 * \file
 * \brief  AD7799�û������ļ�
 * \sa sws_ad7799_usrcfg.c
 *
 * \internal
 * \par Modification history
 * - 1.00 19-06-10  wangpeng, first implementation.
 * \endinternal
 */
#include "stm32f10x.h"
#include "sws_ad7799.h"

/**
 * \copydoc sws_ad7799_usrcfg.c
 * @{
 */

static  sws_gpio_info_t  ad7799_clk = {
      GPIOB,
      GPIO_Pin_13
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
      GPIOA,
      GPIO_Pin_8
};

/** \brief ADCƽ̨��ʼ�� */
static void __plfm_ad7799_init (void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);
    GPIO_InitStructure.GPIO_Pin = ad7799_clk.PIN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;    //  �������
    GPIO_Init(ad7799_clk.POR, &GPIO_InitStructure);
    GPIO_SetBits(ad7799_clk.POR,ad7799_clk.PIN);

    GPIO_InitStructure.GPIO_Pin = ad7799_out.PIN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;    //  �������
    GPIO_Init(ad7799_out.POR, &GPIO_InitStructure);
    GPIO_SetBits(ad7799_out.POR,ad7799_out.PIN);

    GPIO_InitStructure.GPIO_Pin = ad7799_in.PIN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;    //  �������
    GPIO_Init(ad7799_in.POR, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = ad7799_cs.PIN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;       //  ��������
    GPIO_Init(ad7799_cs.POR, &GPIO_InitStructure);
    GPIO_SetBits(ad7799_cs.POR,ad7799_cs.PIN);
}

/** \brief ���ADCƽ̨��ʼ�� */
static void __plfm_ad7799_deinit (void)
{

}




/** \brief ADC�豸��Ϣ */
static const sws_ad7799_dev_info_t __g_ad7799_devinfo = {

    /** \brief IOģ��SPI SCLK����ѡ�� */
     &ad7799_clk,

    /**
     * \brief ad7799_out����ѡ��
     * */
     &ad7799_out,

    /**
     * \brief ad7799_in����ѡ��
     * */
     &ad7799_in,

    /**
     * \brief ad7799_cs����ѡ��
     *
     * \note ���Ӳ��û�ж�DRDY/DOUT���ŷ��룬������Ӧ�ú�out_pin��ͬ
     * */
     &ad7799_cs,

     AD7799_MODE_CONTINUOUS,

     SWS_AD7799_CONFIG_BIPOLAR,

    /**
     * \brief ADC�ο���ѹ����λ��mV
     *
     * \note �òο���ѹ�ɾ���ĵ�·����
     *
     */
    3780,

    /** \brief CLKʱ��ߵ�ƽ��ʱʱ�� ��΢���*/
    20,

    /** \brief CLKʱ��͵�ƽ��ʱʱ�� ��΢���*/
    20,

    /** \brief ����͹���ģʽclk�ߵ�ƽ����ʱ�� ��΢�������ֵ100us*/
    20,



    __plfm_ad7799_init,              /**< \brief ADC��ƽ̨��ʼ�� */
    __plfm_ad7799_deinit             /**< \brief ADC��ƽ̨ȥ��ʼ�� */
};

static sws_ad7799_dev_t  __g_ad7799_dev;   /**< \brief ����ADC �豸 */

/** \brief ADCʵ����ʼ�������ADC��׼������ */
sws_adc_handle_t sws_ad7799_inst_init (void)
{
    return sws_ad7799_init(&__g_ad7799_dev, &__g_ad7799_devinfo);
}

/** \brief ADCʵ�����ʼ�� */
void sws_ad7799_inst_deinit (sws_adc_handle_t handle)
{
  //  sws_adc_deinit(handle);
}

/**
 * @}
 */

/* end of file */
