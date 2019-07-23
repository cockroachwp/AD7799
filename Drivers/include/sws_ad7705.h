
/**
 * \file
 * \brief  AD7705 Ӧ�ýӿ��ļ�
 *
 * \internal
 * \par Modification History
 * - 1.00 19-6-10  wangpeng, first implementation.
 * \endinternal
 */

#ifndef __SWS_AD7705_H
#define __SWS_AD7705_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @addtogroup sws_if_AD7705
 * @copydoc sws_AD7705.h
 * @{
 */

#include "stm32f10x.h"
#include "sws_adc.h"


  /**
 * \brief AD7705 ����ʹ�õ�����
 */
  typedef struct sws_gpio_info {
      GPIO_TypeDef *POR;
      uint16_t      PIN;
  } sws_gpio_info_t;

/**
 * \brief AD7705 �豸��Ϣ�ṹ��
 */
typedef struct sws_AD7705_dev_info{

    /** \brief IOģ��SPI SCLK����ѡ�� */
    sws_gpio_info_t      *clk;

    /**
     * \brief AD7705_out����ѡ��
     * */
    sws_gpio_info_t      *out_pin;

    /**
     * \brief AD7705_in����ѡ��
     * */
    sws_gpio_info_t      *in_pin;

    /** \brief  Ƭѡλ*/
    sws_gpio_info_t      *cs;

    sws_gpio_info_t      *reset;

    /** \brief  ת��ģʽ*/
    uint8_t               mode;

    /** \brief  ת������*/
    uint8_t               polar;

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

    /** \brief ƽ̨��ʼ�����������ʱ�ӣ��������ŵȹ��� */
    void       (*pfn_plfm_init)(void);

    /** \brief ƽ̨���ʼ������ */
    void       (*pfn_plfm_deinit)(void);

}sws_ad7705_dev_info_t;

/** \brief �����ص����� */
typedef void (*sws_AD7705_code_read_cb_t) (void *p_arg,uint32_t code);

/**
 * \brief AD7705 �豸�ṹ��
 */
typedef struct sws_AD7705_dev{

    /** \brief �Ŵ��� */
    uint8_t                        pga;
    /** \brief ������� */
    uint8_t                        out_speed;
    /** \brief ͨ�� */
    uint16_t                       config;
    /** \brief ͨ�� */
    uint16_t                       mode;
    /** \brief INT���ŵĴ�����Ϣ���� */
    struct sws_AD7705_trigger_info {

        /** \brief �����ص����� */
        sws_AD7705_code_read_cb_t   pfn_callback;
        /** \brief �ص������Ĳ��� */
        void                       *p_arg;

    } triginfo;/**< \brief INT���ŵĴ�����Ϣ */

    /** \brief �豸��Ϣ */
    const sws_ad7705_dev_info_t   *p_devinfo;

    /** \brief ADC��׼���� */
    sws_adc_serv_t                 adc_serve;

    /** \brief ����û�����ת����Ļص����� */
    sws_adc_seq_cb_t               pfn_callback;

    /** \brief �û������ص������Ļص��������� */
    void                          *p_arg;

    /** \brief ��ǰת�������������������׵�ַ */
    sws_adc_buf_desc_t             *p_desc;

    /** \brief ��ǰת������������������ */
    uint32_t                       desc_num;

    /** \brief ת���Ĵ��� */
    uint32_t                       count;

    /** \brief ת����־ */
    uint32_t                       flags;

    /** \brief �Ե�ǰ�����������Ѿ������Ĵ��� */
    uint32_t                       conv_cnt;

    /** \brief ����ִ�е�ǰ���������������� */
    uint32_t                       desc_index;

    /** \brief ����������ת����ɵĴ��� */
    uint32_t                       seq_cnt;

}sws_ad7705_dev_t;

typedef sws_ad7705_dev_t * sws_ad7705_adc_handle_t; /**< \brief ������� */

////ת��ģʽ����λ: bit15,14,13
//#define SWS_AD7705_MODE_CONTINUE  0x0000    //v����ת��ģʽ(Ĭ��)
//#define SWS_AD7705_MODE_SINGLE    0x2000    //����ת��ģʽ

//PSW���ؿ���λ�� bit12
//#define SWS_AD7705_MODE_PSW_OFF   0x0000    //�ر�PSW����(Ĭ��)
//define SWS_AD7705_MODE_PSW_ON    0x1000    //��PSW����K


/************************************** ͨѶ�Ĵ��� **************************************/

/* �Ĵ���ѡ�� */

#define SWS_AD7705_COMMUNI_REG           0 << 4   /* ͨѶ�Ĵ��� */
#define SWS_AD7705_SET_REG               1 << 4   /* ���üĴ��� */
#define SWS_AD7705_CLK_REG               2 << 4   /* ʱ�ӼĴ��� */
#define SWS_AD7705_DATA_REG              3 << 4   /* ���ݼĴ��� */
#define SWS_AD7705_TEST_REG              4 << 4   /* ���ԼĴ��� */
#define SWS_AD7705_DEVI_REG              6 << 4   /* ƫ�ƼĴ��� */
#define SWS_AD7705_PAG_REG               7 << 4   /* ����Ĵ��� */

#define SWS_AD7705_WRITE                 0 << 3   /* ������ */
#define SWS_AD7705_READ                  1 << 3   /* д���� */

#define SWS_AD7705_CHANNEL_1         0      /**< \brief ͨ�� 1 */
#define SWS_AD7705_CHANNEL_2         1      /**< \brief ͨ�� 2 */


/************************************** ���üĴ��� **************************************/


#define SWS_AD7705_MODE_NAR			       0 << 6		/* ����ת��ģʽ */
#define SWS_AD7705_MODE_CALIB_SELF	       1 << 6		/* ��У׼ģʽ */
#define SWS_AD7705_MODE_CALIB_ZERO	       2 << 6		/* ����У׼ģʽ */
#define SWS_AD7705_MODE_CALIB_FULL		   3 << 6		/* ����У׼ģʽ */

/**
 * \nswse AD7705 ����ѡ��
 * @{
 */

#define SWS_AD7705_CONFIG_GAIN_1      0 << 3   /*����=1(�Ǳ�Ŵ�������) */
#define SWS_AD7705_CONFIG_GAIN_2      1 << 3   /*����=2(�Ǳ�Ŵ�������) */
#define SWS_AD7705_CONFIG_GAIN_4      2 << 3   /*����=4 */
#define SWS_AD7705_CONFIG_GAIN_8      3 << 3   /*����=8 */
#define SWS_AD7705_CONFIG_GAIN_16     4 << 3   /*����=16 */
#define SWS_AD7705_CONFIG_GAIN_32     5 << 3   /*����=32 */
#define SWS_AD7705_CONFIG_GAIN_64     6 << 3   /*����=64 */
#define SWS_AD7705_CONFIG_GAIN_128    7 << 3   /*����=128 */


//��������źŵĵ�/˫���Ա������: bit12
#define SWS_AD7705_CONFIG_BIPOLAR     0 << 2     /*˫���Ա���(Ĭ��) */
#define SWS_AD7705_CONFIG_UNIPOLAR    1 << 2     /*v�����Ա��� */


/************************************** ʱ�ӼĴ��� **************************************/


/**
 * \nswse AD7705 �������ѡ��
 * @{
 */

#define SWS_AD7705_CLK_IN       0 << 4  /* �ⲿʱ�� */
#define SWS_AD7705_CLK_OUT      1 << 4  /* �ڲ�ʱ�� */


#define SWS_AD7705_MODE_20HZ    0    /* ת������=20hz; */
#define SWS_AD7705_MODE_25HZ    1    /* ת������=25hz; */
#define SWS_AD7705_MODE_100HZ   2    /* ת������=100hz;*/
#define SWS_AD7705_MODE_200HZ   3    /* ת������=200hz */
#define SWS_AD7705_MODE_50HZ    4    /* ת������=50hz; */
#define SWS_AD7705_MODE_60HZ    5    /* ת������=60hz; */
#define SWS_AD7705_MODE_250HZ   6    /* ת������=250hz;*/
#define SWS_AD7705_MODE_500HZ   7    /* ת������=500hz;*/
/** @} */


/**
 * \brief AD7705 �豸��ʼ��
 *
 * \parsws[in] p_dev      :ָ��AD7705�豸�ṹ���ָ��
 * \parsws[in] p_devinfo  :ָ��AD7705�豸��Ϣ�ṹ���ָ��
 *
 * \return AD7705����������,���Ϊ NULL��������ʼ��ʧ��
 */
sws_adc_handle_t sws_ad7705_init(sws_ad7705_dev_t             *p_dev,
                                 const sws_ad7705_dev_info_t  *p_devinfo);

/**
 * \brief AD7705 �豸���ʼ��
 *
 * \parsws[in] handle : AD7705�������
 *
 * \return ��
 */
void sws_ad7705_deinit (sws_ad7705_adc_handle_t handle);

/**
 * \brief AD7705 ���üĴ���pgaд
 *
 * \parsws[in] p_dev : AD7705�������
 * \parsws[in] pga   : pga��غ�
 *
 * \retval  SWS_OK     : ���óɹ�
 *          SWS_ERROR  : ����ʧ�ܣ�ADCδ׼����
 */
int sws_ad7705_pga_set(sws_ad7705_dev_t  *p_dev, uint16_t pga);

/**
 * \brief AD7705 ���üĴ���chд
 *
 * \parsws[in] p_dev : AD7705�������
 * \parsws[in] ch    : ch��غ�
 *
 * \retval  SWS_OK     : ���óɹ�
 *          SWS_ERROR  : ����ʧ�ܣ�ADCδ׼����
 */
int sws_ad7705_ch_set(sws_ad7705_dev_t  *p_dev, uint32_t ch);

/**
 * \brief AD7705 ���üĴ���speedд
 *
 * \parsws[in] p_dev : AD7705�������
 * \parsws[in] speed : speed��غ�
 *
 * \retval  SWS_OK     : ���óɹ�
 *          SWS_ERROR  : ����ʧ�ܣ�ADCδ׼����
 */
int sws_ad7705_out_speed_set(sws_ad7705_dev_t  *p_dev, uint16_t speed);

/**
 * \brief AD7705 pga�Ŵ�����
 *
 * \parsws[in] p_dev : AD7705�������
 *
 * \return pga�Ŵ���
 */
uint32_t sws_ad7705_pga_get(sws_ad7705_dev_t  *p_dev);

/**
 * \brief AD7705 ͨ���Ŷ�
 *
 * \parsws[in] p_dev : AD7705�������
 *
 * \return chͨ����
 */
uint8_t sws_ad7705_ch_get(sws_ad7705_dev_t  *p_dev);

/**
 * \brief AD7705 adc������ʶ�
 *
 * \parsws[in] p_dev : AD7705�������
 *
 * \return adc�������
 */
int sws_ad7705_out_speed_get(sws_ad7705_dev_t  *p_dev);


/**
 * @}
 */

#ifdef __cplusplus
}
#endif

#endif /* __SWS_SPI_H */

/*end of file */


