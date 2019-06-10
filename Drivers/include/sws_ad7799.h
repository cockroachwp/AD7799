
/**
 * \file
 * \brief  AD7799 Ӧ�ýӿ��ļ�
 *
 * \internal
 * \par Modification History
 * - 1.00 19-6-10  wangpeng, first implementation.
 * \endinternal
 */

#ifndef __SWS_AD7799_H
#define __SWS_AD7799_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @addtogroup sws_if_ad7799
 * @copydoc sws_ad7799.h
 * @{
 */

#include "stm32f10x.h"
#include "sws_adc.h"
  
  /**
 * \brief AD7799 ����ʹ�õ�����
 */
  typedef struct sws_gpio_info {
      GPIO_TypeDef *POR;
      uint16_t      PIN;
  } sws_gpio_info_t;
  
/**
 * \brief AD7799 �豸��Ϣ�ṹ��
 */
typedef struct sws_ad7799_dev_info{

    /** \brief IOģ��SPI SCLK����ѡ�� */
    sws_gpio_info_t      *clk;

    /**
     * \brief ad7799_out����ѡ��
     * */
    sws_gpio_info_t      *out_pin;

    /**
     * \brief ad7799_in����ѡ��
     * */
    sws_gpio_info_t      *in_pin;
    
    /** \brief  */
    sws_gpio_info_t      *cs; 

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
    void     (*pfn_plfm_init)(void);

    /** \brief ƽ̨���ʼ������ */
    void     (*pfn_plfm_deinit)(void);

}sws_ad7799_dev_info_t;

/** \brief �����ص����� */
typedef void (*sws_ad7799_code_read_cb_t) (void *p_arg,uint32_t code);

/**
 * \brief AD7799 �豸�ṹ��
 */
typedef struct sws_ad7799_dev{

    /** \brief �Ŵ��� */
    uint8_t                        pga;
    /** \brief ������� */
    uint8_t                        out_speed;
    /** \brief ͨ�� */
    uint8_t                        ch;
//    /** \brief �Ƿ����ж�ģʽ */
//    sws_bool_t                      is_int;

    /** \brief INT���ŵĴ�����Ϣ���� */
    struct sws_ad7799_trigger_info {

        /** \brief �����ص����� */
        sws_ad7799_code_read_cb_t   pfn_callback;
        /** \brief �ص������Ĳ��� */
        void                      *p_arg;

    } triginfo;/**< \brief INT���ŵĴ�����Ϣ */

    /** \brief �豸��Ϣ */
    const sws_ad7799_dev_info_t   *p_devinfo;

    /** \brief ADC��׼���� */
    sws_adc_serv_t                 adc_serve;

    /** \brief ����û�����ת����Ļص����� */
    sws_adc_seq_cb_t                pfn_callback;

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

}sws_ad7799_dev_t;

typedef sws_ad7799_dev_t * sws_ad7799_adc_handle_t; /**< \brief ������� */

//ת��ģʽ����λ: bit15,14,13
#define SWS_AD7799_MODE_CONTINUE  0x0000    //v����ת��ģʽ(Ĭ��)
#define SWS_AD7799_MODE_SINGLE    0x2000    //����ת��ģʽ

//PSW���ؿ���λ�� bit12
#define SWS_AD7799_MODE_PSW_OFF   0x0000    //�ر�PSW����(Ĭ��)
#define SWS_AD7799_MODE_PSW_ON    0x1000    //��PSW����


//��������źŵĵ�/˫���Ա������: bit12
#define SWS_AD7799_CONFIG_BIPOLAR     0x0000    //˫���Ա���(Ĭ��)
#define SWS_AD7799_CONFIG_UNIPOLAR    0x1000    //v�����Ա���

/**
 * \nswse AD7799ͨ��ѡ��
 * @{
 */
#define SWS_AD7799_CHANNEL_0         0      /**< \brief ͨ�� 0,Ĭ�� */
#define SWS_AD7799_CHANNEL_1         1      /**< \brief ͨ�� 1 */
#define SWS_AD7799_CHANNEL_2         2      /**< \brief ͨ�� 2 */
#define SWS_AD7799_CHANNEL_3         3      /**< \brief ͨ�� 3 */

/** @} */

/**
 * \nswse AD7799 ����ѡ��
 * @{
 */

#define SWS_AD7799_CONFIG_GAIN_1      0x0000    //����=1(�Ǳ�Ŵ�������)
#define SWS_AD7799_CONFIG_GAIN_2      0x0100    //����=2(�Ǳ�Ŵ�������)
#define SWS_AD7799_CONFIG_GAIN_4      0x0200    //����=4
#define SWS_AD7799_CONFIG_GAIN_8      0x0300    //����=8
#define SWS_AD7799_CONFIG_GAIN_16     0x0400    //����=16
#define SWS_AD7799_CONFIG_GAIN_32     0x0500    //����=32
#define SWS_AD7799_CONFIG_GAIN_64     0x0600    //����=64
#define SWS_AD7799_CONFIG_GAIN_128    0x0700    //����=128
#define SWS_AD7799_CONFIG_GAIN_AUTO   0X5A5A    //�Զ����� - AD7799Ӳ���޸ù��ܣ��ù���������Զ�ʵ��

/** @} */

/**
 * \nswse AD7799 �������ѡ��
 * @{
 */
#define SWS_AD7799_MODE_4_17HZ    0x000f    //ת������=4.17hz;  ����:74db(50,60hz)
#define SWS_AD7799_MODE_6_25HZ    0x000e    //ת������=6.25hz;  ����:72db(50,60hz)
#define SWS_AD7799_MODE_8_33HZ    0x000d    //ת������=8.33hz;  ����:70db(50,60hz)
#define SWS_AD7799_MODE_10HZ      0x000c    //ת������=10hz  ;  ����:69db(50,60hz)
#define SWS_AD7799_MODE_12_5HZ    0x000b    //ת������=12.5hz;  ����:66db(50,60hz)
#define SWS_AD7799_MODE_16_7HZ    0x000a    //ת������=16.7hz;  ����:65db(50,60hz)
#define SWS_AD7799_MODE_16_50HZ   0x0009    //ת������=16.7hz;  ����:80db(��50hz)
#define SWS_AD7799_MODE_19_6HZ    0x0008    //ת������=19.6hz;  ����:90db(��60hz)
#define SWS_AD7799_MODE_50HZ      0x0005    //ת������=50hz;    ����:-
#define SWS_AD7799_MODE_470HZ     0x0001    //ת������=470hz;   ����:-
/** @} */

//��׼��ѹ���λ: bit5
#define SWS_AD7799_CONFIG_REFDET_EN   0x0020    //��׼��ѹ��⹦����Ч�����ⲿ��׼��ѹ��·��С��0.5Vʱ,״̬�Ĵ�����NOXREFλ��λ
#define SWS_AD7799_CONFIG_REFDET_DIS  0x0000    //��׼��ѹ��⹦�ܽ���
//BUFλ: bit4
#define SWS_AD7799_CONFIG_BUF_EN      0x0010    //���幤��ģʽ
#define SWS_AD7799_CONFIG_BUF_DIS     0x0000    //�޻��幤��ģʽ
//ͨ��ѡ��λ: bit2,1,0
#define SWS_AD7799_CONFIG_AIN1        0x0000    //AIN1�������
#define SWS_AD7799_CONFIG_AIN2        0x0001    //AIN2�������
#define SWS_AD7799_CONFIG_AIN3        0x0002    //AIN3�������


/**
 * \brief AD7799 �豸��ʼ��
 *
 * \parsws[in] p_dev      :ָ��AD7799�豸�ṹ���ָ��
 * \parsws[in] p_devinfo  :ָ��AD7799�豸��Ϣ�ṹ���ָ��
 *
 * \return AD7799����������,���Ϊ NULL��������ʼ��ʧ��
 */
sws_adc_handle_t sws_ad7799_init(sws_ad7799_dev_t            *p_dev,
                                  const sws_ad7799_dev_info_t           *p_devinfo);

/**
 * \brief AD7799 �豸���ʼ��
 *
 * \parsws[in] handle : AD7799�������
 *
 * \return ��
 */
void sws_ad7799_deinit (sws_ad7799_adc_handle_t handle);

/**
 * \brief AD7799 ���üĴ�����
 *
 * \parsws[in] p_dev : AD7799�������
 *
 * \return adc���üĴ���ֵ
 */
uint8_t sws_ad7799_config_reg_read(sws_ad7799_dev_t  *p_dev);

/**
 * \brief AD7799 ���üĴ���pgaд
 *
 * \parsws[in] p_dev : AD7799�������
 * \parsws[in] pga   : pga��غ�
 *
 * \retval  SWS_OK     : ���óɹ�
 *          SWS_ERROR  : ����ʧ�ܣ�ADCδ׼����
 */
int sws_ad7799_pga_set(sws_ad7799_dev_t  *p_dev, uint32_t pga);

/**
 * \brief AD7799 ���üĴ���chд
 *
 * \parsws[in] p_dev : AD7799�������
 * \parsws[in] ch    : ch��غ�
 *
 * \retval  SWS_OK     : ���óɹ�
 *          SWS_ERROR  : ����ʧ�ܣ�ADCδ׼����
 */
int sws_ad7799_ch_set(sws_ad7799_dev_t  *p_dev, uint32_t ch);

/**
 * \brief AD7799 ���üĴ���speedд
 *
 * \parsws[in] p_dev : AD7799�������
 * \parsws[in] speed : speed��غ�
 *
 * \retval  SWS_OK     : ���óɹ�
 *          SWS_ERROR  : ����ʧ�ܣ�ADCδ׼����
 */
int sws_ad7799_out_speed_set(sws_ad7799_dev_t  *p_dev, uint32_t speed);

/**
 * \brief AD7799 pga�Ŵ�����
 *
 * \parsws[in] p_dev : AD7799�������
 *
 * \return pga�Ŵ���
 */
uint32_t sws_ad7799_pga_get(sws_ad7799_dev_t  *p_dev);

/**
 * \brief AD7799 ͨ���Ŷ�
 *
 * \parsws[in] p_dev : AD7799�������
 *
 * \return chͨ����
 */
uint8_t sws_ad7799_ch_get(sws_ad7799_dev_t  *p_dev);

/**
 * \brief AD7799 adc������ʶ�
 *
 * \parsws[in] p_dev : AD7799�������
 *
 * \return adc�������
 */
uint32_t sws_ad7799_out_speed_get(sws_ad7799_dev_t  *p_dev);

/**
 * \brief AD7799 �����
 *
 * \parsws[in] p_dev   : AD7799�������
 *
 * \retval  SWS_OK     : �����ɹ�
 */
int sws_ad7799_read_int_enable(sws_ad7799_dev_t  *p_dev);

/**
 * \brief AD7799 ��ֹ��
 *
 * \parsws[in] p_dev   : AD7799�������
 *
 * \retval  SWS_OK     : �����ɹ�
 */
int sws_ad7799_read_int_disable(sws_ad7799_dev_t  *p_dev);

/**
 * \brief AD7799 ��ѭ��adc�ɼ�ֵ
 *
 * \parsws[in]  p_dev   : AD7799�������
 * \parsws[out] val     : ��������
 *
 * \retval  SWS_OK �ɼ��ɹ�
 */
int sws_ad7799_read_polling(sws_ad7799_dev_t *p_dev, uint32_t *val);

/**
 * \brief AD7799 ����͹���ģʽ
 *
 * \parsws[in] p_dev   : AD7799�������
 *
 * \retval  ��
 */
void sws_ad7799_power_down_enter(sws_ad7799_dev_t  *p_dev);

/**
 * \brief AD7799 �˳��͹���ģʽ
 *
 * \parsws[in] p_dev   : AD7799�������
 *
 * \retval  ��
 */
void sws_ad7799_power_down_out(sws_ad7799_dev_t  *p_dev);

/**
 * \brief AD7799 �����жϻص�����
 *
 * \parsws[in] p_dev   : AD7799�������
 * \parsws[in] p_fun   : AD7799�жϻص�����
 * \parsws[in] p_arg   : AD7799�жϻص���������
 *
 * \retval  ��
 */
void sws_ad7799_int_connect(sws_ad7799_dev_t      *p_dev ,
                           sws_ad7799_code_read_cb_t  p_fun,
                           void                     *p_arg);

/**
 * \brief AD7799 ɾ���жϻص�����
 *
 * \parsws[in] p_dev   : AD7799�������
 * \parsws[in] p_fun   : AD7799�жϻص�����
 * \parsws[in] p_arg   : AD7799�жϻص���������
 *
 * \retval  ��
 */
void sws_ad7799_int_disconnect(sws_ad7799_dev_t      *p_dev ,
                              sws_ad7799_code_read_cb_t  p_fun,
                              void                     *p_arg);

/**
 * \brief AD7799 ��ñ�׼adc���
 * \parsws[in] p_dev : AD7799�������
 *
 * \retval ��׼adc�������
 */
sws_adc_handle_t sws_ad7799_standard_adc_handle_get(sws_ad7799_dev_t *p_dev);
/**
 * @}
 */

#ifdef __cplusplus
}
#endif

#endif /* __SWS_SPI_H */

/*end of file */


