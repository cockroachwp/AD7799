
/**
 * \file
 * \brief ����ʵ����ʼ����������
 *
 * ���������豸��ʵ����ʼ���������ڱ�ͷ�ļ���������ʹ��ʵ��
 * ��ʼ���������������ɻ��һ������ľ����Ȼ��ʹ����صĽӿں�����������в�����
 *
 * \internal
 * \par Modification history
 * - 1.00 16-10-26  wangpeng, first implementation.
 * \endinternal
 */

/** \brief AD7799ʵ����ʼ�������ADC��׼������ */
sws_adc_handle_t sws_ad7799_inst_init (void)
{
    return sws_ad7799_init(&__g_ad7799_dev, &__g_ad7799_devinfo);
}

/** \brief AD7799ʵ�����ʼ�� */
void sws_ad7799_inst_deinit (sws_adc_handle_t handle)