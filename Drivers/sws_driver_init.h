
/**
 * \file
 * \brief 外设实例初始化函数声明
 *
 * 所有外设设备的实例初始化函数均在本头文件中声明，使用实例
 * 初始化函数，可以轻松获得一个外设的句柄，然后使用相关的接口函数对外设进行操作。
 *
 * \internal
 * \par Modification history
 * - 1.00 16-10-26  wangpeng, first implementation.
 * \endinternal
 */

/** \brief AD7799实例初始化，获得ADC标准服务句柄 */
sws_adc_handle_t sws_ad7799_inst_init (void)
{
    return sws_ad7799_init(&__g_ad7799_dev, &__g_ad7799_devinfo);
}

/** \brief AD7799实例解初始化 */
void sws_ad7799_inst_deinit (sws_adc_handle_t handle)