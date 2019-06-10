
/**
 * \file
 * \brief  AD7799 应用接口文件
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
 * \brief AD7799 输入使用的引脚
 */
  typedef struct sws_gpio_info {
      GPIO_TypeDef *POR;
      uint16_t      PIN;
  } sws_gpio_info_t;
  
/**
 * \brief AD7799 设备信息结构体
 */
typedef struct sws_ad7799_dev_info{

    /** \brief IO模拟SPI SCLK引脚选择 */
    sws_gpio_info_t      *clk;

    /**
     * \brief ad7799_out引脚选择
     * */
    sws_gpio_info_t      *out_pin;

    /**
     * \brief ad7799_in引脚选择
     * */
    sws_gpio_info_t      *in_pin;
    
    /** \brief  */
    sws_gpio_info_t      *cs; 

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
    
    /** \brief 平台初始化函数，如打开时钟，配置引脚等工作 */
    void     (*pfn_plfm_init)(void);

    /** \brief 平台解初始化函数 */
    void     (*pfn_plfm_deinit)(void);

}sws_ad7799_dev_info_t;

/** \brief 触发回调函数 */
typedef void (*sws_ad7799_code_read_cb_t) (void *p_arg,uint32_t code);

/**
 * \brief AD7799 设备结构体
 */
typedef struct sws_ad7799_dev{

    /** \brief 放大倍数 */
    uint8_t                        pga;
    /** \brief 输出速率 */
    uint8_t                        out_speed;
    /** \brief 通道 */
    uint8_t                        ch;
//    /** \brief 是否开启中断模式 */
//    sws_bool_t                      is_int;

    /** \brief INT引脚的触发信息类型 */
    struct sws_ad7799_trigger_info {

        /** \brief 触发回调函数 */
        sws_ad7799_code_read_cb_t   pfn_callback;
        /** \brief 回调函数的参数 */
        void                      *p_arg;

    } triginfo;/**< \brief INT引脚的触发信息 */

    /** \brief 设备信息 */
    const sws_ad7799_dev_info_t   *p_devinfo;

    /** \brief ADC标准服务 */
    sws_adc_serv_t                 adc_serve;

    /** \brief 存放用户启动转换后的回调函数 */
    sws_adc_seq_cb_t                pfn_callback;

    /** \brief 用户启动回调函数的回调函数参数 */
    void                          *p_arg;

    /** \brief 当前转换的序列描述符数组首地址 */
    sws_adc_buf_desc_t             *p_desc;

    /** \brief 当前转换的序列描述符数量 */
    uint32_t                       desc_num;

    /** \brief 转换的次数 */
    uint32_t                       count;

    /** \brief 转换标志 */
    uint32_t                       flags;

    /** \brief 对当前序列描述符已经采样的次数 */
    uint32_t                       conv_cnt;

    /** \brief 正在执行当前序列描述符的索引 */
    uint32_t                       desc_index;

    /** \brief 对整个序列转换完成的次数 */
    uint32_t                       seq_cnt;

}sws_ad7799_dev_t;

typedef sws_ad7799_dev_t * sws_ad7799_adc_handle_t; /**< \brief 句柄定义 */

//转换模式控制位: bit15,14,13
#define SWS_AD7799_MODE_CONTINUE  0x0000    //v连续转换模式(默认)
#define SWS_AD7799_MODE_SINGLE    0x2000    //单次转换模式

//PSW开关控制位： bit12
#define SWS_AD7799_MODE_PSW_OFF   0x0000    //关闭PSW开关(默认)
#define SWS_AD7799_MODE_PSW_ON    0x1000    //打开PSW开关


//差分输入信号的单/双极性编码控制: bit12
#define SWS_AD7799_CONFIG_BIPOLAR     0x0000    //双极性编码(默认)
#define SWS_AD7799_CONFIG_UNIPOLAR    0x1000    //v单极性编码

/**
 * \nswse AD7799通道选择
 * @{
 */
#define SWS_AD7799_CHANNEL_0         0      /**< \brief 通道 0,默认 */
#define SWS_AD7799_CHANNEL_1         1      /**< \brief 通道 1 */
#define SWS_AD7799_CHANNEL_2         2      /**< \brief 通道 2 */
#define SWS_AD7799_CHANNEL_3         3      /**< \brief 通道 3 */

/** @} */

/**
 * \nswse AD7799 增益选择
 * @{
 */

#define SWS_AD7799_CONFIG_GAIN_1      0x0000    //增益=1(仪表放大器不用)
#define SWS_AD7799_CONFIG_GAIN_2      0x0100    //增益=2(仪表放大器不用)
#define SWS_AD7799_CONFIG_GAIN_4      0x0200    //增益=4
#define SWS_AD7799_CONFIG_GAIN_8      0x0300    //增益=8
#define SWS_AD7799_CONFIG_GAIN_16     0x0400    //增益=16
#define SWS_AD7799_CONFIG_GAIN_32     0x0500    //增益=32
#define SWS_AD7799_CONFIG_GAIN_64     0x0600    //增益=64
#define SWS_AD7799_CONFIG_GAIN_128    0x0700    //增益=128
#define SWS_AD7799_CONFIG_GAIN_AUTO   0X5A5A    //自动增益 - AD7799硬件无该功能，该功能由软件自动实现

/** @} */

/**
 * \nswse AD7799 输出速率选择
 * @{
 */
#define SWS_AD7799_MODE_4_17HZ    0x000f    //转换速率=4.17hz;  抑制:74db(50,60hz)
#define SWS_AD7799_MODE_6_25HZ    0x000e    //转换速率=6.25hz;  抑制:72db(50,60hz)
#define SWS_AD7799_MODE_8_33HZ    0x000d    //转换速率=8.33hz;  抑制:70db(50,60hz)
#define SWS_AD7799_MODE_10HZ      0x000c    //转换速率=10hz  ;  抑制:69db(50,60hz)
#define SWS_AD7799_MODE_12_5HZ    0x000b    //转换速率=12.5hz;  抑制:66db(50,60hz)
#define SWS_AD7799_MODE_16_7HZ    0x000a    //转换速率=16.7hz;  抑制:65db(50,60hz)
#define SWS_AD7799_MODE_16_50HZ   0x0009    //转换速率=16.7hz;  抑制:80db(仅50hz)
#define SWS_AD7799_MODE_19_6HZ    0x0008    //转换速率=19.6hz;  抑制:90db(仅60hz)
#define SWS_AD7799_MODE_50HZ      0x0005    //转换速率=50hz;    抑制:-
#define SWS_AD7799_MODE_470HZ     0x0001    //转换速率=470hz;   抑制:-
/** @} */

//基准电压检测位: bit5
#define SWS_AD7799_CONFIG_REFDET_EN   0x0020    //基准电压检测功能有效：当外部基准电压开路或小于0.5V时,状态寄存器的NOXREF位置位
#define SWS_AD7799_CONFIG_REFDET_DIS  0x0000    //基准电压检测功能禁用
//BUF位: bit4
#define SWS_AD7799_CONFIG_BUF_EN      0x0010    //缓冲工作模式
#define SWS_AD7799_CONFIG_BUF_DIS     0x0000    //无缓冲工作模式
//通道选择位: bit2,1,0
#define SWS_AD7799_CONFIG_AIN1        0x0000    //AIN1差分输入
#define SWS_AD7799_CONFIG_AIN2        0x0001    //AIN2差分输入
#define SWS_AD7799_CONFIG_AIN3        0x0002    //AIN3差分输入


/**
 * \brief AD7799 设备初始化
 *
 * \parsws[in] p_dev      :指向AD7799设备结构体的指针
 * \parsws[in] p_devinfo  :指向AD7799设备信息结构体的指针
 *
 * \return AD7799服务操作句柄,如果为 NULL，表明初始化失败
 */
sws_adc_handle_t sws_ad7799_init(sws_ad7799_dev_t            *p_dev,
                                  const sws_ad7799_dev_info_t           *p_devinfo);

/**
 * \brief AD7799 设备解初始化
 *
 * \parsws[in] handle : AD7799操作句柄
 *
 * \return 无
 */
void sws_ad7799_deinit (sws_ad7799_adc_handle_t handle);

/**
 * \brief AD7799 配置寄存器读
 *
 * \parsws[in] p_dev : AD7799操作句柄
 *
 * \return adc配置寄存器值
 */
uint8_t sws_ad7799_config_reg_read(sws_ad7799_dev_t  *p_dev);

/**
 * \brief AD7799 配置寄存器pga写
 *
 * \parsws[in] p_dev : AD7799操作句柄
 * \parsws[in] pga   : pga相关宏
 *
 * \retval  SWS_OK     : 设置成功
 *          SWS_ERROR  : 设置失败，ADC未准备好
 */
int sws_ad7799_pga_set(sws_ad7799_dev_t  *p_dev, uint32_t pga);

/**
 * \brief AD7799 配置寄存器ch写
 *
 * \parsws[in] p_dev : AD7799操作句柄
 * \parsws[in] ch    : ch相关宏
 *
 * \retval  SWS_OK     : 设置成功
 *          SWS_ERROR  : 设置失败，ADC未准备好
 */
int sws_ad7799_ch_set(sws_ad7799_dev_t  *p_dev, uint32_t ch);

/**
 * \brief AD7799 配置寄存器speed写
 *
 * \parsws[in] p_dev : AD7799操作句柄
 * \parsws[in] speed : speed相关宏
 *
 * \retval  SWS_OK     : 设置成功
 *          SWS_ERROR  : 设置失败，ADC未准备好
 */
int sws_ad7799_out_speed_set(sws_ad7799_dev_t  *p_dev, uint32_t speed);

/**
 * \brief AD7799 pga放大倍数读
 *
 * \parsws[in] p_dev : AD7799操作句柄
 *
 * \return pga放大倍数
 */
uint32_t sws_ad7799_pga_get(sws_ad7799_dev_t  *p_dev);

/**
 * \brief AD7799 通道号读
 *
 * \parsws[in] p_dev : AD7799操作句柄
 *
 * \return ch通道号
 */
uint8_t sws_ad7799_ch_get(sws_ad7799_dev_t  *p_dev);

/**
 * \brief AD7799 adc输出速率读
 *
 * \parsws[in] p_dev : AD7799操作句柄
 *
 * \return adc输出速率
 */
uint32_t sws_ad7799_out_speed_get(sws_ad7799_dev_t  *p_dev);

/**
 * \brief AD7799 允许读
 *
 * \parsws[in] p_dev   : AD7799操作句柄
 *
 * \retval  SWS_OK     : 操作成功
 */
int sws_ad7799_read_int_enable(sws_ad7799_dev_t  *p_dev);

/**
 * \brief AD7799 禁止读
 *
 * \parsws[in] p_dev   : AD7799操作句柄
 *
 * \retval  SWS_OK     : 操作成功
 */
int sws_ad7799_read_int_disable(sws_ad7799_dev_t  *p_dev);

/**
 * \brief AD7799 轮循读adc采集值
 *
 * \parsws[in]  p_dev   : AD7799操作句柄
 * \parsws[out] val     : 采样数据
 *
 * \retval  SWS_OK 采集成功
 */
int sws_ad7799_read_polling(sws_ad7799_dev_t *p_dev, uint32_t *val);

/**
 * \brief AD7799 进入低功耗模式
 *
 * \parsws[in] p_dev   : AD7799操作句柄
 *
 * \retval  无
 */
void sws_ad7799_power_down_enter(sws_ad7799_dev_t  *p_dev);

/**
 * \brief AD7799 退出低功耗模式
 *
 * \parsws[in] p_dev   : AD7799操作句柄
 *
 * \retval  无
 */
void sws_ad7799_power_down_out(sws_ad7799_dev_t  *p_dev);

/**
 * \brief AD7799 连接中断回调函数
 *
 * \parsws[in] p_dev   : AD7799操作句柄
 * \parsws[in] p_fun   : AD7799中断回调函数
 * \parsws[in] p_arg   : AD7799中断回调函数参数
 *
 * \retval  无
 */
void sws_ad7799_int_connect(sws_ad7799_dev_t      *p_dev ,
                           sws_ad7799_code_read_cb_t  p_fun,
                           void                     *p_arg);

/**
 * \brief AD7799 删除中断回调函数
 *
 * \parsws[in] p_dev   : AD7799操作句柄
 * \parsws[in] p_fun   : AD7799中断回调函数
 * \parsws[in] p_arg   : AD7799中断回调函数参数
 *
 * \retval  无
 */
void sws_ad7799_int_disconnect(sws_ad7799_dev_t      *p_dev ,
                              sws_ad7799_code_read_cb_t  p_fun,
                              void                     *p_arg);

/**
 * \brief AD7799 获得标准adc句柄
 * \parsws[in] p_dev : AD7799操作句柄
 *
 * \retval 标准adc操作句柄
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


