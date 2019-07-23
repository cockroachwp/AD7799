
/**
 * \file
 * \brief  AD7705 应用接口文件
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
 * \brief AD7705 输入使用的引脚
 */
  typedef struct sws_gpio_info {
      GPIO_TypeDef *POR;
      uint16_t      PIN;
  } sws_gpio_info_t;

/**
 * \brief AD7705 设备信息结构体
 */
typedef struct sws_AD7705_dev_info{

    /** \brief IO模拟SPI SCLK引脚选择 */
    sws_gpio_info_t      *clk;

    /**
     * \brief AD7705_out引脚选择
     * */
    sws_gpio_info_t      *out_pin;

    /**
     * \brief AD7705_in引脚选择
     * */
    sws_gpio_info_t      *in_pin;

    /** \brief  片选位*/
    sws_gpio_info_t      *cs;

    sws_gpio_info_t      *reset;

    /** \brief  转换模式*/
    uint8_t               mode;

    /** \brief  转换极性*/
    uint8_t               polar;

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
    void       (*pfn_plfm_init)(void);

    /** \brief 平台解初始化函数 */
    void       (*pfn_plfm_deinit)(void);

}sws_ad7705_dev_info_t;

/** \brief 触发回调函数 */
typedef void (*sws_AD7705_code_read_cb_t) (void *p_arg,uint32_t code);

/**
 * \brief AD7705 设备结构体
 */
typedef struct sws_AD7705_dev{

    /** \brief 放大倍数 */
    uint8_t                        pga;
    /** \brief 输出速率 */
    uint8_t                        out_speed;
    /** \brief 通道 */
    uint16_t                       config;
    /** \brief 通道 */
    uint16_t                       mode;
    /** \brief INT引脚的触发信息类型 */
    struct sws_AD7705_trigger_info {

        /** \brief 触发回调函数 */
        sws_AD7705_code_read_cb_t   pfn_callback;
        /** \brief 回调函数的参数 */
        void                       *p_arg;

    } triginfo;/**< \brief INT引脚的触发信息 */

    /** \brief 设备信息 */
    const sws_ad7705_dev_info_t   *p_devinfo;

    /** \brief ADC标准服务 */
    sws_adc_serv_t                 adc_serve;

    /** \brief 存放用户启动转换后的回调函数 */
    sws_adc_seq_cb_t               pfn_callback;

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

}sws_ad7705_dev_t;

typedef sws_ad7705_dev_t * sws_ad7705_adc_handle_t; /**< \brief 句柄定义 */

////转换模式控制位: bit15,14,13
//#define SWS_AD7705_MODE_CONTINUE  0x0000    //v连续转换模式(默认)
//#define SWS_AD7705_MODE_SINGLE    0x2000    //单次转换模式

//PSW开关控制位： bit12
//#define SWS_AD7705_MODE_PSW_OFF   0x0000    //关闭PSW开关(默认)
//define SWS_AD7705_MODE_PSW_ON    0x1000    //打开PSW开关K


/************************************** 通讯寄存器 **************************************/

/* 寄存器选择 */

#define SWS_AD7705_COMMUNI_REG           0 << 4   /* 通讯寄存器 */
#define SWS_AD7705_SET_REG               1 << 4   /* 设置寄存器 */
#define SWS_AD7705_CLK_REG               2 << 4   /* 时钟寄存器 */
#define SWS_AD7705_DATA_REG              3 << 4   /* 数据寄存器 */
#define SWS_AD7705_TEST_REG              4 << 4   /* 测试寄存器 */
#define SWS_AD7705_DEVI_REG              6 << 4   /* 偏移寄存器 */
#define SWS_AD7705_PAG_REG               7 << 4   /* 增益寄存器 */

#define SWS_AD7705_WRITE                 0 << 3   /* 读操作 */
#define SWS_AD7705_READ                  1 << 3   /* 写操作 */

#define SWS_AD7705_CHANNEL_1         0      /**< \brief 通道 1 */
#define SWS_AD7705_CHANNEL_2         1      /**< \brief 通道 2 */


/************************************** 设置寄存器 **************************************/


#define SWS_AD7705_MODE_NAR			       0 << 6		/* 正常转换模式 */
#define SWS_AD7705_MODE_CALIB_SELF	       1 << 6		/* 自校准模式 */
#define SWS_AD7705_MODE_CALIB_ZERO	       2 << 6		/* 零标度校准模式 */
#define SWS_AD7705_MODE_CALIB_FULL		   3 << 6		/* 满度校准模式 */

/**
 * \nswse AD7705 增益选择
 * @{
 */

#define SWS_AD7705_CONFIG_GAIN_1      0 << 3   /*增益=1(仪表放大器不用) */
#define SWS_AD7705_CONFIG_GAIN_2      1 << 3   /*增益=2(仪表放大器不用) */
#define SWS_AD7705_CONFIG_GAIN_4      2 << 3   /*增益=4 */
#define SWS_AD7705_CONFIG_GAIN_8      3 << 3   /*增益=8 */
#define SWS_AD7705_CONFIG_GAIN_16     4 << 3   /*增益=16 */
#define SWS_AD7705_CONFIG_GAIN_32     5 << 3   /*增益=32 */
#define SWS_AD7705_CONFIG_GAIN_64     6 << 3   /*增益=64 */
#define SWS_AD7705_CONFIG_GAIN_128    7 << 3   /*增益=128 */


//差分输入信号的单/双极性编码控制: bit12
#define SWS_AD7705_CONFIG_BIPOLAR     0 << 2     /*双极性编码(默认) */
#define SWS_AD7705_CONFIG_UNIPOLAR    1 << 2     /*v单极性编码 */


/************************************** 时钟寄存器 **************************************/


/**
 * \nswse AD7705 输出速率选择
 * @{
 */

#define SWS_AD7705_CLK_IN       0 << 4  /* 外部时钟 */
#define SWS_AD7705_CLK_OUT      1 << 4  /* 内部时钟 */


#define SWS_AD7705_MODE_20HZ    0    /* 转换速率=20hz; */
#define SWS_AD7705_MODE_25HZ    1    /* 转换速率=25hz; */
#define SWS_AD7705_MODE_100HZ   2    /* 转换速率=100hz;*/
#define SWS_AD7705_MODE_200HZ   3    /* 转换速率=200hz */
#define SWS_AD7705_MODE_50HZ    4    /* 转换速率=50hz; */
#define SWS_AD7705_MODE_60HZ    5    /* 转换速率=60hz; */
#define SWS_AD7705_MODE_250HZ   6    /* 转换速率=250hz;*/
#define SWS_AD7705_MODE_500HZ   7    /* 转换速率=500hz;*/
/** @} */


/**
 * \brief AD7705 设备初始化
 *
 * \parsws[in] p_dev      :指向AD7705设备结构体的指针
 * \parsws[in] p_devinfo  :指向AD7705设备信息结构体的指针
 *
 * \return AD7705服务操作句柄,如果为 NULL，表明初始化失败
 */
sws_adc_handle_t sws_ad7705_init(sws_ad7705_dev_t             *p_dev,
                                 const sws_ad7705_dev_info_t  *p_devinfo);

/**
 * \brief AD7705 设备解初始化
 *
 * \parsws[in] handle : AD7705操作句柄
 *
 * \return 无
 */
void sws_ad7705_deinit (sws_ad7705_adc_handle_t handle);

/**
 * \brief AD7705 配置寄存器pga写
 *
 * \parsws[in] p_dev : AD7705操作句柄
 * \parsws[in] pga   : pga相关宏
 *
 * \retval  SWS_OK     : 设置成功
 *          SWS_ERROR  : 设置失败，ADC未准备好
 */
int sws_ad7705_pga_set(sws_ad7705_dev_t  *p_dev, uint16_t pga);

/**
 * \brief AD7705 配置寄存器ch写
 *
 * \parsws[in] p_dev : AD7705操作句柄
 * \parsws[in] ch    : ch相关宏
 *
 * \retval  SWS_OK     : 设置成功
 *          SWS_ERROR  : 设置失败，ADC未准备好
 */
int sws_ad7705_ch_set(sws_ad7705_dev_t  *p_dev, uint32_t ch);

/**
 * \brief AD7705 配置寄存器speed写
 *
 * \parsws[in] p_dev : AD7705操作句柄
 * \parsws[in] speed : speed相关宏
 *
 * \retval  SWS_OK     : 设置成功
 *          SWS_ERROR  : 设置失败，ADC未准备好
 */
int sws_ad7705_out_speed_set(sws_ad7705_dev_t  *p_dev, uint16_t speed);

/**
 * \brief AD7705 pga放大倍数读
 *
 * \parsws[in] p_dev : AD7705操作句柄
 *
 * \return pga放大倍数
 */
uint32_t sws_ad7705_pga_get(sws_ad7705_dev_t  *p_dev);

/**
 * \brief AD7705 通道号读
 *
 * \parsws[in] p_dev : AD7705操作句柄
 *
 * \return ch通道号
 */
uint8_t sws_ad7705_ch_get(sws_ad7705_dev_t  *p_dev);

/**
 * \brief AD7705 adc输出速率读
 *
 * \parsws[in] p_dev : AD7705操作句柄
 *
 * \return adc输出速率
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


