
/**
 * \file
 * \brief ADC标准接口
 *
 * \internal
 * \par Modification History
 * - 1.00 19-01-10  wangpeng, first implementation.
 * \endinternal
 */

#ifndef __SWS_ADC_H
#define __SWS_ADC_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * \addtogroup sws_if_adc
 * \copydoc sws_adc.h
 * \{
 */
  
#include "stdint.h"
  
/**
 * \nswse ADC启动转换的标志，用于sws_adc_start()函数的flags参数
 * @{
 */

/** 
 * \brief 转换结果数据左对齐
 *
 * 数据缓冲区类型数据位数超过ADC转换位数时，数据低位补0。
 */
#define SWS_ADC_DATA_ALIGN_LEFT    0x01

/** 
 * \brief 转换结果数据右对齐，这种方式往往能够取得更快的速度(默认)
 *
 * 数据缓冲区类型数据位数超过ADC转换位数时，数据高位补0。
 */
#define SWS_ADC_DATA_ALIGN_RIGHT   0x00

/** @} */

/**
 * \brief 缓冲区完成回调函数类型定义
 *
 *     启动ADC转换时，每个缓冲区都有一个对应的缓冲区完成回调函数，该缓冲区数据
 * 填充满后调用该回调函数。状态参数值(state)表明了本缓冲区数据的有效性:
 *    - SWS_OK        : 数据正确，可以正常使用
 *    - SWS_ERROR     : 数据有误，该缓冲区的数据应该丢弃。
 *
 *     处理转换数据前应该确保转换结束的状态为SWS_OK。如果不为SWS_OK，则序列的后续
 * 转换将自动取消，不会再转换。通常情况下，转换出错可能是由于缓冲区的buf过小，
 * 使得buf切换需要很频繁，数据处理时间稍长，数据就溢出了。可以增大缓冲区的buf大
 * 小以增加buf切换的时间间隔。
 */
typedef void (*sws_adc_buf_cb_t) (void *p_arg, int state);

/**
 * \brief 转换完成回调函数类型定义
 *
 * 序列完成回调函数在一个序列中各缓冲区填充满后调用,以下情况可视为序列转换完成：
 *  - 单个缓冲区，单个缓冲区填充完成即序列转换完成
 *  - 多个缓冲区，从第一个缓冲区到最后一个缓冲区依次填充满才为序列转换完成
 * 
 *     序列正常完成时，状态参数值为SWS_OK。如果填充某个缓冲区时数据出错，也会调
 * 用该函数，且传递参数为SWS_ERROR,表明该序列错误结束。此时，转换自动停止，如果
 * 该序列中还有未转换的序列，则会自动取消，不再继续转换。
 *
 * 该函数仅作为序列完成的一种通知，数据处理应该在每个缓冲区的回调函数中完成。
 */
typedef void (*sws_adc_seq_cb_t) (void *p_arg, int state);

/**
 * \brief ADC缓冲区描述
 */
typedef struct sws_adc_buf_desc {
    
    /** 
     * \brief ADC数据缓冲区，其具体类型与ADC位数有关，
     *        ADC位数可以由函数sws_adc_bits_get()获取。
     *
     *    ADC bits    |    buffer type
     *  ------------- | -------------------
     *     1  ~  8    |  uint8_t/int8_t
     *     9  ~  16   |  uint16_t/int16_t
     *    17  ~  32   |  uint32_t/int32_t
     */
    void                    *p_buf;
    
    /** \brief 数据缓冲区的长度 */
    uint32_t                 length;
    
    /** \brief 该缓冲区数据填充满后的回调函数 */
    sws_adc_buf_cb_t         pfn_complete;
    
    /** \brief 回调函数的参数 */
    void                    *p_arg;
} sws_adc_buf_desc_t;
    
 
/**
 * \brief ADC驱动函数
 */
struct sws_adc_drv_funcs {
    
    /** \brief 启动ADC转换        */
    int (*pfn_adc_start) (void                   *p_drv,
                          int                     chan,
                          sws_adc_buf_desc_t       *p_desc,
                          uint32_t                desc_num,
                          uint32_t                count,
                          uint32_t                flags,
                          sws_adc_seq_cb_t         pfn_callback,
                          void                   *p_arg);
     
    /** \brief 停止转换           */
    int (*pfn_adc_stop) (void *p_drv, int chan);
                          
    /** \brief 获取ADC的采样率    */
    int (*pfn_rate_get) (void      *p_drv, 
                         int        chan,
                         uint32_t   *p_rate);

    /** \brief 设置ADC的采样率，实际采样率可能存在差异 */
    int (*pfn_rate_set) (void     *p_drv, 
                         int       chan, 
                         uint32_t  rate);
    
    /** \brief 获取ADC转换精度 */
    uint32_t (*pfn_bits_get)(void *p_drv, int chan);

    /** \brief 获取ADC参考电压 */
    uint32_t (*pfn_vref_get)(void *p_drv, int chan);
};

/**
 * \brief ADC标准服务
 */
typedef struct sws_adc_serv {

    /** \brief 指向ADC驱动函数的指针 */
    const struct sws_adc_drv_funcs *p_funcs;

    /** \brief 驱动函数第一个入口参数 */
    void                          *p_drv;
} sws_adc_serv_t;

/** \brief ADC标准服务操作句柄定义 */
typedef sws_adc_serv_t *sws_adc_handle_t;

/**
 * \brief 配置一个缓冲区描述符
 * 
 * \parsws[in] p_desc       : 指向一个ADC缓冲区描述符的指针
 * \parsws[in] p_buf        : 数据缓冲区，具体类型与ADC位数有关
 * \parsws[in] length       : 数据缓冲区的长度，决定了可以存放多少次转换结果
 * \parsws[in] pfn_complete : 本次数据缓冲区填充满后的回调函数
 * \parsws[in] p_arg        : 回调函数用户自定义参数
 *
 * \note 关于p_buf数据缓冲区的类型，具体类型应根据ADC转换位数设置,
 *       ADC位数可以由函数sws_adc_bits_get()获取。
 *
 *      ADC bits    |    buffer type
 *    ------------- | -------------------
 *       1  ~  8    |  uint8_t/int8_t
 *       9  ~  16   |  uint16_t/int16_t
 *      17  ~  32   |  uint32_t/int32_t
 */
static inline
void sws_adc_mkbufdesc (sws_adc_buf_desc_t      *p_desc,
                       void                   *p_buf,
                       uint32_t                length,
                       sws_adc_buf_cb_t         pfn_complete,
                       void                   *p_arg)
{
    p_desc->p_buf        = p_buf;
    p_desc->length       = length;
    p_desc->pfn_complete = pfn_complete;
    p_desc->p_arg        = p_arg;
}


/**
 * \brief 启动ADC转换
 * 
 * \parsws[in] handle       ： ADC标准服务操作句柄
 * \parsws[in] chan         ：待转换的通道
 * \parsws[in] p_desc       ：指向缓冲区描述的指针
 * \parsws[in] desc_num     ：缓冲区描述的个数
 * \parsws[in] count        ：序列的转换次数（一个序列表示desc_num指定个数的缓
 *                           冲区依次填充满），为0表示需要持续不断的转换。
 * \parsws[in] flags        : 可用标志见"ADC启动转换的标志"
 * \parsws[in] pfn_callback ：序列完成回调函数
 * \parsws[in] p_arg        ：序列完成回调函数用户自定义参数
 *
 * \retval  SWS_OK     : 启动转换成功
 * \retval -SWS_ENXIO  : 指定的通道不存在
 * \retval -SWS_EINVAL : 无效参数
 *
 * \par 范例
 * \code
 *  sws_adc_buf_desc_t   desc[2];                // 使用两个缓冲区描述
 *  uint16_t            adc_val0[80];           // 数据缓冲区1
 *  uint16_t            adc_val1[80];           // 数据缓冲区2
 *  
 *  uint8_t             g_flag_complete = 0;    // 完成标志
 *
 *  void adc_callback(void *p_arg, int state)
 *  {
 *      
 *      if (state != SWS_OK) {
 *         return;               // 数据有误
 *      }
 *      if ((int)p_arg == 0) {    // 缓冲区0转换完成
 *         // 处理 adc_val0[]中的80个数据
 *      } else {
 *         // 处理 adc_val1[]中的80个数据
 *      } 
 *  }
 *
 *  void adc_seq_complete (void *p_arg, int state) 
 *  {
 *      g_flag_complete = 1; 
 *  }
 * 
 *  int main()
 *  {
 *     // 配置缓冲区描述符0
 *     sws_adc_mkbufdesc(&desc[0],
 *                      (void *)&adc_val0, // 数据存放缓冲区
 *                      80,                // 大小：80
 *                      adc_callback,      // 转换完成回调函数
 *                      (void *)0);        // 传递参数0
 *
 *     // 配置缓冲区描述符1
 *     sws_adc_mkbufdesc(&desc[1],
 *                      (void *)&adc_val1, // 数据存放缓冲区
 *                      80,                // 大小：80
 *                      adc_callback,      // 转换完成回调函数
 *                      (void *)1);        // 传递参数1
 *   
 *     sws_adc_start(adc_handle,
 *                  0,
 *                  &desc[0],
 *                  2,                     // 描述符个数为2
 *                  1,                     // 整个序列仅需完成一次
 *                  0,                     // 无特殊标志，数据右对齐
 *                  NULL,                  // 序列完成无需回调函数
 *                  NULL);                 // 无回调函数参数
 *       
 *      while(g_flag_complete == 0);       // 等待序列转换完成
 *
 *      //...
 *      while(1) {
 *          //...
 *      }
 *  }
 *
 * \endcode
 *
 */
static inline
int sws_adc_start (sws_adc_handle_t         handle,
                  int                     chan,
                  sws_adc_buf_desc_t      *p_desc,
                  uint32_t                desc_num,
                  uint32_t                count,
                  uint32_t                flags,
                  sws_adc_seq_cb_t         pfn_callback,
                  void                   *p_arg)
{
   return handle->p_funcs->pfn_adc_start(handle->p_drv,
                                         chan,
                                         p_desc,
                                         desc_num,
                                         count,
                                         flags,
                                         pfn_callback,
                                         p_arg);
}

/**
 * \brief 停止ADC转换
 * 
 * \parsws[in] handle  : ADC标准服务操作句柄
 * \parsws[in] chan    ：停止转换的通道
 *
 * \retval  SWS_OK     : 停止转换成功
 * \retval -SWS_ENXIO  ：指定的通道不存在
 * \retval -SWS_EINVAL ：无效参数
 */
static inline
int sws_adc_stop (sws_adc_handle_t handle, int chan)
{
   return handle->p_funcs->pfn_adc_stop(handle->p_drv, chan);
}

/**
 * \brief 获取ADC转换精度
 *
 * \parsws[in] handle       : ADC标准服务操作句柄
 * \parsws[in] chan         ：停止转换的通道
 *
 * \retval     >0     : ADC转换位数
 * \retval -SWS_EINVAL ：无效参数
 */
static inline
uint32_t sws_adc_bits_get (sws_adc_handle_t handle, int chan)
{
    return handle->p_funcs->pfn_bits_get(handle->p_drv, chan);
}

/**
 * \brief 获取ADC参考电压，单位：mV
 *
 * \parsws[in] handle  : ADC标准服务操作句柄
 * \parsws[in] chan    ：停止转换的通道
 *
 * \retval     >0     : ADC参考电压
 * \retval -SWS_EINVAL ：无效参数
 */
static inline
int sws_adc_vref_get (sws_adc_handle_t handle, int chan)
{
    return handle->p_funcs->pfn_vref_get(handle->p_drv, chan);
}

/**
 * \brief 获取ADC的采样率，单位Hz
 *
 * \parsws[in] handle  : ADC标准服务操作句柄
 * \parsws[in] chan    ：停止转换的通道
 * \parsws[out] p_rate ： 获取采样率
 *
 * \retval  SWS_OK     : 停止转换成功
 * \retval -SWS_EINVAL ：无效参数
 */
static inline
int sws_adc_rate_get (sws_adc_handle_t handle, int chan, uint32_t  *p_rate)
{
    return handle->p_funcs->pfn_rate_get(handle->p_drv, chan, p_rate);
}


/**
 * \brief 设置ADC的采样率，单位Hz
 *
 * \parsws[in]  handle : ADC标准服务操作句柄
 * \parsws[in]  chan   ：停止转换的通道
 * \parsws[out] rate   ：设置采样率
 *
 * \retval  SWS_OK     : 停止转换成功
 * \retval -SWS_EINVAL ：无效参数
 */
static inline
int sws_adc_rate_set (sws_adc_handle_t handle, int chan, uint32_t rate)
{
    return handle->p_funcs->pfn_rate_set(handle->p_drv, chan, rate);
}

/**
 * \brief 读取指定通道的ADC转换值（读取结束后才返回）
 *
 * \parsws[in] handle   : ADC标准服务操作句柄
 * \parsws[in] chan     : ADC通道号
 * \parsws[in] p_val    : 转换结果存放的缓冲区，数据右对齐
 * \parsws[in] length   : 缓冲区的长度
 *
 * \retval   SWS_OK     : 操作成功
 * \retval  -SWS_ENXIO  : ADC通道号不存在
 * \retval  -SWS_EINVAL : 无效参数
 */
int sws_adc_read(sws_adc_handle_t  handle, 
                int              chan, 
                void            *p_val,
                uint32_t         length);

/**
 * \brief 将ADC转换结果值转换为电压值
 *
 * \parsws[in] handle : ADC标准服务操作句柄
 * \parsws[in] chan   : ADC通道号
 * \parsws[in] val    : 转换结果采样值存放的缓冲区
 *     
 * \note val应为右对齐转换得到的数据
 *
 * \note 仅支持默认情况下右对齐转换得到的数据， 可以使用如下方式得到
 *       采样值对应的电压值：
 *  adc_bits = sws_adc_bits_get(handle,chan);
 *  ref_mv   = sws_adc_vref_get(handle,chan);
 *
 *  电压值 = ref_mv * 采样值 / ((1 << adc_bits) - 1);
 */
#define SWS_ADC_VAL_TO_MV(handle, chan, val)   \
    ((sws_adc_vref_get(handle,chan) * val) /    \
    ((1 << sws_adc_bits_get(handle,chan)) - 1))

/**
 * @}
 */

#ifdef __cplusplus
}
#endif

#endif /* __SWS_ADC_H */

/* end of file */
