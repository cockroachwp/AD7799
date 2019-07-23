/**
 * \file
 * \brief ad7705软件包实现
 *
 * \internal
 * \par Modification history
 * - 1.00 19-06-10  lqy, first implementation.
 * \endinternal
 */

#include "stdlib.h"
#include "sws_adc.h"
#include "sws_errno.h"
#include "sws_ad7705.h"

/**
 * \brief AD7705读指令和写指令宏定义
 */

/*****************************************************************************
 * 静态声明
 ****************************************************************************/

static u8 a = 0;

 /** \brief AD7705 adc时序 */
void __sws_ad7705_ad_clk (sws_ad7705_dev_t  *p_dev);


static void SPI_Write(sws_ad7705_dev_t *p_dev, u8 ad_register);

static u16 SPI_Read( sws_ad7705_dev_t *p_dev);

 /** \brief 启动ADC转换 */
 static int __pfn_adc_start (void                     *p_drv,
                              int                      chan,
                              sws_adc_buf_desc_t      *p_desc,
                              uint32_t                 desc_num,
                              uint32_t                 count,
                              uint32_t                 flags,
                              sws_adc_seq_cb_t         pfn_callback,
                              void                    *p_arg);

 /** \brief 停止转换 */
 static int __pfn_adc_stop (void *p_drv, int chan);

 /** \brief 获取ADC的采样率    */
 static int __pfn_adc_rate_get (void       *p_drv,
                                 int         chan,
                                 uint32_t   *p_rate);

 /** \brief 设置ADC的采样率，实际采样率可能存在差异 */
 static int __pfn_adc_rate_set (void     *p_drv,
                                 int       chan,
                                 uint32_t  rate);

 /** \brief 获取ADC转换精度 */
 static uint32_t __pfn_bits_get (void *p_drv, int chan);

 /** \brief 获取ADC参考电压 */
 static uint32_t __pfn_vref_get (void *p_drv, int chan);

 /**
  * \brief ADC服务函数
  */
 static const struct sws_adc_drv_funcs __g_adc_drvfuncs = {
     __pfn_adc_start,
     __pfn_adc_stop,
     __pfn_adc_rate_get,
     __pfn_adc_rate_set,
     __pfn_bits_get,
     __pfn_vref_get
 };

/*******************************************************************************
* Function Name  : Delay_7705
* Description    : 延时函数
* Input          : timecount - 延时参数(0~65535)
* Output         : None
* Return         : None
*******************************************************************************/
static void __delay_7705(uint16_t timecount)
{
  u8 i;
  while(timecount--)
  {
    i = 8;
    while(--i);
  }
}

/** \brief 启动ADC转换 */
static int __pfn_adc_start (void                    *p_drv,
                             int                     chan,
                             sws_adc_buf_desc_t     *p_desc,
                             uint32_t                desc_num,
                             uint32_t                count,
                             uint32_t                flags,
                             sws_adc_seq_cb_t        pfn_callback,
                             void                   *p_arg)
{
    sws_ad7705_dev_t *p_dev;
    uint32_t i = 0;
    static uint16_t  data;

    if (NULL == p_drv) {
        return -SWS_EINVAL;
    }
    uint32_t * p_buf = (uint32_t*)p_desc->p_buf;
    p_dev  = (sws_ad7705_dev_t *)p_drv;

    p_dev->p_desc       = p_desc;
    p_dev->desc_num     = desc_num;
    p_dev->count        = count;
    p_dev->flags        = flags;
    p_dev->pfn_callback = pfn_callback;
    p_dev->p_arg        = p_arg;

   // sws_ad7705_ch_set(p_dev,chan);

    for (i = 0; i < p_desc->length; i++) {
 //      while((__read_status_reg7705(p_dev) & 0x80));
       //*p_buf++  = __read_data_reg7705(p_dev);

         while(GPIO_ReadInputDataBit(p_dev->p_devinfo->out_pin->POR, p_dev->p_devinfo->cs->PIN));
//        while(1) {
//            if (a == 1) {
//              a == 0;
//              break;
//             }
//        }


        SPI_Write( p_dev, SWS_AD7705_DATA_REG     |
                          SWS_AD7705_CHANNEL_1    |
                          SWS_AD7705_READ);

        // __delay_7705(200);
        //data_h =  SPI_Read( p_dev);
       // data_l =  SPI_Read( p_dev);
	    *p_buf++  = (uint32_t)SPI_Read( p_dev);

    }

    if ((i >= p_desc->length) && (p_desc->pfn_complete != NULL)) {
        p_desc->pfn_complete(p_desc->p_arg, SWS_OK);
    }

    return SWS_OK;
}

/** \brief 停止转换 */
static int __pfn_adc_stop (void *p_drv, int chan)
{

    return -SWS_EPERM;
}

/** \brief 获取ADC的采样率    */
static int __pfn_adc_rate_get (void       *p_drv,
                               int         chan,
                               uint32_t   *p_rate)
{
    sws_ad7705_dev_t *p_dev;

    if (NULL == p_drv) {
        return -SWS_EINVAL;
    }

    p_dev = (sws_ad7705_dev_t *)p_drv;

    //*p_rate = sws_ad7705_out_speed_get(p_dev);

    return SWS_OK;
}

/** \brief 设置ADC的采样率，实际采样率可能存在差异 */
static int __pfn_adc_rate_set (void     *p_drv,
                               int       chan,
                               uint32_t  rate)
{
//    sws_ad7705_dev_t *p_dev;
//
//    if (NULL == p_drv) {
//        return -SWS_EINVAL;
//    }
//
//    p_dev = (sws_ad7705_dev_t *)p_drv;
//
//    switch(p_dev->out_speed){
//
//    case 470:
//        p_dev->mode = (p_dev->mode & (~0x0f)) | SWS_AD7705_MODE_470HZ;
//        break;
//    case 242:
//        p_dev->mode = (p_dev->mode & (~0x0f)) | SWS_AD7705_MODE_242HZ;
//        break;
//    case 123:
//        p_dev->mode = (p_dev->mode & (~0x0f)) | SWS_AD7705_MODE_123HZ;
//        break;
//    case 62:
//        p_dev->mode = (p_dev->mode & (~0x0f)) | SWS_AD7705_MODE_62HZ;
//        break;
//    case 50:
//        p_dev->mode = (p_dev->mode & (~0x0f)) | SWS_AD7705_MODE_50HZ;
//        break;
//    case 39:
//        p_dev->mode = (p_dev->mode & (~0x0f)) | SWS_AD7705_MODE_39HZ;
//        break;
//    case 33:
//        p_dev->mode = (p_dev->mode & (~0x0f)) | SWS_AD7705_MODE_33_2HZ;
//        break;
//    case 20:
//        p_dev->mode = (p_dev->mode & (~0x0f)) | SWS_AD7705_MODE_19_6HZ;
//        break;
//    case 17:
//        p_dev->mode = (p_dev->mode & (~0x0f)) | SWS_AD7705_MODE_16_7HZ;
//        break;
//    case 13:
//        p_dev->mode = (p_dev->mode & (~0x0f)) | SWS_AD7705_MODE_12_5HZ;
//        break;
//    case 10:
//        p_dev->mode = (p_dev->mode & (~0x0f)) | SWS_AD7705_MODE_10HZ;
//        break;
//    case 8:
//        p_dev->mode = (p_dev->mode & (~0x0f)) | SWS_AD7705_MODE_8_33HZ;
//        break;
//    case 6:
//        p_dev->mode = (p_dev->mode & (~0x0f)) | SWS_AD7705_MODE_6_25HZ;
//        break;
//    case 4:
//        p_dev->mode = (p_dev->mode & (~0x0f)) | SWS_AD7705_MODE_4_17HZ;
//        break;
//
//    default:
//        return -SWS_EPERM;
//    }
//
//    sws_ad7705_out_speed_set(p_dev, p_dev->mode);
//
//    return SWS_OK;
}

/** \brief 获取ADC转换精度 */
static uint32_t __pfn_bits_get (void *p_drv, int chan)
{

    if (NULL == p_drv) {
        return (uint32_t)(-SWS_EINVAL);
    }

   return 16;
}

/** \brief 获取ADC参考电压 */
static uint32_t __pfn_vref_get (void *p_drv, int chan)
{
    sws_ad7705_dev_t *p_dev;

    if (NULL == p_drv) {
        return (uint32_t)(-SWS_EINVAL);
    }

    p_dev = (sws_ad7705_dev_t *)p_drv;

    return p_dev->p_devinfo->vref;
}


///** \brief
// * Function Name  : Set_CS_7705
// * Description    : 设置AD7705的CS为高电平(1)或低电平(0)
// * Input          : chipn - AD7705芯片编号：0～(AD7705_PCS-1)，最大3
// *                    val - CS的电平值: 0=低电平; 1=高电平
// * Output         : None
// * Return         : None
// */
//static void __set_cs_7705(sws_ad7705_dev_t  *p_dev,u8 val)
//{
//    if(val == 0) {
//        GPIO_ResetBits(p_dev->p_devinfo->cs->POR, p_dev->p_devinfo->cs->PIN); //AD7705_CS_L;
//    } else {
//        GPIO_SetBits(p_dev->p_devinfo->cs->POR, p_dev->p_devinfo->cs->PIN);   //AD7705_CS_H;
//    }
//}


/******************************************************************************/
/*                 STM32F10x Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f10x_xx.s).                                            */
/******************************************************************************/



/**
 * \brief 写寄存器
 */
//SPI Write and Read
static void SPI_Write(sws_ad7705_dev_t *p_dev, u8 ad_register)
{
	u8 i = 0;
	u8 temp = 0x01<<7;
	for(;i<8;i++)
	{
		if((temp & ad_register) == 0)
			GPIO_ResetBits(p_dev->p_devinfo->in_pin->POR,p_dev->p_devinfo->in_pin->PIN); //AD7705_DIN_L;
		else
			GPIO_SetBits(p_dev->p_devinfo->in_pin->POR,p_dev->p_devinfo ->in_pin->PIN);   //AD7705_DIN_H;
		temp = temp>>1;

		GPIO_ResetBits(p_dev->p_devinfo->clk->POR,p_dev->p_devinfo->clk->PIN);  //AD7705_CLK_L;
		__delay_7705(p_dev->p_devinfo->low_clk_delay);
		GPIO_SetBits(p_dev->p_devinfo->clk->POR,p_dev->p_devinfo->clk->PIN);    //AD7705_CLK_H;
		__delay_7705(p_dev->p_devinfo->high_clk_delay);
	}
    GPIO_SetBits(p_dev->p_devinfo->in_pin->POR,p_dev->p_devinfo->in_pin->PIN);   //AD7705_DIN_H;

    __delay_7705(200);
}


static u16 SPI_Read( sws_ad7705_dev_t *p_dev)
{
	u8 i = 0;
	u16 Data_Receive = 0;
	for(;i<16;i++)
	{
		GPIO_ResetBits(p_dev->p_devinfo->clk->POR,p_dev->p_devinfo->clk->PIN);  //AD7705_CLK_L;
		__delay_7705(p_dev->p_devinfo->low_clk_delay);
		GPIO_SetBits(p_dev->p_devinfo->clk->POR,p_dev->p_devinfo->clk->PIN);    //AD7705_CLK_H;


		if(GPIO_ReadInputDataBit(p_dev->p_devinfo->out_pin->POR, p_dev->p_devinfo->out_pin->PIN) == 0)
			Data_Receive = Data_Receive<<1;
		else
		{
			Data_Receive = Data_Receive<<1;
			Data_Receive += 0x01;
		}
        __delay_7705(p_dev->p_devinfo->high_clk_delay);
	}
	return Data_Receive;
}


 /******************************************************************************
** 函数名称 ：AD7705_Reset
** 函数功能 ：AD7705复位
** 入口函数 ：无
** 出口函数 ：无
*******************************************************************************/
void AD7705_Reset( )
{
  GPIO_ResetBits(GPIOC, GPIO_Pin_4);
  __delay_7705(200);
  GPIO_SetBits(GPIOC, GPIO_Pin_4);
  __delay_7705(200);
}


/*******************************************************************************
* Function Name  : EXTI4_IRQHandler
* Description    : This function handles External interrupt Line 4 request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/

static sws_ad7705_dev_t *p_dev1 = NULL;

void EXTI4_IRQHandler(void)
{
  /* Clear the EXTI line 12 pending bit */
  EXTI_ClearITPendingBit(EXTI_Line4);
  //PressureSensorReadTime = 0;
  //ReadPressure();

   a = 1;

}


void __hard_ad7705(sws_ad7705_dev_t *p_dev)
{
//    static u16 a = 0;
//
   AD7705_Reset();

//    SPI_Write(p_dev, SWS_AD7705_SET_REG | SWS_AD7705_CHANNEL_1);
//	SPI_Write(p_dev, 0);
//
//    SPI_Write(p_dev, SWS_AD7705_CLK_REG | SWS_AD7705_CHANNEL_1);
//	SPI_Write(p_dev, 0);
//
//    SPI_Write( p_dev, SWS_AD7705_DATA_REG     |
//                      SWS_AD7705_CHANNEL_1    |
//                      SWS_AD7705_READ);
//
//    a = (u16)SPI_Read( p_dev ) << 8;
//    a = a + SPI_Read( p_dev );

    u8 i;
    for(i=0;i<4;i++) {//发送32个空时钟
        SPI_Write( p_dev, 0xff);
    }

    SPI_Write( p_dev, SWS_AD7705_CLK_REG      |
                      SWS_AD7705_CHANNEL_1    |
                      SWS_AD7705_WRITE);
    __delay_7705(200);
	SPI_Write( p_dev, SWS_AD7705_MODE_50HZ    |
                      SWS_AD7705_CLK_OUT);

   	SPI_Write( p_dev, SWS_AD7705_SET_REG      |
                      SWS_AD7705_CHANNEL_1    |
                      SWS_AD7705_WRITE);
	SPI_Write( p_dev, SWS_AD7705_MODE_CALIB_SELF |
                      SWS_AD7705_CONFIG_GAIN_32  |
                      SWS_AD7705_CONFIG_BIPOLAR);


// //   __delay_7705(20000);
//    u8 i;
//    for(i=0;i<4;i++) {//发送32个空时钟
//    SPI_Write( p_dev, 0xff);
//    //__delay_7705(20000);
//    //delay_ms(20);
//    }
//
//    // __delay_7705(20000);
//    SPI_Write( p_dev, 0x20);
//  //  delay_ms(100);
//   // __delay_7705(20000);
//    SPI_Write( p_dev, 0x12);
//   // delay_ms(100);
//   // __delay_7705(20000);
//    //__delay_7705(20000);
//    SPI_Write( p_dev, 0x10);
//   // delay_ms(100);
//   // __delay_7705(20000);
//    SPI_Write( p_dev, 0x68);
//   // delay_ms(100);
//   // __delay_7705(20000);

}



/**
 * \brief 初始化函数
 */
sws_adc_handle_t sws_ad7705_init(sws_ad7705_dev_t              *p_dev,
                                 const sws_ad7705_dev_info_t   *p_devinfo)
{
    uint16_t mode, config;
    u8  p_buf[2] = {0};

    /* 验证参数有效性 */
    if ((p_dev == NULL) || (p_devinfo == NULL)) {
        return NULL;
    }
    /* 验证参数有效性 */
    if (p_devinfo->pfn_plfm_init != NULL) {
         p_devinfo->pfn_plfm_init();
    }
     p_dev1 =    p_dev;
    //p_dev->pga = p_devinfo->;
    //p_dev->out_speed = SWS_AD7705_MODE_470HZ;

    p_dev->adc_serve.p_funcs = &__g_adc_drvfuncs;
    p_dev->adc_serve.p_drv   = p_dev;
    p_dev->pfn_callback      = NULL;
    p_dev->p_desc            = NULL;
    p_dev->p_arg             = NULL;
    p_dev->desc_num          = 0;
    p_dev->flags             = 0;
    p_dev->count             = 0;
    p_dev->seq_cnt           = 0;
    p_dev->desc_index        = 0;
    p_dev->conv_cnt          = 0;
    p_dev->p_devinfo         = p_devinfo;

    //__reset_ad7705(p_dev);


    __hard_ad7705(p_dev);


    return &p_dev->adc_serve;
}

/**
 * \brief 解初始化函数
 */
void sws_ad7705_deinit (sws_ad7705_adc_handle_t handle)
{
    handle->adc_serve.p_funcs = NULL;
    handle->adc_serve.p_drv   = NULL;
    handle->pfn_callback      = NULL;
    handle->p_desc            = NULL;
    handle->p_arg             = NULL;
    handle->p_devinfo         = NULL;

    return ;
}

/**
 * \brief AD7705 配置寄存器pga写
 */
int sws_ad7705_pga_set(sws_ad7705_dev_t  *p_dev, uint16_t pga)
{
    uint8_t p_buf[2] = {0};

    if((p_dev->config & 0x0700) == pga) {
        return SWS_OK;
    }

    switch(p_dev->config & 0x0700){

    case SWS_AD7705_CONFIG_GAIN_1:
        p_dev->config = (p_dev->config & (~0x0700)) | SWS_AD7705_CONFIG_GAIN_1;
        break;
    case SWS_AD7705_CONFIG_GAIN_2:
        p_dev->config = (p_dev->config & (~0x0700)) | SWS_AD7705_CONFIG_GAIN_2;
        break;
    case SWS_AD7705_CONFIG_GAIN_4:
        p_dev->config = (p_dev->config & (~0x0700)) | SWS_AD7705_CONFIG_GAIN_4;
        break;
    case SWS_AD7705_CONFIG_GAIN_8:
        p_dev->config = (p_dev->config & (~0x0700)) | SWS_AD7705_CONFIG_GAIN_8;
        break;
    case SWS_AD7705_CONFIG_GAIN_16:
        p_dev->config = (p_dev->config & (~0x0700)) | SWS_AD7705_CONFIG_GAIN_16;
        break;
    case SWS_AD7705_CONFIG_GAIN_32:
        p_dev->config = (p_dev->config & (~0x0700)) | SWS_AD7705_CONFIG_GAIN_32;
        break;
    case SWS_AD7705_CONFIG_GAIN_64:
        p_dev->config = (p_dev->config & (~0x0700)) | SWS_AD7705_CONFIG_GAIN_64;
        break;
    case SWS_AD7705_CONFIG_GAIN_128:
        p_dev->config = (p_dev->config & (~0x0700)) | SWS_AD7705_CONFIG_GAIN_128;
        break;
    default:
        return -SWS_EPERM;
    }

    //写config寄存器
    p_buf[0] = p_dev->config >> 8;   //H8
    p_buf[1] = p_dev->config;        //L8
//    __write_reg7705(p_dev,0x10, 2, p_buf);	//写config寄存器

    return SWS_OK;
}

/**
 * \brief AD7705 pga放大倍数读
 */
uint32_t sws_ad7705_pga_get(sws_ad7705_dev_t  *p_dev)
{

    switch(p_dev->config & 0x0700){

    case SWS_AD7705_CONFIG_GAIN_1:
        return 1;
    case SWS_AD7705_CONFIG_GAIN_2:
        return 2;
    case SWS_AD7705_CONFIG_GAIN_4:
        return 4;
    case SWS_AD7705_CONFIG_GAIN_8:
        return 8;
    case SWS_AD7705_CONFIG_GAIN_16:
        return 16;
    case SWS_AD7705_CONFIG_GAIN_32:
        return 32;
    case SWS_AD7705_CONFIG_GAIN_64:
        return 64;
    case SWS_AD7705_CONFIG_GAIN_128:
        return 128;
    default:
        return -SWS_EPERM;
    }
}

/**
 * \brief AD7705 配置寄存器ch写
 */
int sws_ad7705_ch_set(sws_ad7705_dev_t  *p_dev, uint32_t ch)
{
    uint8_t p_buf[2] = {0};

    if(p_dev->config & 0x07 == ch) {
        return SWS_OK;
    }

    p_dev->config = ( p_dev->config & (~0x07)) | ch;

    //写config寄存器
    p_buf[0] = p_dev->config >> 8;   //H8
    p_buf[1] = p_dev->config;        //L8
//    __write_reg7705(p_dev, 0x10, 2, p_buf); //写config寄存器

    return SWS_OK;
}

/**
 * \brief AD7705 配置寄存器ch读
 */
uint8_t sws_ad7705_ch_get(sws_ad7705_dev_t  *p_dev)
{
    uint8_t ch_reg = 0;

    ch_reg = p_dev->config & 0x03;

    return ch_reg;
}
/**
 * \brief AD7705 配置寄存器speed写
 */
int sws_ad7705_out_speed_set(sws_ad7705_dev_t  *p_dev, uint16_t speed)
{
    uint8_t p_buf[2] = {0};

    if(p_dev->mode & 0x0f == speed) {
        return SWS_OK;
    }

    p_dev->mode = (p_dev->mode & (~0x0f)) | speed;

    //写config寄存器
    p_buf[0] = p_dev->mode >> 8;   //H8
    p_buf[1] = p_dev->mode;        //L8
//    __write_reg7705(p_dev,0x08,2,p_buf);	//写mode寄存器

    return SWS_OK;

}

///**
// * \brief AD7705 adc输出速率读
// */
//int sws_ad7705_out_speed_get(sws_ad7705_dev_t  *p_dev)
//{
//    switch(p_dev->out_speed & 0x000f){
//
//    case SWS_AD7705_MODE_20HZ:
//        return 470;
//    case SWS_AD7705_MODE_242HZ:
//        return 242;
//    case SWS_AD7705_MODE_123HZ:
//        return 123;
//    case SWS_AD7705_MODE_62HZ:
//        return 62;
//    case SWS_AD7705_MODE_50HZ:
//        return 50;
//    case SWS_AD7705_MODE_39HZ:
//        return 39;
//    case SWS_AD7705_MODE_33_2HZ:
//        return 33;
//    case SWS_AD7705_MODE_19_6HZ:
//        return 20;
//    case SWS_AD7705_MODE_16_7HZ:
//        return 17;
//    case SWS_AD7705_MODE_16_70HZ:
//        return 17;
//    case SWS_AD7705_MODE_12_5HZ:
//        return 13;
//    case SWS_AD7705_MODE_10HZ:
//        return 10;
//    case SWS_AD7705_MODE_8_33HZ:
//        return 8;
//    case SWS_AD7705_MODE_6_25HZ:
//        return 6;
//    case SWS_AD7705_MODE_4_17HZ:
//        return 4;
//
//    default:
//        return -SWS_EPERM;
//    }
//}

/* end of file */
