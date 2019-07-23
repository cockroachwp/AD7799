/**
 * \file
 * \brief ad7799�����ʵ��
 *
 * \internal
 * \par Modification history
 * - 1.00 19-06-10  lqy, first implementation.
 * \endinternal
 */

#include "stdlib.h"
#include "sws_adc.h"
#include "sws_errno.h"
#include "sws_ad7799.h"

/**
 * \brief AD7799��ָ���дָ��궨��
 */

/*****************************************************************************
 * ��̬����
 ****************************************************************************/

 /** \brief AD7799 adcʱ�� */
void __sws_ad7799_ad_clk (sws_ad7799_dev_t  *p_dev);

static u32 __read_data_reg7799(sws_ad7799_dev_t *p_dev);

static u8 __read_status_reg7799(sws_ad7799_dev_t *p_dev);

 /** \brief ����ADCת�� */
 static int __pfn_adc_start (void                   *p_drv,
                              int                     chan,
                              sws_adc_buf_desc_t      *p_desc,
                              uint32_t                desc_num,
                              uint32_t                count,
                              uint32_t                flags,
                              sws_adc_seq_cb_t         pfn_callback,
                              void                   *p_arg);

 /** \brief ֹͣת�� */
 static int __pfn_adc_stop (void *p_drv, int chan);

 /** \brief ��ȡADC�Ĳ�����    */
 static int __pfn_adc_rate_get (void       *p_drv,
                                 int         chan,
                                 uint32_t   *p_rate);

 /** \brief ����ADC�Ĳ����ʣ�ʵ�ʲ����ʿ��ܴ��ڲ��� */
 static int __pfn_adc_rate_set (void     *p_drv,
                                 int       chan,
                                 uint32_t  rate);

 /** \brief ��ȡADCת������ */
 static uint32_t __pfn_bits_get (void *p_drv, int chan);

 /** \brief ��ȡADC�ο���ѹ */
 static uint32_t __pfn_vref_get (void *p_drv, int chan);

 /**
  * \brief ADC������
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
* Function Name  : Delay_7799
* Description    : ��ʱ����
* Input          : timecount - ��ʱ����(0~65535)
* Output         : None
* Return         : None
*******************************************************************************/
static void __delay_7799(uint16_t timecount)
{
    while(timecount>0)
    timecount--;
}

/** \brief ����ADCת�� */
static int __pfn_adc_start (void                  *p_drv,
                             int                     chan,
                             sws_adc_buf_desc_t     *p_desc,
                             uint32_t                desc_num,
                             uint32_t                count,
                             uint32_t                flags,
                             sws_adc_seq_cb_t        pfn_callback,
                             void                   *p_arg)
{
    sws_ad7799_dev_t *p_dev;
    uint32_t i = 0;

    if (NULL == p_drv) {
        return -SWS_EINVAL;
    }
    uint32_t * p_buf = (uint32_t*)p_desc->p_buf;
    p_dev  = (sws_ad7799_dev_t *)p_drv;


    p_dev->p_desc       = p_desc;
    p_dev->desc_num     = desc_num;
    p_dev->count        = count;
    p_dev->flags        = flags;
    p_dev->pfn_callback = pfn_callback;
    p_dev->p_arg        = p_arg;

    sws_ad7799_ch_set(p_dev,chan);

    for (i = 0; i < p_desc->length; i++) {
       while((__read_status_reg7799(p_dev) & 0x80));
       *p_buf++  = __read_data_reg7799(p_dev);
    }

    if ((i >= p_desc->length) && (p_desc->pfn_complete != NULL)) {
        p_desc->pfn_complete(p_desc->p_arg, SWS_OK);
    }

    return SWS_OK;
}

/** \brief ֹͣת�� */
static int __pfn_adc_stop (void *p_drv, int chan)
{

    return -SWS_EPERM;
}

/** \brief ��ȡADC�Ĳ�����    */
static int __pfn_adc_rate_get (void       *p_drv,
                               int         chan,
                               uint32_t   *p_rate)
{
    sws_ad7799_dev_t *p_dev;

    if (NULL == p_drv) {
        return -SWS_EINVAL;
    }

    p_dev = (sws_ad7799_dev_t *)p_drv;

    *p_rate = sws_ad7799_out_speed_get(p_dev);

    return SWS_OK;
}

/** \brief ����ADC�Ĳ����ʣ�ʵ�ʲ����ʿ��ܴ��ڲ��� */
static int __pfn_adc_rate_set (void     *p_drv,
                               int       chan,
                               uint32_t  rate)
{
    sws_ad7799_dev_t *p_dev;

    if (NULL == p_drv) {
        return -SWS_EINVAL;
    }

    p_dev = (sws_ad7799_dev_t *)p_drv;

    switch(p_dev->out_speed){

    case 470:
        p_dev->mode = (p_dev->mode & (~0x0f)) | SWS_AD7799_MODE_470HZ;
        break;
    case 242:
        p_dev->mode = (p_dev->mode & (~0x0f)) | SWS_AD7799_MODE_242HZ;
        break;
    case 123:
        p_dev->mode = (p_dev->mode & (~0x0f)) | SWS_AD7799_MODE_123HZ;
        break;
    case 62:
        p_dev->mode = (p_dev->mode & (~0x0f)) | SWS_AD7799_MODE_62HZ;
        break;
    case 50:
        p_dev->mode = (p_dev->mode & (~0x0f)) | SWS_AD7799_MODE_50HZ;
        break;
    case 39:
        p_dev->mode = (p_dev->mode & (~0x0f)) | SWS_AD7799_MODE_39HZ;
        break;
    case 33:
        p_dev->mode = (p_dev->mode & (~0x0f)) | SWS_AD7799_MODE_33_2HZ;
        break;
    case 20:
        p_dev->mode = (p_dev->mode & (~0x0f)) | SWS_AD7799_MODE_19_6HZ;
        break;
    case 17:
        p_dev->mode = (p_dev->mode & (~0x0f)) | SWS_AD7799_MODE_16_7HZ;
        break;
    case 13:
        p_dev->mode = (p_dev->mode & (~0x0f)) | SWS_AD7799_MODE_12_5HZ;
        break;
    case 10:
        p_dev->mode = (p_dev->mode & (~0x0f)) | SWS_AD7799_MODE_10HZ;
        break;
    case 8:
        p_dev->mode = (p_dev->mode & (~0x0f)) | SWS_AD7799_MODE_8_33HZ;
        break;
    case 6:
        p_dev->mode = (p_dev->mode & (~0x0f)) | SWS_AD7799_MODE_6_25HZ;
        break;
    case 4:
        p_dev->mode = (p_dev->mode & (~0x0f)) | SWS_AD7799_MODE_4_17HZ;
        break;

    default:
        return -SWS_EPERM;
    }

    sws_ad7799_out_speed_set(p_dev, p_dev->mode);

    return SWS_OK;
}

/** \brief ��ȡADCת������ */
static uint32_t __pfn_bits_get (void *p_drv, int chan)
{

    if (NULL == p_drv) {
        return (uint32_t)(-SWS_EINVAL);
    }

   return 24;
}

/** \brief ��ȡADC�ο���ѹ */
static uint32_t __pfn_vref_get (void *p_drv, int chan)
{
    sws_ad7799_dev_t *p_dev;

    if (NULL == p_drv) {
        return (uint32_t)(-SWS_EINVAL);
    }

    p_dev = (sws_ad7799_dev_t *)p_drv;

    return p_dev->p_devinfo->vref;
}


/** \brief
 * Function Name  : Set_CS_7799
 * Description    : ����AD7799��CSΪ�ߵ�ƽ(1)��͵�ƽ(0)
 * Input          : chipn - AD7799оƬ��ţ�0��(AD7799_PCS-1)�����3
 *                    val - CS�ĵ�ƽֵ: 0=�͵�ƽ; 1=�ߵ�ƽ
 * Output         : None
 * Return         : None
 */
static void __set_cs_7799(sws_ad7799_dev_t  *p_dev,u8 val)
{
    if(val == 0) {
        GPIO_ResetBits(p_dev->p_devinfo->cs->POR, p_dev->p_devinfo->cs->PIN); //AD7799_CS_L;
    } else {
        GPIO_SetBits(p_dev->p_devinfo->cs->POR, p_dev->p_devinfo->cs->PIN);   //AD7799_CS_H;
    }
}

/** \brief
 * Function Name  : Wr1Byte7799
 * Description    : ��AD7799д��1�ֽ�(MSB��ǰ,������)��������
 * Input          : chipn - AD7799оƬ��ţ�0��(AD7799_PCS-1)�����3
 *                  data - д���8bit����
 * Output         : None
 * Return         : None
 */
static void __wr_byte7799(sws_ad7799_dev_t *p_dev, u8 data)  //ģ��SPI
{
    u8 xi;
    for(xi = 0; xi < 8; xi++) {
        GPIO_ResetBits(p_dev->p_devinfo->clk->POR,p_dev->p_devinfo->clk->PIN);         //AD7799_CLK_L;
        if((data & 0x80) == 0x80 ) {
          GPIO_SetBits(p_dev->p_devinfo->in_pin->POR,p_dev->p_devinfo->in_pin->PIN);   //AD7799_DIN_H;
        } else {
          GPIO_ResetBits(p_dev->p_devinfo->in_pin->POR,p_dev->p_devinfo->in_pin->PIN); //AD7799_DIN_L;
        }
        __delay_7799(p_dev->p_devinfo->low_clk_delay);
        data = data << 1;                                                              //����1λ
        GPIO_SetBits(p_dev->p_devinfo->clk->POR,p_dev->p_devinfo->clk->PIN);           //AD7799_CLK_H;
        __delay_7799(p_dev->p_devinfo->high_clk_delay);
    }
    GPIO_ResetBits(p_dev->p_devinfo->in_pin->POR,p_dev->p_devinfo->in_pin->PIN);       //AD7799_DIN_L;
}


/** \brief
 * Function Name  : Rd1Byte7799
 * Description    : ��1�ֽ�, AD7799�������������,������ʱ������Ч,MSB��ǰ
 * Input          : chipn - AD7799оƬ��ţ�0��(AD7799_PCS-1)�����3
 * Output         : None
 * Return         : 8λ����
 */
static u8 __readd_byte7799(sws_ad7799_dev_t *p_dev)  //ģ��SPI
{
    u8 xi,xd = 0;

    GPIO_SetBits(p_dev->p_devinfo->clk->POR,p_dev->p_devinfo->clk->PIN);    //AD7799_CLK_H;
    for(xi = 0,xd = 0; xi < 8; xi++) {
        __delay_7799(p_dev->p_devinfo->low_clk_delay);
        GPIO_ResetBits(p_dev->p_devinfo->clk->POR,p_dev->p_devinfo->clk->PIN);     //AD7799_CLK_L;
        __delay_7799(p_dev->p_devinfo->low_clk_delay);
        xd = xd << 1;   //����һλ
        if(GPIO_ReadInputDataBit(p_dev->p_devinfo->out_pin->POR, p_dev->p_devinfo->out_pin->PIN)) {
            xd++;
        }
        GPIO_SetBits(p_dev->p_devinfo->clk->POR,p_dev->p_devinfo->clk->PIN);    //AD7799_CLK_H;
    }
    return xd;
}

/**
 * \brief д�Ĵ���
 */
static void __write_reg7799(sws_ad7799_dev_t *p_dev,u8 xcomm, u8 xlen, u8 *s)  //ģ��SPI
{
    u8 xi;

    __set_cs_7799(p_dev, 0);           //AD7799_CS_L;
    __wr_byte7799(p_dev,xcomm & 0xbf); //bit6��Ϊ0 (0=д)
    for(xi = 0; xi < xlen; xi++)
    __wr_byte7799(p_dev,s[xi]);
    __set_cs_7799(p_dev, 1);           //AD7799_CS_H;
}


/** \brief
 * Function Name  : read_status_reg7799
 * Description    : ��AD7799��״̬�Ĵ���
 * Input          : chipn - AD7799оƬ��ţ�0��(AD7799_PCS-1)�����3
 * Output         : None
 * Return         : 8λ״ֵ̬
 */
static u8 __read_status_reg7799(sws_ad7799_dev_t *p_dev)  //ģ��SPI
{
    u8 xa;

    __set_cs_7799(p_dev, 0);      //AD7799_CS_L;
    __wr_byte7799(p_dev,0x40);
    xa = __readd_byte7799(p_dev);
    __set_cs_7799(p_dev, 1);      //AD7799_CS_H;
    return(xa);
}

/** \brief
 * Function Name  : read_data_reg7799
 * Description    : ��AD7799��24λ���ݼĴ���
 * Input          : chipn - AD7799оƬ��ţ�0��(AD7799_PCS-1)�����3
 * Output         : None
 * Return         : 24λADֵ
 */
static u32 __read_data_reg7799(sws_ad7799_dev_t *p_dev)  //ģ��SPI
{
    u32 xa;
    xa = 0;
     __set_cs_7799(p_dev, 0);      //AD7799_CS_L;
    __wr_byte7799(p_dev,0x58);
    xa = __readd_byte7799(p_dev);
    xa = (xa << 8) & 0xffffff00;   //���8bit��Ϊ0,׼����һ�ֽڵ�����
    xa = xa + __readd_byte7799(p_dev);
    xa = (xa << 8) & 0xffffff00;   //���8bit��Ϊ0,׼����һ�ֽڵ�����
    xa = xa + __readd_byte7799(p_dev);

    __set_cs_7799(p_dev, 1);      //AD7799_CS_H;
    return(xa);
}

/** \brief
 * Function Name  : Reset_AD7799
 * Description    : AD7799��λ: ����32��1,���ɶ�AD7799��λ
 * Input          : chipn - AD7799оƬ��ţ�0��(AD7799_PCS-1)�����3
 * Output         : None
 * Return         : None
 */
void __reset_ad7799(sws_ad7799_dev_t                    *p_dev)
{
    u8 xi;

     __set_cs_7799(p_dev, 0);      //AD7799_CS_L;
    for(xi = 0; xi < 4; xi++)      //����32��
    {
      __wr_byte7799(p_dev,0xff);
    }
    __set_cs_7799(p_dev, 1);      //AD7799_CS_H;
}


/*************************************************************************************************************************
* ����		:	bool AD7799_Calibration(AD7799_HANDLE *pHandle, AD7799_SPEED_TYPE speed)
* ����		:	AD7799У׼ģʽ
* ����		:	pHandle�����;speed��ת���ٶ�
* ����		:	FALSE:У׼ʧ��;TRUE:У׼�ɹ�
* ����		:	�ײ�궨��
* ����		:	cp1300@139.com
* ʱ��		:	2017-04-27
* ����޸�	: 	2018-06-06
* ˵��		: 	У׼ǰ��Ҫ������ͨ��,ʹ�õ����ڲ�У׼ģʽ
				Ƭ������У׼����������Ϊ128������½��С�Ϊ��������������ÿ�ε�ͨ��������ı䶼��Ҫ��������У׼��
*************************************************************************************************************************/
int AD7799_Calibration(sws_ad7799_dev_t     *p_dev, u8 ch)
{
    u8 delay = 0;
    u8 p_buf[2] = {0};

    sws_ad7799_ch_set(p_dev, ch);

    p_dev->mode =  (p_dev->mode & (0x1fff) | (AD7799_MODE_IDLE << 13)) ; //�Ƚ������ģʽ
    p_buf[0]    = p_dev->mode >> 8;      //H8
    p_buf[1]    = p_dev->mode;           //L8
    __write_reg7799(p_dev,0x08,2,p_buf); //дMODE�Ĵ���:

    p_dev->mode =  (p_dev->mode & (0x1fff) | (AD7799_MODE_ZEROCALI << 13));
    p_buf[0]    = p_dev->mode >> 8;      //H8s
    p_buf[1]    = p_dev->mode;           //L8
    __write_reg7799(p_dev,0x08,2,p_buf); //дMODE�Ĵ���:

    __set_cs_7799(p_dev, 0);             //AD7799_CS_L;

	if(GPIO_ReadInputDataBit(p_dev->p_devinfo->out_pin->POR, p_dev->p_devinfo->out_pin->PIN)==0) {
		//У׼ʧ�ܣ���ǰ�е͵�ƽ!
        __set_cs_7799(p_dev, 1);      //AD7799_CS_L;
		return -SWS_EINVAL;
	}
	delay = 0;
	while(GPIO_ReadInputDataBit(p_dev->p_devinfo->out_pin->POR, p_dev->p_devinfo->out_pin->PIN)==1) {
		delay_ms(10);
		delay ++;
		if(delay > 100) {
            //У׼ʧ�ܣ���ǰ�е͵�ƽ!
            __set_cs_7799(p_dev, 1);    //AD7799_CS_L;
			return -SWS_EINVAL;
		}
	}

	__set_cs_7799(p_dev, 1);            //AD7799_CS_L;
    p_dev->mode =  (p_dev->mode & (0x1fff) | (AD7799_MODE_FULLCALI << 13));
    p_buf[0]    = p_dev->mode >> 8;      //H8
    p_buf[1]    = p_dev->mode;           //L8
    __write_reg7799(p_dev,0x08,2,p_buf); //дMODE�Ĵ���:

    __set_cs_7799(p_dev, 0);             //AD7799_CS_L;
	if(GPIO_ReadInputDataBit(p_dev->p_devinfo->out_pin->POR, p_dev->p_devinfo->out_pin->PIN)==0) {
		//У׼ʧ��1����ǰ�е͵�ƽ!
        __set_cs_7799(p_dev, 1);     //AD7799_CS_L;
		return -SWS_EINVAL;
	}
	delay = 0;
	while(GPIO_ReadInputDataBit(p_dev->p_devinfo->out_pin->POR, p_dev->p_devinfo->out_pin->PIN)==1) {
		delay_ms(10);
		delay ++;
		if(delay > 100) {
            __set_cs_7799(p_dev, 1); //AD7799_CS_L;
			return -SWS_EINVAL;
		}
	}
    __set_cs_7799(p_dev, 1);         //AD7799_CS_L;

	return SWS_OK;
}

/**
 * \brief ��ʼ������
 */
sws_adc_handle_t sws_ad7799_init(sws_ad7799_dev_t              *p_dev,
                                 const sws_ad7799_dev_info_t   *p_devinfo)
{
    uint16_t mode, config;
    u8  p_buf[2] = {0};

    /* ��֤������Ч�� */
    if ((p_dev == NULL) || (p_devinfo == NULL)) {
        return NULL;
    }
    /* ��֤������Ч�� */
    if (p_devinfo->pfn_plfm_init != NULL) {
         p_devinfo->pfn_plfm_init();
    }

    p_dev->pga = SWS_AD7799_CONFIG_GAIN_1;
    p_dev->out_speed = SWS_AD7799_MODE_470HZ;

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

    __reset_ad7799(p_dev);

    mode   = 0;
    config = 0;

    /*��ʼ��config�Ĵ���*/
    mode = mode                     |
           p_dev->p_devinfo->mode   |
           SWS_AD7799_MODE_33_2HZ;

    p_buf[0] = mode >> 8;   //H8
    p_buf[1] = mode;        //L8
    __write_reg7799(p_dev,0x08,2,p_buf); //дMODE�Ĵ���:
    p_dev->mode = mode;

    config = config                    |
             SWS_AD7799_CONFIG_BUF_EN  |
             p_dev->p_devinfo->polar   |
             SWS_AD7799_CHANNEL_1      |
             SWS_AD7799_CONFIG_GAIN_1;

    p_dev->config = config;

    //дconfig�Ĵ���
    p_buf[0] = config >> 8;   //H8
    p_buf[1] = config;        //L8
    __write_reg7799(p_dev,0x10,2,p_buf);	//дconfig�Ĵ���

    if (AD7799_Calibration(p_dev, 0) != 0) {
       return NULL;
    }
    if (AD7799_Calibration(p_dev, 1) != 0) {
       return NULL;
    }
    if (AD7799_Calibration(p_dev, 2) != 0) {
       return NULL;
    }

    //p_dev->mode = p_dev->mode| SWS_AD7799_MODE_CONTINUE;
    p_dev->mode =  (p_dev->mode & (0x1fff) | ( p_dev->p_devinfo->mode << 13));
    p_buf[0] = p_dev->mode >> 8;   //H8
    p_buf[1] = p_dev->mode;        //L8
    __write_reg7799(p_dev,0x08, 2, p_buf); //дMODE�Ĵ���:

    return &p_dev->adc_serve;
}

/**
 * \brief ���ʼ������
 */
void sws_ad7799_deinit (sws_ad7799_adc_handle_t handle)
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
 * \brief AD7799 ���üĴ���pgaд
 */
int sws_ad7799_pga_set(sws_ad7799_dev_t  *p_dev, uint16_t pga)
{
    uint8_t p_buf[2] = {0};

    if((p_dev->config & 0x0700) == pga) {
        return SWS_OK;
    }

    switch(p_dev->config & 0x0700){

    case SWS_AD7799_CONFIG_GAIN_1:
        p_dev->config = (p_dev->config & (~0x0700)) | SWS_AD7799_CONFIG_GAIN_1;
        break;
    case SWS_AD7799_CONFIG_GAIN_2:
        p_dev->config = (p_dev->config & (~0x0700)) | SWS_AD7799_CONFIG_GAIN_2;
        break;
    case SWS_AD7799_CONFIG_GAIN_4:
        p_dev->config = (p_dev->config & (~0x0700)) | SWS_AD7799_CONFIG_GAIN_4;
        break;
    case SWS_AD7799_CONFIG_GAIN_8:
        p_dev->config = (p_dev->config & (~0x0700)) | SWS_AD7799_CONFIG_GAIN_8;
        break;
    case SWS_AD7799_CONFIG_GAIN_16:
        p_dev->config = (p_dev->config & (~0x0700)) | SWS_AD7799_CONFIG_GAIN_16;
        break;
    case SWS_AD7799_CONFIG_GAIN_32:
        p_dev->config = (p_dev->config & (~0x0700)) | SWS_AD7799_CONFIG_GAIN_32;
        break;
    case SWS_AD7799_CONFIG_GAIN_64:
        p_dev->config = (p_dev->config & (~0x0700)) | SWS_AD7799_CONFIG_GAIN_64;
        break;
    case SWS_AD7799_CONFIG_GAIN_128:
        p_dev->config = (p_dev->config & (~0x0700)) | SWS_AD7799_CONFIG_GAIN_128;
        break;
    default:
        return -SWS_EPERM;
    }

    //дconfig�Ĵ���
    p_buf[0] = p_dev->config >> 8;   //H8
    p_buf[1] = p_dev->config;        //L8
    __write_reg7799(p_dev,0x10, 2, p_buf);	//дconfig�Ĵ���

    return SWS_OK;
}

/**
 * \brief AD7799 pga�Ŵ�����
 */
uint32_t sws_ad7799_pga_get(sws_ad7799_dev_t  *p_dev)
{

    switch(p_dev->config & 0x0700){

    case SWS_AD7799_CONFIG_GAIN_1:
        return 1;
    case SWS_AD7799_CONFIG_GAIN_2:
        return 2;
    case SWS_AD7799_CONFIG_GAIN_4:
        return 4;
    case SWS_AD7799_CONFIG_GAIN_8:
        return 8;
    case SWS_AD7799_CONFIG_GAIN_16:
        return 16;
    case SWS_AD7799_CONFIG_GAIN_32:
        return 32;
    case SWS_AD7799_CONFIG_GAIN_64:
        return 64;
    case SWS_AD7799_CONFIG_GAIN_128:
        return 128;
    default:
        return -SWS_EPERM;
    }
}

/**
 * \brief AD7799 ���üĴ���chд
 */
int sws_ad7799_ch_set(sws_ad7799_dev_t  *p_dev, uint32_t ch)
{
    uint8_t p_buf[2] = {0};

    if(p_dev->config & 0x07 == ch) {
        return SWS_OK;
    }

    p_dev->config = ( p_dev->config & (~0x07)) | ch;

    //дconfig�Ĵ���
    p_buf[0] = p_dev->config >> 8;   //H8
    p_buf[1] = p_dev->config;        //L8
    __write_reg7799(p_dev, 0x10, 2, p_buf); //дconfig�Ĵ���

    return SWS_OK;
}

/**
 * \brief AD7799 ���üĴ���ch��
 */
uint8_t sws_ad7799_ch_get(sws_ad7799_dev_t  *p_dev)
{
    uint8_t ch_reg = 0;

    ch_reg = p_dev->config & 0x03;

    return ch_reg;
}
/**
 * \brief AD7799 ���üĴ���speedд
 */
int sws_ad7799_out_speed_set(sws_ad7799_dev_t  *p_dev, uint16_t speed)
{
    uint8_t p_buf[2] = {0};

    if(p_dev->mode & 0x0f == speed) {
        return SWS_OK;
    }

    p_dev->mode = (p_dev->mode & (~0x0f)) | speed;

    //дconfig�Ĵ���
    p_buf[0] = p_dev->mode >> 8;   //H8
    p_buf[1] = p_dev->mode;        //L8
    __write_reg7799(p_dev,0x08,2,p_buf);	//дmode�Ĵ���

    return SWS_OK;

}

/**
 * \brief AD7799 adc������ʶ�
 */
int sws_ad7799_out_speed_get(sws_ad7799_dev_t  *p_dev)
{
    switch(p_dev->out_speed & 0x000f){

    case SWS_AD7799_MODE_470HZ:
        return 470;
    case SWS_AD7799_MODE_242HZ:
        return 242;
    case SWS_AD7799_MODE_123HZ:
        return 123;
    case SWS_AD7799_MODE_62HZ:
        return 62;
    case SWS_AD7799_MODE_50HZ:
        return 50;
    case SWS_AD7799_MODE_39HZ:
        return 39;
    case SWS_AD7799_MODE_33_2HZ:
        return 33;
    case SWS_AD7799_MODE_19_6HZ:
        return 20;
    case SWS_AD7799_MODE_16_7HZ:
        return 17;
    case SWS_AD7799_MODE_16_70HZ:
        return 17;
    case SWS_AD7799_MODE_12_5HZ:
        return 13;
    case SWS_AD7799_MODE_10HZ:
        return 10;
    case SWS_AD7799_MODE_8_33HZ:
        return 8;
    case SWS_AD7799_MODE_6_25HZ:
        return 6;
    case SWS_AD7799_MODE_4_17HZ:
        return 4;

    default:
        return -SWS_EPERM;
    }
}

/* end of file */
