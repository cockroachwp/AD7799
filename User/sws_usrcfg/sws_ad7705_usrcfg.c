
/**
 * \file
 * \brief  AD7705�û������ļ�
 * \sa sws_ad7705_usrcfg.c
 *
 * \internal
 * \par Modification history
 * - 1.00 19-06-10  wangpeng, first implementation.
 * \endinternal
 */
#include "stm32f10x.h"
#include "stm32f10x_exti.h"
#include "sws_ad7705.h"
#include "sws_adc.h"

/**
 * \copydoc sws_ad7705_usrcfg.c
 * @{
 */

static  sws_gpio_info_t  ad7705_clk = {
      GPIOA,
      GPIO_Pin_5
};

static  sws_gpio_info_t  ad7705_out = {
      GPIOA,
      GPIO_Pin_6
};

static  sws_gpio_info_t  ad7705_in = {
      GPIOA,
      GPIO_Pin_7
};

static  sws_gpio_info_t  ad7705_cs = {
      GPIOA,
      GPIO_Pin_4
};

static  sws_gpio_info_t  ad7705_reset = {
      GPIOC,
      GPIO_Pin_4
};


/** \brief ADCƽ̨��ʼ�� */
static void __plfm_ad7705_init (void)
{
    NVIC_InitTypeDef NVIC_InitStructure;

  /* Configure one bit for preemption priority */
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);

          /* Enable the EXTI4 global Interrupt */
    NVIC_InitStructure.NVIC_IRQChannel = EXTI4_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 4;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    EXTI_InitTypeDef EXTI_InitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA |
                           RCC_APB2Periph_GPIOB |
                           RCC_APB2Periph_GPIOC |
                           RCC_APB2Periph_AFIO, ENABLE);
      /* TIM2,TIM3,TIM4,USART2,SPI2 clock enable */
    RCC_APB1PeriphClockCmd( RCC_APB1Periph_TIM3, ENABLE);
    GPIO_InitStructure.GPIO_Pin = ad7705_clk.PIN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;    //  �������
    GPIO_Init(ad7705_clk.POR, &GPIO_InitStructure);
    GPIO_SetBits(ad7705_clk.POR,ad7705_clk.PIN);

    GPIO_InitStructure.GPIO_Pin = ad7705_out.PIN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;    //  �������
    GPIO_Init(ad7705_out.POR, &GPIO_InitStructure);
    GPIO_ResetBits(ad7705_out.POR,ad7705_out.PIN);

    GPIO_InitStructure.GPIO_Pin = ad7705_in.PIN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;    //  �������
    GPIO_Init(ad7705_in.POR, &GPIO_InitStructure);
    GPIO_ResetBits(ad7705_in.POR, ad7705_in.PIN);

      /***************AD7705 ʱ�Ӷ˿�GPIO��TIM3����*******************************/
    GPIO_InitStructure.GPIO_Pin=GPIO_Pin_0;
    GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    TIM_OCInitTypeDef  TIM_OCInitStructure;

    TIM_TimeBaseStructure.TIM_Period = 3;
    TIM_TimeBaseStructure.TIM_Prescaler = 8;//17;//72/18=4M
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = 2;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;
    TIM_OC3Init(TIM3, &TIM_OCInitStructure);
    TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Enable);
    TIM_Cmd(TIM3, ENABLE);    //ʹ��TIM3

    GPIO_InitStructure.GPIO_Pin = ad7705_cs.PIN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;       //  ��������
    GPIO_Init(ad7705_cs.POR, &GPIO_InitStructure);
    GPIO_SetBits(ad7705_cs.POR,ad7705_cs.PIN);

      /***************AD7705 DRDY�����ж�GPIO��EXTI����****************************/
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
    GPIO_InitStructure.GPIO_Mode =GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource4);
    EXTI_InitStructure.EXTI_Line = EXTI_Line4;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);

    /***************AD7705��RESET�˿ڼ����ݲɼ�ָʾ�ƿ��ƶ˿�GPIO����*************/
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 ;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    /***************AD7705��RESET�˿ڼ����ݲɼ�ָʾ�ƿ��ƶ˿�GPIO����*************/
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 ;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

}

/** \brief ���ADCƽ̨��ʼ�� */
static void __plfm_ad7705_deinit (void)
{

}




/** \brief ADC�豸��Ϣ */
static const sws_ad7705_dev_info_t __g_ad7705_devinfo = {

    /** \brief IOģ��SPI SCLK����ѡ�� */
     &ad7705_clk,

    /**
     * \brief ad7705_out����ѡ��
     * */
     &ad7705_out,

    /**
     * \brief ad7705_in����ѡ��
     * */
     &ad7705_in,

    /**
     * \brief ad7705_cs����ѡ��
     *
     * \note ���Ӳ��û�ж�DRDY/DOUT���ŷ��룬������Ӧ�ú�out_pin��ͬ
     * */
     &ad7705_cs,

     &ad7705_reset,


     SWS_AD7705_MODE_NAR,

     SWS_AD7705_CONFIG_BIPOLAR,

    /**
     * \brief ADC�ο���ѹ����λ��mV
     *
     * \note �òο���ѹ�ɾ���ĵ�·����
     *
     */
    2490,

    /** \brief CLKʱ��ߵ�ƽ��ʱʱ�� ��΢���*/
    10,

    /** \brief CLKʱ��͵�ƽ��ʱʱ�� ��΢���*/
    10,

    /** \brief ����͹���ģʽclk�ߵ�ƽ����ʱ�� ��΢�������ֵ100us*/
    10,

    __plfm_ad7705_init,              /**< \brief ADC��ƽ̨��ʼ�� */
    __plfm_ad7705_deinit             /**< \brief ADC��ƽ̨ȥ��ʼ�� */
};

static sws_ad7705_dev_t  __g_ad7705_dev;   /**< \brief ����ADC �豸 */

/** \brief ADCʵ����ʼ�������ADC��׼������ */
sws_adc_handle_t sws_ad7705_inst_init (void)
{
    return sws_ad7705_init(&__g_ad7705_dev, &__g_ad7705_devinfo);
}

/** \brief ADCʵ�����ʼ�� */
void sws_ad7705_inst_deinit (sws_adc_handle_t handle)
{
  //  sws_adc_deinit(handle);
}

/**
 * @}
 */

/* end of file */
