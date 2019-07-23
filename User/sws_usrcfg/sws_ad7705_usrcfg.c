
/**
 * \file
 * \brief  AD7705用户配置文件
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


/** \brief ADC平台初始化 */
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
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;    //  推挽输出
    GPIO_Init(ad7705_clk.POR, &GPIO_InitStructure);
    GPIO_SetBits(ad7705_clk.POR,ad7705_clk.PIN);

    GPIO_InitStructure.GPIO_Pin = ad7705_out.PIN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;    //  推挽输出
    GPIO_Init(ad7705_out.POR, &GPIO_InitStructure);
    GPIO_ResetBits(ad7705_out.POR,ad7705_out.PIN);

    GPIO_InitStructure.GPIO_Pin = ad7705_in.PIN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;    //  推挽输出
    GPIO_Init(ad7705_in.POR, &GPIO_InitStructure);
    GPIO_ResetBits(ad7705_in.POR, ad7705_in.PIN);

      /***************AD7705 时钟端口GPIO及TIM3配置*******************************/
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
    TIM_Cmd(TIM3, ENABLE);    //使能TIM3

    GPIO_InitStructure.GPIO_Pin = ad7705_cs.PIN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;       //  上拉输入
    GPIO_Init(ad7705_cs.POR, &GPIO_InitStructure);
    GPIO_SetBits(ad7705_cs.POR,ad7705_cs.PIN);

      /***************AD7705 DRDY引脚中断GPIO及EXTI配置****************************/
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
    GPIO_InitStructure.GPIO_Mode =GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource4);
    EXTI_InitStructure.EXTI_Line = EXTI_Line4;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);

    /***************AD7705的RESET端口及数据采集指示灯控制端口GPIO配置*************/
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 ;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    /***************AD7705的RESET端口及数据采集指示灯控制端口GPIO配置*************/
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 ;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

}

/** \brief 解除ADC平台初始化 */
static void __plfm_ad7705_deinit (void)
{

}




/** \brief ADC设备信息 */
static const sws_ad7705_dev_info_t __g_ad7705_devinfo = {

    /** \brief IO模拟SPI SCLK引脚选择 */
     &ad7705_clk,

    /**
     * \brief ad7705_out引脚选择
     * */
     &ad7705_out,

    /**
     * \brief ad7705_in引脚选择
     * */
     &ad7705_in,

    /**
     * \brief ad7705_cs引脚选择
     *
     * \note 如果硬件没有对DRDY/DOUT引脚分离，该引脚应该和out_pin相同
     * */
     &ad7705_cs,

     &ad7705_reset,


     SWS_AD7705_MODE_NAR,

     SWS_AD7705_CONFIG_BIPOLAR,

    /**
     * \brief ADC参考电压，单位：mV
     *
     * \note 该参考电压由具体的电路决定
     *
     */
    2490,

    /** \brief CLK时序高电平延时时间 ，微妙级别*/
    10,

    /** \brief CLK时序低电平延时时间 ，微妙级别*/
    10,

    /** \brief 进入低功耗模式clk高电平持续时间 ，微妙级别，理论值100us*/
    10,

    __plfm_ad7705_init,              /**< \brief ADC的平台初始化 */
    __plfm_ad7705_deinit             /**< \brief ADC的平台去初始化 */
};

static sws_ad7705_dev_t  __g_ad7705_dev;   /**< \brief 定义ADC 设备 */

/** \brief ADC实例初始化，获得ADC标准服务句柄 */
sws_adc_handle_t sws_ad7705_inst_init (void)
{
    return sws_ad7705_init(&__g_ad7705_dev, &__g_ad7705_devinfo);
}

/** \brief ADC实例解初始化 */
void sws_ad7705_inst_deinit (sws_adc_handle_t handle)
{
  //  sws_adc_deinit(handle);
}

/**
 * @}
 */

/* end of file */
