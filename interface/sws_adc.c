
/**
 * \file
 * \brief ADC implementation
 *
 * \internal
 * \par Modification history
 * - 1.00 19-06-11 tee, first implementation.
 * \endinternal
 */
#include "sws_adc.h"
#include "stdlib.h"

/*******************************************************************************/

volatile static int  g_adc_satus = 0;


/*******************************************************************************/

int sws_adc_read (sws_adc_handle_t  handle,
                  int               chan,
                  void             *p_val,
                  uint32_t          length)
{

    sws_adc_buf_desc_t desc;

    sws_adc_mkbufdesc(&desc, p_val, length, NULL,(void *) NULL);

    sws_adc_start(handle, chan, &desc, 1, 1, SWS_ADC_DATA_ALIGN_RIGHT, NULL, NULL);

    return g_adc_satus;
}

/* end of file */
