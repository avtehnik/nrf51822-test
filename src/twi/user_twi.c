#include "user_twi.h"

#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>
#include "ble_advdata.h"
#include "nordic_common.h"
#include "softdevice_handler.h"
#include "bsp.h"
#include "app_timer.h"

#include "boards.h"
#include "app_util_platform.h"
#include "app_error.h"
#include "nrf_delay.h"
#include "user_uart.h"
#include "user_twi.h"

/*Pins to connect shield. */
#define TWI_SCL_PIN 14
#define TWI_SDA_PIN 13

static nrf_drv_twi_t const * m_twi_instance;

/**
 * @brief TWI events handler.
 */
//void twi_handler(nrf_drv_twi_evt_t const * p_event, void * p_context)
//{
////    ret_code_t err_code;
////    static sample_t m_sample;
//	 if(p_event->type == NRF_DRV_TWI_TX_DONE)
//	 {
//		 printf("twi_handler\r\n");
//	 }
//
//}

/**
 * @brief twi initialization.
 */
void twi_init_a(nrf_drv_twi_t const * const  p_instance)
{
	m_twi_instance = p_instance;
    ret_code_t err_code;
    const nrf_drv_twi_config_t twi_mma_7660_config = {
       .scl                = TWI_SCL_PIN,
       .sda                = TWI_SDA_PIN,
       .frequency          = NRF_TWI_FREQ_100K,
       .interrupt_priority = APP_IRQ_PRIORITY_HIGH
    };
    err_code = nrf_drv_twi_init(p_instance, &twi_mma_7660_config, NULL, NULL);
    APP_ERROR_CHECK(err_code);
    nrf_drv_twi_enable(p_instance);
}

