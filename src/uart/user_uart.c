#include "user_uart.h"
#include "app_trace.h"

#define UART_TX_BUF_SIZE           256                                /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE           1                                  /**< UART RX buffer size. */
#define RX_PIN_NUMBER              28
#define TX_PIN_NUMBER              29
#define RTS_PIN_NUMBER             24
#define CTS_PIN_NUMBER             25


void uart_error_handle(app_uart_evt_t * p_event)
{
    if (p_event->evt_type == APP_UART_COMMUNICATION_ERROR)
    {
        APP_ERROR_HANDLER(p_event->data.error_communication);
    }
    else if (p_event->evt_type == APP_UART_FIFO_ERROR)
    {
        APP_ERROR_HANDLER(p_event->data.error_code);
    }
}




/**
 * @brief UART initialization.
 */
void uart_config(void)
{
    uint32_t err_code;
    const app_uart_comm_params_t comm_params =
    {
        RX_PIN_NUMBER,
        TX_PIN_NUMBER,
        RTS_PIN_NUMBER,
        CTS_PIN_NUMBER,
        APP_UART_FLOW_CONTROL_DISABLED,
        false,
        UART_BAUDRATE_BAUDRATE_Baud38400
    };

    APP_UART_FIFO_INIT(&comm_params,
                       UART_RX_BUF_SIZE,
                       UART_TX_BUF_SIZE,
					   uart_error_handle,
                       APP_IRQ_PRIORITY_LOW,
                       err_code);

    APP_ERROR_CHECK(err_code);
    printf("uart_config \r\n");

}
