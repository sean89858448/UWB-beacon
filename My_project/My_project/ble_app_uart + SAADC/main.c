#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include "app_uart.h"
#include "app_error.h"
#include "nrf_delay.h"
#include "nrf.h"
#include "bsp.h"
#include "app_uart.h"

#include "nrf_pwr_mgmt.h"

#include "nrf_drv_saadc.h"
 
#define UART_TX_BUF_SIZE 256
#define UART_RX_BUF_SIZE 256 

#define SAMPLES_IN_BUFFER 6

void saadc_callback(nrf_drv_saadc_evt_t const *p_event){
    if (p_event->type == NRF_DRV_SAADC_EVT_DONE)
    {
        ret_code_t err_code;

        uint16_t ADC_SUM = 0;

        err_code = nrf_drv_saadc_buffer_convert(p_event->data.done.p_buffer, SAMPLES_IN_BUFFER);
        APP_ERROR_CHECK(err_code);

        for(int i = 0 ; i < SAMPLES_IN_BUFFER ; i++)
        {
            ADC_SUM += p_event->data.done.p_buffer[i] * 3600 >> 10;
        }

        ADC_SUM = ADC_SUM / 100;
        nrf_delay_ms(300);

    }
}

void saadc_init(void)
{
    ret_code_t err_code;

    nrf_saadc_channel_config_t channel_config = NRF_DRV_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_AIN2);
    

    channel_config.gain = NRF_SAADC_GAIN1_6;
    channel_config.reference = NRF_SAADC_REFERENCE_INTERNAL;

    err_code = nrf_drv_saadc_init(NULL,saadc_callback);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_saadc_channel_init(0, &channel_config);
    APP_ERROR_CHECK(err_code);
    
}

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

void uart_init()
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
        UART_BAUDRATE_BAUDRATE_Baud115200 
    };
    APP_UART_FIFO_INIT(&comm_params,
                         UART_RX_BUF_SIZE,
                         UART_TX_BUF_SIZE,
                         uart_error_handle,
                         APP_IRQ_PRIORITY_LOWEST,
                         err_code);
 
    APP_ERROR_CHECK(err_code);  
    
}


int main(void){
    nrf_saadc_value_t saadc_val;

    ret_code_t ret_code = nrf_pwr_mgmt_init();
    APP_ERROR_CHECK(ret_code);

    uart_init();
    saadc_init();

    while(true){
        nrf_pwr_mgmt_run();
    }
}