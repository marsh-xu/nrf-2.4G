/* Copyright (c) 2012 Nordic Semiconductor. All Rights Reserved.
*
* The information contained herein is property of Nordic Semiconductor ASA.
* Terms and conditions of usage are described in detail in NORDIC
* SEMICONDUCTOR STANDARD SOFTWARE LICENSE AGREEMENT.
*
* Licensees are granted free, non-transferable use of the information. NO
* WARRANTY of ANY KIND is provided. This heading must NOT be removed from
* the file.
*
* $LastChangedRevision: 15516 $
*/

/**
 * @file
 * @brief Enhanced Shockburst Transmitter (PTX) example
 * @defgroup esb_ptx_example Enhanced Shockburst Transmitter (PTX)
 * @{
 * @ingroup esb_03_examples
 *
 * This project requires that a device running the
 * @ref esb_ptx_example example be used as a counterpart for
 * receiving the data. This can be on either nRF51 device or a nRF24Lxx device
 * running the \b esb_ptx_example in the nRFgo SDK.
 *
 * This example sends a packet and adds a new packet to the TX queue every time
 * it receives an ACK. The contents of GPIO Port BUTTONS are
 * sent in the first payload byte (byte 0). The host sends an acknowledgement
 * that is received by the device. The contents of the first payload byte of
 * the acknowledgement is output on GPIO Port LEDS.
 */


#include "nrf_esb.h"
#include "nrf_gpio.h"

#include "nrf51.h"
#include "nrf51_bitfields.h"

#include "nrf_delay.h"

#include "SEGGER_RTT.h"
#include "app_util_platform.h"

/*****************************************************************************/
/** @name Configuration */
/*****************************************************************************/

// Define pipe
#define PIPE_NUMBER 0 ///< We use pipe 0 in this example

// Define payload length
#define TX_PAYLOAD_LENGTH NRF_ESB_CONST_MAX_PAYLOAD_LENGTH ///< We use 1 byte payload length when transmitting

#define MAX_RTC_TASKS_DELAY     47 
#define RTC1_IRQ_PRI            APP_IRQ_PRIORITY_LOW 

// Data and acknowledgement payloads
static uint8_t my_tx_payload[NRF_ESB_CONST_MAX_PAYLOAD_LENGTH]; ///< Payload to send to PRX.
static uint8_t my_rx_payload[NRF_ESB_CONST_MAX_PAYLOAD_LENGTH]; ///< Placeholder for received ACK payloads from PRX.

static uint8_t index = 0;

static uint32_t time_tick = 0;

static void radio_condig(void)
{
    nrf_esb_set_datarate(NRF_ESB_DATARATE_2_MBPS);
}


/*****************************************************************************/
/**
* @brief Main function.
*
* @return ANSI required int return type.
*/
/*****************************************************************************/
int main()
{
    // Initialize ESB
    (void)nrf_esb_init(NRF_ESB_MODE_PTX);

    (void)nrf_esb_enable();

    for (uint8_t i = 0; i < 32; i++)
    {
        my_tx_payload[i] = i;
    }

    NRF_CLOCK->TASKS_LFCLKSTART = 1;
    while((NRF_CLOCK->EVENTS_LFCLKSTARTED) == 0);


    NRF_RTC1->PRESCALER = 0;
    //NVIC_SetPriority(RTC1_IRQn, RTC1_IRQ_PRI);


    //NRF_RTC1->EVTENSET = RTC_EVTEN_COMPARE0_Msk;
    //NRF_RTC1->INTENSET = RTC_INTENSET_COMPARE0_Msk;

    //NVIC_ClearPendingIRQ(RTC1_IRQn);
    //NVIC_EnableIRQ(RTC1_IRQn);

    NRF_RTC1->TASKS_STOP = 1;
    nrf_delay_us(MAX_RTC_TASKS_DELAY);
    NRF_RTC1->TASKS_CLEAR = 1;
    nrf_delay_us(MAX_RTC_TASKS_DELAY);
    NRF_RTC1->TASKS_START = 1;
    nrf_delay_us(MAX_RTC_TASKS_DELAY);

    radio_condig();
    nrf_esb_datarate_t datarate = nrf_esb_get_datarate();
    nrf_esb_crc_length_t crc_len = nrf_esb_get_crc_length();
    SEGGER_RTT_printf(0, "datarate = %p\r\n",datarate);
    SEGGER_RTT_printf(0, "crc_len = %p\r\n",crc_len);


    // Add packet into TX queue
    my_tx_payload[0] = 0x55;
    my_tx_payload[1] = index;
    (void)nrf_esb_add_packet_to_tx_fifo(PIPE_NUMBER, my_tx_payload, NRF_ESB_CONST_MAX_PAYLOAD_LENGTH, NRF_ESB_PACKET_USE_ACK);
    index ++;

    while(1)
    {
        // Optionally set the CPU to sleep while waiting for a callback.
        // __WFI();
    }
}


/*****************************************************************************/
/** @name ESB callback function definitions  */
/*****************************************************************************/


// If an ACK was received, we send another packet.
void nrf_esb_tx_success(uint32_t tx_pipe, int32_t rssi){
    // Read buttons and load data payload into TX queue
    my_tx_payload[0] = 0x55;
    my_tx_payload[1] = index;
    (void)nrf_esb_add_packet_to_tx_fifo(PIPE_NUMBER, my_tx_payload, TX_PAYLOAD_LENGTH, NRF_ESB_PACKET_USE_ACK);
    time_tick = NRF_RTC1->COUNTER;
    SEGGER_RTT_printf(0, "%p :: Send success!  data:%p\r\n",time_tick, *(uint32_t*)&my_tx_payload[0]);
    index ++;
}


// If the transmission failed, send a new packet.
void nrf_esb_tx_failed(uint32_t tx_pipe){
    SEGGER_RTT_printf(0, "Send failed!\r\n");
    my_tx_payload[0] = 0x55;
    (void)nrf_esb_add_packet_to_tx_fifo(PIPE_NUMBER, my_tx_payload, TX_PAYLOAD_LENGTH, NRF_ESB_PACKET_USE_ACK);}


void nrf_esb_rx_data_ready(uint32_t rx_pipe, int32_t rssi){
    uint32_t my_rx_payload_length;
    time_tick = NRF_RTC1->COUNTER;
    // Pop packet and write first byte of the payload to the GPIO port.
    (void)nrf_esb_fetch_packet_from_rx_fifo(PIPE_NUMBER, my_rx_payload, &my_rx_payload_length);
    SEGGER_RTT_printf(0, "%p :: received success!  data:%p %p %p %p %p %p %p %p\r\n",time_tick, *(uint32_t*)&my_rx_payload[0], *(uint32_t*)&my_rx_payload[4], *(uint32_t*)&my_rx_payload[8], *(uint32_t*)&my_rx_payload[12], *(uint32_t*)&my_rx_payload[16], *(uint32_t*)&my_rx_payload[20], *(uint32_t*)&my_rx_payload[24], *(uint32_t*)&my_rx_payload[28]);
}

// Callbacks not needed in this example.
void nrf_esb_disabled(void)
{}

/** @} */
/** @} */



