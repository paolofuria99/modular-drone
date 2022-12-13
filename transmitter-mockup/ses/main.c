/**
 * Copyright (c) 2014 - 2017, Nordic Semiconductor ASA
 * 
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 * 
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 * 
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 * 
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 * 
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 * 
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 * 
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * 
 */
/** @file
 * @defgroup nrf_dev_timer_example_main main.c
 * @{
 * @ingroup nrf_dev_timer_example
 * @brief Timer Example Application main file.
 *
 * This file contains the source code for a sample application using Timer0.
 *
 */
#include "main_conf.h"

/**
 * @brief Function for main application entry.
 */

const nrf_drv_timer_t TIMER_CH1 = NRF_DRV_TIMER_INSTANCE(0);
const nrf_drv_timer_t TIMER_CH2 = NRF_DRV_TIMER_INSTANCE(1);

channel_t *ch;
channel_t *ch1;
channel_t *ch2;

int main(void) {
   ch = channel_init_void();
   ch1 = channel_init_default(CH1, TIMER_CH1, gpiote_handler);
   ch2 = channel_init_default(PIN_DEBUG, TIMER_CH2, gpiote_handler);
  
  uint32_t err_code = NRF_SUCCESS;

  // ---------------- UART INIT ---------------------------------------- //
  boUART_Init();
  printf("Transmitter Module \r\n");
  printf("UART SET \r\n");
  // ----------------- DWM initialization ----------------------------------- //
  /* Setup DW1000 IRQ pin */
  nrf_gpio_cfg_input(DW1000_IRQ, NRF_GPIO_PIN_NOPULL); //irq

  /* Reset DW1000 */
  reset_DW1000();

  /* Set SPI clock to 2MHz */
  port_set_dw1000_slowrate();

  /* Init the DW1000 */
  if (dwt_initialise(DWT_LOADUCODE) == DWT_ERROR) {
    //Init of DW1000 Failed
    while (1) {
    };
  }
  port_set_dw1000_fastrate();
  dwt_configure(&dwt_config);

  // -------------------- End DWM configuration ----------------------------- //

  printf("DWM configured\r\n");

  //Configure all leds on board.
  //bsp_board_leds_init();

  // TIMER Configuration
  
  // You have to check that the timer reload does not occur during a measurement

  boUART_Init();
  nrf_drv_gpiote_in_event_enable(CH1, true);

  nrf_gpio_cfg_output(PIN_DEBUG);

  printf("Running main loop \r\n");
  while (1) {
    if (data_ready == 0b00000001) {
      channel_set_dc(ch1,  100 - (channel_get_t1(ch1) - channel_get_t2(ch1)) * to_dc);
      tx_msg[1] = channel_get_dc(ch1);
      data_ready = data_ready & 0b11111110; // Reset the last bit
      send_message(tx_msg);                 // Send data and wait
      nrf_gpio_pin_toggle(PIN_DEBUG);
      printf("Duty cycle is: %d[%%] \r\n",  channel_get_dc(ch1));
      printf("Ton duration: %d[ms] \r\n",  (channel_get_t1(ch1) - channel_get_t2(ch1))/1000);
    }
  }
}

// Function definition
static void send_message(uint8 *msg) {
  /* Write frame data to DW1000 and prepare transmission. See NOTE 4 below.*/
  dwt_writetxdata(sizeof(msg), msg, 0); /* Zero offset in TX buffer. */
  dwt_writetxfctrl(sizeof(msg), 0, 0);  /* Zero offset in TX buffer, no ranging. */

  dwt_starttx(DWT_START_TX_IMMEDIATE);

  /* Poll DW1000 until TX frame sent event set. See NOTE 5 below.
         * STATUS register is 5 bytes long but, as the event we are looking at is in the first byte of the register, we can use this simplest API
         * function to access it.*/
  while (!(dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS)) {
  };
  /* Clear TX frame sent event. */
  dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS);
}

static void timer_ch1_event_handler(nrf_timer_event_t event_type, void *p_context) {
  printf("Timer reload Interrupt\r\n");
}

static void timer_reload_handler(nrf_timer_event_t event_type, void *p_context) {
  printf("Timer reload Interrupt\r\n");
}


static void gpiote_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action) {
  if (action == NRF_GPIOTE_POLARITY_TOGGLE) {
    switch (pin){
    case CH1:
      ch = ch1;
      break;
    case PIN_DEBUG:
      ch = ch2;
      break;
    }
    if (nrf_gpio_pin_read(channel_get_pin(ch))) {
      channel_set_t1(ch, nrf_drv_timer_capture_get(channel_get_timer_channel(ch), 0));
      data_ready = data_ready | channel_1_ready;

      //      printf("Interrupt Low to High -- %d\r\n", ch1_times.t2);

    } else {
      channel_set_t2(ch, nrf_drv_timer_capture_get(channel_get_timer_channel(ch), 0));
      //      printf("Interrupt High to Low -- %d\r\n", ch1_times.t1);
    }
  }
}