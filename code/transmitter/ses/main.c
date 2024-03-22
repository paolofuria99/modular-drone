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

// Srt this to the right amount of channel you want to read
int main(void) {
  nrf_gpio_cfg_output(PIN_DEBUG);
  nrf_gpio_pin_set(PIN_DEBUG);

  ch = channel_init_void();
  ch0 = channel_init_default(CH0_PIN, channel_0_ready, TIMER_CH0, gpiote_handler);
  ch1 = channel_init_default(CH1_PIN, channel_1_ready, TIMER_CH1, gpiote_handler);
  ch2 = channel_init_default(CH2_PIN, channel_2_ready, TIMER_CH2, gpiote_handler);
  ch3 = channel_init_default(CH3_PIN, channel_3_ready, TIMER_CH3, gpiote_handler);

  nrf_drv_timer_config_t timer_config = NRF_DRV_TIMER_DEFAULT_CONFIG;
  timer_config.frequency = NRF_TIMER_FREQ_1MHz;

  nrf_drv_timer_init(&TIMER_CH4, &timer_config, empty_timer_callback);
  nrf_drv_timer_extended_compare(&TIMER_CH4,
                                  NRF_TIMER_CC_CHANNEL0, 
                                  UINT32_MAX,
                                  NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK, 
                                  true); // Configure to reload after uint32 max ticks
  nrf_drv_timer_enable(&TIMER_CH4);

  channel_t *all_channels[] = {ch0, ch1, ch2, ch3};
  uint32_t err_code = NRF_SUCCESS;
  float dc = 0;

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

  // Send configuration signal
  uint8_t tx_conf[tx_conf_n] = {0xff, 0xc5, 0x66, 0xa2, 0xb5, 0, 0, 0};
  __disable_irq();
  nrf_delay_ms(10000); // Wait before enabling the interrupts
  __enable_irq();
//  printf("Sent configuration message");

//  printf("DWM configured\r\n");

//  printf("Running main loop \r\n");
  uint32_t cnt = 0;
  uint8_t cnt_print = 40;
  while (1) {

    if (data_ready == BITMASK_WHEN_READY) {
//    nrf_gpio_pin_clear(PIN_DEBUG);

    if(!is_init){
        send_message(tx_conf, tx_conf_n);
        is_init = true;
        data_ready = data_ready & 0x00; 
//        nrf_gpio_pin_set(PIN_DEBUG);
    }
    else{
      for (int j = 1; j < NUMBER_OF_CHANNELS_ENABLED * 2; j += 2) {
        dc = channel_get_dc(all_channels[(j - 1) / 2]);
        tx_msg[j] = dc;
      }
      send_message(tx_msg, tx_msg_n);
//      nrf_gpio_pin_set(PIN_DEBUG);

      data_ready = data_ready & 0x00; 
      // Reset the last bit
      // Send data and wait
//      if (cnt % cnt_print == 0) {
//        print_msg(tx_msg, NUMBER_OF_CHANNELS_ENABLED * 2);
//       }
//      cnt ++;
//      if(cnt%cnt_print == 0){
//        print_msg(tx_msg, NUMBER_OF_CHANNELS_ENABLED * 2);
//        }
//      cnt ++;
    }
    }
  }
}

// Function definition
static void send_message(uint8_t * msg, uint8_t n) {
 dwt_writetxdata(sizeof(uint8_t)*n, msg, 0); /* Zero offset in TX buffer. */
  dwt_writetxfctrl(sizeof(uint8_t)*n, 0, 0);     /* Zero offset in TX buffer, no ranging. */

  dwt_starttx(DWT_START_TX_IMMEDIATE);

  /* Poll DW1000 until TX frame sent event set. See NOTE 5 below.
     * STATUS register is 5 bytes long but, as the event we are looking at is in the first byte of the register, we can use this simplest API
     * function to access it.*/
  while (!(dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS)) {
  };
  /* Clear TX frame sent event. */
  dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS);
}

static void gpiote_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action) {
  if (action == NRF_GPIOTE_POLARITY_TOGGLE && (pin == CH0_PIN | pin == CH1_PIN | pin == CH2_PIN | pin == CH3_PIN)) {
//    nrf_gpio_pin_clear(PIN_DEBUG);
    switch (pin) {
    case CH0_PIN:
      ch = ch0;
      break;
    case CH1_PIN:
      ch = ch1;
      break;
    case CH2_PIN:
      ch = ch2;
      break;
    case CH3_PIN:
      ch = ch3;
      break;
    default:
      printf("Wrong Channel\r\n");
      exit(EXIT_FAILURE);
    }
    if (nrf_gpio_pin_read(channel_get_pin(ch))) {
      channel_set_t1(ch, nrf_drv_timer_capture_get(channel_get_timer_channel(ch), 0));
      data_ready = data_ready | channel_get_mask(ch);
      channel_set_dc(ch, (uint8_t)(100 - (channel_get_t1(ch) - channel_get_t2(ch)) * to_dc));

      //      printf("Interrupt Low to High -- %d\r\n", ch1_times.t2);

    } else {
      channel_set_t2(ch, nrf_drv_timer_capture_get(channel_get_timer_channel(ch), 0));
      //      printf("Interrupt High to Low -- %d\r\n", ch1_times.t1);
    }
  }
//  nrf_gpio_pin_set(PIN_DEBUG);
}

static void print_msg(uint8 *msg, int n) {
  printf("Message:\t[");
  for (int i = 0; i < n; i++) {
    printf(" %d ", msg[i]);
  }
  printf("]\r\n");
}

static void empty_timer_callback(nrf_timer_event_t event_type, void *p_context) {
  printf("Timer reload Interrupt\r\n");
}
//    dc = (100 - (channel_get_t1(ch) - channel_get_t2(ch)) * PWM_IN_FREQ / 1e6 * 100);