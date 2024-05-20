#include "main_conf.h"

// Function prototypes
static void send_message(uint8_t *msg, uint8_t n);
static void gpiote_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action);
static void print_msg(uint8_t *msg, int n);
static void empty_timer_callback(nrf_timer_event_t event_type, void *p_context);

// -----------------------START-MAIN----------------------------------
int main(void) {
  // Initialize GPIO
  nrf_gpio_cfg_output(PIN_DEBUG);
  nrf_gpio_pin_set(PIN_DEBUG);
  
  // Initialize channels
  ch = channel_init_void();
  ch0 = channel_init_default(CH0_PIN, channel_0_ready, TIMER_CH0, gpiote_handler);
  ch1 = channel_init_default(CH1_PIN, channel_1_ready, TIMER_CH1, gpiote_handler);
  ch2 = channel_init_default(CH2_PIN, channel_2_ready, TIMER_CH2, gpiote_handler);
  ch3 = channel_init_default(CH3_PIN, channel_3_ready, TIMER_CH3, gpiote_handler);
  
  // Initialize Timer
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

  // Initialize UART
  boUART_Init();
  //printf("Transmitter Module \r\n");
  //printf("UART SET \r\n");
  
  // Initialize DWM
  nrf_gpio_cfg_input(DW1000_IRQ, NRF_GPIO_PIN_NOPULL); //Setup DW1000 IRQ pin
  reset_DW1000(); // Reset DW1000
  port_set_dw1000_slowrate(); // Set SPI clock to 2MHz 
  if (dwt_initialise(DWT_LOADUCODE) == DWT_ERROR) {
    //Init of DW1000 Failed
    while (1) {
    };
  }
  port_set_dw1000_fastrate();
  dwt_configure(&dwt_config);

  // Send configuration signal
  uint8_t tx_conf[tx_conf_n] = {0xff, 0xc5, 0x66, 0xa2, 0xb5, 0, 0, 0};
  __disable_irq();
  nrf_delay_ms(10000); // Wait before enabling the interrupts
  __enable_irq();

  printf("Sent configuration message");
//  printf("DWM configured\r\n");
//  printf("Running main loop \r\n");

  uint32_t cnt = 0;
  uint8_t cnt_print = 40;
  
  // Main loop
  while (1) {
    if (data_ready == BITMASK_WHEN_READY) {
      nrf_gpio_pin_clear(PIN_DEBUG);

      if(!is_init){
          send_message(tx_conf, tx_conf_n);
          is_init = true;
          data_ready = data_ready & 0x00; 
          nrf_gpio_pin_set(PIN_DEBUG);
      }
      else{
        for (int j = 1; j < NUMBER_OF_CHANNELS_ENABLED * 2; j += 2) {
          dc = channel_get_dc(all_channels[(j - 1) / 2]);
          tx_msg[j] = dc;
        }
        send_message(tx_msg, tx_msg_n);
        nrf_gpio_pin_set(PIN_DEBUG);

        data_ready = data_ready & 0x00; 
        // Reset the last bit
        // Send data and wait
//        if (cnt % cnt_print == 0) {
//          print_msg(tx_msg, NUMBER_OF_CHANNELS_ENABLED * 2);
//         }
//        cnt ++;

      }
    }
  }
}

// Function that send message through UWB
static void send_message(uint8_t * msg, uint8_t n) {
  dwt_writetxdata(sizeof(uint8_t)*n, msg, 0); // Zero offset in TX buffer.
  dwt_writetxfctrl(sizeof(uint8_t)*n, 0, 0);  // Zero offset in TX buffer, no ranging.
  dwt_starttx(DWT_START_TX_IMMEDIATE);

  /* Poll DW1000 until TX frame sent event set. See NOTE 5 below.
     * STATUS register is 5 bytes long but, as the event we are looking at is in the first byte of the register, we can use this simplest API
     * function to access it.*/
  while (!(dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS)) {
  };
  // Clear TX frame sent event.
  dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS);
}

// GPIOTE handler. Calculate the duty cycle
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
      // printf("Wrong Channel\r\n");
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

// Function to print message
static void print_msg(uint8 *msg, int n) {
  printf("Message:\t[");
  for (int i = 0; i < n; i++) {
    printf(" %d ", msg[i]);
  }
  printf("]\r\n");
}

// Empty timer callback
static void empty_timer_callback(nrf_timer_event_t event_type, void *p_context) {
  printf("Timer reload Interrupt\r\n");
}
