#include "channel_library.h"

typedef struct channel{
  uint8_t pin_number;
  nrf_drv_timer_t timer_channel;
  uint32_t t1, t2;
  uint8_t dc;
} channel_t; 


static void empty_timer_callback(nrf_timer_event_t event_type, void *p_context);

channel_t * channel_init_void(){
  channel_t *channel = calloc(1, sizeof(channel_t));
  if (!channel){
    printf("could not allocate channel");
  }
  return channel;
}

channel_t * channel_init_default(uint8_t pin_number, 
                                  const nrf_drv_timer_t timer_channel,
                                  nrf_drv_gpiote_evt_handler_t gpiote_channel_handler){
  uint32_t err_code = NRF_SUCCESS;

  channel_t *channel = calloc(1, sizeof(channel_t));
  if (!channel){
    printf("could not allocate channel");
  }

  channel->pin_number = pin_number;
  channel->timer_channel = timer_channel;
  nrf_drv_timer_config_t timer_config = NRF_DRV_TIMER_DEFAULT_CONFIG;
  timer_config.frequency = NRF_TIMER_FREQ_1MHz;

  err_code = nrf_drv_timer_init(&timer_channel, &timer_config, empty_timer_callback);
  APP_ERROR_CHECK(err_code);

  nrf_drv_timer_extended_compare(&timer_channel,
                                  NRF_TIMER_CC_CHANNEL0, 
                                  2000000,
                                  NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK, 
                                  true); // Configure to reload after uint32 max ticks
  nrf_drv_timer_enable(&timer_channel);

  // GPIOTE Configuration
  nrf_drv_gpiote_init();
  nrf_drv_gpiote_in_config_t ch1_gpiote_config = GPIOTE_CONFIG_IN_SENSE_TOGGLE(false);
  ch1_gpiote_config.pull = NRF_GPIO_PIN_PULLUP;
  nrf_drv_gpiote_in_init(channel->pin_number, &ch1_gpiote_config, gpiote_channel_handler);

  // PPI Connection
  nrf_ppi_channel_t ppi_ch;
  nrf_drv_ppi_channel_alloc(&ppi_ch);
  nrf_drv_ppi_channel_assign(ppi_ch,
  nrf_drv_gpiote_in_event_addr_get(channel->pin_number),
  nrf_drv_timer_task_address_get(&channel->timer_channel, NRF_TIMER_TASK_CAPTURE0)); // configure interrupt on ch1_1 to save cnt value in cc0
  nrf_drv_ppi_channel_enable(ppi_ch);

  return channel;
}

void channel_set_t1(channel_t * channel, uint32_t t){
  channel->t1 = t;
}
void channel_set_t2(channel_t * channel, uint32_t t){
  channel->t2 = t;
}
void channel_set_dc(channel_t * channel, uint8_t dc){
  channel->dc = dc;
}
uint32_t channel_get_t1(channel_t * channel){
  return channel->t1;
}
uint32_t channel_get_t2(channel_t * channel){
  return channel->t2;
}
uint8_t channel_get_dc(channel_t * channel){
  return channel->dc;
}
uint8_t channel_get_pin(channel_t * channel){
  return channel->pin_number;
}
nrf_drv_timer_t * channel_get_timer_channel(channel_t *channel){
  return &channel->timer_channel;
}

static void empty_timer_callback(nrf_timer_event_t event_type, void *p_context) {
  printf("Timer reload Interrupt\r\n");
}
