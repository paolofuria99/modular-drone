#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include "nrf_drv_timer.h"
#include "nrf_drv_gpiote.h"
#include "nrf_drv_ppi.h"
#include "app_error.h"

typedef struct channel channel_t;


channel_t * channel_init_default(uint8_t pin_number, 
                                 uint8_t bitmask, 
                                 const nrf_drv_timer_t timer_channel, 
                                 nrf_drv_gpiote_evt_handler_t gpiote_channel_handler);
channel_t * channel_init_void();

void channel_set_t1(channel_t * channel, uint32_t t);
void channel_set_t2(channel_t * channel, uint32_t t);
void channel_set_dc(channel_t * channel, uint8_t dc);

uint32_t channel_get_t1(channel_t * channel);
uint32_t channel_get_t2(channel_t * channel);
uint8_t channel_get_dc(channel_t * channel);
uint8_t channel_get_mask(channel_t * channel);
uint8_t channel_get_pin(channel_t * channel);
nrf_drv_timer_t * channel_get_timer_channel(channel_t *channel);

