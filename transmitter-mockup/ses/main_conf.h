#include "app_error.h"
#include "bsp.h"
#include "deca_device_api.h"
#include "deca_param_types.h"
#include "deca_regs.h"
#include "deca_types.h"
#include "port_platform.h"
#include "nrf.h"
#include "nrf_delay.h"
#include "nrf_drv_gpiote.h"
#include "nrf_drv_ppi.h"
#include "nrf_drv_timer.h"
#include "nrf_uart.h"
#include "uart.h"
#include "channel_library.h"
#include <stdbool.h>
#include <stdint.h>

/* --------------------- Definitions -------------------------- */
enum {channel_0_ready = 0b00000001, 
      channel_1_ready = 0b00000010,
      channel_2_ready = 0b00000100,
      channel_3_ready = 0b00001000};

// Bitmask used to check when the data is ready.
// if mask_ready_1_channel is useed the data will be send when ch0 is ready
enum {mask_ready_1_channel = 0b00000001,
      mask_ready_2_channel = 0b00000011,
      mask_ready_3_channel = 0b00000111,
      mask_ready_4_channel = 0b00001111};

/* --------------------- User parameters -------------------------- */

#define NUMBER_OF_CHANNELS_ENABLED 3
#define ADDR_CH0 0xC0
#define ADDR_CH1 0xC1
#define ADDR_CH2 0xC2
#define ADDR_CH3 0xC3

#define CH0_PIN 12
#define CH1_PIN 27
#define CH2_PIN 13
#define CH3_PIN 15
#define PIN_DEBUG 8
#define PWM_IN_FREQ 400 // Signal frequency [Hz]



/* --------------------- Global variables -------------------------- */
const nrf_drv_timer_t TIMER_CH0 = NRF_DRV_TIMER_INSTANCE(0);
const nrf_drv_timer_t TIMER_CH1 = NRF_DRV_TIMER_INSTANCE(1);
const nrf_drv_timer_t TIMER_CH2 = NRF_DRV_TIMER_INSTANCE(2);
const nrf_drv_timer_t TIMER_CH3 = NRF_DRV_TIMER_INSTANCE(3);

channel_t *ch;
channel_t *ch0;
channel_t *ch1;
channel_t *ch2;
channel_t *ch3;


//// General ////
float to_dc = PWM_IN_FREQ / 1e6 * 100; // Scaled to be used when the ton is expressed in us

//// Transmission ////
uint8_t tx_msg[] = {ADDR_CH0, 0, ADDR_CH1, 0, ADDR_CH2, 0, ADDR_CH3, 0, 0, 0}; // Buffer to send

static dwt_config_t dwt_config = {
    5,               /* Channel number. */
    DWT_PRF_64M,     /* Pulse repetition frequency. */
    DWT_PLEN_128,    /* Preamble length. Used in TX only. */
    DWT_PAC8,        /* Preamble acquisition chunk size. Used in RX only. */
    10,              /* TX preamble code. Used in TX only. */
    10,              /* RX preamble code. Used in RX only. */
    0,               /* 0 to use standard SFD, 1 to use non-standard SFD. */
    DWT_BR_6M8,      /* Data rate. */
    DWT_PHRMODE_STD, /* PHY header mode. */
    (129 + 8 - 8)    /* SFD timeout (preamble length + 1 + SFD length - PAC size). Used in RX only. */
};
uint8_t data_ready = 0b00000000; // Mask to indicate when data has to be send



/* --------------------- Functions -------------------------- */
static void send_message(uint8 msg);
static void print_msg(uint8 *msg, int n);
static void timer_ch1_event_handler(nrf_timer_event_t event_type, void *p_context);
static void timer_reload_handler(nrf_timer_event_t event_type, void *p_context);
static void gpiote_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action);
static channel_t * channel_array_slice(channel_t * array, uint8_t i);


/* ------------------------ Macros ------------------------------ */
#define BITMASK_WHEN_READY mask_ready_3_channel