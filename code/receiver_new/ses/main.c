#include <string.h>
#include <stdbool.h>
#include <stdint.h>

#include "app_error.h"
#include "app_pwm.h"
#include "bsp.h"
#include "deca_device_api.h"
#include "deca_param_types.h"
#include "deca_regs.h"
#include "deca_types.h"
#include "nrf.h"
#include "nrf_delay.h"
#include "nrf_drv_gpiote.h"
#include "nrf_drv_ppi.h"
#include "nrf_drv_pwm.h"
#include "nrf_drv_timer.h"
#include "nrf_uart.h"
#include "port_platform.h"
#include "UART.h"

// Function declaration
uint16_t wait_reception();

//-----------------DW1000 Configuration ----------------------------
static dwt_config_t config = {
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

// Uncomment the right motor address
//#define RECEIVER_ADDR 0xC5
//#define RECEIVER_ADDR 0xC6
#define RECEIVER_ADDR 0xC7
//#define RECEIVER_ADDR 0xC8

// Define the IO
#define FRAME_LEN_MAX 127
#define CH 8          // Output PWM pin connected to the ESC
#define PIN_DEBUG 23  // Debug PIN

// PWM Configuration in Hz
#define PWM_FREQUENCY			400
#define PWM_CLK				1000000
#define PWM_TOP_VALUE_FIX		(PWM_CLK / PWM_FREQUENCY)
#define PWM_POLARITY_OFFSET		(1 << 15)

// private variables
static uint16_t nrf_pwm_sequence[4] = {0 + PWM_POLARITY_OFFSET , 0 + PWM_POLARITY_OFFSET, 0 + PWM_POLARITY_OFFSET, 0 + PWM_POLARITY_OFFSET};

// Private function prototypes
static void temp_pwm_init(void);
static void temp_pwm_set_val(float duty);
static void print_msg(uint8_t *msg, int n);

// Global variables
static volatile bool ready_flag;
int new_data = 0;
static uint32 status_reg = 0;           // Hold copy of status register state here for reference so that it can be examined at a debug breakpoint.
static uint8 rx_buffer[FRAME_LEN_MAX];
static uint16 frame_len = 0;            // Hold copy of frame length of frame received (if good) so that it can be examined at a debug breakpoint.


// ------------......--START MAIN-----------------------------
int main(void) {
  // Setup DW1000 IRQ pin
  nrf_gpio_cfg_input(DW1000_IRQ, NRF_GPIO_PIN_NOPULL); 

  // Initialization UART
  boUART_Init();
  printf("Simple receiver \r\n");

  // Reset DW1000
  reset_DW1000();

  // Set SPI clock to 2MHz
  port_set_dw1000_slowrate();

  // Init the DW1000
  if (dwt_initialise(DWT_LOADUCODE) == DWT_ERROR) {
    //Init of DW1000 Failed
    while (1) {
    };
  }

  // Set SPI to 8MHz clock
  port_set_dw1000_fastrate();

  // Configure DW1000.
  dwt_configure(&config);
  
  // Initialize PWM
  temp_pwm_init();

  // Variables for printing purposes
  uint32_t cnt = 0;
  uint32_t cnt_print = 40;
  
  // Initialise PWM and debug pin
  nrf_gpio_cfg_output(CH);
  //nrf_gpio_cfg_output(PIN_DEBUG);
  //nrf_gpio_pin_clear(PIN_DEBUG);

  // Start clock for accurate frequencies
  NRF_CLOCK->TASKS_HFCLKSTART = 1;
  // Wait for clock to start
  while (NRF_CLOCK->EVENTS_HFCLKSTARTED == 0);
  
  
  // Waits indefinitely for the message to be received
  while (rx_buffer[0] != 0xff) {
    wait_reception();
  }
  
//  nrf_gpio_pin_set(PIN_DEBUG);
//  nrf_gpio_pin_clear(PIN_DEBUG);

  // Wait indefinitely for reception and update PWM duty cycle
  while (1) {
     if(wait_reception()){
        // Set DEBUG pin
        // nrf_gpio_pin_set(PIN_DEBUG);
        for (int i = 0; i < FRAME_LEN_MAX; i++) {
            if (rx_buffer[i] == RECEIVER_ADDR) {
              // Update PWM Duty Cycle
              temp_pwm_set_val(rx_buffer[i + 1]);
              //printf("id: %d, duty %d\r\n", RECEIVER_ADDR, rx_buffer[i + 1]);
              break;
            }
        }
        // Clear DEBUG pin
        // nrf_gpio_pin_clear(PIN_DEBUG);
     }
  }

}
//--------------------------END MAIN--------------------------------------------------


/**
 * wait_reception()
 *
 * @brief Wait for reception
 * @param
 * @return
 */
uint16_t wait_reception(){
    // Set all elements of the array to 0
    memset(rx_buffer, 0, FRAME_LEN_MAX * sizeof(rx_buffer[0]));

    // Activate reception immediately
    dwt_rxenable(DWT_START_RX_IMMEDIATE);

    // Poll until a frame is properly received or an error/timeout occurs.
    while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG | SYS_STATUS_ALL_RX_ERR))) {
    };

    if (status_reg & SYS_STATUS_RXFCG) {
      // A frame has been received, copy it to our local buffer.
      frame_len = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFL_MASK_1023;
      if (frame_len <= FRAME_LEN_MAX) {
        dwt_readrxdata(rx_buffer, frame_len, 0);
      }

      // Clear good RX frame event in the DW1000 status register. 
      dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG);
    } else {
      // Clear RX error events in the DW1000 status register. 
      dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_ERR);
    }
    
    return (status_reg & SYS_STATUS_RXFCG);
}

/**
 * temp_pwm_init()
 *
 * @brief inits the pwm controller
 * @param
 * @return
 */
static void temp_pwm_init(void)
{       
    // Output PWM PIN     
    NRF_PWM0->PSEL.OUT[0] = CH;
    
    // PWM in upmode
    NRF_PWM0->MODE = PWM_MODE_UPDOWN_Up;

    // select clock source prescaler
    NRF_PWM0->PRESCALER = PWM_PRESCALER_PRESCALER_DIV_16;

    // sets pwm count top to be at 1 KHz  and decode pwm in common mode
    NRF_PWM0->COUNTERTOP = PWM_CLK / PWM_FREQUENCY;
    NRF_PWM0->LOOP = 0;
    NRF_PWM0->DECODER =  PWM_DECODER_LOAD_Individual;
    
    NRF_PWM0->SEQ[0].PTR  = (uint32_t)nrf_pwm_sequence;
    NRF_PWM0->SEQ[0].CNT  = sizeof(nrf_pwm_sequence) / sizeof(uint16_t);
    
    NRF_PWM0->SEQ[0].REFRESH = 0;
    NRF_PWM0->SEQ[0].ENDDELAY = 0;

    // trigger it!
    NRF_PWM0->TASKS_SEQSTART[0] = 1;
}

/**
 * temp_pwm_set_val()
 *
 * @brief set the pwm with a new duty cicle
 * @param duty: New duty cycle
 * @return
 */
static void temp_pwm_set_val(float duty)
{ 
    // Enable PWM
    NRF_PWM0->ENABLE = PWM_ENABLE_ENABLE_Enabled;

    // obtains the duty cicle  and sums it up with polarity offset
    uint16_t dutyy = (uint16_t)((duty) * (float)PWM_TOP_VALUE_FIX/100);
    dutyy |= PWM_POLARITY_OFFSET;

    // update the duty cicle 
    nrf_pwm_sequence[0] = dutyy;
    nrf_pwm_sequence[1] = dutyy;
    nrf_pwm_sequence[2] = dutyy;
    nrf_pwm_sequence[3] = dutyy;

    NRF_PWM0->TASKS_SEQSTART[0] = 1;
}


/**
 * pwm_ready_callback()
 *
 * @brief pwm ready callback
 * @param 
 * @return
 */
void pwm_ready_callback(uint32_t pwm_id) 
{
  ready_flag = true;
}

/**
 * print_msg()
 *
 * @brief Function used to print the received message
 * @param 
 * @return
 */
static void print_msg(uint8 *msg, int n) {
  printf("Message:\t[");
  for (int i = 0; i < n; i++) {
    printf(" %d ", msg[i]);
  }
  printf("]\r\n");
}