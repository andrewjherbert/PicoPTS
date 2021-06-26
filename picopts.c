// Elliott 900 Paper Tape Station  emulator for Raspberry Pi Pico

// Copyright (c) Andrew Herbert - 26/06/2021

// MIT Licence.

// Emulator for Elliott 900 Series computer Paper Tape Station.
// Does not implement 'undefined' effects.
// Has simplified handling of priority levels and initial orders.
// Only supports paper tape input and output and teleprinter
// peripherals.

// This program is intended to run on a Raspberry Pi Pico and
// depends upon Pico specific functions and data types.

// The Pico emulates the paper tape station  interface of a 920M 
// using GPIO pins.  The pin numbers are listed below.  The use 
// of eachpin is as follows:
//
// RDRReq_PIN, high signals a reader request.  The paper tape
// station is expected to load 8 bits of data on pins RDR_1_PIN
// (lsb) to RDR_128_PIN (msb) and then raise ACK-PIN high for
// approximately 5uS to indicate the input data is ready. Once
// the computer has read in the data it lowers RDRReq-PIN to
// signal transfer complete.  
//
// PUNReq_PIN, high signals a punch request. The paper tape
// station is expected to load 8 bits of data from pins PUN_1_PIN
// (lsb) to PUN_128_PIN (msb) and then raise ACK_PIN high for
// approximately 5uS to indicate the output data have been copied.
// Once the ACK_PIN is lowered the computer then lowers PUNReq-PIN.
//
// TTYSEL_PIN, high signals that paper tape input and output is
// to be directed to the online teleprinter, if present. Low
// signals that input should come from the paper tape reader
// and output to the paper tape punch.
//
// ACK_PIN, used as described above to signal completion of a
// data transfer to/from the paper tape station.
//
// II_AUTO_PIN, high selects that on a reset / restart the
// computer should execute an autostart, i.e., jump to location
// 8177.  If low the computer should obey the initial orders
// to load in a program from paper tape.
//
// NOPOWER_PIN, high signals that the computer should stop
// execution and enter a reset state.  Low signals that
// the computecd r whould wake up if in the reset state and
// execute an autostart or initial instructions as determined
// by II_AUTO.  NOPOWER should remain low so that the computer
// can continue to execute until next reset by raising NOPOWER
// high again.
//
// An additional GPIO pins are used to set options for the
// emulation:
//
// LOG_PIN, high signals that diagnostic logging messages
// should be sent to the USB port.This pin only read at the start
// of an emulation. If it is desired to change the setting the Pico
// must be restarted either by removing, then replacing the USB power
// cable, or using the  button provided for this purpose.
//
// The onboard LED (LED_PIN) is used to signal emulation status.
// Immediately after loaded the LED is flashed 4 times to
// signal the emulation is ready to start.  When in the NOPOWER
// state the LED is extinguished.  When running the LED is lit
// continuously.

// If a catastrophic error occurs the LED shows
// a code indicating the error:
//    3 - attempt to address outside bounds of store
//    4 - attempt to execute an unimplemented type 14 instruction -
//    5 - attempt to execute an unimplemented type 15 instruction
//
// The Pico is equipped with a push button which can used used
// to force a restart of the Pico system.  This completely
// restarts the emulation system from the beginning.  It can be
// used to restart after a dynamic stop, infinite loop or
// catastrophic error.


/**********************************************************/
/*                     HEADER FILES                       */
/**********************************************************/


#include <stdio.h>
#include <inttypes.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "pico/binary_info.h"
#include "tusb.h"
#include "pico/multicore.h"
#include <setjmp.h>


/**********************************************************/
/*                         DEFINES                        */
/**********************************************************/

// Booleans
#define TRUE  1
#define FALSE 0

// Limit on polling loops
#define POLL_LIMIT 2000

// GPIO pins

#define RDR_1_PIN    0 // Set lsb of reader output
#define RDR_2_PIN    1 // these pins are assumed consecutive
#define RDR_4_PIN    2
#define RDR_8_PIN    3
#define RDR_16_PIN   4
#define RDR_32_PIN   5
#define RDR_64_PIN   6
#define RDR_128_PIN  7 // Set msb of reader input

#define RDR_PINS_MASK 0377 // PINs to bit mask 

#define PUN_1_PIN    8 // Set lsb of punch input
#define PUN_2_PIN    9 // these pins are assumed consecutive 
#define PUN_4_PIN   10
#define PUN_8_PIN   11
#define PUN_16_PIN  12
#define PUN_32_PIN  13
#define PUN_64_PIN  14 
#define PUN_128_PIN 15 // Set msb of punch output

#define PUN_PINS_MASK 0177400 // PINs to bit mask

/* N.B. the following pins are different to those assigned in pico900 */
#define NOPOWER_PIN 16 // set HIGH to stop and reset, set LOW to run
#define ACK_PIN     17 // pulse HIGH to acknowledge RDR or PUN request
#define II_AUTO_PIN 18 // set HIGH to autostart on reset, or LOW to execute
                       // initial instructions
#define TTYSEL_PIN  19 // Computer sets HIGH to select teleprinter,
                        // LOW for paper tape
#define PUNREQ_PIN  20 // Pico sets HIGH to request punch output and
                       // awaits ACK
#define RDRREQ_PIN  21 // Computer sets HIGH to request reader input and
                       // awaits ACK
#define LOG_PIN     22 // set HIGH to enable logging
// GPIO 23, 24 internal
#define LED_PIN     25 // onboard LED
// GPIO26 spare
// GPIO27 spare
// GPIO28 spare

#define IN_PINS  11 // count of input pins
#define OUT_PINS 12 // count of output pins

#define READ  1 // return values from wait_for_request
#define PUNCH 2

#define REQUEST_FAIL       2
#define REQUEST_END_FAIL   3
#define LOGGING_FAIL       4
#define TEST_FAIL          5
#define EXIT_FAIL          6


/**********************************************************/
/*                         GLOBALS                        */
/**********************************************************/


const uint32_t out_pins[OUT_PINS] =
  { NOPOWER_PIN, ACK_PIN, II_AUTO_PIN, LED_PIN, 
    RDR_1_PIN, RDR_2_PIN, RDR_4_PIN, RDR_8_PIN,
    RDR_16_PIN, RDR_32_PIN, RDR_64_PIN,
    RDR_128_PIN };

const uint32_t in_pins[IN_PINS] =
  { RDRREQ_PIN, PUNREQ_PIN, TTYSEL_PIN, PUN_1_PIN,
    PUN_2_PIN, PUN_4_PIN, PUN_8_PIN, PUN_16_PIN,
    PUN_32_PIN, PUN_64_PIN, PUN_128_PIN };

uint32_t logging_enabled   = 0; // 1 = enable logging,
                                // 0 = disable logging to usb
                                // must default to 0 until set by reference to
                                // LOG_PIN

uint32_t read_request_bit   = 1 << RDRREQ_PIN;
uint32_t punch_request_bit  = 1 << PUNREQ_PIN;

static jmp_buf jbuf;            // used by setjmp in main

static uint64_t cycles;         // used in diagnostic code
static uint32_t max_poll = 0;
static uint32_t monitoring = FALSE;


/**********************************************************/
/*                         FUNCTIONS                      */
/**********************************************************/

static  void     set_up_gpios();               // initialize GPIO interface 
static  void     led_on();                     // turn onboard LED on
static  void     led_off();                    // turn onboard LED off
static  uint32_t logging();                    // TRUE is logging enabled
static  uint32_t ack();                        // signal an ACK
static  void     set_power_on();               // set NOPOWER LOW
static  void     set_power_off();              // set NOPOWER HIGH
static  uint32_t wait_for_request();           // wait for RDR or PUN request
static  void     put_pts_ch(const uint32_t ch);// send from paper tape reader
static  uint32_t get_pts_ch();                 // receive from paper tape punch
static  uint32_t wait_for_request();           // wait for reader or punch request
static  void     wait_for_no_request();        // wait until request cleared
static  uint32_t teletype();                   // TRUE if teletype selected
static void      master();                     // code to run in core1

// Test routines used during development only 
static void signals();
static void reader_test(uint64_t max_cycles);
static void punch_test(uint64_t max_cycles);
static void master();
static void monitor();


/**********************************************************/
/*                           MAIN                         */
/**********************************************************/

int main() {

  int32_t fail_code;

  bi_decl(bi_program_description("Elliott 900 PTS Emulator by Andrew Herbert"));

  stdio_init_all(); // initialise stdio
  set_up_gpios(); // configure interface to outside world

  // set local flags based on external inputs
  logging_enabled = logging();     // print logging messages to usb?

  if ( logging_enabled )
    {
      while ( !tud_cdc_connected() )
	sleep_ms(500); // wait for usb to wake up
      puts("\n\n\nPicoPTS Starting");
    }

  // long jump to here on error
  if ( fail_code = setjmp(jbuf) ) // test if a longjmp occurred
    {
      monitoring = FALSE; // disable conflicting monitoring i/o
      set_power_off(); // and stop 920M
      sleep_ms(2000); // allow system to quiesce
      if ( logging_enabled )
	printf("PicoPTS halted after error with code %d"
	       " - push reset to restart\n",fail_code);
      while ( TRUE )  // loop until reset flashing the code
	{
	  sleep_ms(1000);
	  for ( uint32_t j = 1 ; j <= fail_code ; j++ )
	    {
	      led_on(); sleep_ms(250);led_off(); sleep_ms(100);
	    }
	}
    }
  
  // Reset 920M
  gpio_put(ACK_PIN, 0);
  if ( logging_enabled ) puts("Power on 920M now!");
  set_power_off();
  sleep_ms(5000); // wait for 920M
  if ( logging_enabled ) puts("Resetting 920M");
  set_power_on();
  
  multicore_launch_core1(master);
  monitor();
  longjmp(jbuf, EXIT_FAIL); // STOP HERE

  // start PTS  emulation
  while ( TRUE ) // run emulation for ever
  {
    /* main loop */
    
    // will only end up here on a failure or reset, in either case, restart
  }
}

static void master()
{
  while ( TRUE )
    {
      punch_test(10000000);
      sleep_ms(1);
      reader_test(10000000);
    }
}

static inline void reader_test(uint64_t max)
{
  if ( logging_enabled )
    puts("PicoPTS reader test starting");
  else
    {
      puts("PicoPTS reader test - no logging - stopping");
      longjmp(jbuf, LOGGING_FAIL);
      /* NOT REACHED */
    }
  for ( cycles = 0 ; cycles < max ; cycles++ )
    {
      if ( wait_for_request() != READ )
	{
	  printf("Got PUNREQ in reader test at cycle %"PRIu64"\n", cycles);
	  longjmp(jbuf, REQUEST_FAIL);
	  /* NOT REACHED */
	}
      put_pts_ch(cycles&255);
    }
  puts("Reader test complete");
}

static inline void punch_test(uint64_t max)
{
  if ( logging_enabled )
    puts("PicoPTS punch test starting");
  else
    {
      puts("PicoPTS punch test - no logging - stopped");
      longjmp(jbuf, LOGGING_FAIL);
      /* NOT REACHED */
    }
  for ( cycles = 0 ; cycles < max; cycles++ )
    {
      uint32_t got, expected = cycles&255;
      if ( wait_for_request() != PUNCH )
	{
	  printf("Got RDRREQUEST in punch test at cycle %"PRIu64"\n",
		 cycles);
	  longjmp(jbuf, REQUEST_FAIL);
	  /* NOT REACHED */
	}
      got = get_pts_ch(); 
      if  ( got != expected )
	{
	  printf("Failed after %"PRIu64" cycles got %u, expected %u\n",
		 cycles, got, expected);
	  longjmp(jbuf, TEST_FAIL);
	  /* NOT REACHED */
	}
    }
  puts("Punch test complete");
}

static inline void monitor()
{
  monitoring = TRUE;
  for ( uint32_t tick = 1 ; ; tick++ )
    {
      for ( uint32_t i = 0 ; i < 5 ; i++ )
	{
	  led_on();
	  sleep_ms(1000);
          led_off();
	  sleep_ms(1000);
	}
      if ( monitoring && logging_enabled )
	printf("Time %7u secs %10"PRIu64" cycles, max poll %4u\n",
	       tick*10, cycles, max_poll);
    }
}

/**********************************************************/
/*                    PAPER TAPE SYSTEM                   */
/**********************************************************/


/*  Output a character to paper tape station */ 
/*  i.e., punch or tty output                */

static inline void put_pts_ch(const uint32_t ch)
{
  gpio_put_masked(RDR_PINS_MASK, ch << RDR_1_PIN); // write 8 bits
  ack();
  wait_for_no_request(); // wait for request to clear
}

/* Input a character from the paper tape station */
/* i.e., reader or tty input                     */

static inline uint32_t get_pts_ch()
{
  uint32_t ch;
  ch = (gpio_get_all() >> PUN_1_PIN) & 255; // read 8 bits
  ack();
  wait_for_no_request();
  return(ch);
}


/**********************************************************/
/*                         PICO GPIO                      */
/**********************************************************/


/*  initialize GPIOs */

static inline void set_up_gpios()
{
  uint32_t in_pins_mask = 0, out_pins_mask = 0;
  // calculate masks
  for ( uint32_t i = 0; i < IN_PINS;  i++ ) in_pins_mask  |= (1 << in_pins[i]);
  for ( uint32_t i = 0; i < OUT_PINS; i++ ) out_pins_mask |= (1 << out_pins[i]);
  // initialize GPIOs
  gpio_init_mask(in_pins_mask | out_pins_mask);
  // set up GPIO directions
  gpio_set_dir_masked(in_pins_mask | out_pins_mask, out_pins_mask);
  gpio_put_masked(out_pins_mask, 0); // initialize all GPIOs to LOW
  // set NOPOWER high
  gpio_put(NOPOWER_PIN, 1);
  // set pull up on LOG_PIN
  gpio_pull_up(LOG_PIN);
}

/* LED blinking */

static inline void led_on()
{
  gpio_put(LED_PIN, 1);
}

static inline void led_off()
{
  gpio_put(LED_PIN, 0);
}

/* Read logging status */

static inline uint32_t logging() {
  return gpio_get(LOG_PIN);
}

/* Power status signalling */

static inline void set_power_on()
{
  gpio_put(NOPOWER_PIN, 0);
}

static inline void set_power_off()
{
  gpio_put(NOPOWER_PIN, 1);
}

/* Acknowledge a transfer */

static inline uint32_t ack()
{
  gpio_put(ACK_PIN, 1);
  busy_wait_us_32(4);
  gpio_put(ACK_PIN, 0);
}

/* Wait for transfer request */

static inline uint32_t wait_for_request()
{
  uint32_t pins;
  while ( !((pins = gpio_get_all()) &
	   (read_request_bit | punch_request_bit)) )
    busy_wait_us_32(1);
  if ( (pins & read_request_bit) )
    return READ;
  else if ( (pins & punch_request_bit)  )
    return PUNCH;
}

/* Wait for request to clear */

static inline void wait_for_no_request()
{
  uint32_t count = 0, pins;
  while ( ((pins = gpio_get_all())
	   & (read_request_bit | punch_request_bit)) )
    {
      if ( count++ > max_poll ) max_poll = count;
      if ( count > POLL_LIMIT )
        {
	  if ( logging_enabled )
	    {
	      printf("Time out waiting for %s%s request to clear "
		     "at cycle %"PRIu64" after polling %u times\n",
		     ( (pins & read_request_bit)  ) ? "RDRREQ" : "",
		     ( (pins & punch_request_bit) ) ? "PUNREQ" : "",
		     cycles, --count);
	    }
	    longjmp(jbuf, REQUEST_END_FAIL);
	    /* NOT REACHED */
        }
      busy_wait_us_32(1);
    }
}


/* Status of TTYSel */

static inline uint32_t teletype()
{
  return gpio_get(TTYSEL_PIN);
}

static void signals()
{
  printf("RDRREQ %u PUNREQ %u TTYSEL %u\n", gpio_get(RDRREQ_PIN),
	 gpio_get(PUNREQ_PIN), gpio_get(TTYSEL_PIN));
}
