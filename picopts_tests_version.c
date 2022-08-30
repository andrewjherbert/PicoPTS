// Elliott 900 Paper Tape Station  emulator for Raspberry Pi Pico

// Copyright (c) Andrew Herbert - 10/08/2022

// MIT Licence.

// Emulator for Elliott 900 Series computer Paper Tape Station.
// Does not implement 'undefined' effects.
// Has simplified handling of priority levels and initial orders.
// Only supports paper tape input and output and teleprinter
// peripherals.

// This program is intended to run on a Raspberry Pi Pico and
// depends upon Pico specific functions and data types.

// The Pico emulates the paper tape station interface of a 920M 
// using GPIO pins.  The pin numbers are listed below.  The use 
// of eachpin is as follows:

// RDRREQ_PIN, high signals a reader request.  The paper tape
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
//    2 - wrong request type seen in test
//    3 - timed out waiting for request in test
//    4 - current test failed
//    5 - emulation exited 
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
/* #include "tusb.h" // only needed if stdio routed to usb */
#include "pico/multicore.h"
#include <setjmp.h>


/**********************************************************/
/*                         DEFINES                        */
/**********************************************************/

// Test flags
#define PINS_TEST    1
#define READER_TEST  2
#define PUNCH_TEST   3
#define COPY_TEST1   4
#define COPY_TEST2   5
#define GPIO_TIMIMG  6
#define EMULATION    7

#define TEST COPY_TEST2 // set this to desired test if any

// Booleans
#define TRUE  1
#define FALSE 0

// Monitoring tick rate
#define TICK_SECS    5 // seconds

// Duration of ACK pulse, should be at least 1uS
// so 920M can detect the ACK
#define ACK_TIME     2 // microseconds

// Duration of i/o operations in microseconds
// These can be modified but need to be long
// enough so that PTS doesn't overrun the 920M.
// Empiracally the minimum time is 2uS.
#define RDR_TIME    4000  //  250 ch/s 
#define PUN_TIME    9091  //  110 ch/s
#define TTY_TIME  100000  //   10 ch/s

// GPIO pins

// GPIO0, GPIO1 in uses as serial output
#define RDR_1_PIN    2 // Set lsb of reader input
#define RDR_2_PIN    3 // these pins are assumed consecutive
#define RDR_4_PIN    4
#define RDR_8_PIN    5
#define RDR_16_PIN   6
#define RDR_32_PIN   7
#define RDR_64_PIN   8
#define RDR_128_PIN  9 // Set msb of reader input

#define RDR_PINS_MASK 01774 // PINs to bit mask 

#define PUN_1_PIN   10 // Set lsb of punch output
#define PUN_2_PIN   11 // these pins are assumed consecutive 
#define PUN_4_PIN   12
#define PUN_8_PIN   13
#define PUN_16_PIN  14
#define PUN_32_PIN  15
#define PUN_64_PIN  16 
#define PUN_128_PIN 17 // Pico sets to msb of punch output

#define PUN_PINS_MASK 0776000 // PINs to bit mask

#define NOPOWER_PIN 18 // set HIGH to stop and reset, set LOW to run
#define ACK_PIN     19 // pulse HIGH to acknowledge RDR or PUN request
#define II_AUTO_PIN 20 // set HIGH to autostart on reset, or LOW to execute
                       // initial instructions
#define TTYSEL_PIN  21 // Pico sets HIGH to request reader input and awaits ACK
#define PUNREQ_PIN  22 // Pico sets HIGH to request punch output and awiats ACK
// There is no GPIO23, GPIO24
#define LED_PIN     25 // onboard LED    
#define RDRREQ_PIN  26 // Pico sets HIGH to select teleprinter,
                       // LOW for paper tape
#define LOG_PIN     27 // set HIGH to enable logging
// GPIO28 spare

#define IN_PINS  12 // count of input pins
#define OUT_PINS 12 // count of output pins

#define RDRREQ_BIT   (1 << RDRREQ_PIN)
#define PUNREQ_BIT   (1 << PUNREQ_PIN)
#define TTYSEL_BIT   (1 << TTYSEL_PIN)
#define REQ_BITS     (RDRREQ_BIT | PUNREQ_BIT)
#define ACK_BIT      (1 << ACK_PIN)
#define NOPOWER_BIT  (1 << NOPOWER_PIN)

#define READ  1 // return values from wait_for_request
#define PUNCH 2

#define REQUEST_FAIL       2 // failure codes
#define TIMEOUT_FAIL       3
#define LOGGING_FAIL       4
#define TEST_FAIL          5
#define EXIT_FAIL          6

const char* error_messages[] =
                          { "0 - undefined",
			    "1 - Undefined",
			    "2 - wrong request type seen",
			    "3 - wait for request timed out",
			    "4 - test failed",
			    "5 - program exited" };


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
    PUN_32_PIN, PUN_64_PIN, PUN_128_PIN, LED_PIN };

uint32_t in_pins_mask = 0, out_pins_mask = 0; // initialized in setup_gpios

jmp_buf jbuf;                   // used by setjmp in main

// I/O timing - all devices initially idle
absolute_time_t reader_free, punch_free, tty_free;

// monitoring 
volatile  uint64_t cycles;
volatile  uint32_t monitoring = FALSE;


/**********************************************************/
/*                         FUNCTIONS                      */
/**********************************************************/

static  void     setup_gpios();                // initialize GPIO interface 
static  void     led_on();                     // turn onboard LED on
static  void     led_off();                    // turn onboard LED off
static  uint32_t logging();                    // TRUE is logging enabled
static  uint32_t ack();                        // signal an ACK
static  void     set_power_on();               // set NOPOWER LOW
static  void     set_power_off();              // set NOPOWER HIGH
static  void     put_pts_ch(const uint32_t ch,
			    const uint32_t tty);// send character 
static  uint32_t get_pts_ch(const uint32_t tty);// receive character
static  uint32_t wait_for_request();           // wait for RDR or PUN request
static  uint32_t teletype();                   // TRUE if teletype selected
static void      signals(uint32_t);            // display current input signals
static  void     pts_emulation();              // paper tape station emulation

// Test routines used during development only 

#if TEST == PINS_TEST
static void pins_test();                        // check GPIO pins working
#elif TEST == READER_TEST
static void reader_test();                      // check read protocol
#elif TEST == PUNCH_TEST
static void punch_test();                       // check punch protocol
#elif TEST == COPY_TEST1
static void copy_test1();                       // test copying from reader
                                                // to punch
#elif TEST == COPY_TEST2
static void copy_test2();                       // run a copy program

#elif TEST == GPIO_TIMING
static void gpio_timing();                      // time GPIO operations
#endif

static void monitor();                          // monitor activity


/**********************************************************/
/*                           MAIN                         */
/**********************************************************/

int main() {

  int32_t fail_code;  // failure type ferom longjmp

  bi_decl(bi_program_description("Elliott 900 PTS Emulator by Andrew Herbert"));

  stdio_init_all(); // initialise stdio
  setup_gpios(); // configure interface to outside world
  reader_free = punch_free = tty_free = get_absolute_time();

  // 4 fast blinks to signal waking up
  for ( uint32_t i = 1 ; i <= 4 ; i++ )
    {
      led_on();
      sleep_ms(250);
      led_off();
      sleep_ms(250);
    }
  
  // wait for usb if required
  if ( logging() )
    {
      /* while ( !tud_cdc_connected() ) */
      /* 	sleep_ms(500); // wait for usb to wake up */
      printf("\n\n\nPicoPTS Starting\n");
    }

  multicore_launch_core1(pts_emulation); // run emulation on core1
  monitor();
  /* System stops if get here */
}


/**********************************************************/
/*                         TESTING                        */
/**********************************************************/


#if TEST == PINS_TEST

static void pins_test()
{
  printf("Pins test\n");
  gpio_put_masked(out_pins_mask, 0);
  sleep_ms(1500);
    while ( TRUE )
    {
      gpio_put(NOPOWER_PIN, 1);
      sleep_ms(1000);
      gpio_put(ACK_PIN, 1);
      sleep_ms(1000);
      gpio_put(II_AUTO_PIN, 1);
      sleep_ms(1000);
      gpio_put_masked(out_pins_mask, 0);
      for ( uint32_t bit = 0 ; bit < 8 ; bit++ )
	{
	  gpio_put_masked(RDR_PINS_MASK, 1 << (RDR_1_PIN+bit));
	  sleep_ms(1000);
	}
      gpio_put_masked(out_pins_mask, 0);
      sleep_ms(1000);
    }
}

#elif TEST == READER_TEST

static void reader_test()
{
  printf("PicoPTS reader test starting\n");
  printf("PicoPTS warming up for 10s\n");
  sleep_ms(10000);
  set_power_on(); // wake up 920M
  for ( cycles = 0 ; ; cycles++ )
    {
      if ( wait_for_request() != READ )
	{
	  printf("Got PUNREQ in reader test at cycle %"PRIu64"\n", cycles);
	  longjmp(jbuf, REQUEST_FAIL);
	  /* NOT REACHED */
	}
      put_pts_ch(cycles&255, 0);
    }
  printf("Reader test complete\n");
}

#elif TEST == PUNCH_TEST

static void punch_test()
{
  printf("PicoPTS punch test starting\n");
  printf("PicoPTS warming up for 10s\n");
  sleep_ms(10000);
  set_power_on(); // wake up 920M
  for ( cycles = 0 ; ; cycles++ )
    {
      uint32_t got, expected = cycles&255;
      if ( wait_for_request() != PUNCH )
	{
	  printf("Failed after %"PRIu64" cycles - got RDRREQUEST\n",
		 cycles);
	  longjmp(jbuf, REQUEST_FAIL);
	  /* NOT REACHED */
	}
      got = get_pts_ch(0); 
      if  ( got != expected )
	{
	  printf("Failed after %"PRIu64" cycles - got %u, expected %u\n",
		 cycles, got, expected);
	  longjmp(jbuf, TEST_FAIL);
	  /* NOT REACHED */
	}
    }
  printf("Punch test complete");
}

#elif TEST == COPY_TEST1

static void copy_test1()
{
  printf("PicoPTS copy test 1 starting\n");
  printf("PicoPTS warming up for 10s\n");
  sleep_ms(10000);
  set_power_on();  // wake up 920M
  for ( cycles = 0 ; ; cycles++ )
    {
      uint32_t got, expected = cycles&255;
      if ( wait_for_request() != READ )
	{
	  printf("Failed - no RDRREQ at cycle "
		 "%" PRIu64 "\n", cycles);
	  longjmp(jbuf, REQUEST_FAIL);
	  /* NOT REACHED */
	}
      put_pts_ch(cycles&255, 0);
      if ( wait_for_request() != PUNCH )
	{
	  printf("Failed - no PUNREQ, at cycle "
		 "%" PRIu64 "\n", cycles);
	  longjmp(jbuf, REQUEST_FAIL);
	  /* NOT REACHED */
	}
      if ( (got = get_pts_ch(0)) != expected )
	{
       	  printf("Failed - got %u, expected %u at cycle %"PRIu64"\n",
		 got, expected, cycles); 
       	  longjmp(jbuf, TEST_FAIL);
	  /* NOT REACHED */
	}
    }
  // printf("Copy test 1 complete\n");
}

#elif TEST == COPY_TEST2

// load simple copy program and run
static void copy_test2()
{
  uint32_t program  [] =
    {
      15 * 8192 + 2048,
      15 * 8192 + 6144,
       8 * 8192 + 8177
    };
  
  printf("PicoPTS copy test 2 starting\n");
  printf("PicoPTS warming up for 10s\n");
  sleep_ms(10000);
  set_power_on();  // wake up 920M
  for ( uint32_t e = 8177 ; e < 8180 ; e++ ) // load program
    {
      uint32_t word = program[e - 8177];
      for ( uint32_t n = 1 ; n < 5 ; n++ ) // load instruction
	{
	  if ( wait_for_request() != READ )
	    {
	      printf("PUNREQ received while loading program word %u "
		     "fragment %u\n",
		     e, n);
	      longjmp(jbuf, TEST_FAIL);
	      /* NOT REACHED */
	    }
	  switch ( n )
	    {
	    case 1:
	      put_pts_ch(8, 0); break;
	    case 2:
	      put_pts_ch(((word / 128) / 128) & 127, 0); break; 
	    case 3:
	      put_pts_ch((word / 128) & 127, 0); break;
	    case 4:
	      put_pts_ch(word & 127, 0); break;
	    }
	}
    }
  for ( cycles = 0 ; ; cycles++ ) // run a copy loop
    {
      uint32_t got, expected = cycles&127;
      if ( wait_for_request() != READ )
	{
	  printf("Got PUNREQ, expected RDRREQ, at cycle "
		 "%" PRIu64 "\n", cycles);
	  longjmp(jbuf, REQUEST_FAIL);
	  /* NOT REACHED */
	}
      put_pts_ch(cycles&255, 0);
      sleep_ms(1);
      if ( wait_for_request() != PUNCH )
	{
	  printf("Got RDRREQ, expected PUNREQ, at cycle "
		 "%" PRIu64 "\n", cycles);
	  longjmp(jbuf, REQUEST_FAIL);
	  /* NOT REACHED */
	}
      if ( (got = (get_pts_ch(0) & 127)) != expected )
	{
       	  printf("Failed - got %u, expected %u at cycle %" PRIu64 "\n",
		 got, expected, cycles); 
       	  longjmp(jbuf, TEST_FAIL);
	  /* NOT REACHED */
	}
    }
  
  // printf("Copy test 2 completed\n");
}

#elif TEST == GPIO_TIMING

void gpio_timing()
{
  printf("GPIO timing test\n10,000,000 cycles per function\n");
  monitoring = FALSE;
  for (uint32_t test = 0 ; test < 4 ; test++)
    {
      const uint64_t start = time_us_64();
      uint64_t time_us;
      float time_gpio;
      char *function[4] = {"gpio_put", "gpio_get", "gpio_put_masked",
			   "gpio_get_all"};
      const uint32_t loops = 10000000;
      switch (test)
        {
	  case 0:
	    for (uint32_t i = 0 ; i < loops ; i++)
	      gpio_put(ACK_PIN, 0); // 0.040 uS
	    break;
	  case 1:
	    for (uint32_t i = 0 ; i < loops ; i++)
	      gpio_get(RDRREQ_PIN); //0.036 uS
	    break;
	  case 2:
	    for (uint32_t i = 0 ; i < loops ; i++)
	      gpio_put_masked(RDR_PINS_MASK, 0); // 0.056 uS
	    break;
	  case 3:
	    for (uint32_t i = 0 ; i < loops ; i++)
	      gpio_get_all(); // 0.036 uS
	    break;
        }
      time_us = time_us_64() - start;
      time_gpio = ((float) time_us) / ((float) loops);
      printf("Function %-15s time = %f uS, %.0f per uS\n", function[test],
	     time_gpio, 1.0 / time_gpio);
    }
  printf("GPIO timing test complete\n");
  while ( TRUE ) sleep_ms(100000000);
}

#else

  #error "Unknown test name (TEST)"

#endif

static void monitor()
{
#if TEST == PINS_TEST
  uint32_t last = 0, next= ~0;
#endif
  monitoring = TRUE;  // turned off by error handling
  sleep_ms(15000); // let system get started
  for ( uint32_t tick = 1 ; ; tick++ )
    {
#if TEST != PINS_TEST // monitor every TICK_SECS seconds
      if ( !monitoring ) while ( TRUE ) sleep_ms(UINT32_MAX);
      led_on();
      sleep_ms(TICK_SECS * 500);
      led_off();
      sleep_ms(TICK_SECS * 500);
      if ( monitoring && logging() )
	{
      	  printf("Time %7u secs %10"PRIu64" cycles\n",
      	         tick * TICK_SECS, cycles);
	}
#else // monitor when pins change
      next = gpio_get_all() & in_pins_mask;
      if ( last != next )
	{
	  signals(next);
	  last = next;
	}
      sleep_ms(500);
#endif
    }
}

/**********************************************************/
/*                    PAPER TAPE SYSTEM                   */
/**********************************************************/

static void pts_emulation()
{
  int32_t fail_code; // set by longjmp
  // long jump to here on error
  if ( fail_code = setjmp(jbuf) ) // test if a longjmp occurred
    {
      monitoring = FALSE;   // disable conflicting monitoring i/o
      set_power_off();      //    stop 920M
      gpio_put(ACK_PIN, 0); //      and abort any transfer
      if ( logging() )
	printf("PicoPTS halted after error %s"
	       " - push reset to restart\n",error_messages[fail_code]);
      while ( TRUE )  // loop until reset flashing the code
	{
	  sleep_ms(1000);
	  for ( uint32_t j = 1 ; j <= fail_code ; j++ )
	    {
	      led_on(); sleep_ms(250);led_off(); sleep_ms(100);
	    }
	}
    }

#if TEST == PINS_TEST
  pins_test();
#elif TEST == READER_TEST
  reader_test();
#elif TEST == PUNCH_TEST
  punch_test();
#elif TEST == COPY_TEST1
  copy_test1();
#elif TEST == COPY_TEST2
  copy_test2();
#elif TEST == GPIO_TIMING
  gpio_timing();
#elif defined(TEST)
  #error "Unknown test TEST"
#else
  printf("920M PTS emulation not impelemented yet\n");
  longjmp(jbuf, TEST_FAIL);
  
  if ( logging() ) printf("Resetting 920M\n");
  set_power_off(); // Reset 920M */
  if ( logging() ) printf("Power on 920M now!\n");
  sleep_ms(5000);
  set_power_on();
  if ( logging() ) printf("920M PTS emulation starting\n");
  
#endif
}

/*  Output a character to paper tape station */ 
/*  i.e., punch or tty output                */

static inline void put_pts_ch(const uint32_t ch, const uint32_t tty)
{
  if ( tty) {
    sleep_until(tty_free); // wait until tty idle
    tty_free = make_timeout_time_us(TTY_TIME); // set tty busy
  } else {
    sleep_until(reader_free); // wait until reader idle
    reader_free = make_timeout_time_us(RDR_TIME); // set reader busy
  }
  gpio_put_masked(RDR_PINS_MASK, ch << RDR_1_PIN); // write 8 bits
  ack();
}

/* Input a character from the paper tape station */
/* i.e., reader or tty input                     */

static inline uint32_t get_pts_ch(const uint32_t tty)
{
  if ( tty) {
    sleep_until(tty_free); // wait until tty idle
    tty_free = make_timeout_time_us(TTY_TIME); // set tty busy
  } else {
    sleep_until(punch_free); // wait until punch idle
    punch_free = make_timeout_time_us(PUN_TIME); // set punch busy
  }
  const uint32_t ch = (gpio_get_all() & PUN_PINS_MASK)
                          >> PUN_1_PIN; // read 8 bits
  ack();
  return ch;
}


/**********************************************************/
/*                         PICO GPIO                      */
/**********************************************************/


/*  initialize GPIOs */

static void setup_gpios()
{
  // calculate masks
  for ( uint32_t i = 0; i < IN_PINS;  i++ ) in_pins_mask  |= (1 << in_pins[i]);
  for ( uint32_t i = 0; i < OUT_PINS; i++ ) out_pins_mask |= (1 << out_pins[i]);
  // initialize GPIOs
  gpio_init_mask(in_pins_mask | out_pins_mask);
  // set up GPIO directions
  gpio_set_dir_masked(in_pins_mask | out_pins_mask, out_pins_mask);
  // initialize all output GPIOs to LOW
  gpio_put_masked(out_pins_mask, 0); 
  // set NOPOWER high
  gpio_put(NOPOWER_PIN, 1);
  // set pull up on LOG_PIN so defaults to logging enabled
  gpio_pull_up(LOG_PIN);
  // set pull downs on request lines to avoid spurious signals
  gpio_pull_down(RDRREQ_PIN);
  gpio_pull_down(PUNREQ_PIN);
  gpio_pull_down(TTYSEL_PIN);
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
#if TEST == EMULATION
  return gpio_get(LOG_PIN);
#else
  return TRUE;
#endif
}

/* Power status signalling */

static inline void set_power_on()
{
  if ( logging() ) printf("Setting NOPOWER LOW\n");
  // set NOPOWER LOW and clear ACK 
  gpio_put_masked(NOPOWER_BIT | ACK_BIT, 0);
}

static inline void set_power_off()
{
  if ( logging() ) printf("Setting NPOWER HIGH\n");
  // set NOPOWER HIGH and clear ACK 
  gpio_put_masked(NOPOWER_BIT | ACK_BIT, NOPOWER_BIT);
}

/* Acknowledge a transfer */

static inline uint32_t ack()
{
  gpio_put(ACK_PIN, 1);
  sleep_us(ACK_TIME);
  gpio_put(ACK_PIN, 0);
}

/* Wait for transfer request for up to timeout_ms milliseconds,
   0 => indefinite timeout */

static inline uint32_t wait_for_request()
{
  uint32_t request;
  while ( (request = (gpio_get_all() & REQ_BITS)) == 0 ); // wait for request  
  if ( request & RDRREQ_BIT )
    return READ;
  if ( request & PUNREQ_BIT ) 
    return PUNCH;
}

/* Status of TTYSel */

static inline uint32_t teletype()
{
  return gpio_get(TTYSEL_PIN);
}

static void signals(uint32_t pins)
{
  printf("RDRREQ %1u PUNREQ %1u TTYSEL %1u PUN DATA %3u ",
	 (pins >> RDRREQ_PIN) & 1,
	 (pins >> PUNREQ_PIN) & 1,
	 (pins >> TTYSEL_PIN) & 1,
	 (pins >> PUN_1_PIN) & 255);
  for ( uint32_t bit = 0 ; bit < 8 ; bit++ )
    printf("%1u",(pins >> (PUN_128_PIN-bit)) & 1);
  printf("\n");
}
