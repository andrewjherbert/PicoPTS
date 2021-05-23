
// Elliott 900 Paper Tape Station  emulator for Raspberry Pi Pico

// Copyright (c) Andrew Herbert - 23/05/2021

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
// (lsb) to  RDR_128_PIN (msb) and then raise ACK-PIN high for
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
#include <pico/stdlib.h>
#include <hardware/gpio.h>
#include <pico/binary_info.h>
#include <tusb.h>
#include <setjmp.h>

// Each 900 18 bit word is stored as a 32 bit unsigned value.
// For some operations we compute with a double word so we
// use 64 bit unsigned quantities for these. Arithmetic is
// performed using 32 and 64 bit integers.

// Address calculations 16 bit unsigned arithmetic is used.

// Input/output uses 8 bit characters.

typedef int_fast32_t   INT32;
typedef int_fast64_t   INT64;
typedef uint_fast8_t   UINT8;
typedef uint_fast16_t  UINT16;
typedef uint_fast32_t  UINT32;
typedef uint_fast64_t  UINT64;


/**********************************************************/
/*                         DEFINES                        */
/**********************************************************/

// Booleans
#define TRUE  1
#define FALSE 0

// Store size
#define STORE_SIZE 8192

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


/**********************************************************/
/*                         GLOBALS                        */
/**********************************************************/


const UINT8 out_pins[OUT_PINS] = { NOPOWER_PIN, ACK_PIN, II_AUTO_PIN, LED_PIN, 
		                   RDR_1_PIN, RDR_2_PIN, RDR_4_PIN, RDR_8_PIN,
				   RDR_16_PIN, RDR_32_PIN, RDR_64_PIN,
				   RDR_128_PIN };

const UINT8 in_pins[IN_PINS] = { RDRREQ_PIN, PUNREQ_PIN, TTYSEL_PIN, PUN_1_PIN,
				 PUN_2_PIN, PUN_4_PIN, PUN_8_PIN, PUN_16_PIN,
				 PUN_32_PIN, PUN_64_PIN, PUN_128_PIN };

UINT8 logging_enabled   = 1; // 1 = enable logging, 0 = disable logging to usb


static jmp_buf jbuf;  // used by setjmp in main


/**********************************************************/
/*                         FUNCTIONS                      */
/**********************************************************/

static inline void   set_up_gpios();               // initialize GPIO interface 
static inline void   led_on();                     // turn onboard LED on
static inline void   led_off();                    // turn onboard LED off
static inline UINT8  logging();                    // TRUE is logging enabled
static inline UINT8  ack();                        // signal an ACK
static inline void   set_power_on();               // set NOPOWER LOW
static inline void   set_power_off();              // set NOPOWER HIGH
static inline UINT8  wait_for_request();           // wait for RDR or PUN request
static inline void   put_pts_ch(const UINT8 ch);   // send from paper tape reader
static inline UINT8  get_pts_ch();                 // receive from paper tape punch
static inline UINT8  Wait_for_request();           // wait for reader or punch request
static inline UINT8  teletype();                   // TRUE if teletype selected

void loopback_test();
void reader_test();
void punch_test();



/**********************************************************/
/*                           MAIN                         */
/**********************************************************/

int main() {

  static UINT32 restarts = 0;

  bi_decl(bi_program_description("Elliott 900 PTS Emulator by Andrew Herbert"));

  stdio_init_all(); // initialise stdio
  set_up_gpios(); // configure interface to outside world

  // long jump to here resets simulation
  setjmp(jbuf);

  // 4 blinks to signal waking up
  for ( UINT8 i = 1 ; i <= 4 ; i++ )
    {
      led_on();
      sleep_ms(250);
      led_off();
      sleep_ms(250);
    }
    
  while ( !tud_cdc_connected() ) sleep_ms(100); // wait for usb to wake up

  // set local flags based on external inputs
  logging_enabled = logging();     // print logging messages to usb?
  printf("\n\n\n\nPTS logging = %d\n", logging_enabled);
  logging_enabled = TRUE; 
  if ( logging_enabled )
    printf("\n\n\nPicoPTS Starting (%u)\n", ++restarts);

  // local tests
  loopback_test();
  longjmp(jbuf, 0);
 
  // power cycle 920M to reset it
  puts("Powering off");
  set_power_off();
  sleep_ms(1000); // give 920M time to respond
  puts("Powering on");
  set_power_on();

  // remote tests
  reader_test();
  //punch_test();
    
  longjmp(jbuf, 0);
  
  while ( TRUE ) // run emulation for ever
  {
    led_on(); // set LED to indicate running

    /* main loop */
    
    // will only end up here on a failure or reset, in either case, restart
  }
}

/* The following three test routines are only for use during
the development phase of pico900.  They will not be used in the operational
system */

void loopback_test()
{
  UINT32 cycles = 50000, errors, ch;
  absolute_time_t start;
  UINT64 time_in_us;
  
  puts("picopts loopback test");

  start = get_absolute_time();
  for ( UINT32 i = 0 ; i < cycles ; i++ )
    {
      errors = 0;
      for ( UINT32 j = 0 ; j < 256 ; j++ )
	{
	  //
	  gpio_put_masked(RDR_PINS_MASK, j<<RDR_1_PIN); // write 8 bits
	  busy_wait_us(0);
	  ch = (gpio_get_all() >> PUN_1_PIN) & 255; // read 8 bits
	  if ( ch != j )
	    {
	      printf("Cycle %6d,%3d: sent %3d (%4o), got %3d (%4o)\n",
		     i, j, j, j, ch, ch);
	      if ( ++errors > 10 )
		{
		  sleep_ms(10000);
		  puts("Giving up after errors");
		  return;
		}
	    }
	}
      }
  time_in_us = absolute_time_diff_us(start, get_absolute_time());
  
  printf("Loopback test complete after %d cycles, %.1f uS per cycle\n",
	 cycles, ((float) time_in_us) / ((float) (cycles * 256)));
}
	
void reader_test()
{
  puts("Reader test starting");
  for ( UINT32 c = 0 ; c < 10000 ; c++ )
    {
      printf("Cycle %u (%u)\n", c, c%256);
      if ( wait_for_request() != 0 )
	{
	  puts("Got punch request in reader test!");
	  longjmp(jbuf, 0);
	}
				       
      put_pts_ch(c%256);
      sleep_us(10);
    }
  puts("Reader tests complete");
}


void punch_test()
{
  UINT8 errors = 0;
  puts("Punch test starting");
  // simple test loop emulating read
  for ( UINT32 c = 0 ; c < 1000 ; c++ )
    {
      printf("Cycle %u (%u)\n", c, c%256);
      int ch;
      if ( wait_for_request() != 1 )
	{
	  puts("Got read request in punch test!");
	  longjmp(jbuf,0);
	}
	ch = get_pts_ch(c);
      if  (ch != (c%256) ) 
	{
	  printf("Failed after %d got %d, expected %d\n",c, ch, c%256);
          c++;
	  if ( ++errors > 10 )
	    {
	      puts("Giving up");
	      longjmp(jbuf, 0);
	    }
	 }
      sleep_us(10);
    }
  puts("Punch test complete");
}

void in_signal_test()
{
  static int cycle = 0;
  puts("In Signal Test");
  while ( TRUE )
    {
      printf("%PicoPTS: %4d: RDRREQ %d PUNREQ %d TTYSEL %d PUNCH %3d\n",
	     cycle++,
             gpio_get(RDRREQ_PIN),
             gpio_get(PUNREQ_PIN),
	     gpio_get(TTYSEL_PIN),
             gpio_get(PUN_1_PIN)                 |
	              (gpio_get(PUN_2_PIN)<<1)   |
	              (gpio_get(PUN_4_PIN)<<2)   |
	              (gpio_get(PUN_8_PIN)<<3)   |
	              (gpio_get(PUN_16_PIN)<<4)  |
	              (gpio_get(PUN_32_PIN)<<5)  |
	              (gpio_get(PUN_64_PIN)<<6)  |
	              (gpio_get(PUN_128_PIN)<<7));
      sleep_ms(1000);
    }
}

void out_signal_test()
{
  static int cycle = 0;
  puts("Out Signal Test");
  while ( TRUE )
    {
      printf("PTS %4d\n", cycle++);
      gpio_put(RDR_1_PIN, 1);     sleep_ms(1200);
      gpio_put(RDR_2_PIN, 1);     sleep_ms(1200);
      gpio_put(RDR_4_PIN, 1);     sleep_ms(1200);
      gpio_put(RDR_8_PIN, 1);     sleep_ms(1200);
      gpio_put(RDR_16_PIN, 1);    sleep_ms(1200);
      gpio_put(RDR_32_PIN, 1);    sleep_ms(1200);
      gpio_put(RDR_64_PIN, 1);    sleep_ms(1200);
      gpio_put(RDR_128_PIN, 1);   sleep_ms(1200);
      gpio_put(NOPOWER_PIN, 1);   sleep_ms(1200);
      gpio_put(ACK_PIN, 1);       sleep_ms(1200);
      gpio_put(II_AUTO_PIN, 1);   sleep_ms(1200);
      gpio_put(RDR_1_PIN, 0);
      gpio_put(RDR_2_PIN, 0);
      gpio_put(RDR_4_PIN, 0);
      gpio_put(RDR_8_PIN, 0);
      gpio_put(RDR_16_PIN, 0);
      gpio_put(RDR_32_PIN, 0);
      gpio_put(RDR_64_PIN, 0);
      gpio_put(RDR_128_PIN, 0);
      gpio_put(NOPOWER_PIN, 0);
      gpio_put(ACK_PIN, 0);
      gpio_put(II_AUTO_PIN, 0);    sleep_ms(1200);
    }
}  


/**********************************************************/
/*                    PAPER TAPE SYSTEM                   */
/**********************************************************/


/*  Output a character to paper tape station */ 
/*  i.e., punch or tty output                */

static inline void put_pts_ch(const UINT8 ch)
{
  puts("put_pts_ch");
  gpio_put_masked(RDR_PINS_MASK, ch << RDR_1_PIN); // write 8 bits
  ack();
}

/* Input a character from the paper tape station */
/* i.e., reader or tty input                     */

static inline UINT8 get_pts_ch()
{
  static UINT8 ch;
  puts("get_pts_ch");
  ch = (gpio_get_all() >> PUN_1_PIN) & 255; // write 8 bits
  ack();
  return(ch);
}


/**********************************************************/
/*                         PICO GPIO                      */
/**********************************************************/


/*  initialize GPIOs - n.b. no point in pulling up/down inputs */

static inline void set_up_gpios()
{
  UINT32 in_pins_mask = 0, out_pins_mask = 0;
  // calculate masks
  for ( UINT8 i = 0; i < IN_PINS;  i++ ) in_pins_mask  |= (1 << in_pins[i]);
  for ( UINT8 i = 0; i < OUT_PINS; i++ ) out_pins_mask |= (1 << out_pins[i]);
  // initialize GPIOs
  gpio_init_mask(in_pins_mask | out_pins_mask);
  // set up GPIO directions
  gpio_set_dir_masked(in_pins_mask | out_pins_mask,
		      out_pins_mask); 
  // set all outputs 0
  gpio_clr_mask(out_pins_mask);
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

static inline UINT8 logging() {
  return gpio_get(LOG_PIN);
}

/* Power status signalling */

static inline void set_power_on() {
  gpio_put(NOPOWER_PIN, 0);
}

static inline void set_power_off() {
  gpio_put(NOPOWER_PIN, 1);
}

/* Acknowledge a transfer */

static inline UINT8 ack() {
  gpio_put(ACK_PIN, 1);
  sleep_us(5);
  gpio_put(ACK_PIN, 0);
}

/* Wait for transfer request */

static inline UINT8 wait_for_request()
{
  UINT32 pins;
  while ( TRUE )
    {
      if ( gpio_get(RDRREQ_PIN) == 1 )
        return 0;
      else if (gpio_get(PUNREQ_PIN) == 1 )
        return 1;
      sleep_us(1);
    }
}

/* Status of TTYSel */
static inline UINT8 teletype()
{
  return gpio_get(TTYSEL_PIN);
}

	
       


