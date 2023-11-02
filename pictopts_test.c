// Elliott 900 Paper Tape Station emulator for Raspberry Pi Pico

// TEST SUITE

// Copyright (c) Andrew Herbert - 29/10/2022

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
// us

/**********************************************************/
/*                     HEADER FILES                       */
/**********************************************************/


#include <stdio.h>
#include <inttypes.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "pico/binary_info.h"
#include "pico/multicore.h"
#include <setjmp.h>


/**********************************************************/
/*                         DEFINES                        */
/**********************************************************/


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
#define PUN_1_PIN   10 // Set lsb of punch output
#define PUN_2_PIN   11 // these pins are assumed consecutive 
#define PUN_4_PIN   12
#define PUN_8_PIN   13
#define PUN_16_PIN  14
#define PUN_32_PIN  15
#define PUN_64_PIN  16 
#define PUN_128_PIN 17 // Pico sets to msb of punch output
#define ACK_PIN     19 // pulse HIGH to acknowledge RDR or PUN request
// GPIO18 IO LED
#define II_AUTO_PIN 20 // set HIGH to autostart on reset, or LOW to execute
                       // initial instructions
#define TTYSEL_PIN  21 // Pico sets HIGH to select teleprinter,
                       // LOW for paper tape
#define PUNREQ_PIN  22 // Pico sets HIGH to request punch output and awiats ACK
// There is no GPIO23, GPIO24
#define LED_PIN     25 // onboard LED    
#define RDRREQ_PIN  26 // Pico sets HIGH to request reader input and awaits ACK
#define LOG_PIN     27 // set HIGH to enable logging
// GPIO28 STATUS LED

#define IN_PINS  11 // count of input pins
#define OUT_PINS 11 // count of output pins


/**********************************************************/
/*                         GLOBALS                        */
/**********************************************************/


const uint32_t out_pins[OUT_PINS] =
  { ACK_PIN, II_AUTO_PIN, LED_PIN, 
    RDR_1_PIN,  RDR_2_PIN,  RDR_4_PIN,  RDR_8_PIN,
    RDR_16_PIN, RDR_32_PIN, RDR_64_PIN, RDR_128_PIN };

const uint32_t in_pins[IN_PINS] =
  { RDRREQ_PIN, PUNREQ_PIN, TTYSEL_PIN, PUN_1_PIN,
    PUN_2_PIN,  PUN_4_PIN,  PUN_8_PIN,  PUN_16_PIN,
    PUN_32_PIN, PUN_64_PIN, PUN_128_PIN };

uint32_t in_pins_mask = 0, out_pins_mask = 0; // initialized in setup_gpios


/**********************************************************/
/*                         FUNCTIONS                      */
/**********************************************************/

static  void     setup_gpios();                // initialize GPIO interface 
static void      signals(uint32_t);            // display current input signals
static void      pins_test();                  // check GPIO pins working
static void      monitor();                    // monitor activity


/**********************************************************/
/*                           MAIN                         */
/**********************************************************/

int main() {

  int32_t fail_code;  // failure type ferom longjmp

  bi_decl(bi_program_description("Elliott 900 PTS Emulator by Andrew Herbert"));

  stdio_init_all(); // initialise stdio
  setup_gpios(); // configure interface to outside world
  printf("\n\n\nPicoPTS Test Starting\n");
  multicore_launch_core1(pins_test); // run test on core1
  monitor();
  /* System stops if get here */
}


/**********************************************************/
/*                         TESTING                        */
/**********************************************************/

static void pins_test()
{
  gpio_put_masked(out_pins_mask, 0);
  sleep_ms(1500);
    while ( TRUE )
    {
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

static void monitor()
{
  uint32_t last = 0, next= ~0;
  for ( uint32_t tick = 1 ; ; tick++ )
    {
      // monitor when pins change
      next = gpio_get_all() & in_pins_mask;
      if ( last != next )
	{
	  signals(next);
	  last = next;
	}
      sleep_ms(500);
    }
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
  // set pull downs on request lines to avoid spurious signals
  gpio_pull_down(RDRREQ_PIN);
  gpio_pull_down(PUNREQ_PIN);
  gpio_pull_down(TTYSEL_PIN);
  gpio_pull_down(PUN_1_PIN);
  gpio_pull_down(PUN_2_PIN);
  gpio_pull_down(PUN_4_PIN);
  gpio_pull_down(PUN_8_PIN);
  gpio_pull_down(PUN_16_PIN);
  gpio_pull_down(PUN_32_PIN);
  gpio_pull_down(PUN_64_PIN);
  gpio_pull_down(PUN_128_PIN);
}
