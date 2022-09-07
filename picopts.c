// Elliott 900 Paper Tape Station  emulator for Raspberry Pi Pico

// Copyright (c) Andrew Herbert - 07/09/2022

// MIT Licence.

// Emulator for Elliott 900 Series computer Paper Tape Station.

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
// PUNREQ_PIN, high signals a punch request. The paper tape
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
// 8177. If low the computer should obey the initial orders
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
// An additional GPIO pin LOG_PIN, high signals that diagnostic
// logging messages should be sent to the serial port.
//
// When an emulation is running the onbaord LED is illumintaed,
// otherwise the LED is extinguished.

// The Pico is equipped with a push button which can used used
// to force a restart of the Pico system.  This completely
// restarts the emulation system from the beginning.  It can be
// used to restart after a dynamic stop, infinite loop or
// catastrophic error.

// PicoPTS is intended to be connected to an "900 Operator"
// application via the serial port.  The protocol is as follows:
//
// Operator        PicoPTS         Effect
// -------         -------         ------
//
// send N                          Set NOPOWER HIGH for 1 sec
//                                 (to reset 920M and clear any
//                                  pending R or S command))
//
//                 send L text\n   treat text as 7 bit ASCII
//                                 logging message
//
//                 send R          request tape reader input
// data                            next 8 bit reader character
//
//                 send Sx         request teletype input
// data                            next 8 bit reader character
//
// 255, N    = NOPOWER break
// 255, 255  = data  255
//
//                 send Pdata      request punch output of
//                                 8 bit punch character
//
//                 send Qdata      request teletype output of
//                                 8 bit reader character



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

// Booleans
#define TRUE  1
#define FALSE 0

// Duration of ACK pulse, should be at least 1uS
// so 920M can detect the ACK
#define ACK_TIME     2 // microseconds

// Duration of i/o operations in microseconds
// These can be modified but need to be long
// enough so that PTS doesn't overrun the 920M.
// Empiracally the minimum time is 2uS.
#define FAST_TTY          5
#define SLOW_READER    4000  //  250 ch/s
#define FAST_READER       5
#define SLOW_TTY     100000  //   10 ch/s
#define FAST_PUNCH        5  
#define SLOW_PUNCH     9091  //  110 ch/s

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

#define RDR_PINS_MASK (255 << RDR_1_PIN) // PINs to bit mask 

#define PUN_1_PIN   10 // Set lsb of punch output
#define PUN_2_PIN   11 // these pins are assumed consecutive 
#define PUN_4_PIN   12
#define PUN_8_PIN   13
#define PUN_16_PIN  14
#define PUN_32_PIN  15
#define PUN_64_PIN  16 
#define PUN_128_PIN 17 // Pico sets to msb of punch output

#define PUN_PINS_MASK (255 << PUN_1_PIN) // PINs to bit mask

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

#define RDRREQ_BIT   (1 << RDRREQ_PIN)
#define PUNREQ_BIT   (1 << PUNREQ_PIN)
#define TTYSEL_BIT   (1 << TTYSEL_PIN)
#define REQ_BITS     (RDRREQ_BIT | PUNREQ_BIT)
#define ACK_BIT      (1 << ACK_PIN)
#define NOPOWER_BIT  (1 << NOPOWER_PIN)

#define READ  1 // return values from wait_for_request
#define PUNCH 2

#define COMMAND_FAIL      1
#define PROTOCOL_FAIL     2

const char* error_messages[] =
                          { "1 - Unknown operator command",
			    "2 - program exited" };


/**********************************************************/
/*                         GLOBALS                        */
/**********************************************************/

#define IN_PINS  12 // count of input pins
#define OUT_PINS 12 // count of output pins


const uint32_t in_pins[IN_PINS] =
  { RDRREQ_PIN, PUNREQ_PIN, TTYSEL_PIN,  PUN_1_PIN,
    PUN_2_PIN,  PUN_4_PIN,  PUN_8_PIN,   PUN_16_PIN,
    PUN_32_PIN, PUN_64_PIN, PUN_128_PIN, LOG_PIN };

const uint32_t out_pins[OUT_PINS] =
  { NOPOWER_PIN, ACK_PIN,   II_AUTO_PIN, LED_PIN, 
    RDR_1_PIN,  RDR_2_PIN,  RDR_4_PIN,   RDR_8_PIN,
    RDR_16_PIN, RDR_32_PIN, RDR_64_PIN,  RDR_128_PIN };

uint32_t in_pins_mask = 0, out_pins_mask = 0; // initialized in setup_gpios

jmp_buf jbuf;                   // used by setjmp in main

// I/O timing - all devices initially idle
absolute_time_t reader_free, punch_free, tty_free;

// Set standard 900 series peripherals
uint32_t reader_time = SLOW_READER,
         punch_time  = SLOW_PUNCH,
         tty_time    = SLOW_TTY;


/**********************************************************/
/*                         FUNCTIONS                      */
/**********************************************************/

static  void     setup_gpios();                // initialize GPIO interface 
static  void     led_on();                     // turn onboard LED on
static  void     led_off();                    // turn onboard LED off
static  uint32_t logging();                    // TRUE is logging enabled
static  uint32_t ack();                        // signal an ACK
static  void     stop_computer();              // assert NOPOWER
static  void     restart_computer();           // stop, then assert -NOPOWER
static  void     put_pts_ch(const uint32_t ch, const uint32_t request);
                                               // send character 
static  uint32_t get_pts_ch(const uint32_t request);
                                               // receive character
static  uint32_t wait_for_request();           // wait for RDR or PUN request
static  void     pts_emulation();              // paper tape station emulation


/**********************************************************/
/*                           MAIN                         */
/**********************************************************/


int main() {

  int32_t fail_code;  // failure type ferom longjmp

  bi_decl(bi_program_description("Elliott 900 PTS Emulator by Andrew Herbert"));

  stdio_init_all(); // initialise stdio
  setup_gpios(); // configure interface to outside world

  led_off();
  
  reader_free = punch_free = tty_free = get_absolute_time(); // set device
                                                             // timer
  pts_emulation();
  
  /* NOT REACHED */
  
}


/**********************************************************/
/*                    PAPER TAPE SYSTEM                   */
/**********************************************************/

static void pts_emulation()
{
  int32_t fail_code; // set by longjmp

  // long jump to here on error
  if ( fail_code = setjmp(jbuf) ) { // test if a longjmp occurred
    led_off();
    stop_computer();      // stop 920M
    gpio_put(ACK_PIN, 0); //   and abort any transfer

    if ( logging() ) {
      printf("LPicoPTS - Halted after error - %s\n",
	     error_messages[fail_code-1]);
      while ( TRUE ) sleep_ms(1000);
    };    
    
  };

  if ( logging() ) printf("LPicoPTS - Starting\n");
  led_on();
  
  // start polling
  while ( TRUE ) {
    int32_t data, request;
    switch ( data = getchar_timeout_us(0) )
      {
      case PICO_ERROR_TIMEOUT:
	break;
      case 255:
	if  ( logging() ) printf("LPicoPTS - DEL ignored\n");
	break;
      case 'N':
	restart_computer();
	break;
      case 'D':
	if ( reader_time == FAST_READER) {
	  if ( logging() )
	    printf("LPicoPTS - switching to slow devices\n");
	  reader_time = SLOW_READER;
	  tty_time = SLOW_TTY;
	  punch_time = SLOW_PUNCH;
	}
	else {
	  if ( logging() )
	    printf("LPicoPTS - switching to fast devices\n");
	  reader_time = FAST_READER;
	  tty_time    = FAST_TTY;
	  punch_time  = FAST_PUNCH;
	};
	break;
      default:
	if ( logging() )
	  printf("LPicoPTS - Unrecognized command %d\n", data);
	else
	  longjmp(jbuf, COMMAND_FAIL);
	
	/* NOT REACHED */
      };

    // look for a request
    if ( (request = gpio_get_all()) & RDRREQ_BIT ) {
      putchar_raw(( request & TTYSEL_BIT ) ? 'S' : 'R');
                                         // S for tty read, R for ptr read
      // wait for character response
      while ( (data = getchar_timeout_us(1000000)) == PICO_ERROR_TIMEOUT );
      // reads can be interrupted by a computer restart
      if (  data == 255  ) {
	while ( (data = getchar_timeout_us(1000000)) == PICO_ERROR_TIMEOUT );
	if ( data == 255 )
	  put_pts_ch(data, request); // 255, 255  = 255 (DEL)
	else
	  restart_computer(); // 255, 0 = restart	
      }
      else {
	put_pts_ch(data, request);
      }
    }
    else if ( request & PUNREQ_BIT ) {
      putchar_raw((request & TTYSEL_BIT) ? 'Q' : 'P');
                                        // Q for tty write, P for punch write
      putchar_raw(get_pts_ch(request)); // wait for handshake
      while ( (data = getchar_timeout_us(1000000000)) == PICO_ERROR_TIMEOUT );
      if ( data ) restart_computer();  // operator wants to restart
    };
  };
}

/*  Send a character to 920M                 */ 
/*  i.e.,reader or tty input                 */

static inline void put_pts_ch(const uint32_t ch, const uint32_t request)
{
  if ( request & TTYSEL_BIT) {
    sleep_until(tty_free); // wait until tty idle
    tty_free = make_timeout_time_us(tty_time); // set tty busy
  }
  else {
    sleep_until(reader_free); // wait until reader idle
    reader_free = make_timeout_time_us(reader_time); // set reader busy
  }
  gpio_put_masked(RDR_PINS_MASK, ch << RDR_1_PIN); // write 8 bits
  ack();
}

/* Receive a character from the 920M             */
/* i.e., punch or tty output                     */

static inline uint32_t get_pts_ch(const uint32_t request)
{
  if ( request & TTYSEL_BIT) {
    sleep_until(tty_free); // wait until tty idle
    tty_free = make_timeout_time_us(tty_time); // set tty busy
  } 
  else {
    sleep_until(punch_free); // wait until punch idle
    punch_free = make_timeout_time_us(punch_time); // set punch busy
  }
  const uint32_t ch = (request & PUN_PINS_MASK) >> PUN_1_PIN; // read 8 bits
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
  return gpio_get(LOG_PIN);
}

/* Power control */

static inline void stop_computer() {
  if ( logging() ) printf("LPicoPTS - Stopping computer\n");
  gpio_put(NOPOWER_PIN, 1);
}

static inline void restart_computer()
{
  if ( logging() ) printf("LPicoPTS - Restarting computer\n");
  gpio_put(NOPOWER_PIN, 1);
  sleep_ms(1000);
  gpio_put(NOPOWER_PIN, 0);
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

