// Elliott 900 Paper Tape Station  emulator for Raspberry Pi Pico

// Copyright (c) Andrew Herbert - 31/10/2023

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
// An additional GPIO pin LOG_PIN, high signals that diagnostic
// logging messages should be sent to the serial port.
//
// The STATUS LED (STATUS_LED_PIN) is used to signal emulation status.
// A regular one second flash indicates emulation is running.
// A fast 0.25 second flash indicates halted after an internal
// error.

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
// send D                          Toggle data rate between fast and slow
//
//                 send L text\n   treat text as 7 bit ASCII
//                                 logging message
//
//                 send R          request tape reader input
// data                            next 8 bit reader character
//
//                 send S          request teletype input
// data                            next 8 bit reader character
//
//                 send P data     request punch output of
//                                 8 bit punch character
//
//                 send Q data     request teletype output of
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
// Empirically the minimum time is 2uS.
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

#define IO_PIN      18 // set HIGH during data transfersdata transfer LED
#define ACK_PIN     19 // pulse HIGH to acknowledge RDR or PUN request
#define II_AUTO_PIN 20 // set HIGH to autostart on reset, or LOW to execute
                       // initial instructions
#define TTYSEL_PIN  21 // Pico sets HIGH to request reader input and awaits ACK
#define RDRREQ_PIN  22 // Pico sets HIGH to request punch output and awaits ACK
// There is no GPIO23, GPIO24
#define LED_PIN     25 // onboard LED  
#define PUNREQ_PIN  26 // Pico sets HIGH to select teleprinter,
                       // LOW for paper tape
#define LOG_PIN     27 // set HIGH to enable logging
#define STATUS_PIN  28 // emulation status LED
// GPIO28 spare

#define RDR_1_BIT    (1 << RDR_1_PIN)
#define PUN_1_BIT    (1 << PUN_1_PIN)
#define RDRREQ_BIT   (1 << RDRREQ_PIN)
#define PUNREQ_BIT   (1 << PUNREQ_PIN)
#define TTYSEL_BIT   (1 << TTYSEL_PIN)
#define REQ_BITS     (RDRREQ_BIT | PUNREQ_BIT)

#define READ  1 // return values from wait_for_request
#define PUNCH 2

#define COMMAND_FAIL        1
#define READ_PROTOCOL_FAIL  2
#define PUNCH_PROTOCOL_FAIL 3

const char* error_messages[] =
                          { "1 - Unknown operator command",
			    "2 - Read protocol error",
			    "3 - Punch protocol error"};

// Blink rates
#define FAST_BLINK  250 // ms
#define SLOW_BLINK 1000 // ms
#define NO_BLINK      0

/**********************************************************/
/*                         GLOBALS                        */
/**********************************************************/

#define IN_PINS  12 // count of input pins
#define OUT_PINS 13 // count of output pins


const uint32_t in_pins[IN_PINS] =
  { RDRREQ_PIN, PUNREQ_PIN, TTYSEL_PIN,  PUN_1_PIN,
    PUN_2_PIN,  PUN_4_PIN,  PUN_8_PIN,   PUN_16_PIN,
    PUN_32_PIN, PUN_64_PIN, PUN_128_PIN, LOG_PIN };

const uint32_t out_pins[OUT_PINS] =
  { ACK_PIN,   II_AUTO_PIN, IO_PIN,      STATUS_PIN, LED_PIN,
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

uint32_t blink = NO_BLINK;

/**********************************************************/
/*                         FUNCTIONS                      */
/**********************************************************/

static void     setup_gpios();                  // initialize GPIO interface
static uint32_t poll_for_request();             // poll for request from 920M
static uint32_t read_request(const uint32_t request); // test if reader request
static uint32_t punch_request(const uint32_t request);// test if punch request
static uint32_t tty_request(const uint32_t request); // test if teletype request
static void     led_on();                       // turn on onboard LED
static void     status_on();                    // turn on STATUS LED
static void     status_off();                   // turn off STATUS LED
static void     io_on();                        // turn on IO LED
static void     io_off();                       // turn off IO LED
static void     status(const uint32_t onoff);   // set STATUS_LED
static void     pack_reader_data(const uint32_t ch);
static uint32_t unpack_punch_data(const uint32_t request);
static void     blinker();                      // blink status LED 
static uint32_t logging();                      // TRUE is logging enabled
static uint32_t ack();                          // signal an ACK
static void     cancel_ack();                   // remove ACK if present
static void     put_pts_ch(const uint32_t tty);
                                                // send reader character 
static uint32_t get_pts_ch(const uint32_t ch, const uint32_t tty);
                                                // receive punch character
static void     pts_emulation();                // paper tape station emulatio

// Testing routines
static void     loopback_test();                
static void     on(const char* name, const uint32_t pin);
static void     off(const uint32_t pin);
static void     pause();
static void     check(const char* pin, const uint32_t bit);
static void     signals(const uint32_t pins);   


/**********************************************************/
/*                           MAIN                         */
/**********************************************************/


int main() {

  int32_t fail_code;  // failure type ferom longjmp

  bi_decl(bi_program_description("Elliott 900 PTS Emulator by Andrew Herbert"));

  stdio_init_all(); // initialise stdio
  setup_gpios(); // configure interface to outside world
  led_on(); // show Pico is alive
  sleep_ms(250); // allow serial link to stabilise
  
  loopback_test();

  multicore_launch_core1(blinker); // set LED blinker running
  pts_emulation(); // set paper tape station running
  /* NOT REACHED */
}


/**********************************************************/
/*                         TESTING                        */
/**********************************************************/

static void loopback_test()
{
  printf("PicoPTS Loopback Test\n");
  while ( TRUE )
    {
      printf((logging() ? "Logging\n" : "Not logging\n"));
      on("STATUS", STATUS_PIN); pause(); off(STATUS_PIN);printf("\n");
      on("IO", IO_PIN); pause(); off(IO_PIN);printf("\n");
      on("II_AUTO", II_AUTO_PIN); check("TTYSEL", TTYSEL_BIT); off(II_AUTO_PIN);
      on("ACK", ACK_PIN); check("RDRREQ", RDRREQ_BIT); off(ACK_PIN);
      on("RDR_1",RDR_1_PIN); check("PUN_1", PUN_1_BIT);off(RDR_1_PIN);
      on("RDR_2",RDR_2_PIN); check("PUN_2", PUN_1_BIT<<1);off(RDR_2_PIN);
      on("RDR_4",RDR_4_PIN); check("PUN_4", PUN_1_BIT<<2);off(RDR_4_PIN);
      on("RDR_8",RDR_8_PIN); check("PUN_8", PUN_1_BIT<<3);off(RDR_8_PIN);
      on("RDR_16",RDR_16_PIN); check("PUN_16", PUN_1_BIT<<4);off(RDR_16_PIN);
      on("RDR_32",RDR_32_PIN); check("PUN_32", PUN_1_BIT<<5);off(RDR_32_PIN);
      on("RDR_64",RDR_64_PIN); check("PUN_64", PUN_1_BIT<<6);off(RDR_64_PIN);
      on("RDR_128",RDR_128_PIN); check("PUN_128", PUN_1_BIT<<7);off(RDR_128_PIN);
      printf("\n\n");
    }
}

inline static void on(const char* name, const uint32_t pin)
{
  printf("%s", name);
  gpio_put(pin, 1);
  sleep_ms(0); // give time for level shifter to change
}

inline static void off(const uint32_t pin)
{
  gpio_put(pin, 0);
  sleep_ms(0); // give time for level shifter to change
}
  
inline static void pause()
{
  sleep_ms(500);
}

inline static void check(const char* pin, const uint32_t bit)
{
  int32_t inputs = (gpio_get_all() & in_pins_mask);
  if ( ((inputs & (RDRREQ_BIT | TTYSEL_BIT | (255 << PUN_1_PIN))) == bit) ) {
    printf("\n", pin);
    }
  else {
    printf(" Not matched by %s\n", pin);
    signals(inputs);
    while ( TRUE ) sleep_ms(100000);
  }
}

static void signals(const uint32_t pins)
{
  printf("TTYSEL %1u RDRREQ %1u PUNREQ %1u PUN DATA %3d ",
	 (pins >> TTYSEL_PIN) & 1,
	 (pins >> RDRREQ_PIN) & 1,
	 (pins >> PUNREQ_PIN) & 1,
	 (pins >> PUN_1_PIN) & 255);
  for ( uint32_t bit = 0 ; bit < 8 ; bit++ )
    printf("%1u",(pins >> (PUN_128_PIN-bit)) & 1);
  printf("\n");
}


/**********************************************************/
/*                    PAPER TAPE SYSTEM                   */
/**********************************************************/

static void pts_emulation()
{
  int32_t fail_code; // set by longjmp

  // long jump to here on error
  if ( fail_code = setjmp(jbuf) ) { // test if a longjmp occurred
    blink = FAST_BLINK;
    cancel_ack(); //   and abort any transfer
    io_off();
    if ( logging() ) {
      printf("LPicoPTS - Halted after error - %s\n",
	     error_messages[fail_code-1]);
      while ( TRUE ) sleep_ms(INT32_MAX);
    }     
  }

  blink = SLOW_BLINK; // indicate emulation running
  reader_free = punch_free = tty_free
                    = get_absolute_time(); // set device timer
  // reset operator terminal
  printf("\x00\n"); // 0 to complete a read, \n to complete a log

  if ( logging() ) printf("\n\n\nLPicoPTS - Starting emulator\n");

  // start polling
  while ( TRUE ) {
    int32_t data, request;
    switch ( data = getchar_timeout_us(0) )
      {
      case PICO_ERROR_TIMEOUT:
	break;
      case 0:
	if  ( logging() ) printf("LPicoPTS - NUL ignored\n");
	break;
      case 255:
	if  ( logging() ) printf("LPicoPTS - DEL ignored\n");
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
      }

    // look for a request
    request = poll_for_request();
    if ( read_request(request) ) 
      put_pts_ch(tty_request(request)); // read input	
    else 
      get_pts_ch(unpack_punch_data(request),
		 tty_request(request)); // punch output
  }
}

/*  Send a character to 920M                 */ 
/*  i.e., reader or tty input                */

static inline void put_pts_ch (const uint32_t tty)
{
  uint32_t ch;
  io_on();
  // wait until device ready
  if ( tty ) {
    sleep_until(tty_free); // wait until tty idle
    tty_free = make_timeout_time_us(tty_time); // set tty busy
  }
  else {
    sleep_until(reader_free); // wait until reader idle
    reader_free = make_timeout_time_us(reader_time); // set reader busy
  }
  // request data from device
  if ( read_request(poll_for_request()) ) // check request is still present
       putchar_raw(tty ? 'S' : 'R');      // S for tty read, R for ptr read
  else {
    longjmp(jbuf, READ_PROTOCOL_FAIL);
    /* NOT REACHED */
  }
  // wait for character response from device
  while ( (ch = getchar_timeout_us(1000)) == PICO_ERROR_TIMEOUT ) {
    if ( read_request(poll_for_request()) )
      continue;
    else {
      longjmp(jbuf, READ_PROTOCOL_FAIL);
      /* NOT REACHED */
    }
    io_off();
  }
  // send data to 920M
  pack_reader_data(ch); // send 8 bits to 920M
  ack();
  io_off();
}

/* Receive a character from the 920M             */
/* i.e., punch or tty output                     */

static inline uint32_t get_pts_ch (const uint32_t ch, const uint32_t tty)
{
  io_on();
  // wait for device to be ready  
  if ( tty ) {
    sleep_until(tty_free); // wait until tty idle
    tty_free = make_timeout_time_us(tty_time); // set tty busy
  }
  else {
    sleep_until(punch_free); // wait until punch idle
    punch_free = make_timeout_time_us(punch_time); // set punch busy
  }
  putchar_raw(tty ? 'Q' : 'P');
                     // Q for tty write, P for punch write
  putchar_raw(ch);  // data to punch
  // only ACK if request still present
  if ( punch_request(poll_for_request()) ) {
    ack();
    io_off();
    return ch;
  }
  else {
    longjmp(jbuf, PUNCH_PROTOCOL_FAIL);
    /* NOT REACHED */
  }
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
  // set pull up on LOG_PIN so defaults to logging enabled
  gpio_pull_up(LOG_PIN);
  // set pull downs on request lines to avoid spurious signals
  gpio_pull_down(RDRREQ_PIN);
  gpio_pull_down(PUNREQ_PIN);
  gpio_pull_down(TTYSEL_PIN);
}

/* Read logging status */

static inline uint32_t logging() {
  return gpio_get(LOG_PIN);
}

/* Acknowledge a transfer */

static inline uint32_t ack()
{
  gpio_put(ACK_PIN, 1);
  sleep_us(ACK_TIME);
  gpio_put(ACK_PIN, 0);
}

static inline void cancel_ack() {
  gpio_put(ACK_PIN, 0);
}

/* Request decoding */

static inline uint32_t poll_for_request() {
  return gpio_get_all();
}

static inline uint32_t read_request(const uint32_t request) {
  return request & RDRREQ_BIT;
}

static inline uint32_t punch_request(const uint32_t request) {
  return request & PUNREQ_BIT;
}

static inline uint32_t tty_request(const uint32_t request) {
  return request & TTYSEL_BIT;
}

static inline void led_on() {
  gpio_put(LED_PIN, 1);
}

static inline void status_on() {
  gpio_put(STATUS_PIN, 1);
}

static inline void status_off() {
  gpio_put(STATUS_PIN, 0);
}

static inline void io_on() {
  gpio_put(IO_PIN, 1);
}

static inline void io_off() {
  gpio_put(IO_PIN, 0);
}

static inline void status_led(const uint32_t onoff) {
  gpio_put(STATUS_PIN, onoff);
}

static inline void pack_reader_data (const uint32_t ch) {
  gpio_put_masked(RDR_PINS_MASK, ch << RDR_1_PIN);
}

static inline uint32_t unpack_punch_data (const uint32_t request) {
  (request & PUN_PINS_MASK) >> PUN_1_PIN;
}


/**********************************************************/
/*                          BLINKER                       */
/**********************************************************/

static inline void blinker()
{
  static uint32_t led_state = 0;
  while (TRUE)
    if ( blink == NO_BLINK) {
      led_state = 0;
      status_led(0);
      sleep_ms(SLOW_BLINK);
    }
    else {
      led_state = (led_state+1)&1;
      status_led(led_state);
      sleep_ms(blink);
    }
}
