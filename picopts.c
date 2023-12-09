// Elliott 900 Paper Tape Station  emulator for Raspberry Pi Pico

// Copyright (c) Andrew Herbert - 20/11/2023

// MIT Licence.

// Emulator for Elliott 900 Series computer Paper Tape Station.

// This program is intended to run on a Raspberry Pi Pico and
// depends upon Pico specific functions and data types.

// The Pico emulates the paper tape station interface of a 920M 
// using GPIO pins.  The pin numbers are listed below.  The use 
// of each pin is as follows:

// RDRREQ_PIN, high signals a reader request.  The paper tape
// station is expected to have loaded 8 bits of data on pins
// RDR_1_PIN (lsb) to RDR_128_PIN (msb) and then raise ACK_PIN
// high for 2uS (or longer) to indicate the input data is ready.
// Once the computer has read in the data it lowers RDRREQ_PIN to
// signal transfer complete.  
//
// PUNREQ_PIN, high signals a punch request. The paper tape
// station is expected to load 8 bits of data from pins PUN_1_PIN
// (lsb) to PUN_128_PIN (msb) and then raise ACK_PIN high for
// 2-5 uS to indicate the output data have been copied. Once the
// data is copied, the computer then lowers PUNREQ_PIN.
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
// Two addition pins are used for emulation control.
// LOG_PIN, high signals that diagnostic logging messages should be
// sent to the serial port.
//
// STATUS_LED_PIN is used to signal emulation status.
// A regular one second flash indicates emulation is running.
// A fast 0.25 second flash indicates halted after an internal
// error.
//
// The onboard LED is illuminated at start once the GPIOs are
// initialized.

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
//
//                 send Z          indicate PTR (re)starting


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
#define TRUE  1u
#define FALSE 0u

// Duration of ACK pulse, should be at least 1uS
// so 920M can detect the ACK
#define ACK_TIME     2 // microseconds

// GPIO pins

// GPIO0, GPIO1 in uses as serial output
#define RDR_1_PIN    2  // Set lsb of reader input
#define RDR_2_PIN    3  // these pins are assumed consecutive
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

// GPIO bit positions
#define LOG_BIT      (1 << LOG_PIN)
#define RDR_1_BIT    (1 << RDR_1_PIN)
#define PUN_1_BIT    (1 << PUN_1_PIN)
#define RDRREQ_BIT   (1 << RDRREQ_PIN)
#define PUNREQ_BIT   (1 << PUNREQ_PIN)
#define TTYSEL_BIT   (1 << TTYSEL_PIN)
#define REQ_BITS     (RDRREQ_BIT | PUNREQ_BIT | TTYSEL_BIT)

// Useful masks
#define RDR_PINS_MASK (255 << RDR_1_PIN) // PINs to bit mask 
#define PUN_PINS_MASK (255 << PUN_1_PIN) // PINs to bit mask

// request types
#define NONE         0 // no request
#define READER       1 // RDRREQ present
#define READ_TTY     2 // RDRREQ+TTYSEL present
#define PUNCH        3 // PUNREQ present
#define WRITE_TTY    4 // PUNREQ+TTYSEL present
#define BAD          5 // Bad request (READ & PUNCH)

#define COMMAND_FAIL        1
#define READ_PROTOCOL_FAIL  2
#define PUNCH_PROTOCOL_FAIL 3
#define REQUEST_FAIL        4

const char* error_messages[] =
                          { "1 - Unknown operator command",
			    "2 - Read protocol error",
			    "3 - Punch protocol error",
			    "4 = Simultaneous read and punch requests"};

// Blink rates
#define FAST_BLINK  250 // ms
#define SLOW_BLINK 1000 // ms
#define NO_BLINK      0

/**********************************************************/
/*                         GLOBALS                        */
/**********************************************************/

#define IN_PINS  12 // count of input pins
#define OUT_PINS 13 // count of output pins

const uint32_t in_pins[IN_PINS] = // input GPIO pins
  { RDRREQ_PIN, PUNREQ_PIN, TTYSEL_PIN,  PUN_1_PIN,
    PUN_2_PIN,  PUN_4_PIN,  PUN_8_PIN,   PUN_16_PIN,
    PUN_32_PIN, PUN_64_PIN, PUN_128_PIN, LOG_PIN };

const uint32_t out_pins[OUT_PINS] = // output GPIO pins
  { ACK_PIN,   II_AUTO_PIN, IO_PIN,      STATUS_PIN, LED_PIN,
    RDR_1_PIN,  RDR_2_PIN,  RDR_4_PIN,   RDR_8_PIN,
    RDR_16_PIN, RDR_32_PIN, RDR_64_PIN,  RDR_128_PIN };

uint32_t in_pins_mask = 0, out_pins_mask = 0; // initialized in setup_gpios

jmp_buf jbuf;                   // used by setjmp in main

uint32_t blink = NO_BLINK; // blink time in ms

/**********************************************************/
/*                         FUNCTIONS                      */
/**********************************************************/

static void     setup_gpios();                  // initialize GPIO interface
static uint32_t get_request();                  // poll for request from 920M
static uint32_t request_type(const uint32_t request); // analyze request
static void     read_input(const uint32_t tty);
                                                // send reader character 
static void     punch_output(const uint32_t ch, const uint32_t tty);
                                                // receive punch character
static void     put_read_data(const uint32_t ch);
static uint32_t get_punch_data(const uint32_t request);
static void     wait_for_no_request();          // poll for absence of request
static uint32_t ack();                          // signal an ACK
static void     cancel_ack();                   // remove ACK if present
static uint32_t logging();                      // TRUE is logging enabled
static uint32_t gpio_debounce();                // debounce input GPIOs
static void     led_on();                       // turn on onboard LED
static void     status(const uint32_t onoff);   // set STATUS_LED
static void     status_on();                    // turn on STATUS LED
static void     status_off();                   // turn off STATUS LED
static void     io_on();                        // turn on IO LED
static void     io_off();                       // turn off IO LED
static void     blinker();                      // blink status LED 
static void     pts_emulation();                // paper tape station emulation
static void     halt();                         // sleep for ever

// Testing routines
/*
static void     loopback_test();                
static void     on(const char* name, const uint32_t pin);
static void     off(const uint32_t pin);
static void     pause();
static void     check(const char* pin, const uint32_t bit);
*/
static void     signals(const uint32_t pins);   


/**********************************************************/
/*                           MAIN                         */
/**********************************************************/


int main() {
  bi_decl(bi_program_description("Elliott 900 PTS Emulator by Andrew Herbert"));

  stdio_init_all(); // initialise stdio
  setup_gpios(); // configure interface to outside world
  led_on(); // show Pico is alive
  sleep_ms(250); // allow serial link to stabilise

  /* loopback_test(); */

  status_on();
  
  multicore_launch_core1(blinker); // set LED blinker running
  pts_emulation(); // set paper tape station running
  /* NOT REACHED */
}

static inline void halt() {
  if ( logging() ) printf("LPicoPTS - Halted\n");
  while ( TRUE) sleep_ms(UINT32_MAX);
}



/**********************************************************/
/*                         TESTING                        */
/**********************************************************/

/*
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
  gpio_put_masked(out_pins_mask, 0);
  pause();
  printf("%s", name);
  gpio_put(pin, 1);
  pause(); // give time for level shifter to change
}

inline static void off(const uint32_t pin)
{
  gpio_put(pin, 0);
  pause(); // give time for level shifter to change
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
    halt();
  }
}
*/

static void signals(const uint32_t pins)
{
  printf("LTTYSEL %1u RDRREQ %1u PUNREQ %1u PUN DATA %3d ",
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
  uint32_t tty; // set in request decoding
  int32_t fail_code; // set by longjmp

  // long jump to here on error
  if ( fail_code = setjmp(jbuf) ) { // test if a longjmp occurred
    blink = FAST_BLINK;
    cancel_ack(); //   clear any transfer in progress
    io_off(); // ensure i/o transfer in progress light is off
    if ( logging() ) {
      printf("LPicoPTS - Halted after error - %s\n",
	     error_messages[fail_code-1]);
      signals(gpio_get_all());
      halt();
    }     
  }

  // here on a restart

  // empty any output from device
  while ( getchar_timeout_us(0) != PICO_ERROR_TIMEOUT );
  // signal a restart
  putchar_raw('\x00'); // will terminate a P, Q
  stdio_flush();
  putchar_raw('\n'); // will terminate an L
  stdio_flush();
  putchar_raw('Z');
  stdio_flush();
 
  if ( logging() ) puts_raw("LPicoPTS - Starting emulator");

  blink = SLOW_BLINK; // indicate emulation running

  if ( logging ())
    puts_raw("LPicoPTS - Waiting for 920M to clear outstanding requests");
  wait_for_no_request();

  // start polling
  if ( logging () )
    puts_raw("LPicoPTS - Starting polling for new requests");
  
  while ( TRUE ) {
    
    uint32_t data  = getchar_timeout_us(0); // used in serial port decoding
    uint32_t request, type; // used in GPIO decoding

    // look for a reader or punch request on GPIOs
    request = get_request();
    switch (request_type(request)) {
    case NONE:  {
      gpio_put(ACK_PIN, 0);
      break;
    }
    case READER: {
      read_input(FALSE); // read from paper tape reader
      break;
    }
    case READ_TTY:  {
      read_input(TRUE); // read from teleprinter
      break;
    }
    case PUNCH: {
      punch_output(get_punch_data(request),
		   FALSE); //  output to paper tape punch
      break;
    }
    case WRITE_TTY: {
      punch_output(get_punch_data(request),
		   TRUE); //  output to teleprinter
      break;
    }
    case BAD: {
      longjmp(jbuf, REQUEST_FAIL); // should not happen
      /* NOT REACHED */
    }
    default: {
      if ( logging() )
	puts_raw("LPicoPTS - Internal error (decoding request GPIOs)");
      halt();/* NOT REACHED */
    }
    }
  }
}

/*  Send a character to 920M                 */ 
/*  i.e., reader or tty input                */

static inline void read_input (const uint32_t tty)
{
  static char buffer[256];
  static uint32_t buf_ptr = 257, buf_len = 0;
  uint32_t ch;
  io_on(); // light i/o in progress LED
  //if ( logging() ) printf("LR%s+\n", ( tty ? "TTY" : "RDR" ));
  if ( tty )
    {
      putchar_raw('S');
      stdio_flush();
      ch = getchar();
    }
  else {
    if ( buf_ptr > buf_len ) {
      // request data from device
      putchar_raw('R');      
      stdio_flush();
      // wait for character response from device
      buf_len = getchar() + 1;
      for ( uint32_t i = 1 ; i <= buf_len ; i++ )
	buffer[i] = getchar();
      ch = buffer[1];
      buf_ptr = 2;
    }
    else {
      ch = buffer[buf_ptr++];
    }
  }
  // send data to 920M
  put_read_data(ch); // send 8 bits to 920M
  ack(); 
  wait_for_no_request(); // allow 920M time to clear the request
  //if ( logging() ) printf("LR- %3d\n", ch);
  io_off(); // extinguish i/o in progress LED
}

/* Receive a character from the 920M             */
/* i.e., punch or tty output                     */

static inline void punch_output (const uint32_t ch, const uint32_t tty)
{
  io_on(); // light i/o in progress LED
  //if ( logging() ) printf("LP%s+ %3u\n", ( tty ? "TTY" : "PUN" ), ch);
  // send data to device
  putchar_raw(tty ? 'Q' : 'P');
                    // Q for tty write, P for punch write
  putchar_raw(ch);  // data to punch
  stdio_flush();
  ack(); // signal transfer complete
  wait_for_no_request();
  //if ( logging() ) printf("LP-\n");
  io_off(); // extinguish i/o in progress LED
}


/**********************************************************/
/*                         PICO GPIO                      */
/**********************************************************/


/*  initialize GPIOs */

static void setup_gpios()
{
  uint32_t external_pins_mask; 
  // calculate masks
  for ( uint32_t i = 0; i < IN_PINS;  i++ ) in_pins_mask  |= (1 << in_pins[i]);
  for ( uint32_t i = 0; i < OUT_PINS; i++ ) out_pins_mask |= (1 << out_pins[i]);
  external_pins_mask = out_pins_mask ^
                          ((1<<LED_PIN) | (1<<STATUS_PIN) | (1<<IO_PIN));
  // initialize GPIOs
  gpio_init_mask(in_pins_mask | out_pins_mask);
  // set up GPIO directions
  gpio_set_dir_masked(in_pins_mask | out_pins_mask, out_pins_mask);
  // initialize all output GPIOs to LOW
  gpio_clr_mask(out_pins_mask); 
  // set pull up on LOG_PIN so defaults to logging enabled
  gpio_pull_up(LOG_PIN);
  // set pull downs on request lines to avoid spurious signals
  gpio_pull_down(RDRREQ_PIN);
  gpio_pull_down(PUNREQ_PIN);
  gpio_pull_down(TTYSEL_PIN);
  for ( uint32_t i = 0 ; i < 8 ; i++ )  gpio_pull_down(PUN_1_PIN+i);
  /**/
  // limit slew rate and set high output for output pins
  for ( uint32_t pin = 0 ; pin < 32 ; pin++ ) {
    if ( external_pins_mask & (1<<pin) ) {
      gpio_set_slew_rate(pin, GPIO_SLEW_RATE_SLOW);
      gpio_set_drive_strength(pin, GPIO_DRIVE_STRENGTH_8MA);
    }
  }
  /**/
}

/* debounce GPIO inputs */

static inline uint32_t gpio_debounce ()
{
  uint32_t count = 1, last = 0, next  = gpio_get_all() & in_pins_mask;
  while ( TRUE ) {
    if ( count >= 2 ) return next;
    if ( last == next ) 
      count++;
    else
      count = 0;
    last = next;
    next = gpio_get_all() & in_pins_mask;
  }
}


/* Read logging status */

static inline uint32_t logging() {
  return gpio_debounce() & LOG_BIT;
}

/* Acknowledge a transfer */

static inline uint32_t ack()
{
  gpio_put(ACK_PIN, 1);
  sleep_us(ACK_TIME);
  gpio_put(ACK_PIN, 0);
}

/* Abandon a transfer */

static inline void cancel_ack() {
  gpio_put(ACK_PIN, 0);
}

/* Request decoding */

static inline uint32_t get_request() {
  return gpio_debounce();
}

/* decode type of read request (reader or tty) */

static inline uint32_t request_type(const uint32_t request) {
  uint32_t rp = request & (RDRREQ_BIT | PUNREQ_BIT);
  if ( rp == RDRREQ_BIT )
    if ( request & TTYSEL_BIT )
      return READ_TTY;
    else
      return READER;
  else if ( rp == PUNREQ_BIT )
    if ( request & TTYSEL_BIT )
      return WRITE_TTY;
    else
      return PUNCH;
  else if ( rp == 0 )
    return NONE;
  else
    return BAD; // RDRREQ+PUNREQ simultaneously
}

/* Check a request has been cleared */

static inline void wait_for_no_request() {
  uint32_t request;
  while ( ((request = gpio_debounce()) & REQ_BITS) != 0 )
    sleep_us(1u); // HACK this sleep is needed
}

/* Turn on on board LED */

static inline void led_on() {
  gpio_put(LED_PIN, 1);
}

/* Set status LED on/off */

static inline void status_led(const uint32_t onoff) {
  gpio_put(STATUS_PIN, onoff);
}


/* Turn on status LED */

static inline void status_on() {
  gpio_put(STATUS_PIN, 1);
}

/* Turn off status LED */
static inline void status_off() {
  gpio_put(STATUS_PIN, 0);
}

/* Turn on IO LED */

static inline void io_on() {
  gpio_put(IO_PIN, 1);
}

/* Turn off IO LED */

static inline void io_off() {
  gpio_put(IO_PIN, 0);
}


/* Set RDR bits for a request */

static inline void put_read_data (const uint32_t ch) {
  gpio_put_masked(RDR_PINS_MASK, ch << RDR_1_PIN);
}

/* Fetch PUN bits from a request */
static inline uint32_t get_punch_data (const uint32_t request) {
  return (request >> PUN_1_PIN) & 255;
}


/**********************************************************/
/*                          BLINKER                       */
/**********************************************************/

/* Status indicator thread */

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
      led_state = (led_state+1) & 1;
      status_led(led_state);
      sleep_ms(blink);
    }
}
