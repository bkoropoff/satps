#include <inttypes.h>
#include <stdbool.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

// MISO pin settings
#define MISO_DDR_REG DDRB
#define MISO_DDR_BIT DDB3

// ACK pin settings
#define ACK_PORT_REG PORTC
#define ACK_PORT_BIT PORTC6
#define ACK_DDR_REG DDRC
#define ACK_DDR_BIT DDC6
// Width of ack pulse in microseconds
#define ACK_WIDTH 2

// Select pins
#define SEL_PORT_REG PORTB
#define SEL_PORT_BIT0 PORTB4
#define SEL_PORT_BIT1 PORTB5
#define SEL_DDR DDRB
#define SEL_DDR_BIT0 DDB4
#define SEL_DDR_BIT1 DDB5

// Data pins
#define DATA_PIN_REG PINF
#define DATA_PIN_SHIFT 4
#define DATA_PORT_REG PORTF
#define DATA_PORT_BITS (_BV(PORTF4) | _BV(PORTF5) | _BV(PORTF6) | _BV(PORTF7))
#define DATA_DDR DDRF
#define DATA_DDR_BITS (_BV(DDF4) | _BV(DDF5) | _BV(DDF6) | _BV(DDF7))

// Setting pins
#define SETTING_PIN_REG PIND
#define SETTING_PIN_BIT0 PIND0
#define SETTING_PIN_BIT1 PIND1
#define SETTING_PORT_REG PORTD
#define SETTING_PORT_BIT0 PORTD0
#define SETTING_PORT_BIT1 PORTD1
#define SETTING_DDR DDRD
#define SETTING_DDR_BIT0 DDD0
#define SETTING_DDR_BIT1 DDD1

// SS pin
#define SS_PIN_REG PINB
#define SS_PIN_BIT PINB0
#define SS_DDR DDRB
#define SS_DDR_BIT DDB0

// Delay in microseconds for controller to settle after changing select lines
#define SS_DELAY 2

#define SET(R, P) ((R) |= _BV(P))
#define CLEAR(R, P) ((R) &= ~_BV(P))
#define TEST(R, P) (((R) & _BV(P)) != 0)

enum State {
  STATE_START,
  STATE_POLL,
  STATE_DATA1,
  STATE_DATA2,
  STATE_IGNORE
} __attribute__((packed));

/* Globals */
static volatile bool doAck = false;
static volatile bool doPoll = false;
/* PS digital controller state */
static volatile uint8_t controllerState[2] = {0xff, 0xff};
/* Protocol state machine state */
static volatile enum State state = STATE_START;

static void setup() {
  // Set MISO as output
  SET(MISO_DDR_REG, MISO_DDR_BIT);
  // Turn on SPI as slave
  SPCR = _BV(SPE) | _BV(DORD) | _BV(CPOL) | _BV(CPHA) | _BV(SPIE);
  SPDR = 0xFF;
  // Set select pins as outputs
  SET(SEL_DDR, SEL_DDR_BIT0);
  SET(SEL_DDR, SEL_DDR_BIT1);
  // Set data pins as inputs with pullups
  DATA_DDR &= ~DATA_DDR_BITS;
  DATA_PORT_REG |= DATA_PORT_BITS;
  // Set setting pins as inputs with pullups
  CLEAR(SETTING_DDR, SETTING_DDR_BIT0);
  CLEAR(SETTING_DDR, SETTING_DDR_BIT1);
  SET(SETTING_PORT_REG, SETTING_PORT_BIT0);
  SET(SETTING_PORT_REG, SETTING_PORT_BIT1);
  // Set up pin change interrupts on SS
  CLEAR(SS_DDR, SS_DDR_BIT);
  SET(PCMSK0, PCINT0);
  SET(PCICR, PCIE0);
  // Turn on interrupts
  sei();
}

static void readSettings(bool* vselect, bool* rotate)
{
   uint8_t reg = SETTING_PIN_REG;
   *vselect = (reg & _BV(SETTING_PIN_BIT0)) == 0;
   *rotate = (reg & _BV(SETTING_PIN_BIT1)) == 0;
}

static inline uint16_t readSSData(void)
{
  return (DATA_PIN_REG >> DATA_PIN_SHIFT) & 0xF;
}

static inline uint16_t readSSController(void)
{
  uint16_t result = 0;

  CLEAR(SEL_PORT_REG, SEL_PORT_BIT0);
  CLEAR(SEL_PORT_REG, SEL_PORT_BIT1);
  _delay_us(SS_DELAY);
  result |= readSSData();
  
  SET(SEL_PORT_REG, SEL_PORT_BIT0);
  CLEAR(SEL_PORT_REG, SEL_PORT_BIT1);
  _delay_us(SS_DELAY);
  result |= readSSData() << 4;
  
  CLEAR(SEL_PORT_REG, SEL_PORT_BIT0);
  SET(SEL_PORT_REG, SEL_PORT_BIT1);
  _delay_us(SS_DELAY);
  result |= readSSData() << 8;

  SET(SEL_PORT_REG, SEL_PORT_BIT0);
  SET(SEL_PORT_REG, SEL_PORT_BIT1);
  _delay_us(SS_DELAY);
  result |= readSSData() << 12;

  return result;
}

static void poll(void)
{
  bool vselect = false;
  bool rotate = false;
  readSettings(&vselect, &rotate);
  
  uint16_t ss = readSSController();
  uint8_t z = (ss >> 0) & 1;
  uint8_t y = (ss >> 1) & 1;
  uint8_t x = (ss >> 2) & 1;
  uint8_t r = (ss >> 3) & 1;
  uint8_t b = (ss >> 4) & 1;
  uint8_t c = (ss >> 5) & 1;
  uint8_t a = (ss >> 6) & 1;
  uint8_t st = (ss >> 7) & 1;
  uint8_t up = (ss >> 8) & 1;
  uint8_t dn = (ss >> 9) & 1;
  uint8_t lt = (ss >> 10) & 1;
  uint8_t rt = (ss >> 11) & 1;
  uint8_t l = (ss >> 15) & 1;
  uint8_t sl = 1;

  // Treat up + start as select if set to
  if (vselect && !up && !st) {
    st = 1;
    sl = 0;
  }

  cli();
  controllerState[0] = (sl << 0) | (1 << 1) | (1 << 2) | (st << 3) | (up << 4) | (rt << 5) | (dn << 6) | (lt << 7);
  if (rotate)
    controllerState[1] = (l << 0) | (r << 1) | (z << 2) | (c << 3) | (x << 4) | (y << 5) | (b << 6) | (a << 7);
  else
    controllerState[1] = (l << 0) | (r << 1) | (z << 2) | (c << 3) | (y << 4) | (b << 5) | (a << 6) | (x << 7);
  sei();
}

static inline void transact(uint8_t data)
{
  switch (state) {
  case STATE_START:
    switch (data) {
    case 0x01:
      SPDR = 0x41;
      doAck = true;
      doPoll = true;
      state = STATE_POLL;
      return;
    default:
      // For reasons I can't fathom, the controller and memory card on
      // a given port share an SPI SS line.  This means we have to
      // avoid responding to commands not addressed to the controller.
      // The ignore state avoids driving MISO and is exited when the
      // pin change interrupt sees SS go inactive again.
      SPDR = 0xFF;
      state = STATE_IGNORE;
      return;
    }
  case STATE_POLL:
    switch (data) {
    case 0x42:
      SPDR = 0x5a;
      doAck = true;
      state = STATE_DATA1;
      return;
    default:
      // FIXME: we do hit this branch, but I haven't yet looked at the
      // command being sent.  Not ACKing and going back to the start
      // state works.
      SPDR = 0xFF;
      state = STATE_START;
      return;
    }
  case STATE_DATA1:
    SPDR = controllerState[0];
    doAck = true;
    state = STATE_DATA2;
    return;
  case STATE_DATA2:
    SPDR = controllerState[1];
    doAck = true;
    state = STATE_START;
    return;
  case STATE_IGNORE:
    SPDR = 0xFF;
    return;
  default:
    __builtin_unreachable();
  }
}

ISR(SPI_STC_vect)
{
  transact(SPDR);
}

ISR(PCINT0_vect)
{
  // Reset state machine when we are unselected
  if (TEST(SS_PIN_REG, SS_PIN_BIT))
    state = STATE_START;
}

static void loop() {
  if (doAck) {
    doAck = false;
    SET(ACK_DDR_REG, ACK_DDR_BIT);
    _delay_us(ACK_WIDTH);
    CLEAR(ACK_DDR_REG, ACK_DDR_BIT);
  }

  if (doPoll) {
    doPoll = false;
    poll();
  }
}

int main(void) {
  setup();
  for (;;) loop();
}
