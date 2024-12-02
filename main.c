#include <inttypes.h>
#include <stdbool.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

// I/O definitions.  (PS) indicates an interface with the
// PlayStation console, (SS) indicates an interface with the
// Saturn controller.

// (PS) MISO pin.  Set as output to use SPI.
#define MISO_DDR_REG DDRB
#define MISO_DDR_BIT DDB3

// (PS) ACK pin.  Side channel to tell PS that
// something was present to receive SPI call.
#define ACK_PORT_REG PORTC
#define ACK_PORT_BIT PORTC6
#define ACK_DDR_REG DDRC
#define ACK_DDR_BIT DDC6
// (PS) Width of ack pulse in microseconds
#define ACK_WIDTH 2

// (PS) SPI select pin.  Used by the console to start an SPI call
// with the controller port assembly.
#define SS_PIN_REG PINB
#define SS_PIN_BIT PINB0
#define SS_DDR DDRB
#define SS_DDR_BIT DDB0


// (SS) Select pins.  Control which buttons controller outputs on data pins.
#define SEL_PORT_REG PORTB
#define SEL_PORT_B0 PORTB4
#define SEL_PORT_B1 PORTB5
#define SEL_DDR DDRB
#define SEL_DDR_B0 DDB4
#define SEL_DDR_B1 DDB5
// (SS) Delay in microseconds for controller to settle after changing select lines
#define SS_DELAY 2

// (SS) Data pins.  Output of controller.
#define DATA_PIN_REG PINF
#define DATA_PIN_SHIFT 4
#define DATA_PORT_REG PORTF
#define DATA_PORT_BITS (_BV(PORTF4) | _BV(PORTF5) | _BV(PORTF6) | _BV(PORTF7))
#define DATA_DDR DDRF
#define DATA_DDR_BITS (_BV(DDF4) | _BV(DDF5) | _BV(DDF6) | _BV(DDF7))

// Setting pins.  Control how controller inputs are converted.
// - VSELECT:
//   * Low: treat SS up + start as PS select button
//   * High: don't
// - ROTATE:
//   * Low: map face buttons so SS A and B correspond to PS square and cross.
//     This looks "rotated" compared to the physical button layouts, but is
//     the most natural way to use the SS controller.
//   * High: map face buttons according to physical layout.  SS A and B correspond
//     to PS cross and circle.
// - SWAP:
//   * Low: map SS shoulder buttons to PS L1/R1 and SS Z/C to PS L2/R2.
//     This is more natural for games that expect you to use L1/R1 in combination
//     with face buttons.
//   * High: map SS shoulder buttons to PS L2/R2 and SS z/C to PS L1/R2.
//     This is more natural for 6-button fighting games.
#define SETTING_PIN_REG PIND
#define SETTING_PIN_VSELECT PIND0
#define SETTING_PIN_ROTATE PIND1
#define SETTING_PIN_SWAP PIND4
#define SETTING_PORT_REG PORTD
#define SETTING_PORT_VSELECT PORTD0
#define SETTING_PORT_ROTATE PORTD1
#define SETTING_PORT_SWAP PORTD4
#define SETTING_DDR DDRD
#define SETTING_DDR_VSELECT DDD0
#define SETTING_DDR_ROTATE DDD1
#define SETTING_DDR_SWAP DDD4

// Helper macros
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

// Globals
static volatile bool doAck = false;
static volatile bool doPoll = false;
// PS digital controller state
static volatile uint8_t controllerState[2] = {0xff, 0xff};
// Protocol state machine state
static volatile enum State state = STATE_START;

static void setup() {
  // Set MISO as output
  SET(MISO_DDR_REG, MISO_DDR_BIT);
  // Turn on SPI as slave
  SPCR = _BV(SPE) | _BV(DORD) | _BV(CPOL) | _BV(CPHA) | _BV(SPIE);
  SPDR = 0xFF;
  // Set select pins as outputs
  SET(SEL_DDR, SEL_DDR_B0);
  SET(SEL_DDR, SEL_DDR_B1);
  // Set data pins as inputs with pullups
  DATA_DDR &= ~DATA_DDR_BITS;
  DATA_PORT_REG |= DATA_PORT_BITS;
  // Set setting pins as inputs with pullups
  CLEAR(SETTING_DDR, SETTING_DDR_VSELECT);
  CLEAR(SETTING_DDR, SETTING_DDR_ROTATE);
  CLEAR(SETTING_DDR, SETTING_DDR_SWAP);
  SET(SETTING_PORT_REG, SETTING_PORT_VSELECT);
  SET(SETTING_PORT_REG, SETTING_PORT_ROTATE);
  SET(SETTING_PORT_REG, SETTING_PORT_SWAP);
  // Set up pin change interrupts on SS
  CLEAR(SS_DDR, SS_DDR_BIT);
  SET(PCMSK0, PCINT0);
  SET(PCICR, PCIE0);
  // Turn on interrupts
  sei();
}

static void readSettings(bool* vselect, bool* rotate, bool* swap)
{
   uint8_t reg = SETTING_PIN_REG;
   *vselect = (reg & _BV(SETTING_PIN_VSELECT)) == 0;
   *rotate = (reg & _BV(SETTING_PIN_ROTATE)) == 0;
   *swap = (reg & _BV(SETTING_PIN_SWAP)) == 0;
}

static inline uint16_t readSSData(void)
{
  return (DATA_PIN_REG >> DATA_PIN_SHIFT) & 0xF;
}

static inline uint16_t readSSController(void)
{
  uint16_t result = 0;

  CLEAR(SEL_PORT_REG, SEL_PORT_B0);
  CLEAR(SEL_PORT_REG, SEL_PORT_B1);
  _delay_us(SS_DELAY);
  result |= readSSData();
  
  SET(SEL_PORT_REG, SEL_PORT_B0);
  CLEAR(SEL_PORT_REG, SEL_PORT_B1);
  _delay_us(SS_DELAY);
  result |= readSSData() << 4;
  
  CLEAR(SEL_PORT_REG, SEL_PORT_B0);
  SET(SEL_PORT_REG, SEL_PORT_B1);
  _delay_us(SS_DELAY);
  result |= readSSData() << 8;

  SET(SEL_PORT_REG, SEL_PORT_B0);
  SET(SEL_PORT_REG, SEL_PORT_B1);
  _delay_us(SS_DELAY);
  result |= readSSData() << 12;

  return result;
}

static void poll(void)
{
  bool vselect = false;
  bool rotate = false;
  bool swap = false;

  readSettings(&vselect, &rotate, &swap);
  
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
  uint8_t cs0 = 0;
  uint8_t cs1 = 0;

  // Treat up + start as select if set to
  if (vselect && !up && !st) {
    st = 1;
    sl = 0;
  }

  cs0 = (sl << 0) | (1 << 1) | (1 << 2) | (st << 3) | (up << 4) | (rt << 5) | (dn << 6) | (lt << 7);
  if (swap)
    cs1 = (z << 0) | (c << 1) | (l << 2) | (r << 3);
  else
    cs1 = (l << 0) | (r << 1) | (z << 2) | (c << 3);
  if (rotate)
    cs1 |= (x << 4) | (y << 5) | (b << 6) | (a << 7);
  else
    cs1 |= (y << 4) | (b << 5) | (a << 6) | (x << 7);

  cli();
  controllerState[0] = cs0;
  controllerState[1] = cs1;
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
