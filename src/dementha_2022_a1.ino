#include <avr/delay.h>
#include <avr/power.h>
#include <avr/sleep.h>
#include <avr/wdt.h>
#include <Arduino.h>
#include <avr/io.h>

#include "lis2dh12_reg.h"
#include "fxpt_atan2.h"


#define CLOCK_PIN PB0
#define DATA_PIN  PB1

#define DATA_PORT PORTB

#define INPUT_PORT PINB
#define DATA_PORT_DDR DDRB

#define CS_PORT_DDR DDRB
#define CS_PORT PORTB
#define CS_PIN PB2

#define READ_BIT 0x80
#define AUTO_INCREMENT_BIT 0x40


typedef enum {
  LIS3DH_DATARATE_400_HZ = 0b0111, //  400Hz
  LIS3DH_DATARATE_200_HZ = 0b0110, //  200Hz
  LIS3DH_DATARATE_100_HZ = 0b0101, //  100Hz
  LIS3DH_DATARATE_50_HZ = 0b0100,  //   50Hz
  LIS3DH_DATARATE_25_HZ = 0b0011,  //   25Hz
  LIS3DH_DATARATE_10_HZ = 0b0010,  // 10 Hz
  LIS3DH_DATARATE_1_HZ = 0b0001,   // 1 Hz
  LIS3DH_DATARATE_POWERDOWN = 0,
  LIS3DH_DATARATE_LOWPOWER_1K6HZ = 0b1000,
  LIS3DH_DATARATE_LOWPOWER_5KHZ = 0b1001,

} lis3dh_dataRate_t;



#define P0 PORTA0 // PA7
#define P1 PORTA1 // PA6
#define P2 PORTA2 // PA5
#define P3 PORTA3 // PA4
#define P4 PORTA4 // physical 2, PA3
#define P5 PORTA5 // physical 3, PA2
#define P6 PORTA6 // physical 4, PA1
#define P7 PORTA7 // physical 5, PA0


#define NUM_LEDS 39


typedef struct {
  const uint8_t cathode: 4;
    const uint8_t anode: 4;
  } led_t;


  const led_t LED[NUM_LEDS] = {
  // top
  {P7, P6},
  {P7, P5},
  {P5, P7},
  {P3, P7},
  {P7, P3},
  {P1, P3},
  {P3, P1},
  {P1, P4},
  {P4, P1},
  {P1, P0},
  {P2, P4},
  {P4, P2},
  {P1, P2},
  {P2, P1},


  {P2, P3},

  {P3, P2},
  {P2, P0},
  {P0, P2},
  {P3, P0},

  // tip
  {P0, P3},

  // bottom
  {P4, P0},
  {P0, P4},
  {P3, P4},
  {P4, P3},
  {P3, P5},
  {P5, P3},
  {P4, P5},
  {P5, P4},
  {P4, P6},
  {P6, P4},
  {P5, P6},
  {P0, P5},
  {P5, P0},
  {P6, P5},
  {P0, P6},
  {P6, P0},
  {P0, P7},
  {P7, P0},
  {P6, P7},
};


void resetPix() {
  PORTA = 0;
  DDRA = 0;
}

void pix(uint8_t sinkPin, uint8_t sourcePin) {
  PORTA = 0;
  DDRA = (1 << sinkPin) | (1 << sourcePin);
  PORTA = (1 << sourcePin);
}

void pix(int8_t pinIx) {
  if (pinIx < 0 || pinIx > NUM_LEDS) {
    resetPix();
  } else {
    pix(LED[pinIx].cathode, LED[pinIx].anode);
  }
}



volatile boolean gotInterrupt = false;

void setup(void) {
  uint8_t reset_reason = MCUSR;  // could check this for brown-out or watchdog reset
  MCUSR = 0;

  wdt_disable();

  DDRA = 0;
  PORTA = 0;

  bitClear(ADCSRA, ADEN);    // disable ADC
  bitSet(ACSR, ACD);         // disable analog comparator

  // disable timer1, timer0, USI, shut down ADC
  PRR |= bit(PRTIM1) | bit(PRTIM0) | bit(PRUSI) | bit(PRADC);

  DIDR0 = 0xff; // disable all digital inputs


  DATA_PORT_DDR |= bit(DATA_PIN) | bit(CLOCK_PIN);
  bitSet(CS_PORT, CS_PIN);
  bitClear(DATA_PORT, CLOCK_PIN);
  bitSet(CS_PORT_DDR, CS_PIN);

  startWatchdog();

  _delay_ms(10); // app note says lis takes 5ms to boot
  if (!setupLis(true, false, false, true)) {
    // I have no idea what state the CS pin should be in this case
    // set it to an input. should be high-Z
    bitClear(CS_PORT_DDR, CS_PIN);
    while (true) {
      pix(0);
      _delay_ms(1);
      resetPix();
      _delay_ms(5);
      wdt_reset();
    }
  }
  _delay_ms(10); // various mode changes take a second to transition
  wdt_reset();
}

void startWatchdog() {
  const uint8_t WATCHDOG_TIME = WDTO_1S;
  wdt_reset();
  wdt_enable(WATCHDOG_TIME);
  wdt_reset();
}

void loop() {

  uint8_t startLed = getStartLed();

  //    pix(startLed);
  //    _delay_us(5);
  //    resetPix();
  //    _delay_us(20);
  //    return;

  for (int8_t i = 0; i <= (NUM_LEDS / 2) + 1; ++i) {
    for (uint8_t j = 0; j < 100; ++j) {
      if (i < NUM_LEDS / 2) {
        pix(offsetLed(i, startLed));
        _delay_us(4);
        resetPix();
        pix(offsetLed(-i, startLed));
        _delay_us(4);
        resetPix();
      }

      if (i > 1) {
        pix(offsetLed(i - 1, startLed));
        _delay_us(2);
        resetPix();
        pix(offsetLed(-(i - 1), startLed));
        _delay_us(2);
        resetPix();
      }
    }

    wdt_reset();
    _delay_ms(15);
  }
  //  _delay_ms(350);
  wdt_reset();

  goToSleep();
}


void goToSleep(void) {

  // it's supposed to be possible to get the accelerometer to go into high/low datarate based on activity automatically
  // return to sleep, sleep to wake

  // decrease ODR
  writeOut(LIS2DH12_CTRL_REG1, lis2dh12_reg_t { .ctrl_reg1 = {
      .xen = 1,
      .yen = 1,
      .zen = 1,
      .lpen = 1, // hr mode cannot be enabled while this is true
      .odr = LIS3DH_DATARATE_10_HZ // 4 bits
    }});

  set_sleep_mode(SLEEP_MODE_PWR_DOWN);

  sleep_enable();
  MCUCR &= ~(_BV(ISC01) | _BV(ISC00));      //INT0 on low level

  bitClear(CS_PORT, CS_PIN); // probably not necessary
  bitClear(CS_PORT_DDR, CS_PIN); // change to INPUT

  PORTA = 0; // shouldn't be necessary

  cli();

  wdt_disable();

  //  do not allow sleep while the pin is still active, can cause freezing
  while (!bitRead(PINB, PB2)) {
    void;
  }

  GIMSK |= _BV(INT0);                       //enable INT0

  //stop interrupts to ensure the BOD timed sequence executes as required
  byte mcucr1 = MCUCR | _BV(BODS) | _BV(BODSE);  //turn off the brown-out detector
  byte mcucr2 = mcucr1 & ~_BV(BODSE);
  MCUCR = mcucr1;
  MCUCR = mcucr2;

  sei();                                    //ensure interrupts enabled so we can wake up again
  sleep_cpu();                              //go to sleep
  sleep_disable();                          //wake up here

  startWatchdog();

  bitSet(CS_PORT_DDR, CS_PIN); // change to OUTPUT

  _delay_us(10); // wait for pin to settle

  // increase ODR
  writeOut(LIS2DH12_CTRL_REG1, lis2dh12_reg_t { .ctrl_reg1 = {
      .xen = 1,
      .yen = 1,
      .zen = 1,
      .lpen = 1, // hr mode cannot be enabled while this is true
      .odr = LIS3DH_DATARATE_200_HZ // 4 bits
    }});

  _delay_ms(10); // I have no idea whether the start-up time is based on the old ODR or the new ODR

}

//external interrupt 0 wakes the MCU
ISR(INT0_vect) {
  GIMSK = 0;                     //disable external interrupts (only need one to wake up)
}

uint8_t offsetLed(uint8_t start, int8_t offset) {
  // offset must be < NUM_LEDS
  // other conditions could be handled with recursion
  int8_t i = start;
  i += offset;
  if (i < 0) {
    return NUM_LEDS + i;
  } else if (i >= NUM_LEDS) {
    return i - NUM_LEDS;
  } else {
    return i;
  }
}

// from lib8tion/fastled
// there may be an asm version that works on attiny
uint8_t scale8( uint8_t i, uint8_t scale) {
  return (((uint16_t)i) * (1 + (uint16_t)(scale))) >> 8;
}



// could easily progmem this to save memory
const uint8_t ANGLE_INDEX[] = {
  0,
  10,
  15,
  19,
  24,
  29,
  35,
  42,
  49,
  58,
  68,
  77,
  87,
  96,
  103,
  109,
  115,
  119,
  123,
  126,
  129,
  132,
  136,
  141,
  147,
  154,
  161,
  170,
  179,
  189,
  199,
  208,
  216,
  222,
  227,
  232,
  237,
  241,
  246
};

uint8_t binarySearchAngle(uint8_t angle) {
  uint8_t lo = 0, hi = NUM_LEDS - 1;
  uint8_t mid;
  // This below check covers all cases , so need to check
  // for mid=lo-(hi-lo)/2
  while (hi - lo > 1) {
    uint8_t mid = (hi + lo) / 2;
    if (ANGLE_INDEX[mid] < angle) {
      lo = mid + 1;
    } else {
      hi = mid;
    }
  }
  if (ANGLE_INDEX[lo] == angle) {
    return lo;
  } else if (ANGLE_INDEX[hi] == angle) {
    return hi;
  } else if (angle - ANGLE_INDEX[lo] < ANGLE_INDEX[hi] - angle) {
    // get closest
    return lo;
  } else {
    return hi;
  }
}



uint8_t getStartLed() {
  uint8_t angle = getFifoAngle();

  angle += 127;

  return binarySearchAngle(angle);

  // lookup table method
  //  return ANGLE_TO_LED[angle];

  // naive method
  //  angle = 255 - angle;
  //  //  angle += 30; // rotate a little
  //  uint8_t start_led = NUM_LEDS - scale8(angle, NUM_LEDS);
  //  start_led = offsetLed(start_led, -1); // there's some off-by-one
  //  return start_led;
}

uint8_t getFifoAngle() {
  // read from the fifo queue, so we can tell what orientation we were in while we were sleeping
  // should read to see how many entries are actually in the fifo queue and only read that many
  // should keep a ring buffer or moving average
  // but I probably don't have the space for it.
  // so just blindly read, regardless of how much data we have

  //  _delay_ms(40); // moar data

  int16_t x = repeatedRead(LIS2DH12_OUT_X_L);
  int16_t y = repeatedRead(LIS2DH12_OUT_Y_L);

  uint8_t angle = fxpt_atan2(y, x) >> 8;
  return angle;


  // 10hz = 100ms
  // 25hz = 40ms between samples
  // 50hz = 20ms
  // 100hz = 10ms
}

void smoothInt(uint16_t sample, uint8_t bits, long *filter) {
  long local_sample = ((long) sample) << 16;

  *filter += (local_sample - *filter) >> bits;
}


short getFilterValue(long filter) {
  return (short)((filter + 0x8000) >> 16);
}

long setFilterValue(short value) {
  return ((long) value) << 16;
}


int16_t repeatedRead(uint8_t addr) {
  const uint8_t size_of_fifo_queue = 32;
  int32_t buf = 0;
  for (uint8_t i = 0; i < size_of_fifo_queue; ++i) {
    buf += readIn2(addr);
  }
  return buf >> 9; // 5 + 4 (5 = divide by 32, 4 = padding we naturally get from the sensor)
}

int16_t repeatedReadSmooth(uint8_t addr) {
  // if I'm doing it this way I could try to get the # of readings actually in the queue
  const uint8_t size_of_fifo_queue = 32;
  int32_t filter = 0; // setFilterValue(readIn2(addr) >> 4);
  for (uint8_t i = size_of_fifo_queue; i > 0; ++i) {
    smoothInt(readIn2(addr) >> 4, (i >> 2) + 1, &filter);
  }
  return getFilterValue(filter) >> 5;
}



void rawWrite(uint8_t val) {
  //  bitSet(DDRB, DATA_PIN); // enable output

  for (int8_t i = 0; i < 8; i++) {
    bitWrite(DATA_PORT, DATA_PIN, bitRead(val, 7 - i));

    _delay_us(1);
    bitSet(DATA_PORT, CLOCK_PIN);
    _delay_us(1);

    bitClear(DATA_PORT, CLOCK_PIN);
  }
}

uint8_t rawRead() {
  uint8_t readVal = 0;

  // clock is expected to be low at this point

  for (int8_t i = 0; i < 8; ++i) {

    bitSet(DATA_PORT, CLOCK_PIN); // may be high already from write operation
    _delay_us(1);

    bitWrite(readVal, 7 - i, bitRead(INPUT_PORT, DATA_PIN));

    bitClear(DATA_PORT, CLOCK_PIN);
    _delay_us(1);
  }

  return readVal;
}

void startTransaction() {
  bitClear(CS_PORT, CS_PIN);

}

void endTransaction() {
  bitSet(CS_PORT, CS_PIN);
}

// test the wake/sleep interrupt
//   could reconfigure the LIS on wake. only necessary if reconfigurating the HPF.
//   otherwise, just need to reconfigure the INT/CS pin on wake/sleep

inline void writeOut(uint8_t addr, lis2dh12_reg_t val) {
  writeOut(addr, val.byte);
}

void writeOut(uint8_t addr, uint8_t val) {
  // should start/end transaction around this
  startTransaction();

  rawWrite(addr);
  rawWrite(val);

  endTransaction();
}

void startRead(uint8_t addr) {

  startTransaction();

  _delay_us(1);

  rawWrite(addr);

  bitClear(DATA_PORT, DATA_PIN);
  bitClear(DATA_PORT_DDR, DATA_PIN); // enable input

  _delay_us(1);
}

void endRead() {
  endTransaction();

  // reset state of other pins
  bitSet(DATA_PORT_DDR, DATA_PIN);
}

uint8_t readIn(uint8_t addr) {

  startRead(addr // should only use bottom 6 bits
            | READ_BIT); // read bit

  uint8_t result = rawRead();

  endRead();

  return result;
}

int16_t readIn2(uint8_t addr) {

  startRead(addr // should only use bottom 6 bits
            | READ_BIT
            | AUTO_INCREMENT_BIT); // read bit

  int16_t output = (int16_t)rawRead() | int16_t(rawRead() << 8);

  endRead();

  return output;
}



uint8_t setupLis(boolean setupInterrupts, boolean enableHpf, boolean enableLatch, boolean enableFifo) {

  const boolean USE_CLICK_INT = false;
  const boolean USE_ACCEL_INT = true;

  //  writeOut(CTRL_REG4,
  //           (SENSITIVITY_2G << 4));
  //           | 1); // 3 wire mode

  // startup sequence, per the app note
  //1. Write CTRL_REG1
  //2. Write CTRL_REG2
  //3. Write CTRL_REG3
  //4. Write CTRL_REG4
  //5. Write CTRL_REG5
  //6. Write CTRL_REG6
  //7. Write REFERENCE
  //8. Write INTx_THS
  //9. Write INTx_DUR
  //10. Write INTx_CFG
  //11. Write CTRL_REG5


  // what next?
  // test an interrupt, maybe click
  // then try wake interrupt (need to enable hpf for sleep, disable on startup/wake)

  // how to set up FIFO:
  // per app note an5005 sec. 9.3.3
  // set FIFO_EN bit in REG3
  // set FM1 and FM0 bits in FIFO_CTRL_REG (streaming mode)

  // how to read:
  // read FSS bits (0-4) in FIFO_SRC_REG to get size of queue
  // read X, Y, Z that many times

  writeOut(LIS2DH12_CTRL_REG1, lis2dh12_reg_t { .ctrl_reg1 = {
      .xen = 1,
      .yen = 1,
      .zen = 1,
      .lpen = 1, // hr mode cannot be enabled while this is true
      .odr = LIS3DH_DATARATE_200_HZ // 4 bits
    }});

  writeOut(LIS2DH12_CTRL_REG2, lis2dh12_reg_t { .ctrl_reg2 = { // high-pass filter config
      .hp = 0b111, // HPCLICK + HP_IA2 + HP_IA1 -> HP; // use HPF for interrupts
      .fds = (enableHpf) ? 1 : 0, // enable hp filter on regular output
      .hpcf = 0, // 2 bits
      .hpm = 0 // 2 bits, filter mode = normal
    }});

  writeOut(LIS2DH12_CTRL_REG3, lis2dh12_reg_t { .ctrl_reg3 = { // various interrupt config
      .not_used_01 = 0,
      .i1_overrun = 0,
      .i1_wtm = 0,
      .not_used_02 = 0,  // .i1_321da = 0, // change from lis3 "321DA interrupt on INT1."
      .i1_zyxda = 0,
      .i1_ia2 = 0,
      .i1_ia1 = (setupInterrupts && USE_ACCEL_INT) ? 1 : 0,
      .i1_click = (setupInterrupts && USE_CLICK_INT) ? 1 : 0,
    }});

  writeOut(LIS2DH12_CTRL_REG4, lis2dh12_reg_t { .ctrl_reg4 = {
      .sim = 1, // 3 wire mode
      .st = 0, // self test mode
      .hr = 0, // high resolution mode (12 bit output). 0 must be set in reg1.lpen. see section 3.2.1 in data sheet.
      .fs = 0, // data range. 00 = 2g, 01 = 4g, 10 = 8g, 11 = 16g
      .ble = 0, // big endian vs little endian. only matters for high resolution mode.
      .bdu = 1, // block data ready. don't allow mis-matched pairs of high and low bytes
    }});

  writeOut(LIS2DH12_CTRL_REG5, lis2dh12_reg_t { .ctrl_reg5 = { // fifo and interrupt latch
      .d4d_int2 = 0,
      .lir_int2 = 0,
      .d4d_int1 = 0,
      .lir_int1 = (enableLatch) ? 1 : 0, // latch. if set, must read LIS2DH12_INT1_SRC to clear. might be useful if we want to see how big the initial impact was.
      .not_used_01 = 0,
      .fifo_en = (enableFifo) ? 1 : 0,
      .boot = 0
    }});

  writeOut(LIS2DH12_CTRL_REG6, lis2dh12_reg_t { .ctrl_reg6 = {
      .not_used_01 = 0,
      .int_polarity = 1, // set int active low
      .not_used_02 = 0,
      .i2_act = 0,
      .i2_boot = 0,
      .i2_ia2 = 0,
      .i2_ia1 = 0,
      .i2_click = 0
    }});


  readIn(LIS2DH12_REFERENCE); // resets HPF calibration (doesn't seem to do much)

  if (enableFifo) {
    writeOut(LIS2DH12_FIFO_CTRL_REG, lis2dh12_reg_t { .fifo_ctrl_reg = {
        .fth = 0,
        .tr = 0,
        .fm = 0b10, // b00 = bypass, b01 = fifo mode, b10 = stream mode, b11 = stream-to-fifo mode
      }});
  }

  if (setupInterrupts) {
    if (USE_ACCEL_INT) { // non-click interrupt
      writeOut(LIS2DH12_INT1_THS, 10); // copying what I did for skully
      writeOut(LIS2DH12_INT1_DURATION, 0); // probably unnecessary

      writeOut(LIS2DH12_INT1_CFG, lis2dh12_reg_t { .int1_cfg = {
          .xlie = 0,
          .xhie = 1,
          .ylie = 0,
          .yhie = 1,
          .zlie = 0,
          .zhie = 1,
          ._6d = 0,
          .aoi = 0,
        }});
    }
    if (USE_CLICK_INT) {
      writeOut(LIS2DH12_CLICK_CFG, lis2dh12_reg_t { .click_cfg = {
          .xs = 0,
          .xd = 0,
          .ys = 0,
          .yd = 0,
          .zs = 1,
          .zd = 1,
          .not_used_01 = 0,
        }});

      writeOut(LIS2DH12_CLICK_THS, lis2dh12_reg_t { .click_ths = {
          .ths = 10 , // 1 LSB = full scale/128.
          .lir_click = 0,
        }});

      //1 LSB = 1/ODR.
      //TLI7 through TLI0 define the maximum time interval that can elapse between the start of
      //the click-detection procedure (the acceleration on the selected channel exceeds the
      //programmed threshold) and when the acceleration falls back below the threshold.
      writeOut(LIS2DH12_TIME_LIMIT, lis2dh12_reg_t { .time_limit = {
          .tli = 10, // chosen b/c adafruit default
          .not_used_01 = 0,
        }});

      writeOut(LIS2DH12_TIME_LATENCY, lis2dh12_reg_t { .time_latency = {
          .tla = 20, // chosen b/c adafruit default
        }});

      writeOut(LIS2DH12_TIME_WINDOW, lis2dh12_reg_t { .time_window = {
          .tw = 255, // chosen b/c adafruit default
        }});
    }


  } else {
    writeOut(LIS2DH12_INT1_CFG, 0);
  }

  return readIn(LIS2DH12_WHO_AM_I) == LIS2DH12_ID;
}

// plan
// figure out "top"
// rotate 45 degrees
// use same animation as 2019, but use that to determine start & end
