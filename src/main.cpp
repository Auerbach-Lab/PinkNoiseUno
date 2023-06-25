/*
  Arduino pink noise generation adapted from Mark Tillotson's adaptation of Stenzel
  Works for Uno/Pro Mini/Mega, using pin 11 or 10 depending on board (the one driven by timer2 output A)
  Noncommercial use only.
 */

#include <Arduino.h>
#include "firtables.h"    // 12 tap FIR filter using 2 lookups on last 12 random bits
#include <debounce.h>

// define the OCR2A timer output pin, depends on which board - only support ATmega328 and ATmega1280/2560
#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
#  define AUDIO_OUTPUT_PIN 10
#else
#  define AUDIO_OUTPUT_PIN 11
#endif

// On the Uno, the output pin is 11, but the high-frequency components of
// the signal bleed onto 10 and 12 so avoid using these.
#define SEQUENCE_BUTTON_PIN 9
#define TEST_BUTTON_PIN 8
#define VTT_OUTPUT_PIN 7

volatile byte outsampl = 0x80;  // cache last sample for ISR
volatile byte phase = 0;  // oscillates between 0 <-> 1 for each interrupt
volatile int error = 0;   // trick to reduce quantization noise
byte bitrevtab[0x40];  // caches bit reversal at cost of 64 bytes RAM

// Stenzel "new shade of pink" algorithm state
long lfsr = 0x5EED41F5;  // feedback RNG
int inc = 0xCCC;         // incrementing bit per octave
int dec = 0xCCC;         // decrementing bit per octave
int accum = 0;           // accumulator
int counter = 0xAAA;     // used to chose bit position

byte bitrev (byte a) { // reverse a 6 bit value 
  byte r = 0;
  for (byte i = 0; i < 6; i++) {
    r <<= 1;
    r |= a & 1;
    a >>= 1;
  }
  return r;
}

void build_revtab() {
  for (byte i = 0; i < 0x40; i++) {
    bitrevtab [i] = bitrev (i);
  }
}

int stenzel_pink_sample () {
  int bit = lfsr < 0 ? 0xFFF : 0x000;  // step the RNG, keeping latest bit as a mask in variable 'bit'
  lfsr += lfsr;
  if (bit) lfsr ^= 0x46000001L;
    
  counter += 1;
  counter &= 0xFFF;                    // step counter
  int bitmask = counter & -counter;    // get lowest bit
  bitmask = bitrevtab [bitmask & 0x3F];// reverse using 2 lookups
  bitmask <<= 6;
  bitmask |= bitrevtab [bitmask >> 6];
  
  dec &= ~bitmask;         // clear the dec bit at position given by bitmask
  dec |= inc & bitmask;    // copy the bit at that position from inc to dec (cancelling the effect of inc)
  inc ^= bit & bitmask;    // depending on latest random bit, perhaps flip the inc bit at that position
  accum += inc - dec;      // difference calculates the linear interpolations for all 12 generators simultaneously
  int result = accum;

  int twelve_bits = ((int) lfsr) & 0xFFF;   // extract last 12 random bits generated and filter to correct spectral
  result += highfirtab [twelve_bits >> 6];  // power density at the higher end of spectrum
  result += lowfirtab  [twelve_bits & 0x3F];
  
  return result;
}

// output the latest sample, regenerate new sample every two interrupts (since it takes longer than 16us)
// Thus PWM is 62.5 kHz, actual sample rate 31.25 kSPS, so a hardware anti-aliasing filter with cutoff below
// 15kHz is recommended
// Since only 8 bit samples are output, the quantization noise probably pollutes the higher frequencies
ISR (TIMER2_OVF_vect) {
  if (phase == 0) {
    OCR2A = outsampl + 0x80;    // PWM output, offset is 50% duty cycle for 8 bit timer2
    int samp = stenzel_pink_sample () + error;
    outsampl = samp >> 8;
    error = samp - (outsampl<<8);  // recalculate error
  }
  phase = 1 - phase;
}

static void sequenceHandler(uint8_t btnId, uint8_t btnState) {
  if (btnState == BTN_PRESSED) {
    Serial.println("Pressed sequence button");
  } else {
    // btnState == BTN_OPEN.
    Serial.println("Released sequence button");
  }
}

static void testHandler(uint8_t btnId, uint8_t btnState) {
  if (btnState == BTN_PRESSED) {
    Serial.println("Testing...");
    analogWrite(AUDIO_OUTPUT_PIN, 128); //set duty cycle to 50%
    digitalWrite(VTT_OUTPUT_PIN, HIGH);
  } else {
    // btnState == BTN_OPEN
    Serial.println("Test stop");
    analogWrite(AUDIO_OUTPUT_PIN, 0); //set duty cycle to 0%
    digitalWrite(VTT_OUTPUT_PIN, LOW);
  }
}

// Define button with a unique id (0) and handler function.
// (The ids are so one handler function can tell different buttons apart if necessary.)
static Button seqButton(0, sequenceHandler);
static Button testButton(1, testHandler);

static void pollButtons() {
  // update() will call buttonHandler() if PIN transitions to a new state and stays there
  // for multiple reads over 25+ ms.
  seqButton.update(digitalRead(SEQUENCE_BUTTON_PIN));
  testButton.update(digitalRead(TEST_BUTTON_PIN));
}

void setup() {
  Serial.begin(115200);
  pinMode(SEQUENCE_BUTTON_PIN, INPUT_PULLUP);
  pinMode(TEST_BUTTON_PIN, INPUT_PULLUP);
  pinMode(VTT_OUTPUT_PIN, OUTPUT);

  build_revtab();
  //analogWrite(AUDIO_OUTPUT_PIN, 128);  // enable the output pin and its timer, set to 50%
  TCCR2A = 0xB3;  // configure as fast 8-bit PWM (mode 011, clock prescale = 1)
  TCCR2B = 0x01;
  TIMSK2 = 0x01;  // timer2 overflow interrupt enabled every 256 cycles (62.5 kHz for a 16MHz ATmega) because 16MHz / 8 bit register / prescaler of only 1 = 256
}

void loop() { // nothing here for ongoing pink noise, all driven by ISR
  pollButtons();
  delay(10);
}