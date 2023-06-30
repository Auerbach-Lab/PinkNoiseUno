#include <Arduino.h>
#include "firtables.h"    // 12 tap FIR filter using 2 lookups on last 12 random bits
#include <debounce.h>

/* Using timings of 1000/500/5000, the current sequence will be:
                  0 s    sequence start button pushed
[offset]          1.0 s  pre-sound imaging starts
[image duration]  1.5 s  pre-sound imaging stops
[offset]          2.5 s  sound starts
[calculated]      4.75 s imaging starts (centered on middle of sound playback)
[image duration]  5.25 s imaging stops
                  7.5 s  sound stops
[offset]          8.5 s  post-sound imaging starts
[image duration]  9.0 s  post-sound imaging stops*/

// EDIT THESE VALUES to adjust timings on existing sequence.
#define OFFSET 1000           // ms between steps in sequence
#define IMAGE_DURATION 500    // ms duration of TTL signal for imaging
#define SOUND_DURATION 5000   // ms duration of sound to play

// EDIT THIS SECTION to define sequence itself
unsigned long currentMillis = 0;
unsigned long imageStart[3] = {0};
unsigned long imageStop[3] = {0};
unsigned long soundStart = 0;
unsigned long soundStop = 0;
bool sendingTTL = false;
bool playingSound = false;
static void sequenceHandler(uint8_t btnId, uint8_t btnState) {
  if ((btnState == BTN_PRESSED) && !imageStop[2]) {
    Serial.println("Pressed sequence button");
    
    //pre image
    imageStart[0] = currentMillis + OFFSET;
    imageStop[0] = imageStart[0] + IMAGE_DURATION;

    //sound
    soundStart = imageStop[0] + OFFSET;
    soundStop = soundStart + SOUND_DURATION;

    //mid-sound image
    imageStart[1] = soundStart + (SOUND_DURATION - IMAGE_DURATION)/2;
    imageStop[1] = imageStart[1] + IMAGE_DURATION;

    //post image
    imageStart[2] = soundStop + OFFSET;
    imageStop[2] = imageStart[2] + IMAGE_DURATION;
    
  } else {
    // btnState == BTN_OPEN.
    Serial.println("Released sequence button");
  }
}

// define the OCR2A timer output pin, depends on which board - only support ATmega328 and ATmega1280/2560
#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
#  define AUDIO_OUTPUT_PIN 10
#else
#  define AUDIO_OUTPUT_PIN 11
#endif

// On the Uno, the relevant timer pin for audio output pin is 11, but the
// high-frequency components of the signal bleed onto 10 and 12. Avoid them.
#define SEQUENCE_BUTTON_PIN 4
#define TEST_BUTTON_PIN 3
#define TTL_OUTPUT_PIN 7
#define SOUND_GATE_PIN 6
#define TRANSISTOR_PIN 9

//#include "costable.h // Cosine volume fade is not possible with PWM (vs analog control of voltage) to a transistor

static void playSound() {
  Serial.println("Sound playing");
  analogWrite(AUDIO_OUTPUT_PIN, 128); 
  digitalWrite(SOUND_GATE_PIN, HIGH);;
  playingSound = true;
  soundStart = 0; //clear assignment
}

static void silenceSound() {
  Serial.println("Sound silenced"); 
  analogWrite(AUDIO_OUTPUT_PIN, 0); //set duty cycle to 0% but doesn't fully silence, so a transistor/mosfet is needed
  digitalWrite(SOUND_GATE_PIN, LOW); 
  playingSound = false;
  soundStop = 0; //clear assignment     
}

static void startImaging(unsigned int i) {
  Serial.println("Start imaging");
  digitalWrite(TTL_OUTPUT_PIN, HIGH);
  sendingTTL = true;
  imageStart[i] = 0; //clear assignment
}

static void stopImaging(unsigned int i) {
  Serial.println("Stop imaging");
  digitalWrite(TTL_OUTPUT_PIN, LOW);
  sendingTTL = false;
  imageStop[i] = 0; //clear assignment
}

static void testHandler(uint8_t btnId, uint8_t btnState) {
  if (btnState == BTN_PRESSED) {
    Serial.println("Testing...");
    playSound();
    startImaging(0);
  } else {
    // btnState == BTN_OPEN
    Serial.println("Test stop");
    silenceSound();
    stopImaging(0);
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

/*
Arduino pink noise generation adapted from Mark Tillotson's adaptation of Stenzel
Works for Uno/Pro Mini/Mega, using pin 11 or 10 depending on board (the one driven by timer2 output A)
Noncommercial use only.
*/
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
/* Arduino pink noise generation ends. */

void setup() {
  Serial.begin(115200);
  build_revtab();
  TCCR2A = 0xB3;  // configure as fast 8-bit PWM (mode 011, clock prescale = 1)
  TCCR2B = 0x01;
  TIMSK2 = 0x01;  // timer2 overflow interrupt enabled every 256 cycles (62.5 kHz for a 16MHz ATmega) because 16MHz / 8 bit register / prescaler of only 1 = 256

  pinMode(SEQUENCE_BUTTON_PIN, INPUT_PULLUP);
  pinMode(TEST_BUTTON_PIN, INPUT_PULLUP);
  pinMode(TTL_OUTPUT_PIN, OUTPUT);
  pinMode(SOUND_GATE_PIN, OUTPUT);
  analogWrite(AUDIO_OUTPUT_PIN, 128); // enable the output pin and its timer, set to 50% which is loudest
  digitalWrite(SOUND_GATE_PIN, HIGH);
  
}

void loop() { // nothing here for ongoing pink noise, all driven by ISR
  pollButtons();
  currentMillis = millis();
  
  for (unsigned int i=0; i < sizeof imageStart / sizeof imageStart[i]; i++) {
    if(!sendingTTL && imageStart[i] && (currentMillis > imageStart[i])) startImaging(i);
    if(sendingTTL && imageStop[i] && (currentMillis > imageStop[i])) stopImaging(i);
    if(!playingSound && soundStart && (currentMillis > soundStart)) playSound();
    if(playingSound && soundStop && (currentMillis > soundStop)) silenceSound();
  }

  delay(10);
}