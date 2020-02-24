#include "resources.h"

// desired samplerate 16MHz/2000 => 8000Hz
static const uint16_t kTimerPrescaler = 64;
static const uint16_t kSamplePrescaler = 2000;
// range of the sample rate is between (16MHz/(kTimerPrescaler*kOCRRegister)) => 250000Hz - 980Hz
static const uint8_t  kOCRRegister = (kSamplePrescaler/kTimerPrescaler) - 1;


ISR(TIMER2_COMPA_vect){
  (*sampleInterrupt)();
}

void setupSampleTimer(){
  // Arduino Uno, Nano
  // Timer2 for samplerate
  // prescaler of timer is set to 64 by wiring.c
  TCCR2B = (TCCR2B & 0xf8) | 0x04;
  // set mode of Timer 2 to CTC Capture
  TCCR2A = (TCCR2A & 0xfc) | 0x02;  
  // set Compare Match A value, can be altered for Samplerate reduction. Hooray Aliasing!
  OCR2A = kOCRRegister;      
  // enable Timer/Counter2 Output Compare Match A Interrupt Enable
  // triggers ISR(TIMER0_COMPA_vect) routine when TCNT0 = OCR0A, (TCNT0 = timer value) 
  TIMSK2 |= (1<<OCIE2A);
}


word phase;
word phase_increment;
word sample;
byte pin_state;

void setup(){
  // initializes and enables our sample timer interrupt
  setupSampleTimer();
  // our phase counter
  phase = 0;
  pin_state = 0;
  // has a useful resolution of 1..32768
  // to calculate phase increment: freq * max_phase/samplerate
  phase_increment = 3000; 
  pinMode(10, OUTPUT);
}

void loop(){
}

void sampleInterrupt(){
  // increment phase by phase_increment  
  phase += phase_increment;
  // get the most significant 8 bits
  if(phase< phase_increment){
    // overflow
    pin_state = !pin_state; // toggle pn, 0,1,0,1...
    digitalWrite(13, pin_state);

  }
  sample = (phase>>8); // get upper 8 bits of phase
}


/*
65535/(Samplerate * freq)
*/
