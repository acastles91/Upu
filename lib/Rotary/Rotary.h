/*
 * Rotary encoder library for Arduino.
 */
#pragma once

#ifndef rotary_h
#define rotary_h

#include "Arduino.h"

// Enable this to emit codes twice per step.
//#define HALF_STEP

// Enable weak pullups
#define ENABLE_PULLUPS

// Values returned by 'process'
// No complete step yet.
#define DIR_NONE 0x0
// Clockwise step.
#define DIR_CW 0x10
// Anti-clockwise step.
#define DIR_CCW 0x20

class Rotary
{
  public:
    Rotary(char, char);
    //Extra
    int encoderTest();
    // Process pin(s)
    unsigned char process();
    volatile int aState;
    volatile int aLastState;
    int counterNew;
    int counterOld;
  private:
    unsigned char state;
    unsigned char pin1;
    unsigned char pin2;
    
};

#endif
 
