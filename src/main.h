#pragma once

#include <Arduino.h>
#include <open-celluloid.h>
#include <HardwareSerial.h>
#include <TeensyStep.h>
#include <TMCStepper.h>
#include <functions.h>
#include <motor.h>
#include <Rotary.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <Bounce2.h>
#include <limits.h>

#define ENCODER_USE_INTERRUPTS
#include <Encoder.h>

#include <vector>
#include <array>


Motor motorObj(dir_Pin, step_Pin, enab_Pin, chipSelect);
Rotary rotaryObj(channelA, channelB);
Sensor sensorObj(sensor);
OpenCelluloid openCelluloidObj{};
volatile State stateObj{};

Encoder encoderObj(channelA, channelB);

Bounce bounceSensor = Bounce();
Bounce bounceStartStop = Bounce();
Bounce bounceDirSwitch = Bounce();
Bounce bounceCapture = Bounce();
Bounce bounceSpeedSelect = Bounce();
Bounce bounceEncoButton = Bounce();

Mode modeObj;

//std::vector<Bounce *> debouncers;
std::array<Bounce, 6> debouncers;
std::array<int, 6> leds;


// debouncers.
// //debouncers
// debouncers.push_back(bounceSensor, 
//                      bounceSensor, 
//                      bounceDirSwitch,
//                      bounceCapture,
//                      bounceSpeedSelect,
//                      bounceEncoButton
//                       );

//Motor Test//
Stepper stepperObj(step_Pin, dir_Pin);
StepControl stepControlObj;
RotateControl rotateControlObj;
volatile Request prevRequest;
IntervalTimer triggerTimerObj;

int counterNew;
int counterOld;

//IntervalTimer captureLed;

volatile uint16_t dummyCounter = 0;
volatile uint16_t triggerCounter = 0;
uint16_t nonInterruptBufferCounter = 0;
volatile uint16_t timerCounterNew = 0;
volatile uint16_t timerCounterOld = 0;
uint16_t oldTriggerCounter = 0;
volatile bool captureFlag;
volatile bool attachTimerFlag;
volatile bool supportBool;


bool recording{};
 
