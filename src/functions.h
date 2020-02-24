#pragma once

#include <Arduino.h>
#include <TMCStepper.h>
#include <SPI.h>
#include <TimerOne.h>
#include <HardwareSerial.h>
#include <Stream.h>
#include "TeensyStep.h"
#include <Rotary.h>
#include <open-celluloid.h>
#include <pinout.h>
#include <motor.h>
#include <sensor.h>
#include <state.h>
#include <Bounce2.h>
#include <limits.h>
#include <request.h>
#include <Encoder.h>

#include <array>

extern volatile uint16_t dummyCounter;
extern uint16_t nonInterruptBufferCounter;
//volatile int triggerCounter;
extern volatile uint16_t triggerCounter;
extern volatile uint16_t timerCounterNew;
extern volatile uint16_t timerCounterOld;

extern volatile bool captureFlag;
extern volatile bool attachTimerFlag;
extern volatile bool supportBool;


void readSensor();
void oneFrame();
void oneFrameSlow();
void serialTask(State &stateArg, Sensor &sensorArg, Rotary &rotaryArg, OpenCelluloid &openCelluloidArg, Motor &motorArg, Bounce &bounceArg, Stepper &stepperArg, StepControl &stepControlArg, RotateControl &rotateControlArg);
void stateSwitch(State &stateArg);
void encoder();
void calibrateShutter();
void testDigital(int button);
void testAnalog(int pin);
void sensorInterruptOn(Bounce &debouncerArg);
void sensorInterruptOff();
void sumTrigger();

void debouncerTriggerCheck(OpenCelluloid &openCelluloidArg, Bounce &debouncerArg);
void debouncerStartStopCheck(Bounce &debouncerArg);
void debouncerCaptureButton(Bounce &debouncerArg);
void debouncerSpeedSelectButton(Bounce &debouncerArg);
void debouncerDirectionSwitchButton(Bounce &debouncerArg);
void debouncerSpeedChangeButton(Bounce &debouncerArg);
//void debouncerToggleCheck(Bounce &debouncerArg);

static void changeSpeed(State &stateArg, 
                        volatile Request &prevRequestArg, 
                        Encoder &encoderArg, 
                        OpenCelluloid &openCelluloidArg, 
                        Motor &motorArg, 
                        Bounce &bounceEncoButtonArg,
                        std::array<int, 6> &ledsArg,
                        Stepper &stepperArg);

//Request checkInput(Stream &streamArg);
Request checkInputStream(Stream &streamArg, volatile Request &prevRequestArg);
Request checkInputStreamOF(Stream &streamArg, volatile Request &prevRequestArg);

Request checkInputButtons(std::array<Bounce, 6> &bouncerArrayArg, volatile Request &prevRequestArg);
Request checkInputButtonsOF(std::array<Bounce, 6> &bouncerArrayArg, volatile Request &prevRequestArg);

void showPosition(Stepper &stepperArg, RotateControl &rotateControlArg);

void switchState(volatile Request &requestArg, 
                 volatile Request &prevRequest,
                 Stepper &stepperArg,
                 StepControl &stepControlArg,
                 RotateControl &rotateControlArg,
                 Sensor &sensorArg,
                 Rotary &rotaryArg,
                 OpenCelluloid &openCelluloidArg,
                 Motor &motorArg,
                 Bounce &bounceArg,
                 std::array<int, 6> &ledsArg,
                 Encoder &encoderArg,
                 Bounce &bounceEncoButtonArg,
                 Mode &modeArg);

void switchStateOF(volatile Request &requestArg, 
                 volatile Request &prevRequest,
                 Stepper &stepperArg,
                 StepControl &stepControlArg,
                 RotateControl &rotateControlArg,
                 Sensor &sensorArg,
                 Rotary &rotaryArg,
                 OpenCelluloid &openCelluloidArg,
                 Motor &motorArg,
                 Bounce &bounceArg,
                 std::array<int, 6> &ledsArg,
                 Encoder &encoderArg,
                 Bounce &bounceEncoButtonArg,
                 IntervalTimer &triggerTimerArg,
                 Mode &modeArg);

void blinkCaptureLed();

void traditionalTriggerInterrupt();
void traditionalTriggerCheck();
void timerTest();
void debouncerPrintAll(Bounce &debouncerArg, volatile int dummyCounter);
void timerTest();

void timerExperimental();
void initMode(Mode &modeArg, Motor &motorArg, Stepper &stepperArg, std::array<int, 6> &ledsArg);


// void switchState(volatile State &stateArg, Request requestArg, Stepper &stepperArg, StepControl &stepControlArg, RotateControl &rotateControlArg, Sensor &sensorArg, Rotary &rotaryArg, OpenCelluloid &openCelluloidArg, Motor &motorArg, Bounce &bounceArg);