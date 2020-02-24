#pragma once

#include <TeensyStep.h>
#include <TMCStepper.h>
#include <SPI.h>
#include <TimerOne.h>
#include <pinout.h>
#include <open-celluloid.h>

#include <array>

#define STALL_VALUE 15
#define R_SENSE 0.11 // Match to your driver
                      // SilentStepStick series use 0.11
                      // UltiMachine Einsy and Archim2 boards use 0.2
                      // Panucatt BSD2660 uses 0.1
                      // Watterott TMC5160 uses 0.075

                      //BOB = 0.3

struct Motor {

    public:

        Motor(const int dirPinArg, const int stepPinArg, const int enabPinArg, const int chipSelectArg, const int mosiSdiArg, const int misoSdoArg, const int clkArg, float rSenseArg);    
        Motor(const int dirPinArg, const int stepPinArg, const int enabPinArg, const int chipSelectArg);
        Motor(const Motor& otherMotor);
        int microsteps = 8;
        const uint32_t steps_per_mm = 80;
        int frameRatio = (200 * microsteps) / 2.55;
        int motorSpeed;
        bool motorState = false;
        bool isMoving = false;
        bool accelerated = false;

        void initializeDriver();
        void setupMotor();
        void accelerate();
        void moveMotor();
        void xFrames(int x, OpenCelluloid &openCelluloidArg, Bounce &debouncerArg);
        //void oneFrameSlow();
        void moveMotorSlow();
        bool isAccelerated();
        void stepperTimer();
        void setSpeed(Motor &motorArg, std::array<int, 6> &ledsArg);

        //int maxSpeed = 15000;   //Effective max speed of the motor, too fast for the interrupt
        //int maxSpeed = 5000; //recording can't keep up
        int maxSpeed = 2000;
        int maxAcceleration = 10000;

        int speeds1[4] {1, 2, 3, 4};
        int selectedSpeed;
        int speedIndex{};


        int spr = 16*200;  // 3200 steps per revolution

        TMC2130Stepper driver;
        //Stepper motor;
        //StepControl controller;

    private:

        
};