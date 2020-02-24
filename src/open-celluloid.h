#pragma once

#include <Arduino.h>
#include <TimerOne.h>
#include <HardwareSerial.h>
#include <Stream.h>
#include <Rotary.h>
#include <pinout.h>
#include <Bounce2.h>



class OpenCelluloid {

    public:

        OpenCelluloid();
        void stepperTimer();
        void homing();
        void checkTriggerOld();
        void checkTrigger(Bounce &debouncerArg);
        void calibrateShutter(Rotary &rotaryArg);
        // void sensorInterruptOn();
        // void sensorInterruptOff();
        // void sumTrigger();
        //HardwareSerial *serial;
        //HardwareSerial &serial1 = Serial;
        //TimerOne *timer1;

        const int sensorThreshold = 300;
        int lastSensorState = 0;
        uint8_t sensorState = 0; //
        uint8_t stepState = 0; //

        //Encoder

		volatile long signed oldPosition  = -999;
		//________________________


        //trigger
        volatile bool gateOpen{};
        volatile bool trigger{};
        volatile uint8_t triggerCounter2{};
        volatile bool sameState = true;
        volatile byte threshold[8];
        volatile uint8_t sum_threshold{};
        volatile uint8_t gate{};
        volatile uint8_t gatePrevious{};
        volatile uint8_t shutterCounter{};
        volatile bool boolGate{};
        //volatile bool boolState = !digitalRead(sensor);
        volatile bool boolState{};
        volatile bool home_position{};      
        volatile bool isCapturing;

        uint8_t doFullRotation{};
        int stepCount = 10000;
        volatile char state;

        //Rotary rotary = Rotary(5, 6);

        

    private:

        Stream *_streamRef;
        TimerOne *_timerRef;
        //Stepper *_motorRef;
        //StepControl *_controllerRef;

};


//
