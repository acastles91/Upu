#pragma once
#include <pinout.h>
#include <Arduino.h>
#include <open-celluloid.h>
#include <Rotary.h>

struct Sensor {
    
    public:
            Sensor(int sensorPinArg);
            void readSensor(); 
            void calibrateSensor(OpenCelluloid openCelluloidArg, Rotary rotaryArg);
            void printSensorValues();  
            uint32_t aLast;
            
            int sensorHigh{};
            int sensorLow{};
};