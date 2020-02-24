#include <motor.h>
#include <TMCStepper.h>
#include <TeensyStep.h>



Motor::Motor(const int dirPinArg, const int stepPinArg, const int enabPinArg, const int chipSelectArg, const int mosiSdiArg, const int misoSdoArg, const int clkArg, float rSenseArg) :

        driver(chipSelectArg, rSenseArg)
        //motor(stepPinArg, dirPinArg),
        //controller{}
        
        {} 
Motor::Motor(const Motor& otherMotor) :

        driver(otherMotor.driver)
        //motor(otherMotor.motor),
        //controller(otherMotor.controller)
        {}

Motor::Motor(const int dirPinArg, const int stepPinArg, const int enabPinArg, const int chipSelectArg) :

        driver(chipSelectArg, R_SENSE)
        //motor(stepPinArg, dirPinArg),
        //controller{}
        {}

void Motor::initializeDriver(){

    // //     //TMCStepper - simple !! Works with hardware constructor, TMCStepper library
  
    driver.begin();                 //  SPI: Init CS pins and possible SW SPI pins                                 // UART: Init SW UART (if selected) with default 115200 baudrate
    driver.toff(1);                 // Enables driver in software
    driver.tbl(1);
    driver.rms_current(1000);        // Set motor RMS current
    driver.microsteps(2);          // Set microsteps to 1/16th

    driver.en_pwm_mode(true);       // Toggle stealthChop on TMC2130/2160/5130/5160
    //driver.pwm_autoscale(true);     // Needed for stealthChop
    driver.semin(5);
    driver.semax(2);
};
//_________________________________________________

void Motor::setupMotor(){


    // motor
    //   .setMaxSpeed(maxSpeed)
    //   .setAcceleration(maxAcceleration)
    //   .setStepPinPolarity(LOW);
        
    
}
void Motor::accelerate(){
    int speedTest = 150;
        
          for(int i = 0; (speedTest * 3) - i == 150; i++){

              digitalWrite(step_Pin, HIGH);
              delayMicroseconds((speedTest * 3) - i);
              digitalWrite(step_Pin, LOW);
              delayMicroseconds((speedTest * 3) - i);
          };
    //3662.109

    // for (float i = 0.000; i == 3662.1; i + 0.1){
    //   analogWriteFrequency(stepPin, i);
    //   analogWrite(stepPin, 128);
    // } 
    // for (int i = 1500; i == 500; i -- ){
    // digitalWrite(step_Pin, LOW);
    // delayMicroseconds(i);
    // digitalWrite(step_Pin, HIGH);
    // delayMicroseconds(i);
    // }


    // for (int i = 0; i == 128; i + 1){
    //     analogWrite(stepPin, i);
    // }

};
//_________________________________________________
bool Motor::isAccelerated(){
 	return	accelerated;
 };

//_________________________________________________

void Motor::moveMotor(){

  //   if (motorSpeed != (map(analogRead(potSpeed), 0, 1023, 0, 500))){
  //     setSpeed();
  //   } 

  //Timer1.pwm(9, 512, 350);
  //analogWrite(step_Pin, 1);
  // _motorRef->setTargetRel(100000000000);
  // _controllerRef->move(_motorRef);

  //motor.setTargetRel(10000000000);
  //controller.move(motor);

    digitalWrite(step_Pin, HIGH);
    delayMicroseconds(150);
    digitalWrite(step_Pin, LOW);
    delayMicroseconds(150);
      // int speedTest = 150;
        
      //     for(int i = 0; ((speedTest * 3) - i) == 150; i++){

      //         digitalWrite(step_Pin, HIGH);
      //         delayMicroseconds((speedTest * 3) - i);
      //         digitalWrite(step_Pin, LOW);
      //         delayMicroseconds((speedTest * 3) - i);
      //     }
      
};

void Motor::xFrames(int x, OpenCelluloid &openCelluloidArg, Bounce &debouncerArg) {
  openCelluloidArg.homing();
  for (int z = 0; z == x; z++){
    if (openCelluloidArg.triggerCounter2 != 4)
    digitalWrite(step_Pin, HIGH);
    delayMicroseconds(200);
    digitalWrite(step_Pin, LOW);
    delayMicroseconds(200);
    openCelluloidArg.checkTrigger(debouncerArg);
  }
};

//_________________________________________________
// void Motor::oneFrameSlow() {
//   for (int i = 0; i < frameRatio ; i ++) {
//     digitalWrite(step_Pin, HIGH);
//     delayMicroseconds(500);
//     digitalWrite(step_Pin, LOW);
//     delayMicroseconds(500);
//   }
// };

//_________________________________________________
void Motor::moveMotorSlow(){
    //oneFrameSlow();    
};

void Motor::setSpeed(Motor &motorArg, std::array<int, 6> &ledsArg){
if(motorArg.speedIndex == 3){  
          digitalWrite(ledsArg[0], HIGH);
          digitalWrite(ledsArg[1], LOW);
          digitalWrite(ledsArg[2], LOW);
          digitalWrite(ledsArg[3], LOW);
        }
        else if(motorArg.speedIndex == 2){
          digitalWrite(ledsArg[0], HIGH);
          digitalWrite(ledsArg[1], HIGH);
          digitalWrite(ledsArg[2], LOW);
          digitalWrite(ledsArg[3], LOW);
        }
        else if(motorArg.speedIndex == 1){
          digitalWrite(ledsArg[0], HIGH);
          digitalWrite(ledsArg[1], HIGH);
          digitalWrite(ledsArg[2], HIGH);
          digitalWrite(ledsArg[3], LOW);
        }
        else if(motorArg.speedIndex == 0){
          digitalWrite(ledsArg[0], HIGH);
          digitalWrite(ledsArg[1], HIGH);
          digitalWrite(ledsArg[2], HIGH);
          digitalWrite(ledsArg[3], HIGH);
        }

}