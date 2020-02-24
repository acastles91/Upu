#include "open-celluloid.h"

OpenCelluloid::OpenCelluloid()
{}
// //_________________________________________________
void OpenCelluloid::stepperTimer(){
  if (stepCount > 0) {
    if (stepState == 0) {
      stepCount--;
    }
    digitalWrite(step_Pin, stepState);
    stepState = !stepState;
  }
};
////_________________________________________________

void OpenCelluloid::homing(){

  digitalWrite(enab_Pin, LOW);
  digitalWrite(dir_Pin, LOW);
  int val = 0;
  bool gate_closed = true; //boolean related roughly to the shutter in front of the sensor
  bool center_not_found = true; //boolean related precisely to the sensor being at the end of the shutter
  byte gate[8]; //array used to make an average of the values; 0 means the shutter is in front of the sensor
  int sum_gate = 0;

  while (gate_closed == true && center_not_found == true) {
    for (int i = 0; i < 8; i ++) {
      digitalWrite(step_Pin, HIGH);
      delayMicroseconds(300);
      digitalWrite(step_Pin, LOW);
      delayMicroseconds(300);
      val = !digitalRead(sensor);
      gate[i] = val;
    }
    for (int i = 0; i < 8; i ++) {
      sum_gate += gate[i];
      //Serial.println(gate[i]);
    }
    if (sum_gate == 0) {
      gate_closed = false;
      continue;
    } else {
      sum_gate = 0;
      continue;
    }
  }

  if (gate_closed == false) {
    //val = digitalRead(sensor);
    while (center_not_found == true) {
      //Serial.println("buscando el 8");
      for (int i = 0; i < 8; i ++) {
        digitalWrite(step_Pin, HIGH);
        delayMicroseconds(300);
        digitalWrite(step_Pin, LOW);
        delayMicroseconds(300);
        val = !digitalRead(sensor);
        gate[i] = val;
      }
      for (int i = 0; i < 8; i ++) {
        sum_gate += gate[i];
        //Serial.println(gate[i]);
      }
      //Serial.println(sum_gate);
      if (sum_gate == 8) {
        center_not_found = false;
        continue;
      } else {
        sum_gate = 0;
        continue;
      }
    }
  }
  digitalWrite(dir_Pin, HIGH);
  gateOpen = true;
  //moveMotorSlow();
};

void OpenCelluloid::checkTriggerOld(){
  if(gateOpen){
   if(triggerCounter2 < 8){
      threshold[triggerCounter2] = digitalRead(sensor);
      sum_threshold += threshold[triggerCounter2];
    }
    else{
      triggerCounter2 = 0;    
    }
    if (sum_threshold == 8){
      sameState = false;
      sum_threshold = 0;
      gateOpen = false;
      shutterCounter += 1;
    }
    else if (sum_threshold > 8){
      sum_threshold = 0;
    }   
  }
  if (!gateOpen){
    triggerCounter2 = 0;
    if(digitalRead(sensor) == 1){
      gateOpen = true;
      shutterCounter +=1;
    }
  }
};
// void OpenCelluloid::sumTrigger(){
//   triggerCounter ++;
// };
// ////_________________________________________________


// void OpenCelluloid::sensorInterruptOn(){

//   attachInterrupt(sensor, sumTrigger, CHANGE);

// };

// void OpenCelluloid::sensorInterruptOff(){
//   detachInterrupt(sensor);
// };


void OpenCelluloid::checkTrigger(Bounce &debouncerArg){

    debouncerArg.update();
    
    if ((debouncerArg.fell()) || (debouncerArg.rose())){
      triggerCounter2 += 1;
      Serial.print("Trigger Counter:        ");
      Serial.println(triggerCounter2); 
    }
    if (triggerCounter2 == 4){
      Serial.println("Frame");
      triggerCounter2 = 0;
  }
};

//////_________________________________________________

void OpenCelluloid::calibrateShutter(Rotary &rotaryArg){

rotaryArg.encoderTest();

 if (rotaryArg.counterNew > rotaryArg.counterOld){
     digitalWrite(dir_Pin, HIGH);
    for (int i = 0; i < 100; i ++){
        digitalWrite(step_Pin, HIGH);
        delayMicroseconds(500);
        digitalWrite(step_Pin, LOW);
        delayMicroseconds(500);
        }
    rotaryArg.counterNew = rotaryArg.counterOld;     
   } else if (rotaryArg.counterNew < rotaryArg.counterOld){
    digitalWrite(dir_Pin, LOW);
    for (int i = 0; i < 100; i ++){
        digitalWrite(step_Pin, HIGH);
        delayMicroseconds(500);
        digitalWrite(step_Pin, LOW);
        delayMicroseconds(500);
        }
    rotaryArg.counterNew = rotaryArg.counterOld;     

    }
};


  // rotaryArg.aState = digitalRead(channelA); // Reads the "current" state of the outputA
  //  // If the previous and the current state of the outputA are different, that means a Pulse has occured
  // if (rotaryArg.aState != rotaryArg.aLastState){     
  //    // If the outputB state is different to the outputA state, that means the encoder is rotating clockwise
  //    if (digitalRead(channelB) != rotaryArg.aState) { 
  //      rotaryArg.counterNew = rotaryArg.counterOld ++;
  //    } else {
  //      rotaryArg.counterNew = rotaryArg.counterOld --;
  //    }
  //    delay(40);
  //    //Serial.print("Position: ");
  //    //Serial.println(counterNew);
  // }

  // unsigned char result = rotaryArg.process();
  // //Serial.println("Encoder test");
  // if (result == DIR_CW) {
  //   rotaryArg.counterNew = rotaryArg.counterOld ++;
  //   Serial.println(rotaryArg.counterNew);
  // } else if (result == DIR_CCW) {
  //   rotaryArg.counterNew = rotaryArg.counterOld --;
  //   Serial.println(rotaryArg.counterNew);
  // }
  //
  //  }
  //  //aLastState = aState;
  //  rotaryArg.counterOld = rotaryArg.counterNew;
  //  //delay(20);




