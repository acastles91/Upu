#include <sensor.h>
//#include <pinout.h>

Sensor::Sensor(int sensorPinArg){};

void Sensor::readSensor(){
  
  uint32_t aNow = 0;
  aNow = analogRead(sensor);
  if ( aNow != aLast ) {
    aLast = aNow;
    Serial.println( aNow );
    delay(3);
  }
  
};

//Deprecated
// void OpenCelluloid::triggerSensor(){
//     if(digitalRead(sensor) == false){
//     boolGate = true;
//   }
//   else{
//     boolGate = false;
//   }
//   if(boolGate != boolState){
//     triggerCounter += 1;
//     boolState = boolGate;
//   }
//   if (triggerCounter == 4){
//     Serial.write('0' + 0);
//     triggerCounter = 0;
//   }
// };

void Sensor::calibrateSensor(OpenCelluloid openCelluloidArg, Rotary rotaryArg){

  //_timerRef->attachInterrupt(calibrateShutter);

  sensorLow = 999;
  sensorHigh = 999;
  int buttonState = digitalRead(dirSwitch);
  Serial.println("Place the sensor in front of the wheel on its white position and press the encoder button");
  while (sensorHigh == 999){
  //Serial.println(digitalRead(dirSwitch));
  openCelluloidArg.calibrateShutter(rotaryArg);

  if (digitalRead(dirSwitch) != buttonState){
    sensorHigh = analogRead(sensor);
    if(digitalRead(dirSwitch == buttonState)){
      Serial.println("Processing...");
      delay(2000);
      Serial.println("Sensor High set to: ");
      Serial.print(sensorHigh);

    }
  }
}
  delay(2000);
  Serial.println("Place the sensor in front of the wheel on its dark position and press the encoder button");
  while (sensorLow == 999){
  openCelluloidArg.calibrateShutter(rotaryArg);
  if (digitalRead(dirSwitch) != buttonState){
    sensorLow = analogRead(sensor);
    if(digitalRead(dirSwitch == buttonState)){
      Serial.println("Processing...");
      delay(2000);
      Serial.println("Sensor Low set to: ");
      Serial.print(sensorLow);
      }  
    }
  }
  Serial.println("Done");
  Serial.println("Low value: ");
  Serial.print(sensorLow);
  Serial.println("High value: ");
  Serial.print(sensorHigh);
  delay(2000);
  //state = auto_end;
  //_timerRef->detachInterrupt();
};

void Sensor::printSensorValues(){

  Serial.print("Sensor High:      ");
  Serial.print(sensorHigh);
  Serial.println("");
  Serial.print("Sensor Low:       ");
  Serial.print(sensorLow);

  delay(2000);

};
  