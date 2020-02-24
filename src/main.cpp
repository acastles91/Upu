#include <main.h>

void setup() {

  // ***********
	pinMode(enab_Pin, OUTPUT);
	pinMode(step_Pin, OUTPUT);
	pinMode(chipSelect, OUTPUT);
	pinMode(dir_Pin, OUTPUT);
  pinMode(sensor, INPUT);

  // pinMode(channelA, INPUT);
  // pinMode(channelB, INPUT);
  pinMode(startStop, INPUT);
  pinMode(dirSwitch, INPUT);
  pinMode(encoButton, INPUT);
  pinMode(speedSelect, INPUT);
  pinMode(capture, INPUT);

  pinMode(led1, OUTPUT);
  pinMode(led2, OUTPUT);
  pinMode(led3, OUTPUT);
  pinMode(led4, OUTPUT);
  pinMode(led5, OUTPUT);
  pinMode(led6, OUTPUT);

  digitalWrite(enab_Pin, LOW);
  digitalWrite(dir_Pin, HIGH);
  digitalWrite(step_Pin, LOW);
  digitalWrite(chipSelect, LOW);

  for (int i = 0; i == leds.size(); i ++){
    digitalWrite(leds[i], LOW);
  }

  // bounceSensor.attach(sensor, INPUT);
  // bounceSensor.interval(1);

  bounceDirSwitch.attach(dirSwitch, INPUT);
  bounceDirSwitch.interval(20);

  bounceStartStop.attach(startStop, INPUT);
  bounceStartStop.interval(20);

  bounceSpeedSelect.attach(speedSelect, INPUT);
  bounceSpeedSelect.interval(20);

  bounceEncoButton.attach(encoButton, INPUT);
  bounceEncoButton.interval(20);

  bounceCapture.attach(capture, INPUT);
  bounceCapture.interval(20);

  
  std::get<0>(debouncers) = bounceSensor;
  std::get<1>(debouncers) = bounceDirSwitch;
  std::get<2>(debouncers) = bounceCapture;
  std::get<3>(debouncers) = bounceStartStop;
  std::get<4>(debouncers) = bounceSpeedSelect;
  std::get<5>(debouncers) = bounceEncoButton;

  std::get<0>(leds) = led1;
  std::get<1>(leds) = led2;
  std::get<2>(leds) = led3;
  std::get<3>(leds) = led4;
  std::get<4>(leds) = led5;
  std::get<5>(leds) = led6;

  motorObj.setupMotor();
  openCelluloidObj.oldPosition = 14;
  motorObj.speedIndex = 0;

  openCelluloidObj.isCapturing = false;
  motorObj.selectedSpeed = motorObj.maxSpeed / motorObj.speeds1[0];

  stepperObj.setMaxSpeed(motorObj.maxSpeed / motorObj.speeds1[motorObj.speedIndex]);
  stepperObj.setAcceleration(motorObj.maxAcceleration);

  prevRequest = Request::auto_reset_request;
  captureFlag = false;
  attachTimerFlag = false;
  supportBool = true;
  
  modeObj =  Mode::continuous;


  //attempt at traditional interrupt and timer

  //noInterrupts();
  // ***********

  interrupts();

  // attachInterrupt(digitalPinToInterrupt(sensor), traditionalTriggerInterrupt, CHANGE);
  // NVIC_SET_PRIORITY(IRQ_PORTC, 128);
  
  Serial.begin(115200);
  delay(2000);

};


void loop() {

// ***********
//Functional
  
  volatile Request requestStream = checkInputStreamOF(Serial, prevRequest);
  if(requestStream != prevRequest){
    switchStateOF(requestStream,
                prevRequest, 
                stepperObj, 
                stepControlObj, 
                rotateControlObj, 
                sensorObj, 
                rotaryObj, 
                openCelluloidObj, 
                motorObj, 
                bounceSensor,
                leds,
                encoderObj,
                bounceEncoButton,
                triggerTimerObj,
                modeObj);
    prevRequest = requestStream;
  }

    volatile Request requestButton = checkInputButtonsOF(debouncers, prevRequest);
    if(requestButton != prevRequest){
    switchStateOF(requestButton,
                prevRequest,
                stepperObj, 
                stepControlObj, 
                rotateControlObj, 
                sensorObj, 
                rotaryObj, 
                openCelluloidObj, 
                motorObj, 
                bounceSensor,
                leds,
                encoderObj,
                bounceEncoButton,
                triggerTimerObj,
                modeObj);
     }
  if (captureFlag == true){
    Serial.print('0');
    captureFlag = false;
  }


  

  //*************************************************
///Experimental

  if (modeObj == Mode::experimental){
    bounceSensor.update();
      if(bounceSensor.rose()){
        attachTimerFlag = true;
        }
      if (attachTimerFlag && supportBool){
        triggerTimerObj.begin(timerExperimental, 7300);
        triggerTimerObj.priority(0);
        attachTimerFlag = false;
        supportBool = false;
        bounceSensor.interval(20);
      }      
  }


};
  
  //*************************************************


