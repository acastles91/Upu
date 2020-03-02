#include <functions.h>



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
                 Mode &modeArg){

  static State stateArg = State::idle;
  //stateArg = State::idle;

  switch (requestArg){
    case Request::auto_reset_request:

      
      // Serial.println("Serial Task, case auto reset");
      // Serial.println("a = Auto Reset");
      // Serial.println("b = Start Moving Forward");
      // Serial.println("c = Start Moving Backward");
      // Serial.println("d = Stop");
      // Serial.println("e = One Frame");
      // Serial.println("f = Hundred Frames");
      // Serial.println("g = Loading");
      // Serial.println("h = Test");
      // Serial.println("i = Test digital");
      // Serial.println("j = Test analog");
      // Serial.println("k = Encoder Test");
      // Serial.println("l = Calibrate Shutter");
      // Serial.println("m = Calibrate Sensor");
      // Serial.println("n = Read Sensor");
      // Serial.println("o = Print Values");
      // Serial.println("q = Direction Toggle");
      // Serial.println("r = Turn on interrupt");
      // Serial.println("s = Turn off interrupt");
      // Serial.println("t = Toggle Capture");
      // Serial.println("u = Homing");
      // Serial.println("v = Test button");
      // Serial.println("x = Test LED");


      // Serial.println("0 = Stopping");
      // Serial.println("1 = Start Moving Forward");
      // Serial.println("2 = Start Moving backward");
      // Serial.println("4 = Auto Reset");
      break;
    
    case Request::start_moving_forward_request:
      if(stateArg == State::is_moving && digitalReadFast(dir_Pin) == HIGH){
        rotateControlArg.stopAsync();
        delay(600);
        stateArg = State::idle;   
      }
      else if(stateArg == State::is_moving){
        stateArg = State::is_moving;
      }
      if (stateArg == State::idle){
      digitalWriteFast(dir_Pin, LOW);
      rotateControlArg.rotateAsync(stepperArg); // start rotation
      stateArg = State::is_moving;
      }
      break;

    case Request::start_moving_backward_request:
      if(stateArg == State::is_moving && digitalReadFast(dir_Pin) == LOW){
        rotateControlArg.stopAsync();
        delay(600);
        stateArg = State::idle;
      }
      else if(stateArg == State::is_moving){
        stateArg = State::is_moving;
      }
      if (stateArg == State::idle){
      digitalWriteFast(dir_Pin, HIGH);
      rotateControlArg.rotateAsync(stepperArg); // start rotation
      stateArg = State::is_moving;
      }
      break;
    
    case Request::stopping_request:
      if (stateArg == State::is_moving){
        //Serial.println("stopRequest");
        //stepControlArg.stop();             //stop will wait until motor stops which is not good in a state machine...
        rotateControlArg.stopAsync();        //...use the async version instead to immediately return;
        stateArg = State::idle;
        }
      break;
    
    case Request::one_frame_request:
      motorArg.xFrames(5, openCelluloidArg, bounceArg);
      stateArg = State::idle;
      break;
    
    case Request::loading_request:
      if (stateArg == State::idle){
        rotateControlArg.rotateAsync(stepperArg); // start rotation
        stateArg = State::is_moving;
        }
        break;
    
    case Request::test_request:
      sensorArg.readSensor();
      break;
    
    case Request::test_digital_request:
      testDigital(sensor);
      break;
    
    case Request::test_analog_request:
      testAnalog(sensor);
      break; 

    case Request::encoder_test_request:
      rotaryArg.encoderTest();
      break;    
    
    case Request::calibrate_shutter_request:
      openCelluloidArg.calibrateShutter(rotaryArg);
      break;

    case Request::calibrate_sensor_request:
      sensorArg.calibrateSensor(openCelluloidArg, rotaryArg);
      break;
    
    case Request::read_sensor_request:
        stateArg = State::read_sensor;
        while (Serial.available()){
          sensorArg.readSensor();
        };
      break;
    
    case Request::print_values_request:
      // sensorArg.printSensorValues();
      break;
    
    case Request::test_acceleration_request:
      // motorArg.accelerate();
      break;

    case Request::turn_on_interrupt_request:
      openCelluloidArg.isCapturing = true;
      // Serial.println("Interrupt turned on");
      stateArg = State::idle;
      break;
    case Request::turn_off_interrupt_request:
      sensorInterruptOff();
      // Serial.println("Interrupt turned off");
      stateArg = State::idle;
      break;
    
    case Request::homing_request:
      openCelluloidArg.homing();
      // Serial.println("Homing");
      stateArg = State::idle;
      break;
    
    case Request::test_leds_request:
      digitalWrite(ledsArg[6], !digitalReadFast(ledsArg[6]));
      stateArg = State::idle;
      break;
    //Toggle requests
    //______________________________________


  case Request::direction_toggle_request:
    // Serial.printf("Direction Toggle request processed \n");
    if (stateArg == State::is_moving){
      rotateControlArg.stopAsync();
      delay(800);
      stateArg = State::idle;     
      // Serial.printf("Direction: %d \n", digitalRead(dir_Pin));
      digitalWrite(dir_Pin, !digitalRead(dir_Pin));
      rotateControlArg.rotateAsync(stepperArg); // start rotation
      stateArg = State::is_moving;      
      }
    if (stateArg == State::idle){
      digitalWrite(dir_Pin, !digitalRead(dir_Pin));
      // Serial.printf("Direction: %d \n", digitalRead(dir_Pin));
    }
      break;

    case Request::capture_toggle_request:

      if (modeArg == Mode::continuous){
        openCelluloidArg.isCapturing = !openCelluloidArg.isCapturing;     
        if(openCelluloidArg.isCapturing){
          digitalWrite(ledsArg[4], HIGH);
          // triggerTimerArg.begin(traditionalTriggerCheck, 200);
          // triggerTimerArg.priority(0);
          attachInterrupt(digitalPinToInterrupt(sensor), traditionalTriggerInterrupt, CHANGE);
          NVIC_SET_PRIORITY(IRQ_PORTC, 0);
          //delay(500);  
        //Serial.write(1); ffmpeg??
        }
        else{
          digitalWrite(ledsArg[4], LOW);        
          detachInterrupt(sensor);
          //triggerTimerArg.end();
          //noInterrupts();
        }
        //stateArg = State::idle;
      }
      else if(modeArg == Mode::discrete){
        openCelluloidArg.isCapturing = !openCelluloidArg.isCapturing;     
        if(openCelluloidArg.isCapturing){
          digitalWrite(ledsArg[4], HIGH); 
          bounceArg.attach(sensor, INPUT);
          bounceArg.interval(1);
          delay(500);           
          }
        else{
          digitalWrite(ledsArg[4], LOW);
          //triggerTimerArg.end();
          supportBool = true;
        }
  }
      

      break;
    
    
    case Request::start_stop_toggle_request:
      // Serial.printf("Start stop request processed \n");
        if (stateArg == State::idle){
          // Serial.println("Request processed");
          rotateControlArg.rotateAsync(stepperArg); // start rotation
          stateArg = State::is_moving;
          }
        else if(stateArg == State::is_moving){
          // Serial.println("Request processed");
          rotateControlArg.stopAsync();
          delay(600);
          stateArg = State::idle;   
        }
        break;
    
    case Request::change_speed_toggle_request:
      // Serial.printf("Change speed \n");
      if(!openCelluloidArg.isCapturing){
      interrupts();
      }
      if (stateArg == State::is_moving){
        rotateControlArg.stopAsync();
        delay(400);
        stateArg = State::change_speed;
      }if (stateArg == State::idle){
        stateArg = State::change_speed;
      }if (stateArg == State::change_speed){
        changeSpeed(stateArg, prevRequest, encoderArg, openCelluloidArg, motorArg, bounceEncoButtonArg, ledsArg, stepperArg);
        stateArg = State::idle;
      if(!openCelluloidArg.isCapturing){
        noInterrupts();
        }
      }
    

      break;

    case Request::set_speed_1_request:
      if (stateArg == State::is_moving){
        rotateControlArg.stopAsync();
        delay(400);
        }
        motorArg.speedIndex = 3;
        initMode(modeArg, motorArg, stepperArg, ledsArg);
        
        break;
    case Request::set_speed_2_request:
      if (stateArg == State::is_moving){
        rotateControlArg.stopAsync();
        delay(400);
        }
        motorArg.speedIndex = 2;
        initMode(modeArg, motorArg, stepperArg, ledsArg);
        
        break;
    
    case Request::set_speed_3_request:
      if (stateArg == State::is_moving){
        rotateControlArg.stopAsync();
        delay(400);
        }
        motorArg.speedIndex = 1;
        initMode(modeArg, motorArg, stepperArg, ledsArg);
        
        break;
    case Request::set_speed_4_request:
      if (stateArg == State::is_moving){
        rotateControlArg.stopAsync();
        delay(400);
        }
        motorArg.speedIndex = 0;
        initMode(modeArg, motorArg, stepperArg, ledsArg);
        
        break;

    case Request::mode_toggle_request:
      if(modeArg == Mode::continuous){
        if (stateArg == State::is_moving){
          rotateControlArg.stopAsync();
          delay(300);
          stateArg = State::idle;
        }
        modeArg = Mode::discrete;
        initMode(modeArg, motorArg, stepperArg, ledsArg);
      }else if(modeArg == Mode::discrete){
        if (stateArg == State::is_moving){
          rotateControlArg.stopAsync();
          delay(300);
          stateArg = State::idle;
        }
        modeArg = Mode::continuous;
        initMode(modeArg, motorArg, stepperArg, ledsArg);
      }
      break;
     
    case Request::idle_request:
      // Serial.printf("Idle request processed \n");
      delay(500);
      // Serial.println("Idle Request");
      break;


  }
};

void showPosition(Stepper &stepperArg, RotateControl &rotateControlArg){
    static elapsedMillis stopwatch = 0;
    static int oldPos;

    int pos = stepperArg.getPosition();
    int speed = rotateControlArg.isRunning() ? rotateControlArg.getCurrentSpeed() : 0;

    if (stopwatch > 150 && oldPos != pos)
    {
        Serial.printf("pos: %d  speed: %d\n", pos, speed);
        oldPos = pos;
        stopwatch = 0;
    }
};
    

Request checkInputStreamOF(Stream &stream, volatile Request &prevRequestArg){

  switch (stream.read()){

        case 'a':
            //Serial.println("Auto reset");
            Serial.print('a');
            return Request::auto_reset_request;
            prevRequestArg = Request::auto_reset_request;
            break;
        case 'b':
            // Serial.println("Start moving forward");
            Serial.print('b');
            return Request::start_moving_forward_request;
            prevRequestArg = Request::start_moving_forward_request;
            break;
        case 'c':
            // Serial.println("Start moving backward");
            Serial.print('c');
            return Request::start_moving_backward_request;
            prevRequestArg = Request::start_moving_backward_request;          
            break;
        case 'd':
            // Serial.println("Stopping");
            Serial.print('d');
            return Request::stopping_request;  
            prevRequestArg = Request::stopping_request;
            break;
        case 'e':
            //Serial.println("One frame");
            Serial.print('e');
            return Request::one_frame_request;
            prevRequestArg = Request::one_frame_request;
            break;
        case 'f':
            //Serial.println("Hundred Frames");
            Serial.print('f');
            return Request::hundred_frames_request;
            prevRequestArg = Request::hundred_frames_request;
            break;
        case 'g':
            //Serial.println("Loading");
            Serial.print('g');
            return Request::mode_toggle_request;
            prevRequestArg = Request::mode_toggle_request;
            break;
        case 'h':
            //Serial.println("Test");
            Serial.print('h');
            return Request::test_request;
            prevRequestArg = Request::test_request;
            break;
        case 'i':
            //Serial.println("Test digital");
            Serial.print('i');
            return Request::test_digital_request;
            prevRequestArg = Request::test_digital_request;
            break;
        case 'j':
            //Serial.println("Test analog");
            Serial.print('j');
            return Request::test_analog_request;
            prevRequestArg = Request::test_analog_request;
            break;
        case 'k':
            //Serial.println("Encoder test");
            Serial.print('k');
            return Request::encoder_test_request;
            prevRequestArg = Request::encoder_test_request;
            break;
        case 'l':
            //Serial.println("Calibrate shutter");
            Serial.print('l');
            return Request::calibrate_shutter_request;
            prevRequestArg = Request::calibrate_shutter_request;
            break;
        case 'm':
            //Serial.println("Calibrate sensor");
            Serial.print('m');
            return Request::calibrate_sensor_request;
            prevRequestArg = Request::calibrate_sensor_request;
            break;
        case 'n':
            //Serial.println("Read sensor");
            Serial.print('n');
            return Request::read_sensor_request;
            prevRequestArg = Request::read_sensor_request;
            break;
        case 'o':
            //Serial.println("Print values");
            Serial.print('o');
            return Request::mode_toggle_request;
            prevRequestArg = Request::mode_toggle_request;
            break;
        case 'p':
            //Serial.println("Test acceleration");
            Serial.print('p');
            return Request::test_acceleration_request;
            prevRequestArg = Request::test_acceleration_request;
            break;
        case 'q':
            //Serial.println("Direction Toggle");
            Serial.print('q');
            return Request::direction_toggle_request;
            prevRequestArg = Request::direction_toggle_request;
            break;
        
        case 'r':
            //Serial.println("Turn on interrupt");
            Serial.print('r');
            return Request::turn_on_interrupt_request;
            prevRequestArg = Request::turn_on_interrupt_request;
            break;

        case 's':
            //Serial.println("Turn off interrupt");
            Serial.print('s');
            return Request::turn_off_interrupt_request;
            prevRequestArg = Request::turn_off_interrupt_request;
            break;

        case 't':
            //Serial.println("Toggle Capture");
            Serial.print('t');
            return Request::capture_toggle_request;
            prevRequestArg = Request::capture_toggle_request;
            break;
          
        case 'u':
            Serial.print('u');
            return Request::homing_request;
            prevRequestArg = Request::homing_request;
            break;

        case 'y':
            Serial.print('y');
            return Request::change_speed_toggle_request;
            prevRequestArg = Request::start_stop_toggle_request;
            break;
        
        case 'x':
            //Serial.println("LEDS test");
            Serial.print('x');
            return Request::test_leds_request;
            prevRequestArg = Request::test_leds_request;
            break;
        
        case 'z':
            Serial.print('z');
            return Request::start_stop_toggle_request;
            prevRequestArg = Request::start_stop_toggle_request;
            break;
        


        
        // case 'v':
        //     return Request::button
        
        case '0':
            return Request::stopping_request;
            break;
        case '1':
            return Request::start_moving_forward_request;
            break;
        case '2':
            return Request::start_moving_backward_request;
            break;
        case '4':
            return Request::auto_reset_request;
            break;
        
        case '5':
            return Request::set_speed_1_request;
            break;
        case '6':
            return Request::set_speed_2_request;
            break;
        case '7':
            return Request::set_speed_3_request;
            break;
        case '8':
            return Request::set_speed_4_request;
            break;
        
        }
};

Request checkInputButtonsOF(std::array<Bounce, 6> &bouncerArrayArg, volatile Request &prevRequestArg){

  Request returnRequest;
  returnRequest = prevRequestArg;

  bouncerArrayArg[3].update();
  if(bouncerArrayArg[3].fell()){
    //Serial.printf("Start stop toggle \n");
    returnRequest = Request::start_stop_toggle_request;
    }
  bouncerArrayArg[1].update();
  if(bouncerArrayArg[1].fell()){
    //Serial.printf("Direction toggle \n");
    returnRequest = Request::direction_toggle_request;
  }
  bouncerArrayArg[2].update();
  if(bouncerArrayArg[2].fell()){
    //Serial.printf("Capture toggle \n");
    returnRequest = Request::capture_toggle_request;
  }
  bouncerArrayArg[4].update();
  if(bouncerArrayArg[4].fell()){
    //Serial.printf("Speed select toggle \n");
    returnRequest = Request::mode_toggle_request;
  }
  bouncerArrayArg[5].update();
  if(bouncerArrayArg[5].fell()){
    //Serial.printf("Speed change request \n");
    returnRequest = Request::change_speed_toggle_request;
  }
  // if(returnRequest != prevRequestArg){
  //   prevRequestArg = returnRequest;
    return returnRequest;
  //}
};

void testDigital(int button){

  int readButton;
  readButton = digitalRead(button);
  while (!Serial && millis() < 4000 ){
    Serial.println(readButton);
  }
};

void blinkCaptureLed(){
  digitalWriteFast(led5, !digitalReadFast(led5));
  //delay(100);

}

void testAnalog(int pin){

  int readPin;
  readPin = analogRead(pin);
  while (!Serial && millis() < 4000 ){
    Serial.println(readPin);
  }
};

void sumTrigger(int &triggerCounter){
  triggerCounter += 1;
  Serial.println("Trigger Counter");
  Serial.print(triggerCounter);
};

void sensorInterruptOn(Bounce &debouncerArg){

    debouncerArg.update();
    
    if ((debouncerArg.fell()) || (debouncerArg.rose())){
      triggerCounter += 1;
      Serial.print("Trigger Counter:        ");
      Serial.write('0' + 0);
      Serial.println(triggerCounter); 
    }
};

void debouncerTriggerCheck(OpenCelluloid &openCelluloidArg, Bounce &debouncerArg){

  if(openCelluloidArg.isCapturing){
   debouncerArg.update();
    
    if ((debouncerArg.fell()) || (debouncerArg.rose())){
      triggerCounter += 1;
      //digitalWriteFast(led5, LOW);
      // while (!Serial && millis() < 400 ){
        //Serial.print("Trigger Counter:        ");
        //Serial.println(triggerCounter);
      
    }
    if (triggerCounter == 4){
      // while (!Serial && millis() < 400 ){
        // Serial.print("Frame");
        Serial.print("0");
        triggerCounter = 0;
      }
  }
  
};

void debouncerPrintAll(Bounce &debouncerArg, volatile int dummyCounter){

debouncerArg.update();
    
    if ((debouncerArg.fell()) || (debouncerArg.rose())){
        dummyCounter += 1;  
    }
    if (dummyCounter == 4){
        dummyCounter = 0;
      }
    
    Serial.println(dummyCounter);
  
};

void debouncerStartStopCheck(Bounce &debouncerArg){

    //Serial.println("Debouncer!");

    debouncerArg.update();

    // if ((debouncerArg.fell()) || (debouncerArg.rose())){
      if (debouncerArg.fell()){

        Serial.println("Toggle!");   

      }
      
    };



void sensorInterruptOff(){
  detachInterrupt(sensor);
};

void traditionalTriggerInterrupt(){
  triggerCounter += 1;
  if(triggerCounter == 4){
      Serial.print('0');
      //captureFlag = false; doesnt work
      //timerTest();
      
      triggerCounter = 0;
  }
};

void traditionalTriggerCheck(){

    // if(timerCounterNew != timerCounterOld && timerCounterNew % 4 == 0){
    //   Serial.print("0");
    //   timerCounterOld = timerCounterNew;
    // }
    if(triggerCounter == 4){
      Serial.print('0');
      timerTest();  
      triggerCounter = 0;
    }
  };

void timerExperimental(){
  captureFlag = true;
}
static void changeSpeed(State &stateArg,
            volatile Request &prevRequestArg,
            Encoder &encoderArg, 
            OpenCelluloid &openCelluloidArg, 
            Motor &motorArg, Bounce &bounceEncoButtonArg,
            std::array<int, 6> &ledsArg,
            Stepper &stepperArg){

  if(prevRequestArg == Request::auto_reset_request){
    motorArg.speedIndex = 1;
  }           
  while (stateArg == State::change_speed){
    long signed newPosition = encoderArg.read();
    if (newPosition != openCelluloidArg.oldPosition) {
      //if((newPosition - openCelluloidArg.oldPosition) >= 4){
      if(newPosition <= 4){
        motorArg.speedIndex = 0;
      //}else if (((newPosition - openCelluloidArg.oldPosition) <= 4)){
      }else if (newPosition > 4 && newPosition <= 8){
        motorArg.speedIndex = 1;
      
      }else if (newPosition > 8 && newPosition <= 12){
        motorArg.speedIndex = 2;
      
      }else if (newPosition > 12 && newPosition <= 16){
        motorArg.speedIndex = 3;
      }
      if (newPosition > 16){
        newPosition = 16;
      }
      if (newPosition < 0){
        newPosition = 0;
      }
      
      Serial.println(newPosition);
      //Serial.printf("Speed index:   %d \n", motorArg.speedIndex);

        if(motorArg.speedIndex == 3){  
          digitalWrite(ledsArg[0], HIGH);
          digitalWrite(ledsArg[1], LOW);
          digitalWrite(ledsArg[2], LOW);
          digitalWrite(ledsArg[3], LOW);
          newPosition = 2;
        }
        else if(motorArg.speedIndex == 2){
          digitalWrite(ledsArg[0], HIGH);
          digitalWrite(ledsArg[1], HIGH);
          digitalWrite(ledsArg[2], LOW);
          digitalWrite(ledsArg[3], LOW);
          newPosition = 6;
        }
        else if(motorArg.speedIndex == 1){
          digitalWrite(ledsArg[0], HIGH);
          digitalWrite(ledsArg[1], HIGH);
          digitalWrite(ledsArg[2], HIGH);
          digitalWrite(ledsArg[3], LOW);
          newPosition = 10;
        }
        else if(motorArg.speedIndex == 0){
          digitalWrite(ledsArg[0], HIGH);
          digitalWrite(ledsArg[1], HIGH);
          digitalWrite(ledsArg[2], HIGH);
          digitalWrite(ledsArg[3], HIGH);
          newPosition = 14;
        }

        // switch (motorArg.speedIndex){
        //   case 0;
        //     newPosition = 14;
        //     break;
        // }
        // {
        // case /* constant-expression */:
        //   /* code */
        //   break;
        
        // default:
        //   break;
        // }
        openCelluloidArg.oldPosition = newPosition;
      }
      bounceEncoButtonArg.update();
      if (bounceEncoButtonArg.fell() || bounceEncoButtonArg.rose()){
        stepperArg.setMaxSpeed(motorArg.maxSpeed / motorArg.speeds1[motorArg.speedIndex]);
        //Serial.printf("Speed changed \n");
        stateArg = State::idle;
        }
    }
    
  }




void timerTest(){

  captureFlag = true;
  //delay(10);
}

void initMode(Mode &modeArg, Motor &motorArg, Stepper &stepperArg, std::array<int, 6> &ledsArg){

  switch (modeArg){
    case Mode::continuous:
      motorArg.maxSpeed = 15000;

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

      digitalWrite(ledsArg[5], HIGH);
      digitalWrite(ledsArg[6], LOW);
      motorArg.selectedSpeed = motorArg.maxSpeed / motorArg.speeds1[0];

      stepperArg.setMaxSpeed(motorArg.maxSpeed / motorArg.speeds1[motorArg.speedIndex]);
      stepperArg.setAcceleration(motorArg.maxAcceleration); 
      
      Serial.print('h');

      break;

    case Mode::discrete:
      motorArg.maxSpeed = 2500;

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

      digitalWrite(ledsArg[5], LOW);
      digitalWrite(ledsArg[6], HIGH);
      //motorArg.selectedSpeed = motorArg.maxSpeed / motorArg.speeds1[0];

      stepperArg.setMaxSpeed(motorArg.maxSpeed / motorArg.speeds1[motorArg.speedIndex]);
      stepperArg.setAcceleration(motorArg.maxAcceleration); 
      
      Serial.print('i');

      break;

  }




}




