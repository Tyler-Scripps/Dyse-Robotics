#include <Sparki.h>

#define Init_State 0
#define Search_State 1
#define Approach_State 2
#define Grab_State 3
#define Return_State 4
#define Trace_State 5

//globals are readings from sensors
// these are only updated when readSensors is called
int cm_dist = 1000;
int left = 1000;
int center = 1000;
int right = 1000;
int curr_state = 0;
int prev_dist = 0;
int ret_dist = 0;
int ir_threshold = 500;

void readSensors() {
  sparki.updateLCD();
  cm_dist = sparki.ping(); 
  left = sparki.lineLeft(); 
  right = sparki.lineRight(); 
  center = sparki.lineCenter();
  delay(25); 
}

void setup() {
  sparki.clearLCD(); 
  sparki.println("Hello World");
  sparki.println("Just warming up, give me a second");
  sparki.updateLCD();
  sparki.RGB(RGB_RED); // Turn on the red LED
  sparki.servo(SERVO_CENTER); // Center the ultrasonic sensor
  sparki.gripperClose();
  delay(2000); // Give the motor time to turn
  sparki.gripperOpen();
  delay(5000); // Give the motor time to open the griper
  sparki.gripperStop(); // 5 seconds should be long enough
  sparki.RGB(RGB_GREEN); // Change LED to green so we know the robot's setup is done!
  sparki.RGB(0,0,0);

}

void init_state_1(){
  sparki.motorRotate(MOTOR_LEFT, DIR_CW, 100);
  sparki.motorRotate(MOTOR_RIGHT, DIR_CW, 100);
}

void init_state_2(){
  sparki.moveStop();
  sparki.moveLeft(18);
  sparki.moveForward(cm_dist);
}

void init_state_3(){
  sparki.gripperClose();
  delay(2000);
}

void init_state_4(){
  sparki.motorStop(MOTOR_GRIPPER);
  sparki.moveRight(180);
  delay(100);
  sparki.moveForward(ret_dist);
}

void state_5(){
  if(left < ir_threshold && right < ir_threshold){
    curr_state = 6;
    sparki.moveStop();
  }
  else if( left < ir_threshold){
    sparki.moveLeft();
  }
  else if( right < ir_threshold){
    sparki.moveRight();
  }
  else if( center < ir_threshold){
    sparki.moveForward();
  }
  else{
    sparki.moveRight();
  }
}

void loop() {
  sparki.clearLCD();
  sparki.print("State: ");
  sparki.println(curr_state);

  if( curr_state == 6){
    sparki.println("Yaay I finished, are you proud of me?");
  }
  sparki.updateLCD();

  readSensors();
  
  switch(curr_state){
    case Init_State:
      curr_state = 1;
      init_state_1();
    case Search_State:
      if(cm_dist <= 30){
        curr_state = 2;
        prev_dist  = cm_dist;
        readSensors();
        ret_dist = (cm_dist + prev_dist) / 2;
        init_state_2();
      }
      break;
      
    case Approach_State:
      if(cm_dist <= 2){
        curr_state = 3;
        init_state_3();
      }
      else if(cm_dist > 30){
        sparki.println("The object seems to have vanished, restarting");
        curr_state = 0;
      }
      break;
      
    case Grab_State:
      curr_state = 4;
      init_state_4();
      break;
      
    case Return_State:
      curr_state = 5;
      break;
      
    case Trace_State:
      state_5();
      break;  
  }
}
