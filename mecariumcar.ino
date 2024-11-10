#include <MotorDriver.h>

// Initialize wheels
MotorDriver m;

#define forn_right 2
#define forn_left  3
#define rear_right 1
#define rear_left  4

#define object A5

// Sensors
const int sensor[5] = { A0, A1, A2, A3, A4 };
int checked[5];

bool lol =  false, 
     task_state_1 = false,
     task_state_2 = false,
     task_state_3 = false,
     task_state_4 = false,
     task_state_5 = false,
     task_state_6 = false,
     task_state_7 = false,
     task_state_8 = false;

unsigned int current_task = 0;

// Enum for driving modes
enum Mode {
  STOP,
  FOR_WARD,
  BACK_WARD,
  TURN_LEFT,
  TURN_RIGHT,
  GO_SIDE,
  GO_AROUND,
  TURN_SIDE,
  TURN_AROUND,
};
Mode currentMode;

// Motor speeds
int FR = 0, FL = 0, RR = 0, RL = 0;


void setup() {
  // Initialize the serial communication
  Serial.begin(115200);

  for (int i = 0; i < 5; i++) {
    pinMode(sensor[i], INPUT);
  }

  pinMode(forn_right, OUTPUT);
  pinMode(forn_left,  OUTPUT);
  pinMode(rear_right, OUTPUT);
  pinMode(rear_left,  OUTPUT);
  pinMode(object,     INPUT);

    // Initialize motor driver
  m.motor(forn_right, RELEASE, 0);
  m.motor(forn_left,  RELEASE, 0);
  m.motor(rear_right, RELEASE, 0);
  m.motor(rear_left,  RELEASE, 0);
  
  currentMode = STOP;
}


// Read forn
int read_Forn(int checked[5]) {

  if (checked[0] == 0 && checked[1] == 1 && checked[2] == 1 && checked[3] == 1 && checked[4] == 1) return -2;  // Far left
  if (checked[1] == 0 && checked[0] == 1 && checked[2] == 1 && checked[3] == 1 && checked[4] == 1) return -1;  // Left
  if (checked[2] == 0) return 0;                                                                               // Center
  if (checked[3] == 0 && checked[0] == 1 && checked[1] == 1 && checked[2] == 1 && checked[4] == 1) return 1;   // Right
  if (checked[4] == 0 && checked[0] == 1 && checked[1] == 1 && checked[2] == 1 && checked[3] == 1) return 2;   // Far right
  return 0;                                                                                                    // Default
}

void PID(float error, Mode mode, int* FR, int* FL, int* RR, int* RL) {
  // PID gains
  static float Kp = 1.2,
               Ki = 0.01,
               Kd = 0.01;

  // PID variables
  static float previous_error = 0,
               integral = 0;
  float dt = 0.01;
  int baseSpeed = 63;

  // PID calculation
  integral += error;
  float derivative = (error - previous_error) / dt;
  float pid_output = Kp * error + Ki * integral + Kd * derivative;
  previous_error = error;

  // Handle different driving modes
  switch (mode) {
    case STOP:
      *FR = 0;
      *FL = 0;
      *RR = 0;
      *RL = 0;
      break;

    case FOR_WARD:
      *FR = baseSpeed - pid_output;
      *FL = baseSpeed + pid_output;
      *RR = baseSpeed - pid_output;
      *RL = baseSpeed + pid_output;
      break;

    case BACK_WARD:
      *FR = baseSpeed - pid_output;
      *FL = baseSpeed + pid_output;
      *RR = baseSpeed - pid_output;
      *RL = baseSpeed + pid_output;
      break;
          
    case TURN_LEFT:
      *FR = baseSpeed + abs(pid_output * 1.5);
      *FL = (baseSpeed * 0.6) - (pid_output * 2);
      *RR = baseSpeed + abs(pid_output * 1.5);
      *RL = (baseSpeed * 0.6) - (pid_output * 2);
      break;

    case TURN_RIGHT:
      *FR = (baseSpeed * 0.6) - (pid_output * 2);
      *FL = baseSpeed + abs(pid_output * 1.5);
      *RR = (baseSpeed * 0.6) - (pid_output * 2);
      *RL = baseSpeed + abs(pid_output * 1.5);
      break;

  }

  // Constrain motor speeds to prevent going beyond limits
  *FR = constrain(*FR, 0, 120); 
  *FL = constrain(*FL, 0, 120); 
  *RR = constrain(*RR, 0, 120); 
  *RL = constrain(*RL, 0, 120); 
}



void loop() {

  if( Serial.available()){
    char cmd = Serial.read();

      if(cmd == 'R'){

        lol = true;

      }
    }
  
  // Read sensor values into the checked array
  for (int i = 0; i < 5; i++) {
    checked[i] = digitalRead(sensor[i]);
  }
  int testing = digitalRead(object);


if(lol){

  // Manage task states and transitions
  if (!task_state_1 && current_task == 0) {
    condition_1();
    if (isTask1Completed()) {
      task_state_1 = true;
      current_task++;
      release();
    }

  } else if (!task_state_2 && current_task == 1) {
    currentMode = TURN_AROUND;
    if (isTask2Completed()) {
      task_state_2 = true;
      current_task++;
      release();
      currentMode = BACK_WARD;
    }

  } else if (!task_state_3 && current_task == 2) {
    condition_3();
    if (isTaskMode()) {
      task_state_3 = true;
      current_task++;
    }
  } else if (!task_state_4 && current_task == 3) {
    if (testing == 0) {
      condition_4();
      if (isTask4Completed()) {
        task_state_4 = true;
        current_task++;
        release();
      }
    } else {
      release();
    }

  } else if (!task_state_5 && current_task == 4) {
    condition_5();
    if (isTaskMode()) {
      task_state_5 = true;
      current_task++;
      release();
    }

  } else if (!task_state_6 && current_task == 5) {
    if (testing == 1) {
      condition_6();
      if (isTask6Completed()) {
        task_state_6 = true;
        current_task++;
        release();
      }
    } else {
      release();
    }

  } else if (!task_state_7 && current_task == 6) {
    condition_7();
    if (isTask2Completed()) {
      task_state_7 = true;
      current_task++;
      release();

  } else if(!task_state_8 && current_task == 7)
      condition_6();
      if (isTaskMode()) {
        task_state_8 = true;
        current_task++;
        release();
      }

  }

}

  // Check if all tasks are completed
  if (task_state_1 && task_state_2 && task_state_3 && task_state_4 && task_state_5 && task_state_6 && task_state_7) {
    // Reset all task states and current task
    task_state_1 = false;
    task_state_2 = false;
    task_state_3 = false;
    task_state_4 = false;
    task_state_5 = false;
    task_state_6 = false;
    task_state_7 = false;
    current_task = 0;
    lol = false;
  }



    // Calculate error and apply PID
    int error_forn = read_Forn(checked);
    PID(error_forn, currentMode, &FR, &FL, &RR, &RL);

  // Apply motor speeds
  applyMotorSpeeds_Red(currentMode);
}


void release(){
     m.motor(forn_right,RELEASE, 0);
     m.motor(forn_left, RELEASE, 0);
     m.motor(rear_right,RELEASE, 0);
     m.motor(rear_left, RELEASE, 0);
     delay(300);
}

void condition_1() {

bool r = false;

  if (checked[0] == 0 && checked[1] == 0 && checked[2] == 0 && checked[3] == 0 && checked[4] == 0) {
        currentMode = GO_SIDE;
        r = false;

  } else{
        r = true;
  }

  if (checked[0] == 1 && checked[1] == 1 && checked[2] == 0 && checked[3] == 1 && checked[4] == 1) {
        currentMode = FOR_WARD;

  } else if ((checked[0] == 1 && checked[1] == 0 && checked[2] == 1 && checked[3] == 1 && checked[4] == 1) || 
             (checked[0] == 0 && checked[1] == 1 && checked[2] == 1 && checked[3] == 1 && checked[4] == 1)) { 
        currentMode = TURN_LEFT;

  } else if ((checked[0] == 1 && checked[1] == 1 && checked[2] == 1 && checked[3] == 0 && checked[4] == 1) || 
             (checked[0] == 1 && checked[1] == 1 && checked[2] == 1 && checked[3] == 1 && checked[4] == 0)) {       
         currentMode = TURN_RIGHT;

  } else {
         if (r) currentMode = FOR_WARD;
  }
}

bool isTask1Completed() {
  return (checked[0] == 1 && checked[1] == 1 && checked[2] == 0 && checked[3] == 0 && checked[4] == 0);
}   


bool isTask2Completed() { // task 9 
  return (checked[0] == 1 && checked[1] == 1 && checked[2] == 0 && checked[3] == 1 && checked[4] == 1);
}



void condition_3() {

        currentMode = BACK_WARD;

}

bool isTaskMode() { // task 10 , task 5
  return  (checked[0] == 0 && checked[1] == 0 && checked[2] == 0 && checked[3] == 0 && checked[4] == 0);
}


void condition_4() {

     currentMode = GO_AROUND; 

}

bool isTask4Completed() {
  return (checked[0] == 1 && checked[1] == 1 && checked[2] == 0 && checked[3] == 1 && checked[4] == 1);
}



void condition_5() {  //task7- 14

bool 

          if ((checked[0] == 1 && checked[1] == 0 && checked[2] == 0 && checked[3] == 0 && checked[4] == 0) || 
              (checked[0] == 1 && checked[1] == 1 && checked[2] == 0 && checked[3] == 0 && checked[4] == 0) || 
              (checked[0] == 1 && checked[1] == 1 && checked[2] == 1 && checked[3] == 0 && checked[4] == 0)) {
              
            currentMode = TURN_SIDE;

          } else if (checked[0] == 1 && checked[1] == 1 && checked[2] == 0 && checked[3] == 1 && checked[4] == 1) {
            currentMode = FOR_WARD;

          } else if ((checked[0] == 1 && checked[1] == 0 && checked[2] == 1 && checked[3] == 1 && checked[4] == 1) || 
                     (checked[0] == 0 && checked[1] == 1 && checked[2] == 1 && checked[3] == 1 && checked[4] == 1)) {
            currentMode = TURN_LEFT;

          } else if ((checked[0] == 1 && checked[1] == 1 && checked[2] == 1 && checked[3] == 0 && checked[4] == 1) || 
                     (checked[0] == 1 && checked[1] == 1 && checked[2] == 1 && checked[3] == 1 && checked[4] == 0)) {
            currentMode = TURN_RIGHT;

          } else {
            currentMode = FOR_WARD;  // Default mode if no other conditions match
          }
}








void condition_6(){

  if (checked[0] == 1 && checked[1] == 1 && checked[2] == 0 && checked[3] == 1 && checked[4] == 1) {
        currentMode = FOR_WARD;

  } else if ((checked[0] == 1 && checked[1] == 0 && checked[2] == 1 && checked[3] == 1 && checked[4] == 1) || 
             (checked[0] == 0 && checked[1] == 1 && checked[2] == 1 && checked[3] == 1 && checked[4] == 1)) { 
        currentMode = TURN_LEFT;

  } else if ((checked[0] == 1 && checked[1] == 1 && checked[2] == 1 && checked[3] == 0 && checked[4] == 1) || 
             (checked[0] == 1 && checked[1] == 1 && checked[2] == 1 && checked[3] == 1 && checked[4] == 0)) {       
         currentMode = TURN_RIGHT;

  } else {
         currentMode = FOR_WARD;
  }

}


bool isTask6Completed() { // task 12
  return ((checked[0] == 0 && checked[1] == 0 && checked[2] == 0 && checked[3] == 1 && checked[4] == 1)||
          (checked[0] == 1 && checked[1] == 1 && checked[2] == 0 && checked[3] == 0 && checked[4] == 0));
}


void condition_7 (){
      currentMode = TURN_SIDE;

}

bool isTask7Completed() { // task 12
  return ((checked[0] == 1 && checked[1] == 1 && checked[2] == 0 && checked[3] == 1 && checked[4] == 1)||
          (checked[0] == 1 && checked[1] == 1 && checked[2] == 0 && checked[3] == 0 && checked[4] == 1)||
          (checked[0] == 1 && checked[1] == 0 && checked[2] == 0 && checked[3] == 1 && checked[4] == 1));
}



void applyMotorSpeeds_Red(Mode currentMode) {

  switch (currentMode) {
    case FOR_WARD:
      m.motor(forn_right, FORWARD, FR);
      m.motor(forn_left,  FORWARD, FL);
      m.motor(rear_right, FORWARD, RR);
      m.motor(rear_left,  FORWARD, RL);
      break;

    case BACK_WARD:
      m.motor(forn_right, BACKWARD,  100);
      m.motor(forn_left,  BACKWARD,  116);
      m.motor(rear_right, BACKWARD,  100);
      m.motor(rear_left,  BACKWARD,  116);
      break;

    case TURN_LEFT:
      m.motor(forn_right, FORWARD, FR);
      m.motor(forn_left,  FORWARD, FL);
      m.motor(rear_right, FORWARD, RR);
      m.motor(rear_left,  FORWARD, RL);
      break;

    case TURN_RIGHT:
      m.motor(forn_right, FORWARD, FR);
      m.motor(forn_left,  FORWARD, FL);
      m.motor(rear_right, FORWARD, RR);
      m.motor(rear_left,  FORWARD, RL);
      break;

    case GO_SIDE:
      m.motor(forn_right, BACKWARD, 140);
      m.motor(forn_left,  FORWARD,  140);
      m.motor(rear_right, FORWARD,  140);
      m.motor(rear_left,  BACKWARD, 140);
      break;

    case TURN_SIDE:
      m.motor(forn_right, RELEASE,   0);
      m.motor(forn_left,  FORWARD,   110);
      m.motor(rear_right, BACKWARD,  110);
      m.motor(rear_left,  FORWARD,   110);
      break;

    case TURN_AROUND:
      m.motor(forn_right, BACKWARD,  90);
      m.motor(forn_left,  FORWARD,   90);
      m.motor(rear_right, BACKWARD,  90);
      m.motor(rear_left,  FORWARD,   90);    
      break;

    case GO_AROUND:
      m.motor(forn_right, FORWARD,   110);
      m.motor(forn_left,  RELEASE,   0);
      m.motor(rear_right, FORWARD,   110);
      m.motor(rear_left,  BACKWARD,  110);
      break;
      
    case STOP:
    default:
      m.motor(forn_right, RELEASE,  0);
      m.motor(forn_left,  RELEASE,  0);
      m.motor(rear_right, RELEASE,  0);
      m.motor(rear_left,  RELEASE,  0);
      break;
  }
}


