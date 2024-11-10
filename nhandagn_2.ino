#include <Servo.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Arduino.h>

LiquidCrystal_I2C lcd(0x27, 16, 2);

// Create Servo objects
Servo myServo_1, myServo_3, myServo_4;

int rotationTime = 350; // Servo rotation time for one turn

// Define values to receive from OpenCV
#define str_1 '1'
#define str_2 '2'
#define str_3 '3'
#define str_4 '4'

#define red    'R'
#define yellow 'Y'

// Declare global variables
int a = 0;
int b = 0;

// Declare variables for millis
bool value_1 = false, value_2 = false;
bool servo3_active = false, servo4_active = false;  // Flags to track servo actions

// Enum for Servo and Sensor pins
enum ServoPin {
  servo_1 = 9,
  servo_2 = 3,
  servo_3 = 11,
  servo_4 = 10,
  bt = 12
};

enum SensorPin {
  sensor_1 = A0,
  sensor_2 = A1,
  sensor_3 = A2,
  sensor_4 = A3
};

int read1 = 0, read2 = 0, read3 = 0, read4 = 0;

void setup() {
  Serial.begin(9600);

  // Set sensor pins as input
  pinMode(sensor_1, INPUT_PULLUP);
  pinMode(sensor_2, INPUT_PULLUP);
  pinMode(sensor_3, INPUT_PULLUP);
  pinMode(sensor_4, INPUT_PULLUP);

  // Set bt pin as output
  pinMode(bt, OUTPUT);
  pinMode(servo_2, OUTPUT);

  // Initialize LCD
  lcd.init();
  lcd.backlight();

  // Attach servos
  myServo_1.attach(servo_1);
  myServo_3.attach(servo_3);
  myServo_4.attach(servo_4);

  myServo_1.write(0);
  myServo_3.write(0);
  myServo_4.write(0);
}

void rotationOneTurn() {
  int pulseWidth = 2000;
  unsigned long startTime = millis();

  while (millis() - startTime < rotationTime) {
    digitalWrite(servo_2, HIGH);
    delayMicroseconds(pulseWidth);
    digitalWrite(servo_2, LOW);
    delay(20 - pulseWidth / 2500); // Adjusted to maintain a 50Hz signal
  }
  stopServo();
}

void stopServo() {
  for (int i = 0; i < 50; i++) {
    digitalWrite(servo_2, HIGH);
    delayMicroseconds(1500); // Neutral position for standard servo
    digitalWrite(servo_2, LOW);
    delay(20 - 1500 / 2500); // Maintain 50Hz signal
  }
}

// Servo 1 moves when condition met
void get_1() {
  myServo_1.write(70);
  a += 1;
}

// Servo 2 turns once
void get_2() {
  rotationOneTurn();
  b += 1;
}

// Servo 3 moves when a = 3 and sensor 3 is triggered
void get_3() {
  if (a == 3 && read3 == 0) {
    myServo_3.write(90);
    servo3_active = true;  // Mark servo_3 as active
    a = 0;
  }
}

// Servo 4 moves when b = 3 and sensor 4 is triggered
void get_4() {
  if (b == 3 && read4 == 0) {
    myServo_4.write(90);
    servo4_active = true;  // Mark servo_4 as active
    b = 0;
  }
}

void loop() {
  read1 = digitalRead(sensor_1);
  read2 = digitalRead(sensor_2);
  read3 = digitalRead(sensor_3);
  read4 = digitalRead(sensor_4);

  if (Serial.available() > 0) {
    char cmd = Serial.read();
    switch (cmd) {
      case '1':
        value_1 = true;
        break;
      case '2':
        value_2 = true;
        break;
      default:
        break;
    }
  }

  // Handle Servo 1 action based on sensor and serial input
  if (read1 == 0 && value_1) {
    delay(950);  // Can adjust this timing to your needs
    get_1();
    delay(2400); 
    value_1 = false;
    if (a == 3) {
      Serial.println(red);
    }
  } else {
    myServo_1.write(0);
    value_1 = false;
  }

  // Handle Servo 2 action based on sensor and serial input
  if (read2 == 0 && value_2) {
    delay(1400);
    get_2();
    value_2 = false;
    if (b == 3) {
      Serial.println(yellow);
    }
  }

  // Handle Servo 3 and Servo 4 based on sensor conditions
  if (read4 == 0 && !servo4_active) {  // If servo4 is not already active
    get_4();
  } else if (read3 == 0 && !servo3_active) {  // If servo3 is not already active
    get_3();
  }

  // Reset bt pin only if both servos are inactive
  if (!servo3_active && !servo4_active) {
    digitalWrite(bt, HIGH);  // Reset bt pin when no servos are moving
  } else {
    digitalWrite(bt, LOW);   // Stop bt pin when a servo is moving
  }

  // If servos are done moving, reset them
  if (servo3_active && read3 == 1) {
    myServo_3.write(0);
    servo3_active = false;  // Reset the active flag when servo 3 is done
  }
  if (servo4_active && read4 == 1) {
    myServo_4.write(0);
    servo4_active = false;  // Reset the active flag when servo 4 is done
  }

  // Update LCD display
  display_lcd();
}

void display_lcd() {
  lcd.setCursor(0, 0);
  lcd.print("Red: ");
  lcd.setCursor(8, 0);
  lcd.print(a);

  lcd.setCursor(0, 1);
  lcd.print("Yellow: ");
  lcd.setCursor(8, 1);
  lcd.print(b);
}
