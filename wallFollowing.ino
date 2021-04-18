#include <Wire.h>
#include <Servo.h>
#include <Zumo32U4.h>

Zumo32U4Buzzer buzzer;
Zumo32U4Motors motors;
Zumo32U4Encoders encoders;

Servo headServo; // create servo object ot control a servo

// switches
const boolean HEAD_DEBUG = false;

// head servo timing
unsigned long headCm;
unsigned long headPm;
const unsigned long HEAD_MOVEMENT_PERIOD = 130;

// head servo constants
const int HEAD_SERVO_PIN = 0;
const int NUM_HEAD_POSITIONS = 3;
const int HEAD_POSITIONS[NUM_HEAD_POSITIONS] = {0, 80, 180};

// head servo data
boolean headDirectionClockwise = true;
int currentHeadPosition = 2;

// Initialize Ultrasonic
const int ECHO_PIN = 30;
const int TRIG_PIN = 17;

// Ultrasonic Maxs
const int MAX_DISTANCE = 200; //(200 cm / 2 meters)

// Ultrasonic timing
unsigned long usCm;
unsigned long usPm;
const unsigned long US_PERIOD = 50;

// current US distance reading
float distance = 0;
float leftDistance = 0;
boolean readFlag = false;


// Motor timing
unsigned long motorCm;
unsigned long motorPm;
const unsigned long MOTOR_PERIOD = 20; // time to wait between adjusting the motor speed

// determine the normalization factor based on MAX_DISTANCE
const float DISTANCE_FACTOR = MAX_DISTANCE / 100;
const float STOP_DISTANCE =5;

// Motor constants
const float MOTOR_BASE_SPEED = 200.0;
const int MOTOR_MIN_SPEED = 30;

// determine the normalizaiton factor based on MOTOR_BASE_SPEED
const float MOTOR_FACTOR = MOTOR_BASE_SPEED / 100;

double desiredState = 15;
double k = 4;
double error;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(57600);

  // initialize the head position to start
  headServo.attach(HEAD_SERVO_PIN);
  headServo.write(40);

  pinMode(ECHO_PIN, INPUT);
  pinMode(TRIG_PIN, OUTPUT);

  // start delay
  delay(3000);
  buzzer.play("c32");

}

void loop() {
  // put your main code here, to run repeatedly:
  // perform head movement
  moveHead();
  
  // update the motor speeds
  setMotors();
}

void moveHead() {
  headCm = millis();
  if(headCm > headPm + HEAD_MOVEMENT_PERIOD) {
    // position head ot the current position in the array
    headServo.write( HEAD_POSITIONS[currentHeadPosition] );
    /**
     * set next head position
     * move servo to the next head position and chnages direction when needed
     */
     if(headDirectionClockwise) {
      if(currentHeadPosition >= (NUM_HEAD_POSITIONS - 1)) {
        headDirectionClockwise = !headDirectionClockwise;
        currentHeadPosition--;
      } else {
        currentHeadPosition++;
      }
     } else {
        if(currentHeadPosition <= 0) {
          headDirectionClockwise = !headDirectionClockwise;
          currentHeadPosition++;
        } else {
          currentHeadPosition--;
        }
     }
     readFlag = false;
     usReadCm();
     // reset pervious millis
     headPm = headCm;
  }
}

void usReadCm() {
   usCm = millis();
   if(usCm > usPm + US_PERIOD && currentHeadPosition == 0 && readFlag == false) {
  
      // Clear the TRIG_PIN (set low)
      digitalWrite(TRIG_PIN, LOW);
      delayMicroseconds(2);
  
      // Set the TRIG_PIN HIGH (ACTIVE) for 10 microseconds
      digitalWrite(TRIG_PIN,HIGH);
      delayMicroseconds(10);
      digitalWrite(TRIG_PIN, LOW);
  
      // Read the ECHO_PIN, returns the sound wave travel time in microseconds
      // note the duration (38000 microseconds) that will allow for reading up max distance supported by the sensor
      long duration = pulseIn(ECHO_PIN, HIGH, 38000);
      // Calculating the distance
      distance = duration * 0.034 / 2; // Time of flight equation: Speed of sound wave devided by 2
  
      // apply limits
      if (distance > MAX_DISTANCE) distance = MAX_DISTANCE;
      if(distance == 0) distance = MAX_DISTANCE;

      if (HEAD_POSITIONS[currentHeadPosition] == 0) {
        leftDistance = distance;
      }

      if (HEAD_POSITIONS[currentHeadPosition] == 0) {
        leftDistance = distance;
      }
      
      if (HEAD_DEBUG == false) {
      Serial.println(leftDistance);
      }
      // head debug output
      if(HEAD_DEBUG) {
        Serial.print("Distance at ");
        Serial.print(HEAD_POSITIONS[currentHeadPosition]);
        // Displays the distance on the Serial Monitor
        Serial.print(" = ");
        Serial.print(distance);
        Serial.println("cm");
      }
      readFlag = true;
      usPm = usCm;
    }
}

void setMotors() {
  motorCm = millis();
  if(motorCm > motorPm + MOTOR_PERIOD) {

    // get error for left angle
    error = leftDistance - desiredState;
    
    //determine error for left turn
    int motorSpeed = k * error;
    int rightSpeed = MOTOR_BASE_SPEED + motorSpeed;
    int leftSpeed = MOTOR_BASE_SPEED - motorSpeed;

    leftSpeed = constrain(leftSpeed, 100, 255);
    rightSpeed = constrain(rightSpeed, 100, 255);

    // apply to motors
    motors.setSpeeds(leftSpeed, rightSpeed);

    motorPm = motorCm;
    
  }
}
