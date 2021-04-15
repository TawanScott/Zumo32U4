#include <Wire.h>
#include <Servo.h>
#include <Zumo32U4.h>

Zumo32U4Buzzer buzzer;

Servo headServo; // create servo object ot control a servo

// switches
const boolean HEAD_DEBUG = true;

// head servo timing
unsigned long headCm;
unsigned long headPm;
const unsigned long HEAD_MOVEMENT_PERIOD = 200;

// head servo constants
const int HEAD_SERVO_PIN = 0;
const int NUM_HEAD_POSITIONS = 7;
const int HEAD_POSITIONS[NUM_HEAD_POSITIONS] = {125, 110, 95, 80, 65, 50, 35};

// head servo data
boolean headDirectionClockwise = true;
int currentHeadPosition = 0;

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
boolean readFlag = false;

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

}

void moveHead() {
  headCm = millis();
  if(headCm > headPm + HEAD_MOVEMENT_PERIOD) {
    
    // position head ot the current position in the array
    headServo.write( HEAD_POSITIONS[currentHeadPosition] );
    usReadCm();
  
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
     // reset pervious millis
     headPm = headCm;
  }
}

void usReadCm() {
   usCm = millis();
   if(usCm > usPm + US_PERIOD && readFlag == false) {
  
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
  
      readFlag = true;
      
      // head debug output
      if(HEAD_DEBUG) {
        Serial.print("Distance at ");
        Serial.print(HEAD_POSITIONS[currentHeadPosition]);
        // Displays the distance on the Serial Monitor
        Serial.print(" = ");
        Serial.print(distance);
        Serial.println("cm");
      }
    }
}
