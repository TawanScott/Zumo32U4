#include <Wire.h>
#include <Zumo32U4.h>

Zumo32U4Buzzer buzzer;
Zumo32U4Motors motors;

// Initialize Ultrasonic
const int ECHO_PIN = 30;
const int TRIG_PIN = 17;

// Ultrasonic Maxs
const int MAX_DISTANCE = 100; //(100 cm / 1 meters)

// determine the normalization factor based on MAX_DISTANCE
const float DISTANCE_FACTOR = MAX_DISTANCE / 100;
const float STOP_DISTANCE =5;

// Motor constants
const float MOTOR_BASE_SPEED = 300.0;
const int MOTOR_MIN_SPEED = 30;

// determine the normalizaiton factor based on MOTOR_BASE_SPEED
const float MOTOR_FACTOR = MOTOR_BASE_SPEED / 100;

// motor compensation
const float L_MOTOR_FACTOR = 1.0;
const float R_MOTOR_FACTOR = .90;
const float L_MOTOR_FACTOR_THRESHOLD = 80;
const float R_MOTOR_FACTOR_THRESHOLD = 80;

// Ultrasonic timing
unsigned long usCm;
unsigned long usPm;
const unsigned long US_PERIOD = 50;

// Motor timing
unsigned long motorCm;
unsigned long motorPm;
const unsigned long MOTOR_PERIOD = 20; // time to wait between adjusting the motor speed

// current US distance reading
int distance = 0;

void setup() {
  // put your setup code here, to run once:
  pinMode(ECHO_PIN, INPUT);
  pinMode(TRIG_PIN, OUTPUT);

  delay(1000);
  buzzer.play("c32");
}

void loop() {
  // put your main code here, to run repeatedly:

  // update the current distance
  usReadCm();

  // update the motor speeds
  setMotors();
}

void usReadCm() {
  usCm = millis();
  if(usCm > usPm + US_PERIOD) {

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

    // Displays the distance on the Serial Monitor
    Serial.print("Distance: ");
    Serial.print(distance);
    Serial.println(" cm");

    // update the prevmillis
    usPm = usCm;
  }
}

void setMotors() {
  motorCm = millis();
  if(motorCm > motorPm + MOTOR_PERIOD) {

    // start out with the MOTOR_BASE_SPEED
    float leftSpeed = MOTOR_BASE_SPEED;
    float rightSpeed = MOTOR_BASE_SPEED;

    // check to see if most current distance measurement is less than / equal to MAX_DISTANCE
    if(distance <= MAX_DISTANCE) {

      // determine the magnitude of the distance by taking the difference (short distance = high magnitude)
      // divide by the Distance_FACTOR to ensure uniform reponse as MAX_DISTANCE changes
      // this maps the distance range (1 - MAX_RANGE) to 0-100 for the magnitude
      float magnitude = (float)(MAX_DISTANCE - distance) / DISTANCE_FACTOR;
      // example 1: MAX_DISTANCE = 80, distance = 40: 80-40 = 40/.8 = 50 (mid range)
      // example 2: MAX_DISTANCE = 160, distance = 40: 160-40 = 120/1.6 = 75 (top 1/4)

      // multiple the magnitude by the MOTOR_FACTOR to map the magnitude range (0-100) to the motors
      // (0 - MOTOR_BASE_SPEED)
      leftSpeed = MOTOR_BASE_SPEED - (magnitude * MOTOR_FACTOR);
      rightSpeed = MOTOR_BASE_SPEED - (magnitude * MOTOR_FACTOR);
      
    }

    // lower limit check
    if(leftSpeed < MOTOR_MIN_SPEED) leftSpeed = MOTOR_MIN_SPEED;
    if(rightSpeed < MOTOR_MIN_SPEED) rightSpeed = MOTOR_MIN_SPEED;

    // add in motor compensation
    if(leftSpeed <= L_MOTOR_FACTOR_THRESHOLD) {
      leftSpeed *= L_MOTOR_FACTOR;
    }
    if(rightSpeed <= R_MOTOR_FACTOR_THRESHOLD) {
      rightSpeed *= R_MOTOR_FACTOR;
    }

    // check stop distance 
    if(distance <= STOP_DISTANCE) leftSpeed = 0;
    if(distance <= STOP_DISTANCE) rightSpeed = 0;

    Serial.print("Left: ");
    Serial.print(leftSpeed);
    Serial.print(" Right: ");
    Serial.println(rightSpeed);

    motors.setSpeeds(leftSpeed, rightSpeed);

    motorPm = motorCm;
    
  }
}
