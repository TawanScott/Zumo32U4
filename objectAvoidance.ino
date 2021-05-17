#include <Wire.h>
#include <Servo.h>
#include <Zumo32U4.h>
#include <math.h>

Zumo32U4Encoders encoders;
Zumo32U4Buzzer buzzer;
Zumo32U4Motors motors;

// create servo object to control a servo
Servo headServo; 

// switches
const boolean HEAD_DEBUG = false;

// head servo timing
unsigned long headCm;
unsigned long headPm;
const unsigned long HEAD_MOVEMENT_PERIOD = 130;

// head servo constants
const int HEAD_SERVO_PIN = 0;
const int NUM_HEAD_POSITIONS = 3;
const int HEAD_POSITIONS[NUM_HEAD_POSITIONS] = {20, 80, 130};

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
float distance = MAX_DISTANCE;
float leftDistance = MAX_DISTANCE;
float rightDistance = MAX_DISTANCE;
float frontDistance = MAX_DISTANCE;

//encoder
long countsLeft = 0;
long countsRight = 0;
long prevLeft = 0;
long prevRight = 0;

//motor physical attributes
const int CLICKS_PER_ROTATION = 12;
const float GEAR_RATIO = 75.81F;
const float WHEEL_DIAMETER = 3.5;
const float WHEEL_CIRCUMFERENCE = 10.9557;
const float DISTANCE_BETWEEN_WHEELS = 9.00;

//motor speed
float rightSpeed;
float leftSpeed;

//change of centerpoint, theta, x, y 
float S;
float currentTheta = 0;
float currentThetaOld = 0;
float theta = 0;
float currentX = 0;
float currentXOld = 0;
float currentY = 0;
float currentYOld = 0;
float x = 0;
float y = 0;

// pid
float v = 300;
float vl = 0;
float vr = 0;
float w = 0;
float delta = 12;
float error = 0;
float errorP = 0;
float errorI = 0;
float p = 0;
float i = 0;
float lasterror = 0;
float Kp = 60;     
float Ki = 0.1;

// goals
const int NUMBER_OF_GOALS = 3;
float xGoals[NUMBER_OF_GOALS] = {30, 30, 0};
float yGoals[NUMBER_OF_GOALS] = {30, 60, 0};
int currentGoal = 0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(57600);
  
  // initialize the head position to start
  headServo.attach(HEAD_SERVO_PIN);
  headServo.write(20);

  pinMode(ECHO_PIN, INPUT);
  pinMode(TRIG_PIN, OUTPUT);
  
  delay(1000);
  buzzer.play("c32");
}

void loop() {
  // put your main code here, to run repeatedly:
  // perform head movement
  moveHead();
  usReadCm();
  // perform go to goal behavior while avoiding obstacles
  goToGoal();
}

// go towards x and y goal coordinates
void goToGoal() {
  checkPosition();

  // reach the last goal
  if (currentGoal > NUMBER_OF_GOALS){
    motors.setSpeeds(0,0);
  }
}

// servo movement
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
     // distinguish distance recordings for each of the positions
     if (HEAD_POSITIONS[currentHeadPosition] == 80) {
        frontDistance = distance;
     } else if (HEAD_POSITIONS[currentHeadPosition] == 20) {
        leftDistance = distance;
     } else {
        rightDistance = distance;
     }
     
     if (HEAD_DEBUG == false) {
        Serial.println(rightDistance);
     }
     // reset pervious millis
     headPm = headCm;
  }
}

// utilize ultrasonic sensors to calculate distance from robot
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

// compare current position to goal and input next goal if satisfied with current position
void checkPosition() {
  nextGoal(); 
  //compare position of x and  y to x and y goals
  if ((currentX > (x + delta)) || (currentX < (x - delta)) || (currentY > (y + delta)) || (currentY < (y - delta))) {
    pid();
    setMotor(vr, vl);   
    delay(100);  
  }
  //initiate next goal
  else if(currentGoal < NUMBER_OF_GOALS) {
    motors.setSpeeds(0,0);
    currentGoal++;
    delay(1000);
    //beeps when reaching each goals
    buzzer.play("c32");
    delay(1000);
    //last goal reached
  } else {
    motors.setSpeeds(0,0);
    delay(1000);
    //continously beep once completing all goals
    buzzer.play("c70");
  }
}

// set x and y goals according to the current goal
void nextGoal() {
  // if there are still more goals coordinates
  if (currentGoal != (NUMBER_OF_GOALS)){
    x = xGoals[currentGoal];
    y = yGoals[currentGoal];
    //reach end goal
  } else {
    motors.setSpeeds(0,0);
  }
}

//calculate theta, pid
void pid() {
  checkEncoders(); 
  //goal theta
  theta = atan2(y - currentY, x - currentX);
  
  //calculate error
  error = theta - currentTheta;
  error = atan2(sin(error), cos (error));
  errorP = error;
  //proportional
  p = Kp*errorP;
  errorI = errorI + error;
  //integral
  i = Ki*errorI;

  //omega
  w = p + i;
 
  lasterror = error;
  //set speed according to error 
  vr = (2*v + w*DISTANCE_BETWEEN_WHEELS)/(2*WHEEL_DIAMETER); 
  vl = (2*v - w*DISTANCE_BETWEEN_WHEELS)/(2*WHEEL_DIAMETER); 
}

//utilize wheel encoders to  calculate distance, x, y, theta
void checkEncoders() {
  countsLeft += encoders.getCountsAndResetLeft();
  countsRight += encoders.getCountsAndResetRight();
  //left and right wheel distance in cm
  float Sl = ((countsLeft - prevLeft) / (CLICKS_PER_ROTATION * GEAR_RATIO) * WHEEL_CIRCUMFERENCE);
  float Sr = ((countsRight - prevRight) / (CLICKS_PER_ROTATION * GEAR_RATIO) * WHEEL_CIRCUMFERENCE);
  //distance of robot
  float S = (Sr + Sl)/2;
  //current x and y positions
  currentX = currentXOld + S*cos(currentThetaOld);
  currentY = currentYOld + S*sin(currentThetaOld);
  //current theta
  currentTheta = currentThetaOld + ((Sr-Sl)/DISTANCE_BETWEEN_WHEELS);
  currentTheta = atan2 (sin(currentTheta),cos(currentTheta));
  
  prevLeft = countsLeft;
  prevRight = countsRight;
  
  currentXOld = currentX;
  currentYOld = currentY;
  currentThetaOld = currentTheta;
}

//set speeds for left and right motors according to pid()
void setMotor(float vr, float vl) {
  /**
  if(frontDistance < 20) {
    motors.setSpeeds(-80, 80);
  }
  **/
  // if an object is detected on the left side of the robot
  if(leftDistance <= 20) {
    motors.setSpeeds(150, 40);
    // if an object is detected on the right side of the robot
  } else if(rightDistance <= 20) {
    motors.setSpeeds(40, 150);
    // no obstacle, continue towards goal using information from pid controller
  } else {
    if (vr > 0 && vl > 0) {   
      rightSpeed = vr;
      leftSpeed = vl;
      motors.setSpeeds(leftSpeed,rightSpeed);
    } else if (vr > 0 && vl < 0) {
      rightSpeed = vr;
      leftSpeed = abs(vl);
      motors.setSpeeds(leftSpeed, rightSpeed);
    } else if (vr < 0 && vl > 0) {
      rightSpeed = abs(vr);
      leftSpeed = vl;
      motors.setSpeeds(leftSpeed,rightSpeed);
    } else if (vr < 0 && vl < 0) {
      rightSpeed = abs(vr);
      leftSpeed = abs(vl);
      motors.setSpeeds(leftSpeed,rightSpeed);
    }
  }
}
