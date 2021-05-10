#include <Wire.h>
#include <Zumo32U4.h>
#include <math.h>

Zumo32U4Encoders encoders;
Zumo32U4Buzzer buzzer;
Zumo32U4Motors motors;

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
  delay(1000);
  buzzer.play("c32");
}

void loop() {
  // put your main code here, to run repeatedly:
  goToGoal();
}

void goToGoal() {
  checkPosition();

  if (currentGoal > NUMBER_OF_GOALS){
    motors.setSpeeds(0,0);
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
  if (currentGoal != (NUMBER_OF_GOALS)){
    x = xGoals[currentGoal];
    y = yGoals[currentGoal];
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
  if (vr>0 && vl>0) {   
    rightSpeed = vr;
    leftSpeed = vl;
    motors.setSpeeds(leftSpeed,rightSpeed);
  } else if (vr>0 && vl<0) {
    rightSpeed = vr;
    leftSpeed = abs(vl);
    motors.setSpeeds(leftSpeed, rightSpeed);
  } else if (vr<0 && vl>0) {
    rightSpeed = abs(vr);
    leftSpeed = vl;
    motors.setSpeeds(leftSpeed,rightSpeed);
  } else if (vr<0 && vl<0) {
    rightSpeed = abs(vr);
    leftSpeed = abs(vl);
    motors.setSpeeds(leftSpeed,rightSpeed);
  }
}
