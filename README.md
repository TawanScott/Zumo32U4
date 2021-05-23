# Zumo32U4
![zumo](https://user-images.githubusercontent.com/36020934/114920902-f0f4f380-9df7-11eb-8084-c9d60dc09ef0.jpg)
YouTube Demo: https://www.youtube.com/watch?v=iiGa7Rmoe9E

Required Components:
=================================================
* Zumo 32U4 Robot (Assembled with 75:1 HP Motors)
* SG90 Servo 0-180 degrees
* HC-SR04 Ultrasonic Sensor
* 470uf 25V Electrolytic Capacitor
* 3 Male-Male Header Pin
* 4 Male-Male Jumper Wires
* 4 Male-Female Jumper Wires
* 55 Points Breadboard

Code:
=================================================
Object Following:
* Object following behavior that utilize the motors and ultrasonic senor for accuracy, smoothness and dynamics control. 

Servo and Ultrasonic Sensor:
* Initial code for testing servo movement as well as accurate ultrasonic sensor readings at different servo angles. 

Wall Following:
* Utilizing a PID controller to keep the robot smoothly following walls.
* The rbot will be able to deal with changes in room layout, wall angles, and random obstacles it might encouter without prior knowledge of the environment.
* It will also act in a natural and efficient manner by moving at a good pace without having to stop or slow to deal with obstacles.

Go to Goal:
* This will implement a full localization controller in 2D cartesian space.
* The robot will be able to store multiple x and y coordinates in 1 program run.
* Additonally, the amount of stops along the run as well as the locations of the stops will be editable within the constants.
* Using a PID controller, the robot will be able to travel to each stops within 2 cm accuracy in both x and y space. 

Object Avoidance:
* Incorporate everything from go to goal and add the ability to avoid obstacles along the way. 
* The robot will be able to avoid randomly placed obstacles while navigating to the goal by taking in multipe readings from the range finding ultrasonic sensor at different angles.
