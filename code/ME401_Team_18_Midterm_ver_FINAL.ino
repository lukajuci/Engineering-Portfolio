#include "ME401_Radio.h"
#include "ME401_PID.h"
#include <Servo.h>
#define MY_ROBOT_ID 18
int leftServoPin = 33;         //servo pins, pin j18 is 32 and pin j20 is 33
int rightServoPin = 32;
Servo leftServo;               // initialize left and right servos
Servo rightServo;

#include <MedianFilterLib.h>
int irSensor1Pin = A7;        //NOT PIN 4, 5, 6, 11
MedianFilter<float> medianFilter(5);


//initialize motor sweep variables
volatile float tar_setpoint;   // volatile float setpoint = 0;
volatile int target;           // targeting global variable

//from IR/zombie
volatile float targetSetpoint;     // volatile float setpoint = 0;




void setup() {
  Serial.begin(115200);             // Set up the serial port in case we want output or input
  pinMode(bumperSwitch1, INPUT);    // set up limit switches for collision function
  state = HUMAN;                    // set initial state
  ME401_Radio_initialize();         // Initialize the RFM69HCW radio
  leftServo.attach(leftServoPin);   // Set up wheel servos
  rightServo.attach(rightServoPin);
  setupPIDandIR();                 // Initialize the PID and IR interrupts


  //IR/ZOMBIE setup

  pinMode(2, INPUT);                // Set up the quadrature inputs
  pinMode(20, INPUT);
  errorLeft = false;
  lastLeftA = digitalRead(2);
  lastLeftB = digitalRead(20);
  attachCoreTimerService(TimerCallback);
  pinMode(3, OUTPUT);           // Set up the motor outputs
  pinMode(4, OUTPUT);
  digitalWrite(3,0);
  digitalWrite(4,0);
  SoftPWMServoPWMWrite(3, 0);
  myPID.SetMode(AUTOMATIC);      //Setup the pid 
  myPID.SetSampleTime(pidSampleTime);
  myPID.SetOutputLimits(-255,255);

  
  
 // TODO: Change the kp, ki, kd in the ME491_PID_IR.h file to match your new tunings that you did after installing the sensor on your robot. Also need to Set up DC motor.
}

void loop() {
  Serial.print("millis:");
  Serial.println(millis());

  updateRobotPoseAndBallPositions();
  RobotPose ourBot = getRobotPose(MY_ROBOT_ID);     // create struct for our robot
  target = 0;                                       // state machine to control the robot
  
  switch (state) {
    case(HUMAN):
      tempstate = HUMAN;
      runHumanState();
      break;
      
    case(HEALER):
      tempstate = HEALER;
      runHealerState();
      break;
      
    case(ZOMBIE):
      tempstate = ZOMBIE;
      runZombieState();
      break;
      
    case(BALL):
      tempstate = BALL;
      runBallState();
      break;

    case(WALL):
      runWallState();
      break;

    default:
      state = HUMAN;
      break; 
  }
  
  
  delay(100);
  delay(20);
}

void runHumanState(){
  RobotPose ourBot = getRobotPose(MY_ROBOT_ID);       // create struct for our robot
  target = targetRobot();                             // run targeting function
  float new_x = 1000;                                 // set target location to middle of field
  float new_y = 1000;
  
  if( target != 0){                                   // check to see if there is a target zombie
    new_x = (getRobotPose(target).x);                 // set target to zombie location, running will be handled in the movement function
    new_y = (getRobotPose(target).y);
  }
  
  movement(ourBot.x, ourBot.y, ourBot.theta * 0.001, new_x, new_y);       // call the movement function
  
  //change states based on updates to status
  if (ourBot.zombie == true)      
  {
    state = ZOMBIE;
  }
   if (ourBot.healing == true)
  {
    state = HEALER;
  }
  if (numBalls > 0)
  {
    state = BALL;
  }
}

void runHealerState(){
  RobotPose ourBot = getRobotPose(MY_ROBOT_ID);       // create struct for our robot
  target = targetRobot();                             // call targeting function to get a target robot and its ID
  float new_x = 1000;
  float new_y = 1000;
  
  if( target != 0){                    // check to see if there is a target zombie
    new_x = getRobotPose(target).x;    // set new_x and new_y to thetarget robot x and y to heal
    new_y = getRobotPose(target).y;  
  }

  movement(ourBot.x, ourBot.y, ourBot.theta * 0.001, new_x, new_y);

   if (ourBot.healing == false)
  {
    state = HUMAN;
  }
}

void runZombieState(){
  RobotPose ourBot = getRobotPose(MY_ROBOT_ID);      // create struct for our robot
  Serial.println("ZOMBIE STATE");
  // Start of IR sensor section
  // If this section of code is included, and the serial monitor is opened, the DC sweep begins to behave erratically, placing your hand in front of the sensor causes expecially erratic behavior.
  int irSensor1Value = analogRead(irSensor1Pin);
  float rawMeasure = irSensor1Value;
  float median = medianFilter.AddValue(rawMeasure);
  float actual_distance = abs(convertADC2Distance(median));
  Serial.print("ACTUAL DISTANCE: ");
  Serial.println(actual_distance);
  //End of IR section


  
  long newPosition = position;      //Boot up the robot with the IR centered for accurate range, sweeping mmotion setup
  if (setpoint < 90)
  {
    setpoint = setpoint + 10;
   // delay(15);
  }
  else
  {
  //  delay(100);
    setpoint = -90;
  }
  Serial.print("SETPOINT: ");
  Serial.println(setpoint);

  // Start of IR sensor section
  // If this section of code is included, and the serial monitor is opened, the DC sweep begins to behave erratically, placing your hand in front of the sensor causes expecially erratic behavior.
  if(actual_distance < 0.37) //this may have to be set to a different value, < 40 perhaps?
  {
    float targetSetpoint = setpoint;
    Serial.print(targetSetpoint);
    zombieMovement( actual_distance,  targetSetpoint);  
    
  }
  if (ourBot.zombie == false){
      state = HUMAN;
  }
}

void runBallState(){
  RobotPose ourBot = getRobotPose(MY_ROBOT_ID);       // create struct for our robot
  float closest = 3000;
  int closest_ID = 0;
  float distance;
  target = targetRobot();

  for(int i=0; i < NUM_BALLS; i++)
  {
      distance = sqrt(pow(ballPositions[i].x - ourBot.x, 2) + pow(ballPositions[i].y - ourBot.y, 2));
    if(distance < closest && ballPositions[i].hue != 0)
    {
        closest = distance;
        closest_ID = i;       
    }
  }

  float new_x = ballPositions[closest_ID].x;
  float new_y = ballPositions[closest_ID].y;

  if( target != 0){                           // check to see if there is a target zombie
    new_x = (getRobotPose(target).x);         // set target to zombie location, running will be handled in the movement function
    new_y = (getRobotPose(target).y);
  }
    
  movement(ourBot.x, ourBot.y, ourBot.theta * 0.001, new_x, new_y);
  
  //if pick up is successfull go to healer state
  if(ourBot.healing == true)
  {
    state = HEALER;
  }
  else if(numBalls == 0){
    state = HUMAN;
  }
}

void movement(float cur_x, float cur_y, float a, float new_x, float new_y){

  RobotPose ourBot = getRobotPose(MY_ROBOT_ID);       // create struct for our robot

    float R_x = ((cos(a)*new_x)+(sin(a)*new_y))-((cos(a)*cur_x)+(sin(a)*cur_y));         // convert to robot coordinates
    float R_y = ((-sin(a)*new_x)+(cos(a)*new_y))-((-sin(a)*cur_x)+(cos(a)*cur_y));
    float R_a_rad = atan2(R_y, R_x);
    
    if(state == HUMAN || state == BALL){            // check if human and need to run make target coords opposite
      if(target != 0){
      R_x = R_x * -1;
      R_y = R_y * -1;
      if(R_a_rad <= 0){
        R_a_rad = R_a_rad + 3.1415;
      }else {
        R_a_rad = R_a_rad - 3.1415;
      }
      }
    }
    
    float d = sqrt(pow(R_x, 2) + pow(R_y, 2));        // find distance from robot to location

    float forward_s = 100;          // forward speed set to constant 100 unless close
    if(d < 50){                     // to prevent hard collision with target
      forward_s = d;
    }
    float turn_s = R_a_rad * 30;    // turn speed will be 0-100
    
    leftServo.writeMicroseconds(1500 + (forward_s - turn_s));         // right servo has to be negative to go forward. add forward and turn speeds
    rightServo.writeMicroseconds(1500 + (-forward_s - turn_s));    
}

int targetRobot(){
  RobotPose ourBot = getRobotPose(MY_ROBOT_ID);       // create struct for our robot
  float x = ourBot.x;                                 // find our robot x,y, and angle (convert angle from milliradians to radians)
  float y = ourBot.y;
  float a = ourBot.theta * 0.001;
  float closest = 10000;
  if(state == HUMAN || state == BALL){
    closest = 400;
  }
  
  int tar_zomb = 0;
  for(int i = 0; i<= NUM_ROBOTS; i++){                              // search for closest zombie
    if(getRobotPose(i).valid == true && i != MY_ROBOT_ID){
      
    float rob_x = getRobotPose(i).x;                                // find robot(i) x and y coord
    float rob_y = getRobotPose(i).y;
    float d = sqrt(pow((rob_x - x),2) + pow((rob_y - y),2));        // calculate distance from our robot to robot(i)
    
    //CHANGE DISTANCE REQUIREMENT BASED ON SENSOR CALIBRATION

    if(getRobotPose(i).zombie == true && d < closest){              // check to see if robot(i) is a zombie and is within desired range. 
                                                                    // if requirements satisfied check to see if this is the closest zombie
        tar_zomb = i;                                               // if closest zombie, set distance to closest and the zombie ID to our target
        closest = d;  
    }
    }
  }
  return tar_zomb;
}

void runWallState(){
  //backwards
  leftServo.writeMicroseconds(1400);
  rightServo.writeMicroseconds(1600);
  delay(2000);
  
  //turn
  leftServo.writeMicroseconds(1500);
  delay(1000);
  
  //forwards
  leftServo.writeMicroseconds(1600);
  rightServo.writeMicroseconds(1400);

  //set state back to previous state
  state = tempstate;
}

//Zombie Movement Function based on the stored angle and distance seen by the IR sensor
void zombieMovement(float actual_distance, float targetSetpoint){
  Serial.println("ZOMBIE MOVEMENT FUNCTION");
 float angle = targetSetpoint ;
 Serial.print("ANGLE: ");
 Serial.println(angle);
    
  if (angle < 0 && actual_distance < 0.30) {                     //turn left if in scanned left quadrant 
    Serial.println("TURNING LEFT");
    rightServo.writeMicroseconds(1500);
    leftServo.writeMicroseconds(1600);
    delay(200);
    leftServo.writeMicroseconds(1600);                          //move straight a bit
    rightServo.writeMicroseconds(1400);
  }
  else if (angle > 0 && actual_distance < 0.30 ) {              //turn right 
    Serial.println("TURNING RIGHT");
    leftServo.writeMicroseconds(1500);
    rightServo.writeMicroseconds(1400); 
    delay(200);
    leftServo.writeMicroseconds(1600);                          //move straight a bit
    rightServo.writeMicroseconds(1400);
  }
  else {
    Serial.println("GOING STRAIGHT");
    leftServo.writeMicroseconds(1600);
    rightServo.writeMicroseconds(1400);
  }
}

float convertADC2Distance(float rawMeasure){                //IR sensor code to convert the raw measure to a distance
    Serial.println("CONVERTADC@DISTANCE WORKING");
    float actual_distance =  -0.0009*rawMeasure + 0.4697;   //Determined from IR distance calibration
    return actual_distance;
}
