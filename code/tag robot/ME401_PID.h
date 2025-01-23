#include <SoftPWMServo.h>
#include <PID_v1.h>

enum StateMachineState{     //create our states. Has to be in the PID file so that the timer interrupt can use it.
  HUMAN = 0,
  ZOMBIE = 1,
  HEALER = 2,
  BALL = 3,
  WALL = 4,
};

StateMachineState state;    //initialize statemachine structs
StateMachineState tempstate;
int bumperSwitch1 = 9;     //pins limit switches are connected to
int bumperSwitch2 = 8;

// Pin definitions. DO NOT CHANGE THESE ONES
const int EncoderAPin = 2;
const int EncoderBPin = 20;
const int MotorDirectionPin = 4;
const int MotorPWMPin = 3;

double ku = 14.0;
double tu = 0.07;
double kp = 0.8 * ku , ki = 0, kd = (kp * tu) / 8; //PD calculations

// Global variables for quadrature decoder
static volatile char lastLeftA;
static volatile char lastLeftB;
static volatile bool errorLeft;
volatile long position = 0;
volatile float conversion = 0.12857;
volatile int angle;

// Global variables for the timer interrupt handling
int pidSampleTime = 10;
long counterPID=1;

// Global variables for the PID controller
double input=0, output=0, setpoint=0;
PID myPID(&input, &output, &setpoint,kp,ki,kd, DIRECT);

// Forward declaration of functions to be used that are defined later than their first use
uint32_t TimerCallback(uint32_t currentTime);

void setupPIDandIR(void)
{
  // Set up the quadrature inputs
  pinMode(EncoderAPin, INPUT);
  pinMode(EncoderBPin, INPUT);

  errorLeft = false;
  lastLeftA = digitalRead(EncoderAPin);
  lastLeftB = digitalRead(EncoderBPin);

  // Set up the motor outputs
  pinMode(MotorPWMPin, OUTPUT);
  pinMode(MotorDirectionPin, OUTPUT);

  digitalWrite(MotorPWMPin,0);
  digitalWrite(MotorDirectionPin,0);
  SoftPWMServoPWMWrite(MotorPWMPin, 0);

  //Setup the pid 
  myPID.SetMode(AUTOMATIC);
  myPID.SetSampleTime(pidSampleTime);
  myPID.SetOutputLimits(-255,255);

  // Initialize the timer interrupt that decodes the IR beacon signal
  attachCoreTimerService(TimerCallback);
}

uint32_t TimerCallback(uint32_t currentTime) {
  char newLeftA = digitalRead(EncoderAPin);
  char newLeftB = digitalRead(EncoderBPin);

  position += (newLeftA ^ lastLeftB) - (lastLeftA ^ newLeftB); 

  if((lastLeftA ^ newLeftA) & (lastLeftB ^ newLeftB))
  {
    errorLeft = true;
  }

  lastLeftA = newLeftA;
  lastLeftB = newLeftB;

  if (counterPID % 100*pidSampleTime == 0)
  {
    angle = position*0.12857;
    input = angle;      
    myPID.Compute();

    if (output > 0)
    {
      digitalWrite(4,1);
    }
    else
    {
      digitalWrite(4,0);
    }  
    SoftPWMServoPWMWrite(3,abs(output));
    counterPID = 0;

    //if the switch hits the wall
    if(digitalRead(bumperSwitch1) == LOW || digitalRead(bumperSwitch2) == LOW){
    //debugging check
    Serial.println("INTERRUPT IS WORKING");

    //set state to wall
    state = WALL;
  }
  }
  counterPID++;
  
  return (currentTime + CORE_TICK_RATE/100);
}
