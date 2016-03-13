//2016-3-12
// ROSRomeoNode - Implements a ROS node for the RomeoBLE for controlling
//     a ROSRev2-class robot.
//

// Needed on Leonardo to force use of USB serial.
#define USE_USBCON
// commented to use hw serial between A* and RPi.

#include <AStar32U4.h>
#include <EnableInterrupt.h>
#include <SimplePID.h>
// TODO trying this to avoid delay()
#include <elapsedMillis.h>

#include <ros.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>

const int ONBOARD_LED_PIN = 13;

// Pins for the A-Star motor encoder outputs.
const int M1_A = 8;  // changed to PCINT pin, leaving INT6 free
const int M1_B = 11;
const int M2_A = 15;
const int M2_B = 16;

ros::NodeHandle  nh;

std_msgs::Int16 lwheelMsg;
ros::Publisher lwheelPub("lwheel", &lwheelMsg);

std_msgs::Int16 rwheelMsg;
ros::Publisher rwheelPub("rwheel", &rwheelMsg);

std_msgs::Float32 lwheelVelocityMsg;
ros::Publisher lwheelVelocityPub("lwheel_velocity", &lwheelVelocityMsg);

std_msgs::Float32 rwheelVelocityMsg;
ros::Publisher rwheelVelocityPub("rwheel_velocity", &rwheelVelocityMsg);

void lwheelTargetCallback(const std_msgs::Float32& cmdMsg);
ros::Subscriber<std_msgs::Float32> lwheelTargetSub("lwheel_vtarget", &lwheelTargetCallback);

void rwheelTargetCallback(const std_msgs::Float32& cmdMsg);
ros::Subscriber<std_msgs::Float32> rwheelTargetSub("rwheel_vtarget", &rwheelTargetCallback);

AStar32U4Motors motors;

// Ziegler-Nichols tuning. See this Wikipedia article for details:
//     https://en.wikipedia.org/wiki/PID_controller#Loop_tuning

const float Ku = .035;  
const float Tu = .235;

const float Kp = 0.6*Ku;  // pr had 0.048 before
const float Ki = 2*Kp/Tu;  // pr had 0.279 before
const float Kd = Kp*Tu/8;  // pr had 0.00323 before

SimplePID leftController = SimplePID(Kp, Ki, Kd);
SimplePID rightController = SimplePID(Kp, Ki, Kd);

volatile long lwheel = 0;
volatile long rwheel = 0;

long lastLwheel = 0;
long lastRwheel = 0;

// Target motors speeds in ticks per second.
int lwheelTargetRate = 0;
int rwheelTargetRate = 0;

// The interval between motor control steps.
// int controlDelayMillis;
elapsedMillis waitMillis = 0;  // TODO trying with elapsedMillis to avoid delay()
unsigned int controlDelayMillis;  // trying with elapsedMillis TODO 

// The number of encoder ticks per meter.
int ticksPerMeter;

// The number of milliseconds without a velocity target when the robot
// will automatically turn off the motors.
int vtargetTimeoutMillis;

unsigned long lastLoopTime;
unsigned long lastMotorCmdTime;

int leftMotorCmd = 0;
int rightMotorCmd = 0;

// Minimum motor control value. Motor output below this will stall.
const int MIN_MOTOR_CMD = 0;  // seems fine below 60

void setup() {
  pinMode(ONBOARD_LED_PIN, OUTPUT);

  // Uncomment to flip a motor's direction:
  motors.flipM1(true);
  //motors.flipM2(true);
  
  enableInterrupt(M1_A, leftAChange, CHANGE);
  // TODO commented out 'B' encoder edges for diagnostics
  // enableInterrupt(M1_B, leftBChange, CHANGE);
  enableInterrupt(M2_A, rightAChange, CHANGE);
  //enableInterrupt(M2_B, rightBChange, CHANGE);

  nh.initNode();

  // TODO experimental to change serial baud rate
  // reference: http://answers.ros.org/question/11022/rosserial_arduino-trouble-with-too-much-messaging/
  nh.getHardware()->setBaud(115200);
  
  nh.advertise(lwheelPub);
  nh.advertise(rwheelPub);
  nh.advertise(lwheelVelocityPub);
  nh.advertise(rwheelVelocityPub);

  nh.subscribe(lwheelTargetSub);
  nh.subscribe(rwheelTargetSub);

  // Wait until the node has initialized before getting parameters.
  while(!nh.connected()) {
    nh.spinOnce();
  }

  int controlRate;
  if (!nh.getParam("~control_rate", &controlRate)) {
    controlRate = 60;  // TODO default = 10 only as diagnostic;  was 60
  }
  controlDelayMillis = 1000.0 / controlRate;
  
  if (!nh.getParam("ticks_meter", &ticksPerMeter)) {
    ticksPerMeter = 5113;  // TODO was 10226
  }
  
  float vtargetTimeout;
  if (!nh.getParam("~vtarget_timeout", &vtargetTimeout)) {
    vtargetTimeout = 0.250;
  }
  vtargetTimeoutMillis = vtargetTimeout * 1000;

  lastLoopTime = micros();
  lastMotorCmdTime = lastLoopTime;
}

// Every loop, publish the encoder and wheel rates.
void loop()
{
  //delay(controlDelayMillis);
  // TODO trying with elapsedMillis
  while (waitMillis < controlDelayMillis){
      ;  // do nothing
  }
  waitMillis = 0;  // reset
  
  long curLoopTime = micros();

  noInterrupts();
  long curLwheel = lwheel;
  long curRwheel = rwheel;
  interrupts();

  lwheelMsg.data = (int) curLwheel;
  rwheelMsg.data = (int) curRwheel;
  lwheelPub.publish(&lwheelMsg);
  rwheelPub.publish(&rwheelMsg);

  float dt = (curLoopTime - lastLoopTime) / 1E6;

  float lwheelRate = ((curLwheel - lastLwheel) / dt);
  float rwheelRate = ((curRwheel - lastRwheel) / dt);

  lwheelVelocityMsg.data = lwheelRate / ticksPerMeter;
  rwheelVelocityMsg.data = rwheelRate / ticksPerMeter;
  lwheelVelocityPub.publish(&lwheelVelocityMsg);
  rwheelVelocityPub.publish(&rwheelVelocityMsg);
  
  int leftControl = leftController.getControlValue(lwheelRate, dt);
  leftMotorCmd += min(255, leftControl);
  leftMotorCmd = constrain(leftMotorCmd, -255, 255);
  /* if (leftMotorCmd > 0) {
    leftMotorCmd = max(leftMotorCmd, MIN_MOTOR_CMD);
  } */
  if (abs(leftMotorCmd) < MIN_MOTOR_CMD){  // avoid deadband
      if (leftMotorCmd < 0){
          leftMotorCmd = -MIN_MOTOR_CMD;
      } else {
          leftMotorCmd = MIN_MOTOR_CMD;
      }
  }
  
  
  int rightControl = rightController.getControlValue(rwheelRate, dt);
  rightMotorCmd += min(255, rightControl);
  rightMotorCmd = constrain(rightMotorCmd, -255, 255);
  /* if (rightMotorCmd > 0) {
    rightMotorCmd = max(rightMotorCmd, MIN_MOTOR_CMD);
  }
  */
  if (abs(rightMotorCmd) < MIN_MOTOR_CMD){  // avoid deadband
       if (rightMotorCmd < 0){
           rightMotorCmd = -MIN_MOTOR_CMD;
       } else {
           rightMotorCmd = MIN_MOTOR_CMD;
       }
  }
 
  // Coast to a stop if target is zero.
  if (lwheelTargetRate == 0) {
    leftMotorCmd = 0;
  }
  if (rwheelTargetRate == 0) {
    rightMotorCmd = 0;
  }
  
  setSpeed(leftMotorCmd, rightMotorCmd);
  
  // Turn off motors if too much time has elapsed since last motor command.
  if (millis() - lastMotorCmdTime > vtargetTimeoutMillis) {
    lwheelTargetRate = 0;
    rwheelTargetRate = 0;
    setSpeed(0, 0);
  }

  lastLwheel = curLwheel;
  lastRwheel = curRwheel;
  
  lastLoopTime = curLoopTime;
  
  nh.spinOnce();
}

void lwheelTargetCallback(const std_msgs::Float32& cmdMsg) {
  lastMotorCmdTime = millis();
  lwheelTargetRate = cmdMsg.data * ticksPerMeter;
  leftController.setSetPoint(lwheelTargetRate);
}

void rwheelTargetCallback(const std_msgs::Float32& cmdMsg) {
  lastMotorCmdTime = millis();
  rwheelTargetRate = cmdMsg.data * ticksPerMeter;
  rightController.setSetPoint(rwheelTargetRate);
}

void leftAChange() {
  // TODO testing FastGPIO; clean up pin refs?; extend to B's?
  //if (digitalRead(M1_A) == digitalRead(M1_B)) {
    if (FastGPIO::Pin<8>::isInputHigh() == FastGPIO::Pin<11>::isInputHigh()){
    ++lwheel;
  } else {
    --lwheel;
  }
}

void leftBChange() {
  if (digitalRead(M1_A) != digitalRead(M1_B)) {
    ++lwheel;
  } else {
    --lwheel;
  }
}

void rightAChange() {
  //if (digitalRead(M2_A) != digitalRead(M2_B)) {
    if (FastGPIO::Pin<15>::isInputHigh() != FastGPIO::Pin<16>::isInputHigh()){
    ++rwheel;
  } else {
    --rwheel;
  }
}

void rightBChange() {
  if (digitalRead(M2_A) == digitalRead(M2_B)) {
    ++rwheel;
  } else {
    --rwheel;
  }
}

// Sets the left and right motor speeds.
void setSpeed(int leftSpeed, int rightSpeed) {
  motors.setSpeeds(leftSpeed, rightSpeed);
}

