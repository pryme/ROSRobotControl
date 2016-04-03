//2016-3-12
// ROSRomeoNode - Implements a ROS node for the RomeoBLE for controlling
//     a ROSRev2-class robot.
//

// Needed on Leonardo to force use of USB serial.
// #define USE_USBCON
// commented to use hw serial between A* and RPi.

#include <AStar32U4.h>
#include <EnableInterrupt.h>
#include <SimplePID.h>
// using this to avoid delay() which is blocking
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

std_msgs::Int16 controlDelayMsg;  // TODO diagnostic
ros::Publisher cDelayPub("cDelay", &controlDelayMsg);

AStar32U4Motors motors;

// Ziegler-Nichols tuning. See this Wikipedia article for details:
//     https://en.wikipedia.org/wiki/PID_controller#Loop_tuning

//const float Ku = .15;  // was 0.035
//const float Tu = .1142857143;  // was 0.235

//const float Kp = 0.6*Ku;  // pr had 0.048 before
///const float Ki = 2*Kp/Tu;  // pr had 0.279 before
//const float Kd = Kp*Tu/8;  // pr had 0.00323 before
//const float Kp = 0.048;  // TODO
//const float Ki = 0.279;  // TODO
//const float Kd = 0.00323;  // TODO
const float Kp = 0.15;  // from pr re-tuning Z-N based on Ku = 0.25
const float Ki = 2.625;
const float Kd = 0.002143;

SimplePID leftController = SimplePID(Kp, Ki, Kd);
SimplePID rightController = SimplePID(Kp, Ki, Kd);

volatile long lwheel = 0;
volatile long rwheel = 0;

long lastLwheel = 0;
long lastRwheel = 0;

// Target motors speeds in ticks per second.
int lwheelTargetRate = 0;
int rwheelTargetRate = 0;

// Current motor setpoints in ticks / sec; allows setpoint ramping
int lwheelCurrentSetpoint = 0;
int rwheelCurrentSetpoint = 0;
float maxSetpointSpeedStep = 0.02;  // abs(max step in setpoint); m/sec

// The interval between motor control steps.
// int controlDelayMillis;
elapsedMillis waitMillis = 0;  
unsigned int controlDelayMillis;  // elapsedMillis is non-blocking

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

// Maximum motor control value
const int MAX_MOTOR_CMD = 400;

void setup() {
  pinMode(ONBOARD_LED_PIN, OUTPUT);

  // Uncomment to flip a motor's direction:
  motors.flipM1(true);
  //motors.flipM2(true);
  
  enableInterrupt(M1_A, leftAChange, CHANGE);
  // not using 'B' encoder edges
  // enableInterrupt(M1_B, leftBChange, CHANGE);
  enableInterrupt(M2_A, rightAChange, CHANGE);
  //enableInterrupt(M2_B, rightBChange, CHANGE);

  nh.initNode();

  // TODO experiment to change serial baud rate
  // reference: http://answers.ros.org/question/11022/rosserial_arduino-trouble-with-too-much-messaging/
  // nh.getHardware()->setBaud(115200);
  
  nh.advertise(lwheelPub);
  nh.advertise(rwheelPub);
  nh.advertise(lwheelVelocityPub);
  nh.advertise(rwheelVelocityPub);
  nh.advertise(cDelayPub);

  nh.subscribe(lwheelTargetSub);
  nh.subscribe(rwheelTargetSub);

  // Wait until the node has initialized before getting parameters.
  while(!nh.connected()) {
    nh.spinOnce();
  }

  int controlRate;
  if (!nh.getParam("~control_rate", &controlRate)) {
    controlRate = 60;  
  }
  controlDelayMillis = 1000.0 / controlRate;
  
  if (!nh.getParam("ticks_meter", &ticksPerMeter)) {
    ticksPerMeter = 5113;  // was 10226
  }
  
  float vtargetTimeout;
  if (!nh.getParam("~vtarget_timeout", &vtargetTimeout)) {
    vtargetTimeout = 0.250;
  }
  vtargetTimeoutMillis = vtargetTimeout * 1000;

  lastLoopTime = micros();
  lastMotorCmdTime = lastLoopTime;
  
  controlDelayMsg.data = (int) controlDelayMillis;
}

// Every loop, publish the encoder and wheel rates.
void loop()
{
  cDelayPub.publish(&controlDelayMsg);  // TODO diagnostic
  //delay(controlDelayMillis);
  // elapsedMillis is non-blocking
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
  leftMotorCmd += min(MAX_MOTOR_CMD, leftControl);
  leftMotorCmd = constrain(leftMotorCmd, -MAX_MOTOR_CMD, MAX_MOTOR_CMD);
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
  rightMotorCmd += min(MAX_MOTOR_CMD, rightControl);
  rightMotorCmd = constrain(rightMotorCmd, -MAX_MOTOR_CMD, MAX_MOTOR_CMD);
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
    lwheelCurrentSetpoint = 0;
    rwheelTargetRate = 0;
    rwheelCurrentSetpoint = 0;
    const int stopDeadband = 20;  // if MotorCmd < this make it zero
    // setSpeed(0 , 0); // TODO temp for test
    
    while((leftMotorCmd != 0) || (rightMotorCmd != 0)) {
        // ramp down motors to limit deceleration
        if (abs(leftMotorCmd) < stopDeadband) {
            leftMotorCmd = 0;
        }
        if (leftMotorCmd < 0) {
            leftMotorCmd += 20;
            if (leftMotorCmd >= 0) {
                leftMotorCmd = 0;  // don't pass zero
            }
        } else {
            leftMotorCmd -= 20;
            if (leftMotorCmd <= 0) {
                leftMotorCmd = 0;  // don't pass zero
            }
        }
        if (abs(rightMotorCmd) < stopDeadband) {
            rightMotorCmd = 0;
        }
        if (rightMotorCmd < 0) {
            rightMotorCmd += 20;
            if (rightMotorCmd >= 0) {
                rightMotorCmd = 0;  // don't pass zero
            }
        } else {
            rightMotorCmd -= 20;
            if (rightMotorCmd <= 0) {
                rightMotorCmd = 0;  // don't pass zero
            }
        }    
        setSpeed(leftMotorCmd, rightMotorCmd);
        delayMicroseconds(8);  // control loop speed;
    }  // end while
  }

  lastLwheel = curLwheel;
  lastRwheel = curRwheel;
  
  lastLoopTime = curLoopTime;
  
  nh.spinOnce();
}

void lwheelTargetCallback(const std_msgs::Float32& cmdMsg) {
  lastMotorCmdTime = millis();
  lwheelTargetRate = cmdMsg.data * ticksPerMeter;
  // ramp the setpoint to control acceleration
  // TODO: do ramp here
  int spDelta = lwheelCurrentSetpoint - lwheelTargetRate;
  if (spDelta == 0) {
      return;  // already at the target setpoint
  }
  if (spDelta < 0) {
      lwheelCurrentSetpoint += (int)(maxSetpointSpeedStep * ticksPerMeter);
      if (lwheelCurrentSetpoint >= lwheelTargetRate) {
          lwheelCurrentSetpoint = lwheelTargetRate;  // constrain max
      }
  } else {
      lwheelCurrentSetpoint -= (int)(maxSetpointSpeedStep * ticksPerMeter);
      if (lwheelCurrentSetpoint <= lwheelTargetRate) {
          lwheelCurrentSetpoint = lwheelTargetRate;  // constrain min
      }
  }
  // leftController.setSetPoint(lwheelTargetRate);
  leftController.setSetPoint(lwheelCurrentSetpoint);
}

void rwheelTargetCallback(const std_msgs::Float32& cmdMsg) {
  lastMotorCmdTime = millis();
  rwheelTargetRate = cmdMsg.data * ticksPerMeter;
  // ramp the setpoint to control acceleration
  int spDelta = rwheelCurrentSetpoint - rwheelTargetRate;
  if (spDelta == 0) {
      return;  // already at the target setpoint
  }
  if (spDelta < 0) {
      rwheelCurrentSetpoint += (int)(maxSetpointSpeedStep * ticksPerMeter);
      if (rwheelCurrentSetpoint >= rwheelTargetRate) {
          rwheelCurrentSetpoint = rwheelTargetRate;  // constrain max
      }
  } else {
      rwheelCurrentSetpoint -= (int)(maxSetpointSpeedStep * ticksPerMeter);
      if (rwheelCurrentSetpoint <= rwheelTargetRate) {
          rwheelCurrentSetpoint = rwheelTargetRate;  // constrain min
      }
  }
  //rightController.setSetPoint(rwheelTargetRate);
  rightController.setSetPoint(rwheelCurrentSetpoint);
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

