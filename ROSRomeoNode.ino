//
// ROSRomeoNode - Implements a ROS node for the RomeoBLE for controlling
//     a ROSRev2-class robot.
//

// Needed on Leonardo to force use of USB serial.
#define USE_USBCON

#include <SimplePID.h>

#include <ros.h>
#include <ros/time.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>
//#include <sensor_msgs/LaserScan.h>

// added this to use Pololu library
#include <AStar32U4.h>

AStar32U4Motors motors;

// const int ONBOARD_LED_PIN = 13;

// The pins for motor control on the Romeo BLE.
//const int M1_DIRECTION = 4;
//const int M1_SPEED = 5;
//const int M2_SPEED = 6;
//const int M2_DIRECTION = 7;

// Pins for the Pololu motor encoder outputs.
//const int M1_A = 2;
//const int M1_B = 8;
//const int M2_A = 3;
//const int M2_B = 9;

// Pin definitions for A-star (PR version)
const int ONBOARD_LED_PIN = 13;

//const int M1_DIRECTION = 12;
//const int M1_SPEED = 9;
//const int M2_SPEED = 10;
//const int M2_DIRECTION = PE2;
//const int M2_DIRECTION = HWB;
//const int M2_DIRECTION = IO_E2;
//
// workaround for broken PE2 connection on my first A-star
// this pin jumpered to PE2 trace
// const int M2_DIRECTION = 23;  

// Pins for the Pololu motor encoder outputs.
const int M1_A = 0;
const int M1_B = 4;
const int M2_A = 1;
const int M2_B = 8;

// Pins for the Sharp IR rangers
const int IR_neg180 = A4;
const int IR_neg45 = A5;
const int IR_0 = A2;
const int IR_pos45 = A3;

ros::NodeHandle  nh;

std_msgs::Int16 lwheelMsg;
ros::Publisher lwheelPub("lwheel", &lwheelMsg);

std_msgs::Int16 rwheelMsg;
ros::Publisher rwheelPub("rwheel", &rwheelMsg);

std_msgs::Float32 lwheelVelocityMsg;
ros::Publisher lwheelVelocityPub("lwheel_velocity", &lwheelVelocityMsg);

std_msgs::Float32 rwheelVelocityMsg;
ros::Publisher rwheelVelocityPub("rwheel_velocity", &rwheelVelocityMsg);

//sensor_msgs::LaserScan scan;
//ros::Publisher scan_pub("scan", &scan);

std_msgs::Int16 IR_neg180_msg;
ros::Publisher IR_neg180_pub("IR_neg180", &IR_neg180_msg);

std_msgs::Int16 IR_neg45_msg;
ros::Publisher IR_neg45_pub("IR_neg45", &IR_neg45_msg);

std_msgs::Int16 IR_0_msg;
ros::Publisher IR_0_pub("IR_0", &IR_0_msg);

std_msgs::Int16 IR_pos45_msg;
ros::Publisher IR_pos45_pub("IR_pos45", &IR_pos45_msg);

void lwheelTargetCallback(const std_msgs::Float32& cmdMsg);
ros::Subscriber<std_msgs::Float32> lwheelTargetSub("lwheel_vtarget", &lwheelTargetCallback);

void rwheelTargetCallback(const std_msgs::Float32& cmdMsg);
ros::Subscriber<std_msgs::Float32> rwheelTargetSub("rwheel_vtarget", &rwheelTargetCallback);

// add diagnostic message to find out what ticksPerMeter is being used
// I suspect the parameter for this is not being grabbed from the launch file
//std_msgs::Int16 ticksPerMeterMsg;
//ros::Publisher ticksPerMeterPub("ticksPerMeter", &ticksPerMeterMsg);

// Ziegler-Nichols tuning. See this Wikipedia article for details:
//     https://en.wikipedia.org/wiki/PID_controller#Loop_tuning
// Ku and Tu were determined by setting Ki and Kd to zero, then increasing
// Kp until steady oscillation occurs. Tu is the oscillation wavelength.
//
// actually formulas at that link are:
//  PID:  Kp= 0.6*Ku; Ki= 2*Kp/Tu; Kd= Kp*Tu/8;  
//  PI:  Kp= 0.45*Ku; Ki= 1.2*Kp/Tu; 
//  and these were correct in RomeoPIDTest.ino
//
//const float Ku = .17;
//const float Tu = .14;
//const float Kp = 0.45*Ku;
//const float Ki = 1.2*Kp/Ku;
//const float Kp = 0.6*Ku;
//const float Ki = 2*Kp/Tu;
//const float Kd = Kp*Tu/8;
//
// try M Rose new PID params
const float Kp = 0.048;
const float Ki = 0.279;
const float Kd = 0.00323;

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
int controlDelayMillis;

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
const int MIN_MOTOR_CMD = 0;  // was 60

// IR range scan
const int num_readings = 8;
double laser_frequency = 50;
double ranges[num_readings];
double intensities[num_readings];

void setup()
{
  pinMode(ONBOARD_LED_PIN, OUTPUT);
  //pinMode(M1_DIRECTION, OUTPUT);
  //pinMode(M2_DIRECTION, OUTPUT);
  
  //pinMode(M1_SPEED, OUTPUT);  // perhaps not needed?
  //pinMode(M2_SPEED, OUTPUT);
  pinMode(M1_A, INPUT);
  pinMode(M1_B, INPUT);
  pinMode(M2_A, INPUT);
  pinMode(M2_B, INPUT);
  
  pinMode(IR_neg180, INPUT);
  pinMode(IR_neg45, INPUT);
  pinMode(IR_0, INPUT);
  pinMode(IR_pos45, INPUT);

  // Uncomment to flip a motor's direction:
  motors.flipM1(true);
  //motors.flipM2(true);
  
  attachInterrupt(digitalPinToInterrupt(M1_A), leftAChange, CHANGE);
  attachInterrupt(digitalPinToInterrupt(M2_A), rightAChange, CHANGE);

  nh.initNode();

  nh.advertise(lwheelPub);
  nh.advertise(rwheelPub);
  nh.advertise(lwheelVelocityPub);
  nh.advertise(rwheelVelocityPub);
  //nh.advertise(scan_pub);
  nh.advertise(IR_neg180_pub);
  nh.advertise(IR_neg45_pub);
  nh.advertise(IR_0_pub);
  nh.advertise(IR_pos45_pub);
  //
  //nh.advertise(ticksPerMeterPub);  // what is this doing?  TODO:

  nh.subscribe(lwheelTargetSub);
  nh.subscribe(rwheelTargetSub);

  // Wait until the node has initialized before getting parameters.
  while(!nh.connected()) {
    nh.spinOnce();
  }

  int controlRate;
  if (!nh.getParam("~control_rate", &controlRate)) {
    controlRate = 50;
  }
  controlDelayMillis = 1000.0 / controlRate;
  
  if (!nh.getParam("ticks_meter", &ticksPerMeter)) {
    ticksPerMeter = 5113;  // was 50 but wasn't getting param from launch
  }
  
    
  float vtargetTimeout;
  if (!nh.getParam("~vtarget_timeout", &vtargetTimeout)) {
    vtargetTimeout = 0.2;
  }
  vtargetTimeoutMillis = vtargetTimeout * 1000;

  lastLoopTime = micros();
  lastMotorCmdTime = lastLoopTime;
}

// Every loop, publish the encoder and wheel rates.
void loop()
{
  delay(controlDelayMillis);

  long curLoopTime = micros();

  noInterrupts();
  long curLwheel = lwheel;
  long curRwheel = rwheel;
  interrupts();

  // diagnostic
  //ticksPerMeterMsg.data = ticksPerMeter;
  //ticksPerMeterPub.publish(&ticksPerMeterMsg);  // why is this here?  TODO:
  
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
  
  do_IR_scan();
  
  lastLoopTime = curLoopTime;
  
  nh.spinOnce();
}

void do_IR_scan(){
    /*
    //populate the LaserScan message
    
    //ros::Time scan_time = ros::Time::now();  // doesn't work; use nh.now()
    //scan.header.stamp = scan_time;
    scan.header.stamp = nh.now();
    scan.header.frame_id = "base_link";
    scan.angle_min = -3.14;
    scan.angle_max = 3.14;
    scan.angle_increment = 6.28 / num_readings;
    scan.time_increment = (1 / laser_frequency) / (num_readings);
    scan.range_min = 0.0;
    scan.range_max = 2.0;
    // TODO: ADC to distance conversion below is wrong, just quick estimate
    scan.ranges[0] = 72.0 / max(10, analogRead(IR_neg180));
    scan.ranges[3] = 72.0 / max(10, analogRead(IR_neg45));
    scan.ranges[4] = 72.0 / max(10, analogRead(IR_0));
    scan.ranges[5] = 72.0 / max(10, analogRead(IR_pos45));
    scan.ranges[1] = 0.0;
    scan.ranges[2] = 0.0;
    scan.ranges[6] = 0.0;
    scan.ranges[7] = 0.0;
    // TODO: below is dummy - should elminate at least from loop
    // maybe compiler optimizes :)
    for(int i = 0; i < num_readings; ++i){
        scan.intensities[i] = 0.0;
    }
    scan_pub.publish(&scan);
    */
    IR_neg180_msg.data = analogRead(IR_neg180);
    IR_neg180_pub.publish(&IR_neg180_msg);
    
    IR_neg45_msg.data = analogRead(IR_neg45);
    IR_neg45_pub.publish(&IR_neg45_msg);
    
    IR_0_msg.data = analogRead(IR_0);
    IR_0_pub.publish(&IR_0_msg);
    
    IR_pos45_msg.data = analogRead(IR_pos45);
    IR_pos45_pub.publish(&IR_pos45_msg);
    
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
  if (digitalRead(M1_A) == digitalRead(M1_B)) {
    ++lwheel;
  } else {
    --lwheel;
  }
}

void rightAChange() {
  if (digitalRead(M2_A) != digitalRead(M2_B)) {
    ++rwheel;
  } else {
    --rwheel;
  }
}

// Sets the left and right motor speeds.
void setSpeed(int leftSpeed, int rightSpeed){
    // wrap the Pololu function
    motors.setSpeeds(leftSpeed, rightSpeed);
}
/*  replaced by Pololu library
void setSpeed(int leftSpeed, int rightSpeed) {
  digitalWrite(M1_DIRECTION, (leftSpeed >= 0 ? HIGH : LOW));
  analogWrite(M1_SPEED, abs(leftSpeed));
  //digitalWrite(M2_DIRECTION, (rightSpeed >= 0 ? HIGH : LOW));
  // reversed sense because L, R motors wired identically
  digitalWrite(M2_DIRECTION, (rightSpeed >= 0 ? LOW : HIGH));
  analogWrite(M2_SPEED, abs(rightSpeed));
}
*/
