/* Include the appropriate ROS headers */
#include <ros.h>
#include <ros/time.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Quaternion.h>
#include <ros_arduino_diff_drive/OdometryLite.h>
#include <std_msgs/UInt32.h>

/* We need sin and cos for odometry calcuations */
#include <math.h>

/* Load the custom typedefs for tracking encoder info */
#include "pid_params.h"

//PINES
#define ON_LED    13
#define BUSY_LED  A0
#define RED_LED   A1
#define SW2o      A2
#define IN1       9
#define IN2       8
#define IN3       7   // Input3 conectada al pin 5
#define IN4       6    // Input4 conectada al pin 4
#define ENB       10   // ENB conectada al pin 3 de Arduino
#define ENA       11

Encoder M1Enc(0, 1);
Encoder M2Enc(2, 3);
long oldPosition1  = 0;
long oldPosition2  = 0;
long newPosition1 = 0;
long newPosition2 = 0;
unsigned char moving = 0; // is the base in motion?

/* Maximum value for a PWM signal */
#define MAXOUTPUT       255

/* Rate at which encoders are sampled and PID loop is updated */
#define PID_RATE        30     // Hz
const float PID_INTERVAL = 1000.0 / PID_RATE;

/* Odometry publishing rate */
#define ODOM_RATE       10     // Hz
const float ODOM_INTERVAL = 1000.0 / ODOM_RATE;

/* The base and odometry frames */
char odomFrame[] = "/odom";

/* Counters to track update rates for PID and Odometry */
unsigned long nextPID = 0;
unsigned long nextOdom = 0;

/* SetPointInfo struct is defined in PIDTypes.h */
SetPointInfo leftPID, rightPID;

/* OdomInfo struct is defined in PIDTypes.h */
OdomInfo odomInfo;

/* Create the ROS node handle */
ros::NodeHandle nh;

/* A publisher for OdometryLite data on the /odometry_lite topic.
 * Thanks to Austin Hendrix for the code.
 */
ros_arduino_diff_drive::OdometryLite odom_msg;
ros::Publisher odomPub("odometry_lite", &odom_msg);

/* A debugging publisher since nh.loginfo() only takes character constants */
std_msgs::UInt32 log_msg;
ros::Publisher logPub("arduino_log", &log_msg);

/* Convert meters per second to ticks per time frame */
int SpeedToTicks(float v) {
  return int(v * cpr / (PID_RATE * PI * wheelDiameter));
}

/* The callback function to convert Twist messages into motor speeds */
void cmdVelCb(const geometry_msgs::Twist& msg) {
  float x = msg.linear.x; // m/s
  float th = msg.angular.z; // rad/s
  float spd_left, spd_right;

  /* Reset the auto stop timer */
  lastMotorCommand = millis();

  if (x == 0 && th == 0) {
    moving = 0;
    drive.setSpeeds(0, 0); //<-CODIGO A CAMBIAR
    return;
  }

  /* Indicate that we are moving */
  moving = 1;

  if (x == 0) {
    // Turn in place
    spd_right = th * wheelTrack / 2.0;
    spd_left = -spd_right;
  }
  else if (th == 0) {
    // Pure forward/backward motion
    spd_left = spd_right = x;
  }
  else {
    // Rotation about a point in space
    spd_left = x - th * wheelTrack / 2.0;
    spd_right = x + th * wheelTrack / 2.0;
  }

  /* Set the target speeds in meters per second */
  leftPID.TargetSpeed = spd_left;
  rightPID.TargetSpeed = spd_right;

  /* Convert speeds to encoder ticks per frame */
  leftPID.TargetTicksPerFrame = SpeedToTicks(leftPID.TargetSpeed);
  rightPID.TargetTicksPerFrame = SpeedToTicks(rightPID.TargetSpeed);
}

/* A subscriber for the /cmd_vel topic */
ros::Subscriber<geometry_msgs::Twist> cmdVelSub("/cmd_vel", &cmdVelCb);

/* PID routine to compute the next motor commands */
void doPID(SetPointInfo * p) {
  long Perror;
  long output;

  Perror = p->TargetTicksPerFrame - (p->Encoder - p->PrevEnc);

  // Derivative error is the delta Perror
  output = (Kp * Perror + Kd * (Perror - p->PrevErr) + Ki * p->Ierror) / Ko;
  p->PrevErr = Perror;
  p->PrevEnc = p->Encoder;

  output += p->output;

  if (output >= MAXOUTPUT)
    output = MAXOUTPUT;
  else if (output <= -MAXOUTPUT)
    output = -MAXOUTPUT;
  else
    p->Ierror += Perror;

  p->output = output;
}

/* Read the encoder values and call the PID routine */
void updatePID() {
  /* Read the encoders */
  leftPID.Encoder = encoders.YAxisGetCount();
  rightPID.Encoder = encoders.XAxisGetCount();

  /* Record the time that the readings were taken */
  odomInfo.encoderTime = millis();
  odomInfo.encoderStamp = nh.now();

  /* If we're not moving there is nothing more to do */
  if (!moving)
    return;

  /* Compute PID update for each motor */
  doPID(&leftPID);
  doPID(&rightPID);

  /* Set the motor speeds accordingly */
  drive.setSpeeds(leftPID.output, rightPID.output);
}

/* Calculate the odometry update and publish the result */
void updateOdom() {
  double dt, dleft, dright, dx, dy, dxy_ave, dth, vxy, vth;

  /* Get the time in seconds since the last encoder measurement */
  //dt = nh.now().toSec() - odomInfo.lastOdom.toSec();
  dt = (odomInfo.encoderTime - odomInfo.lastEncoderTime) / 1000.0;

  /* Save the encoder time for the next calculation */
  odomInfo.lastEncoderTime = odomInfo.encoderTime;

  /* Calculate the distance in meters traveled by the two wheels */
  dleft = (leftPID.Encoder - odomInfo.prevLeftEnc) / ticksPerMeter;
  dright = (rightPID.Encoder - odomInfo.prevRightEnc) / ticksPerMeter;

  odomInfo.prevLeftEnc = leftPID.Encoder;
  odomInfo.prevRightEnc = rightPID.Encoder;

  /* Compute the average linear distance over the two wheels */
  dxy_ave = (dleft + dright) / 2.0;

  /* Compute the angle rotated */
  dth = (dright - dleft) / wheelTrack;

  /* Linear velocity */
  vxy = dxy_ave / dt;

  /* Angular velocity */
  vth = dth / dt;

  /* How far did we move forward? */
  if (dxy_ave != 0) {
    dx = cos(dth) * dxy_ave;
    dy = -sin(dth) * dxy_ave;
    /* The total distance traveled so far */
    odomInfo.linearX += (cos(odomInfo.angularZ) * dx - sin(
    odomInfo.angularZ) * dy);
    odomInfo.linearY += (sin(odomInfo.angularZ) * dx + cos(
    odomInfo.angularZ) * dy);
  }

  /* The total angular rotated so far */
  if (dth != 0)
    odomInfo.angularZ += dth;

  /* Represent the rotation as a quaternion */
  geometry_msgs::Quaternion quaternion;
  quaternion.x = 0.0;
  quaternion.y = 0.0;
  quaternion.z = sin(odomInfo.angularZ / 2.0);
  quaternion.w = cos(odomInfo.angularZ / 2.0);

  /* Publish the distances and speeds on the odom topic. Set the timestamp
      to the last encoder time. */
  odom_msg.header.frame_id = odomFrame;
  odom_msg.child_frame_id = baseFrame;
  odom_msg.header.stamp = odomInfo.encoderStamp;
  odom_msg.pose.position.x = odomInfo.linearX;
  odom_msg.pose.position.y = odomInfo.linearY;
  odom_msg.pose.position.z = 0;
  odom_msg.pose.orientation = quaternion;
  odom_msg.twist.linear.x = vxy;
  odom_msg.twist.linear.y = 0;
  odom_msg.twist.linear.z = 0;
  odom_msg.twist.angular.x = 0;
  odom_msg.twist.angular.y = 0;
  odom_msg.twist.angular.z = vth;

  odomPub.publish(&odom_msg);
}

void clearPID() {
  moving = 0;
  leftPID.PrevErr = 0;
  leftPID.Ierror = 0;
  leftPID.output = 0;
  rightPID.PrevErr = 0;
  rightPID.Ierror = 0;
  rightPID.output = 0;
}

void clearAll() {
  clearPID();
  encoders.XAxisReset();
  encoders.YAxisReset();
}

void setup() {
  Serial.begin(57600);
  Serial.println("ROS PID Test");

  /****************************MOTORES********************************/
  pinMode (ENB, OUTPUT);
  pinMode (IN3, OUTPUT);
  pinMode (IN4, OUTPUT);
  pinMode (ENA, OUTPUT);
  pinMode (IN1, OUTPUT);
  pinMode (IN2, OUTPUT);
  digitalWrite (IN3, HIGH);
  digitalWrite (IN4, LOW);
  digitalWrite (IN1, HIGH);
  digitalWrite (IN2, LOW);
  analogWrite(ENB,0);
  analogWrite(ENA,0);
  /*****************************LEDS********************************/
  pinMode(ON_LED, OUTPUT);
  pinMode(BUSY_LED, OUTPUT);
  digitalWrite(ON_LED, HIGH);
  digitalWrite(BUSY_LED, HIGH);
  /******************************ROS********************************/

  /* Set the target speeds in meters per second */
  leftPID.TargetSpeed = 0.0;
  rightPID.TargetSpeed = 0.0;

  /* Convert speeds to encoder ticks per frame */
  leftPID.TargetTicksPerFrame = SpeedToTicks(leftPID.TargetSpeed);
  rightPID.TargetTicksPerFrame = SpeedToTicks(rightPID.TargetSpeed);

  /* Zero out the encoder counts */
  encoders.XAxisReset();
  encoders.YAxisReset();

  /* Initialize the ROS node */
  nh.initNode();

  nextPID = PID_INTERVAL;
  nextOdom = ODOM_INTERVAL;

  /* Advertise the Odometry publisher */
  nh.advertise(odomPub);
  //nh.advertise(logPub);

  /* Activate the Twist subscriber */
  nh.subscribe(cmdVelSub);
}

void loop() {
  /* Is it time for another PID calculation? */
  if (millis() > nextPID) {
    updatePID();
    nextPID += PID_INTERVAL;
  }

  /* Is it time for another odometry calculation? */
  if (millis() > nextOdom) {
    updateOdom();
    nextOdom += ODOM_INTERVAL;
  }

  if ((millis() - lastMotorCommand) > AUTO_STOP_INTERVAL) {;
    drive.setSpeeds(0, 0);
    moving = 0;
  }

  nh.spinOnce();
}
