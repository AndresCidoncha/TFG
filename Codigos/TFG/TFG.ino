/***********************************************************************************
*
* SketchAmigoBot.ino
*
***********************************************************************************
*
* AUTORES
* Andrés Cidoncha Carballo
*
* FECHA
* 25/06/2016
*
* DESCRIPCION
* Sketch principal para el Arduino MEGA. Encargado de publicar y suscribirse a los topics
* Basado en el proyecto ros_arduino_diff_drive https://github.com/SimonBirrell/ros_arduino_diff_drive
************************************************************************************/

/*********************************LIBRARIES************************************/
/* Include the appropriate ROS headers */
#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Quaternion.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <math.h> /* We need sin and cos for odometry calcuations */
#include "pid_params.h" /* Load the custom typedefs for tracking encoder info */
#include "AmigoBot.h" /* AmigoBot Robot Class */
/******************************************************************************/

/*********************************CONSTANTS************************************/
#define MAXOUTPUT               255    // PWM
#define PID_UPDATE_RATE         30     // Hz
#define ODOM_PUBLISH_RATE       10     // Hz
/******************************************************************************/

void cmdVelCb(const geometry_msgs::Twist& msg);

/**********************************OBJECTS*************************************/
/* Structs defined in PIDTypes.h */
SetPointInfo leftPID, rightPID;
OdomInfo odomInfo;

ros::NodeHandle nh; /* Create the ROS node handle */

/* Odometry publisher */
nav_msgs::Odometry odom_msg;
ros::Publisher odomPub("/odom", &odom_msg);

/* A subscriber for the /cmd_vel topic */
ros::Subscriber<geometry_msgs::Twist> cmdVelSub("/cmd_vel", &cmdVelCb);

/* The Robot Object */
AmigoBot* robot;
/******************************************************************************/

/*********************************VARIABLES************************************/
const float PID_INTERVAL = 1000.0 / PID_UPDATE_RATE;
const float ODOM_INTERVAL = 1000.0 / ODOM_PUBLISH_RATE;
char baseFrame[] = "/base_link";
char odomFrame[] = "/odom";
unsigned long nextPID = 0;
unsigned long nextOdom = 0;
unsigned char moving = 0; // is the base in motion?
/******************************************************************************/

/**********************************METHODS*************************************/
/* Convert meters per second to ticks per time frame */
int SpeedToTicks(float v) {
      return int(v * cpr / (PID_UPDATE_RATE * PI * wheelDiameter));
}

/* The callback function to convert Twist messages into motor speeds */
void cmdVelCb(const geometry_msgs::Twist& msg) {
    float x = msg.linear.x; // m/s
    float th = msg.angular.z; // rad/s
    float spd_left, spd_right;

    robot->busy();

    lastMotorCommand = millis(); /* Reset the auto stop timer */

    if (x == 0 && th == 0) {
        moving = 0;
        robot->stopMotors();
        return;
    }

    moving = 1; /* Indicate that we are moving */

    if (x == 0) { // Turn in place
        spd_right = th * wheelTrack / 2.0;
        spd_left = -spd_right;
    }
    else if (th == 0) { // Pure forward/backward motion
        spd_left = spd_right = x;
    }
    else { // Rotation about a point in space
        spd_left = x - th * wheelTrack / 2.0;
        spd_right = x + th * wheelTrack / 2.0;
    }

    /* Set the target speeds in meters per second */
    leftPID.TargetSpeed = spd_left;
    rightPID.TargetSpeed = spd_right;

    /* Convert speeds to encoder ticks per frame */
    leftPID.TargetTicksPerFrame = SpeedToTicks(leftPID.TargetSpeed);
    rightPID.TargetTicksPerFrame = SpeedToTicks(rightPID.TargetSpeed);

    robot->ready();
}

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
  leftPID.Encoder = robot->leftMotor->read();
  rightPID.Encoder = robot->rightMotor->read();

  /* Record the time that the readings were taken */
  odomInfo.encoderTime = millis();
  odomInfo.encoderStamp = nh.now();

  if (!moving) /* If we're not moving there is nothing more to do */
    return;

  /* Compute PID update for each motor */
  doPID(&leftPID);
  doPID(&rightPID);

  robot->setSpeeds(leftPID.output, rightPID.output); /* Set the motor speeds */
}

/* Calculate the odometry update and publish the result */
void updateOdom() {
  double dt, dleft, dright, dx, dy, dxy_ave, dth, vxy, vth;
  static tf::TransformBroadcaster odom_tf;

  /* Get the time in seconds since the last encoder measurement */
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
  odom_msg.pose.pose.position.x = odomInfo.linearX;
  odom_msg.pose.pose.position.y = odomInfo.linearY;
  odom_msg.pose.pose.position.z = 0;
  odom_msg.pose.pose.orientation = quaternion;
  odom_msg.twist.twist.linear.x = vxy;
  odom_msg.twist.twist.linear.y = 0;
  odom_msg.twist.twist.linear.z = 0;
  odom_msg.twist.twist.angular.x = 0;
  odom_msg.twist.twist.angular.y = 0;
  odom_msg.twist.twist.angular.z = vth;
  odomPub.publish(&odom_msg);

  geometry_msgs::TransformStamped transform;
  transform.header = odom_msg.header;
  transform.child_frame_id = odom_msg.child_frame_id;
  transform.transform.translation.x = odom_msg.pose.pose.position.x;
  transform.transform.translation.y = odom_msg.pose.pose.position.y;
  transform.transform.translation.z = odom_msg.pose.pose.position.z;
  transform.transform.rotation = odom_msg.pose.pose.orientation;
  odom_tf.sendTransform(transform);
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
  robot->resetEncoders();
}
/******************************************************************************/

/***********************************SETUP**************************************/
void setup() {
  Serial.begin(57600);
  robot = new AmigoBot();
  /* Set the target speeds in meters per second */
  leftPID.TargetSpeed = 0.0;
  rightPID.TargetSpeed = 0.0;

  /* Convert speeds to encoder ticks per frame */
  leftPID.TargetTicksPerFrame = SpeedToTicks(leftPID.TargetSpeed);
  rightPID.TargetTicksPerFrame = SpeedToTicks(rightPID.TargetSpeed);

  /* Zero out the encoder counts */
  robot->resetEncoders();

  /* Initialize the ROS node */
  nh.initNode();

  nextPID = PID_INTERVAL;
  nextOdom = ODOM_INTERVAL;

  /* Advertise the Odometry publisher */
  nh.advertise(odomPub);

  /* Activate the Twist subscriber */
  nh.subscribe(cmdVelSub);

  robot->ready();
}
/******************************************************************************/

/************************************LOOP**************************************/
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

  /* AUTOSTOP? */
  if ((millis() - lastMotorCommand) > AUTO_STOP_INTERVAL) {;
    robot->setSpeeds(0, 0);
    moving = 0;
  }

  nh.spinOnce();
}
/******************************************************************************/
