#include <Encoder.h>
#define USE_USBCON
#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Empty.h>

//PINES
#define ON_LED    13
#define BUSY_LED  A0
#define IN1       9
#define IN2       8
#define IN3       7   // Input3 conectada al pin 5
#define IN4       6    // Input4 conectada al pin 4 
#define ENB       10   // ENB conectada al pin 3 de Arduino
#define ENA       11

//ENCODERS
Encoder M1Enc(0, 1);
Encoder M2Enc(2, 3);
long oldPosition1  = -999;
long oldPosition2  = -999;

//MANEJADOR DE ROS
ros::NodeHandle  nh;

//=======================PUBLICADORES==================
std_msgs::Int32 enc1_msg;
ros::Publisher enc1("ArduRos/Encoder1", &enc1_msg);
std_msgs::Int32 enc2_msg;
ros::Publisher enc2("ArduRos/Encoder2", &enc2_msg);

//=======================SUSCRIPTORES==================
void toggleLed(const std_msgs::String& toggle_msg){
  digitalWrite(BUSY_LED, HIGH-digitalRead(BUSY_LED));
}

ros::Subscriber<std_msgs::String> toggle_led("ArduRos/toggle_led", &toggleLed);

/*
void cmdVel(const std_msgs::Int8& cmd_msg){
  digitalWrite(BUSY_LED, LOW);
  analogWrite(ENB,cmd_msg.data);
  analogWrite(ENA,cmd_msg.data);
  digitalWrite(BUSY_LED, HIGH);
}

ros::Subscriber<std_msgs::Int8> cmd_vel("ArduRos/cmd_vel", &cmdVel);*/

//=========================MAIN========================

void setup(){
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
    nh.initNode();
    nh.advertise(enc1);
    nh.advertise(enc2);
    nh.subscribe(toggle_led);
    //nh.subscribe(cmd_vel);
    enc1_msg.data=0;
    enc2_msg.data=0;
}

void loop()
{
  long newPosition1 = M1Enc.read();
  long newPosition2 = M2Enc.read();
  if (newPosition1 != oldPosition1 || newPosition2 != oldPosition2) {
    oldPosition1 = newPosition1;
    oldPosition2 = newPosition2;
    enc1_msg.data=newPosition1;
    enc2_msg.data=newPosition2;
  }
  enc1.publish(&enc1_msg);
  enc2.publish(&enc2_msg);
  nh.spinOnce();
}
