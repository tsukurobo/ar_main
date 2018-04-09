#include "ros/ros.h"
#include <stdlib.h>
#include <math.h>
#include "std_msgs/Int8.h"
#include "sensor_msgs/Joy.h"
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Int16MultiArray.h"

// params
const int servoSum = 7;
const int closeDeg = 90; // close deg
const int openDeg = 0;   // open deg
const int hz = 10;
const int b = 0, c = 1, d = 2, e = 3, f = 4, g = 5, h = 6;
const int delaySmall = 500, delayMedium = 1000, delayLong = 1500, delayShot = 3000;

// inner values
bool delaying = false; // for delay
int delayCounter = 0;

// ros values
ros::Publisher pub;
std_msgs::Int16MultiArray array;

// ===========sub func==========
void sendArr(int servoNo, int degree) {
  array.data.clear();
  array.data.push_back(servoNo);
  array.data.push_back(degree);
  pub.publish(array);
}

void open(int servoNo) {
  sendArr(servoNo, openDeg);  
}

void close(int servoNo) {
  sendArr(servoNo, closeDeg);  
}

void delayCount() {
  if (delayCounter > 0) {
    delayCounter--;
  } else {
    delaying = false;
  }
}

void delay(int ms) {
  ROS_INFO("delay");
  delayCounter = (ms*hz/1000);
  delaying = true;  
}

// =========== routine ==========
void setup() {
  ROS_INFO("setup");
  int i = 0;
  for (i = 0; i < servoSum; i++) {
    close(i);
  }
}

void prepare() {
  // add pressure
  static int mode = 0;
  if (mode == 0) {
    ROS_INFO("prepare");
    open(h);
    delay(delaySmall);
    mode = 1;
  } else if (mode == 1) {
    open(b);
    delay(delaySmall);
    mode = 2;
  } else if (mode == 2) {
    open(f);
    delay(delayMedium);
    mode = 3;
  } else if (mode == 3) {
    close(h);
    close(f);
    delay(delaySmall);
    mode = 0;
  } 
}

void task() {
  //  ROS_INFO("task");
  
  
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "servo_handler");
  ros::NodeHandle n;
  ros::Rate loop_rate(hz);
  pub = n.advertise<std_msgs::Int16MultiArray>("servo_deg", 100);
  setup();
  
  while (ros::ok())
    {
      ros::spinOnce();
      if (delaying) {
	delayCount();
      } else {
	task();
      }
      loop_rate.sleep();
    }

  return 0;
}
