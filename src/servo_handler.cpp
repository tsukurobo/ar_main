#include "ros/ros.h"
#include <stdlib.h>
#include <math.h>
#include "std_msgs/Int8.h"
#include "sensor_msgs/Joy.h"
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Int16MultiArray.h"
#include "conf.h"

// params
const int servoSum = 7;
const int closeDeg = 180; // close deg
const int openDeg = 90;   // open deg
const int hz = 10;
const int b = 0, c = 1, d = 2, e = 3, f = 4, g = 5, h = 6;
const int delaySmall = 500, delayMedium = 1000, delayLong = 1500, delayShot = 5000, delayReset = 3000;

// inner values
bool delaying = false; // for delay
int delayCounter = 0;
int state = SERVO_WAIT;

// ros values
ros::Publisher pub;
ros::Subscriber servoSub;
std_msgs::Int16MultiArray array;


// =========== callback ==========
void servoTaskCallback(const std_msgs::Int8::ConstPtr& m){
  //ROS_INFO("stcb");
  state = m->data;
}


// ===========sub func==========
void sendArr(int servoNo, int degree) {
  array.data.clear();
  array.data.push_back(servoNo);
  array.data.push_back(degree);
  pub.publish(array);
}

void sOpen(int servoNo) {
  sendArr(servoNo, openDeg);  
}

void sClose(int servoNo) {
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
    sClose(i);
  }
}

void prepare() {
  // add pressure
  static int mode = 0;
  if (mode == 0) {
    ROS_INFO("prepare");
    sOpen(h);
    delay(delaySmall);
    mode = 1;
  } else if (mode == 1) {
    sOpen(b);
    delay(delaySmall);
    mode = 2;
  } else if (mode == 2) {
    sOpen(f);
    delay(delayMedium);
    mode = 3;
  } else if (mode == 3) {
    sClose(h);
    sClose(f);
    delay(delaySmall);
    state = -1;
    mode = 0;
  } 
}

void tz1(){
    static int mode = 0;
    if (mode == 0) {
      ROS_INFO("tz1");
      // shot
      sOpen(d);
      delay(delayShot);
      mode = 1;
    } else if (mode ==1) {
      // shot done
      sClose(d);
      sClose(e);
      delay(delaySmall);
      mode = 2;
    } else if (mode == 2) {
      // prepare exhaust
      sOpen(h);
      delay(delaySmall);
      mode = 3;
    } else if ( mode == 3) {
      // reset the arm
      sOpen(f);
      delay(delayReset);
      mode = 4;
    } else if (mode == 4) {
      // reset done
      sClose(f);
      sClose(h);
      delay(delaySmall);
      mode = 5;
    } else if (mode == 5) {
      // prepare exhausting. close 0.45mp
      sOpen(e);
      delay(500);
      mode = 6;
    } else if (mode == 6) {
      // prepare 0.55mp.
      sOpen(c);
      mode = 0;
      state = -1;
    }
}

void tz2(){
    static int mode = 0;
    if (mode == 0) {
      ROS_INFO("tz2");
      // shot
      sOpen(d);
      delay(delayShot);
      mode = 1;
    } else if (mode ==1) {
      // shot done
      sClose(d);
      sClose(e);
      delay(delaySmall);
      mode = 2;
    } else if (mode == 2) {
      // prepare exhaust
      sOpen(h);
      delay(delaySmall);
      mode = 3;
    } else if ( mode == 3) {
      // reset the arm
      sOpen(f);
      delay(delayReset);
      mode = 4;
    } else if (mode == 4) {
      // reset done
      sClose(f);
      delay(delaySmall);
      mode = 5;
    } else if (mode == 5) {
      sClose(h);
      sOpen(e);
      sClose(c);
      delay(delaySmall);
      mode = 6;
    } else if (mode == 6) {
      // prepare 0.71
      sOpen(g);
      delay(delaySmall);
      mode = 0;
      state = -1;
    } 
}

void tz3() {
    static int mode = 0;
    if (mode == 0) {
      ROS_INFO("tz3");
      // shot
      sOpen(d);
      delay(delayShot);
      mode = 1;
    } else if (mode ==1) {
      // shot done
      sClose(d);
      delay(delaySmall);
      mode = 2;
    } else if (mode == 2) {
      // prepare exhaust
      sClose(e);
      delay(delaySmall);
      mode = 3;
    } else if ( mode == 3) {
      // reset the arm
      sOpen(f);
      delay(delayReset);
      mode = 4;
    } else if (mode == 4) {
      // reset done
      sClose(f);
      delay(delaySmall);
      mode = 5;
    } else if (mode == 5) {
      sOpen(e);
      delay(delaySmall);
      mode = 0;
      state = -1;
    }

}


void task() {
  //  ROS_INFO("task");
  if (state == SERVO_WAIT) {
    //pass
  } else if (state == SERVO_PREPARE) {
    prepare();
  } else if (state == SERVO_TZ1SHOT) {
    tz1();
  } else if (state == SERVO_TZ2SHOT) {
    tz2();
  } else if (state == SERVO_TZ3SHOT) {
    tz3();
  }
  
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "servo_handler");
  ros::NodeHandle n;
  ros::Rate loop_rate(hz);
  pub = n.advertise<std_msgs::Int16MultiArray>("servo_deg", 100);
  servoSub = n.subscribe("servo_task", 100, servoTaskCallback);
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
