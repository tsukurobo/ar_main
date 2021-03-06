//main #.ino
// dataset [servoNo, degree(0~180)]
#include <Servo.h>
#include <ros.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Int16MultiArray.h>
#include <stdlib.h>
#include <std_msgs/MultiArrayLayout.h>
#include <std_msgs/MultiArrayDimension.h>

const int servoSum = 12;
const int shotServoSum = 7;
const int close_angle = 0;
const int open_angle = 90;

ros::NodeHandle  nh;
std_msgs::Int16MultiArray array;
Servo servos[servoSum];
Servo shotServos[shotServoSum];

void initServos() {
  int i = 0;
  for(i = 0; i < servoSum; i++) {
    servos[i] = Servo();
    servos[i].attach(i+2);  //pin: 2~14
    servos[i].write(close_angle);
  }
}

// =========== rosCallBack ==========
void servoDegCB(const std_msgs::Int16MultiArray& array)
{
  int index = array.data[0];
  int deg = array.data[1];
  servos[index].write(deg);
}
ros::Subscriber<std_msgs::Int16MultiArray>sub("servo_deg",&servoDegCB);

// ========== main ==========
void setup() {
  initServos();
  nh.initNode();
  nh.subscribe(sub);
}

void loop() {
  nh.spinOnce();
  delay(10);
}
