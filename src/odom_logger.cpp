#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int8.h"
#include "std_msgs/Char.h"
#include <sstream>
#include "conf.h"
#include "sensor_msgs/Joy.h"

int beginButtonIndex = 2;
bool wait;
std_msgs::Int8 task_data; // task
ros::Publisher task_pub;
ros::Subscriber task_sub;


void taskCallback(const std_msgs::Int8::ConstPtr& m) {
  int r = m->data;
  if (r == CORRECTSPACE_END) {
    wait = false;
  } else if (r == ODOMRUN_END) {
    wait = false;
  } else if (r == ODOMRUN_LOGGING_END) {
    wait = false;
  }
}

bool odomRun(int taskData) {
  // @param taskData: data of the task from conf.h
  // @return bool: if true, continue, if false, finish.
  static int mode = 0;  
  if (mode == 0){
    ROS_INFO("odom run (logging) \n");
    task_data.data = ODOMRUN_LOGGING_BEGIN;
    task_pub.publish(task_data);
    mode = 1;
    wait = true;
    return true;
  } else if (mode == 1) {
    // wait
    if (!wait) {
      // end
      mode = 0;
      return false;
    } else {
      return true;
    }
  }
}

void joyCallback(const sensor_msgs::Joy::ConstPtr& joy){
  if (joy->buttons[beginButtonIndex]) {
    ROS_INFO("correct Space\n");
    task_data.data = ODOMRUN_LOGGING_BEGIN;
    task_pub.publish(task_data);
  }
}


int main(int argc, char **argv) {  
  ros::init(argc, argv, "odomlogger");
  ros::NodeHandle n;
  ros::Rate loop_rate(10);
  ros::Subscriber joy = n.subscribe("joy",1000,joyCallback);
  task_sub = n.subscribe("task", 1000, taskCallback);
  task_pub = n.advertise<std_msgs::Int8>("task", 1000);

  ROS_INFO("push contoller 3 button(index is 2), logging starts.\n");
  while (ros::ok())
    {
      ros::spinOnce();
      loop_rate.sleep();
    }

}

