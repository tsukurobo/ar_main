#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int8.h"
#include <sstream>
#include "conf.h"


// ====================definitions====================
static int taskFlow[1000] = {
  PREPARE,
  STARTTOPASS1,
  WAITPASS1,
  PASS1
};


// split: if true, go to next routine.
//        false is for debugging.
bool split = true;

// logging: if true, logging the move.
//        false is auto run.
bool logging = true;

int owner = 0; // 0 means owner is self.


int state = PREPARE;
bool odom_end = false;



// ====================ros callbacks====================
void taskCallback(const std_msgs::Int8::ConstPtr& m) {
  int r = m->data;
  if (r == CORRECTSPACE_END) {
    owner = 0;
  } else if (r == ODOMRUN_END) {
    owner = 0;
  } else if (r == ODOMRUN_LOGGING_END) {
    owner = 0;
  }
}



// ====================ros settings====================
ros::NodeHandle n;
ros::Publisher task_pub = n.advertise<std_msgs::Int8>("task", 1000);
ros::Subscriber task_sub = n.subscribe("task", 1000, taskCallback);



// ==================== sub routines ====================

int nextTask() {
  static int i = 1;
  return taskFlow[i++];
}

bool correctSpace() {
  // @return bool: if true, continue, if false, finish.
  ROS_INFO("correct Space\n");
  static int mode = 0;
  if (mode==0) {
    // begin
    std_msgs::Int8 c;
    c.data = CORRECTSPACE_BEGIN;
    task_pub.publish(c);
    mode = 1;
    return true;
  } else if (mode == 1) {
    // check end
    if (owner == 0) {
      mode = 0;
      return false;
    }
  } else {
    return true;
  }
}

bool odomRun() {
  // @return bool: if true, continue, if false, finish.
  if (logging) {
    ROS_INFO("odom run (logging) \n");
  } else {
    ROS_INFO("odom run (play) \n");
  }
}

// stages
void startToPass1() {
  ROS_INFO("start to pass1\n");
  static int mode = 0;
  if (mode == 0) {
    if(!odomRun()){
      mode = 1;
    }
  } else if (mode == 1) {
    if(!correctSpace()){
      state = nextTask();
    }
  }
  
}

void waitPass1() {
  ROS_INFO("wait pass1\n");
  //
}

void pass1() {
  ROS_INFO("pass1\n");
  //
  
}

void chatterCallback(const std_msgs::String::ConstPtr& msg)
{
  ROS_INFO("I heard: [%s]", msg->data.c_str());
}

void task() {
  if (state == PREPARE) {
    //pass
  } else if (state == STARTTOPASS1) {
    startToPass1();
  } else if (state == WAITPASS1) {
    waitPass1();
  } else if (state == PASS1) {
    pass1();
  }
}



// ==================== main workflow ====================
int main(int argc, char **argv)
{
  ros::init(argc, argv, "main");
  ros::Rate loop_rate(10);

  while (ros::ok()) {
    
    task();
    ros::spinOnce();
    loop_rate.sleep();
  }


  return 0;
}
