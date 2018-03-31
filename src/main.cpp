#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int8.h"
#include "std_msgs/Char.h"
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

bool wait = false; 


int state = PREPARE;
bool odom_end = false;


std_msgs::Int8 task_data; // task

int nextTask(int index = -1) {
  static int i = 1;
  if (index != -1) {
    i = index;
  }
  return taskFlow[i++];
}


// ====================ros callbacks====================
void keyCallback(const std_msgs::Char::ConstPtr& m) {
  char c = m->data;
  ROS_INFO("kcb%c", c);
  if (c == 's'){
    //start
    state = nextTask(1);
  }
}

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



// ====================ros settings====================
ros::Publisher task_pub;
ros::Subscriber task_sub;
ros::Subscriber key_sub;



// ==================== sub routines ====================

bool correctSpace() {
  // @return bool: if true, continue, if false, finish.
  ROS_INFO("correct Space\n");
  static int mode = 0;
  if (mode==0) {
    // begin
    task_data.data = CORRECTSPACE_BEGIN;
    task_pub.publish(task_data);
    mode = 1;
    return true;
  } else if (mode == 1) {
    // check end
    if (!wait) {
      mode = 0;
      state = nextTask();
      return false;
    }
  } else {
    return true;
  }
}

bool odomRun() {
  // @return bool: if true, continue, if false, finish.
  
  if (logging) {
    // logging
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
  } else {
    // playing
    ROS_INFO("odom run (play) \n");
  }
}

// stages
void startToPass1() {
  static int mode = 0;
  if (mode == 0) {
      ROS_INFO("start to pass1\n");
      mode = 1;
  } else if (mode == 1) {
    if (!odomRun()) {
      //end
      mode = 2;
    }
  } else if (mode == 2) {
    if(!correctSpace()){
      mode = 0;
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
  ros::NodeHandle n;
  task_pub = n.advertise<std_msgs::Int8>("task", 1000);
  key_sub = n.subscribe("key", 1000, keyCallback);
  task_sub = n.subscribe("task", 1000, taskCallback);
  
  ros::Rate loop_rate(10);

  while (ros::ok()) {
    
    task();
    //odomRun();
    ros::spinOnce();
    loop_rate.sleep();
  }


  return 0;
}
