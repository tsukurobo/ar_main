#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int8.h"
#include "std_msgs/Char.h"
#include <sstream>
#include "conf.h"


// ====================definitions====================
// split: if true, go to next routine.
//        false is for debugging.
bool split = true;

// logging: if true, logging the move.
//        false is auto run.
bool logging = true;

bool wait = false; 


int state = PREPARE;
bool odom_end = false;

static const int taskFlow[1000] = {
  PREPARE,
  STARTTOPASS1,
  WAITPASS1,
  PASS1,
  PASS1TOSHOT1,
  SHOT1,
  SHOT1TOPASS2,
  WAITPASS2,
  PASS2,
  PASS2TOSHOT2,
  SHOT2,
  SHOT2TOPASS3,
  WAITPASS3,
  PASS3,
  PASS3TOSHOT3,
  SHOT3,  
};




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

bool odomRun(int taskData) {
  // @param taskData: data of the task from conf.h
  // @return bool: if true, continue, if false, finish.
  static int mode = 0;  
  if (logging) {
    // logging
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
    if (mode == 0){
      ROS_INFO("odom run (play) \n");
      task_data.data = taskData;
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
}

// stages
void startToPass1() {
  static int mode = 0;
  if (mode == 0) {
      ROS_INFO("start to pass1\n");
      mode = 1;
  } else if (mode == 1) {
    if (!odomRun(STARTTOPASS1)) {
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

void pass1ToShot1() {

}

void shot1() {

}

void shot1ToPass2() {

}

void waitPass2() {

}

void pass2() {

}

void pass2ToShot2() {

}

void shot2() {

}

void shot2ToPass3() {

}

void waitPass3() {

}

void pass3() {

}

void pass3ToShot3() {

}

void shot3() {

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
  } else if (state == PASS1TOSHOT1) {
    pass1ToShot1();
  } else if (state == SHOT1) {
    shot1();
  } else if (state == SHOT1TOPASS2) {
    shot1ToPass2();
  } else if (state == WAITPASS2) {
    waitPass2();
  } else if (state == PASS2) {
    pass2();
  } else if (state == PASS2TOSHOT2) {
    pass2ToShot2();
  } else if (state == SHOT2) {
    shot2();
  } else if (state == SHOT2TOPASS3) {
    shot2ToPass3();
  } else if (state == WAITPASS3) {
    waitPass3();
  } else if (state == PASS3) {
    pass3();
  } else if (state == PASS3TOSHOT3) {
    pass3ToShot3();
  } else if (state == SHOT3) {
    shot3();
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
