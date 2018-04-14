#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int8.h"
#include "std_msgs/Char.h"
#include <sstream>
#include "conf.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Int32MultiArray.h"
#include "std_msgs/Int8MultiArray.h"
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Float64MultiArray.h"
std_msgs::Float32MultiArray param;
std_msgs::Int8 servoTask;
std_msgs::Int8 motorState;
std_msgs::Int8 armState;

ros::NodeHandle n;
ros::Rate loop_rate(10);

/*
@param[0]:degree 0~360(゜) foward:90,right:0
@param[1]:length 0~1
@param[3]:turn 1~-1
param.data.clear();
param.data.push_back();
param_pub.publish(param);
*/
/*
servoTask.data=;
servo_pub.pub(servoTask);
*/

// ====================definitions====================

int state = PREPARE;
int image_param[2];
float distance=300, lrfparam[6]; 
int delayCounter = 0;
bool delaying = false;
const int hz = 10;

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
  SHOT3TOPASS3,
  PASS3TOSHOT2,  
};





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

void imageCallback(const std_msgs::Int8MultiArray::ConstPtr& image){
    int loop = 0;
    for(std::vector<int8_t>::const_iterator it = image->data.begin(); it != image->data.end(); ++it)
    {
        image_param[loop] = *it;
        loop++;
    }
}

void stateCallback(const std_msgs::Int8::ConstPtr& move_state){
	motorState.data=move_state->data;
}

void servoCallback(const std_msgs::Int8::ConstPtr& servo_task){
	servoTask.data=servo_task->data;
}

void lrfCallback(const std_msgs::Float32MultiArray::ConstPtr& lrfpm)
{
    int lr = 0;
    for(std::vector<float>::const_iterator it = lrfpm->data.begin(); it != lrfpm->data.end(); ++it)
    { 
        lrfparam[lr] = *it;
        lr++;
    }
}

// ====================ros settings====================
ros::Publisher servo_pub;
ros::Publisher state_pub;
ros::Publisher num_pub;
ros::Subscriber key_sub;
ros::Subscriber image_sub;
ros::Subscriber state_sub;
ros::Subscriber servo_sub;
ros::Subscriber lrf_sub;

// ==================== sub routines ====================

// stages
//1
void startToPass1() {
	motorState.data=STARTTOPASS1;
	state_pub.publish(motorState);
	armState.data=ARM_PREPARE;
	num_pub.publish(armState);
	//while(motorState!=-1);
	ROS_INFO("start to pass1\n");
	state=nextTask();
}
//2
void waitPass1() {
  ROS_INFO("wait pass1\n");
  while(lrfparam[6]>distance){
		ros::spinOnce();
		loop_rate.sleep();
	}
	state=nextTask();
}
//3
void pass1() {
	ROS_INFO("pass1\n");
	while(image_param[0]==0){
		ros::spinOnce();
		loop_rate.sleep();
	}
	//pass
	armState.data=ARM_PRE_FRONT;
	num_pub.publish(armState);
	ros::Duration(3).sleep();
	//catch
	state=nextTask();
	
	armState.data=ARM_RETURN;
	num_pub.publish(armState);
	
	ros::Duration(3).sleep();
	

	state=nextTask();
}
//4
void pass1ToShot1() {
	
	motorState.data=PASS1TOSHOT1;
	state_pub.publish(motorState);
	ROS_INFO("pass1 to shot1\n");
	
	armState.data=ARM_PASS;
	num_pub.publish(armState);
	ros::Duration(3).sleep();
	armState.data=ARM_INIT;
	num_pub.publish(armState);
	ros::Duration(3).sleep();
	
	
	state=nextTask();
}
//5
void shot1() {
	servoTask.data=SERVO_TZ1SHOT;
	servo_pub.publish(servoTask);
	ROS_INFO("shot1\n");
	state=nextTask();
}
//6
void shot1ToPass2() {
	motorState.data=SHOT1TOPASS2;
	state_pub.publish(motorState);
	state=nextTask();
}
//7
void waitPass2() {
	while(lrfparam[6]>distance){
		ros::spinOnce();
		loop_rate.sleep();
	}
	state=nextTask();
}
//8
void pass2() {
	while(image_param[0]==0){
		ros::spinOnce();
		loop_rate.sleep();
	}
	//pass
	
	armState.data=ARM_PRE_FRONT;
	num_pub.publish(armState);
	ros::Duration(3).sleep();
	

	//watch and select
	//ok
	if(image_param[1]==1&&image_param[0]==1){
		state=nextTask();
	}
	//far
	else if(image_param[0]==0&&image_param[0]==1){
		state=nextTask(4);
	}
	
	//near
	else if(image_param[0]==1&&image_param[0]==0){
		state=nextTask(4);
	}
	armState.data=ARM_RETURN;
	num_pub.publish(armState);
	ros::Duration(3).sleep();
}
//9
void pass2ToShot2() {
	motorState.data=PASS2TOSHOT2;
	state_pub.publish(motorState);
	state=nextTask();

	armState.data=ARM_PASS;
	num_pub.publish(armState);
	armState.data=ARM_RETURN;
	num_pub.publish(armState);
	ros::Duration(3).sleep();
	
	armState.data=ARM_INIT;
	num_pub.publish(armState);
	armState.data=ARM_RETURN;
	num_pub.publish(armState);
	ros::Duration(3).sleep();
	
	
}
//10
void shot2() {
	servoTask.data=SERVO_TZ2SHOT;
	servo_pub.publish(servoTask);
	state=nextTask();

}
//11
void shot2ToPass3() {
	motorState.data=SHOT2TOPASS3;
	state_pub.publish(motorState);
	state=nextTask();
}
//12
void waitPass3() {
	while(lrfparam[6]>distance){
		ros::spinOnce();
		loop_rate.sleep();
	}
	state=nextTask();
}
//13
void pass3() {
	while(image_param[0]==0){
		ros::spinOnce();
		loop_rate.sleep();
	}
	//pass
	
	armState.data=ARM_PRE_FRONT;
	num_pub.publish(armState);
	ros::Duration(3).sleep();
	

	//watch and select
	//ok
	if(image_param[1]==1&&image_param[0]==1){
		state=nextTask();
	}
	//far
	else if(image_param[0]==0&&image_param[0]==1){
		state=nextTask(17);
	}
	
	//near
	else if(image_param[0]==1&&image_param[0]==0){
		state=nextTask(17);
	}
	
	armState.data=ARM_RETURN;
	num_pub.publish(armState);
	ros::Duration(3).sleep();
}
//14
void pass3ToShot3() {
	armState.data=ARM_PASS;
	num_pub.publish(armState);
	armState.data=ARM_INIT;
	num_pub.publish(armState);
	
	motorState.data=PASS3TOSHOT3;
	state_pub.publish(motorState);
	state=nextTask();
}
//15
void shot3() {
	servoTask.data=SERVO_TZ3SHOT;
	servo_pub.publish(servoTask);
	state=nextTask();

}
//16
void shot3ToPass3() {
	motorState.data=SHOT3TOPASS3;
	state_pub.publish(motorState);
	state=nextTask(12);
}
//17
void pass3ToShot2() {
	motorState.data=PASS3TOSHOT2;
	state_pub.publish(motorState);
	state=nextTask(10);

	armState.data=ARM_PASS;
	num_pub.publish(armState);
	ros::Duration(3).sleep();
	armState.data=ARM_INIT;
	num_pub.publish(armState);
	ros::Duration(3).sleep();
	
}



void chatterCallback(const std_msgs::String::ConstPtr& msg)
{
  ROS_INFO("I heard: [%s]", msg->data.c_str());
}

void task() {
  if (state == PREPARE) {
    servoTask.data=SERVO_PREPARE;
    servo_pub.publish(servoTask);
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
  } else if (state == PASS3TOSHOT2) {
    pass3ToShot2();
  } else if (state == SHOT3TOPASS3) {
    shot3ToPass3();
  } 
}



// ==================== main workflow ====================
int main(int argc, char **argv)
{
  ros::init(argc, argv, "main");
  state_pub = n.advertise<std_msgs::Int8>("state", 100);
  servo_pub = n.advertise<std_msgs::Int8>("servo_task", 100);
  num_pub = n.advertise<std_msgs::Int8>("num", 100);
  state_sub = n.subscribe("state",100,stateCallback);
  key_sub = n.subscribe("key", 1000, keyCallback);
  image_sub = n.subscribe("image",100,imageCallback);
  servo_sub = n.subscribe("servo_task",100,servoCallback);
  lrf_sub = n.subscribe("lrfparam",100,lrfCallback);
  

  while (ros::ok()) {
    if(servoTask.data == SERVO_WAIT && motorState.data == PREPARE){
	    task();
	    //odomRun();
    }
    else{
    }
    ros::spinOnce();
    loop_rate.sleep();
  }


  return 0;
}
