#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int8.h"
#include "std_msgs/Char.h"
#include "sensor_msgs/Joy.h"
#include <sstream>
#include "conf.h"






// ====================definitions====================

int state;
double delayCounter = 0;
bool delaying = false;
const int hz = 1000000000;
int btn0,btn1,btn2,btn3,btn4,btn5,btn6;
int btn0param, btn1param, btn2param,btn3param,btn4param,btn5param,btn6param;


std_msgs::Int8 servoTask;
std_msgs::Int8 armState;



ros::Duration five_seconds(5, 0);

int tmp_state = 0;




void delayCount() {
  if (delayCounter > 0) {
    delayCounter--;
  } else {
    delaying = false;
  }
}

void delay(int ms) {
  ROS_INFO("delay");
  delayCounter = (ms*hz);
  delaying = true;  
}

// ====================ros settings====================

ros::Publisher servo_pub;
ros::Publisher num_pub;
ros::Subscriber joy;
int b1 = 0;

// ====================ros callbacks====================

void joyCallback(const sensor_msgs::Joy::ConstPtr& joy){
        b1 = joy->buttons[11];//tmp
	btn0=joy->buttons[0];//shot 1
	btn1=joy->buttons[2];//shot 2
	btn2=joy->buttons[3];//shot 3
	btn3=joy->axes[5];//set front
	btn4=joy->axes[5];//set back
	btn5=joy->buttons[4];//prepare
	btn6=joy->buttons[5];//close
	if(btn0==1){
		btn0param=1;
	}
	else if(btn1==1){
		btn1param=1;
	}
	else if(btn2==1){
		btn2param=1;
	}
	else if(btn3==1){
		btn3param=1;
	}
	else if(btn4==-1){
		btn4param=-1;
	}
	else if(btn5==1){
		btn5param=1;
	}
	else if(btn6==1){
		btn6param=1;
	}
}






// ==================== sub routines ====================


void shotTz1(){	
	ROS_INFO("shotTz1\n");
	servoTask.data=SERVO_TZ1SHOT;
	servo_pub.publish(servoTask);
}
void shotTz2(){
	ROS_INFO("shotTz2\n");
	servoTask.data=SERVO_TZ2SHOT;
	servo_pub.publish(servoTask);
}
void shotTz3(){
	ROS_INFO("shotTz3\n");
	servoTask.data=SERVO_TZ3SHOT;
	servo_pub.publish(servoTask);
}

void pick(){
	ROS_INFO("pick\n");
	servoTask.data=SERVO_PICK;
	servo_pub.publish(servoTask);
}
void free(){
	ROS_INFO("free\n");
	servoTask.data=SERVO_FREE;
	servo_pub.publish(servoTask);
}
void close(){
	ROS_INFO("close\n");
	servoTask.data=SERVO_CLOSE;
	servo_pub.publish(servoTask);
}
void set_front(){
	armState.data=ARM_PRE_FRONT;
	num_pub.publish(armState);
	ROS_INFO("arm front\n");
	
	ros::Duration(1.5).sleep();
        while(b1==0){
        	ros::spinOnce();
        }
	
	
	pick();
	ros::Duration(1).sleep();

	armState.data=ARM_RETURN;
	num_pub.publish(armState);
	ROS_INFO("arm return\n");
	
	ros::Duration(1.5).sleep();	
	
	armState.data=ARM_PASS;
	num_pub.publish(armState);
	ROS_INFO("arm pass\n");
	
	//ros::Duration(4.5).sleep();
	ros::Duration(10).sleep();
	
	armState.data=ARM_RETURN;
	num_pub.publish(armState);
	ROS_INFO("arm init\n");
	
	free();
	
	ros::Duration(0.5).sleep();
	
	armState.data=ARM_INIT;
	num_pub.publish(armState);
	ROS_INFO("arm init\n");
	
}
void set_back(){
	armState.data=ARM_PRE_BACK;
	num_pub.publish(armState);
	ROS_INFO("arm back\n");
	
	ros::Duration(1.5).sleep();
        while(b1==0){
        	ros::spinOnce();
        }
	
	
	pick();
	ros::Duration(1).sleep();
	
	armState.data=ARM_RETURN;
	num_pub.publish(armState);
	ROS_INFO("arm return\n");
	
	ros::Duration(1.5).sleep();
	
	armState.data=ARM_PASS;
	num_pub.publish(armState);
	ROS_INFO("arm pass\n");
	
	ros::Duration(10).sleep();
	//4.5
	
	armState.data=ARM_RETURN;
	num_pub.publish(armState);
	ROS_INFO("arm init\n");
	
	free();
	
	ros::Duration(0.5).sleep();
	
	armState.data=ARM_INIT;
	num_pub.publish(armState);
	ROS_INFO("arm init\n");
}
void prepare(){
	free();
	armState.data=ARM_PREPARE;
	num_pub.publish(armState);
	servoTask.data=SERVO_PREPARE;
	servo_pub.publish(servoTask);
	ROS_INFO("prepare\n");
}

void task() {

	if(btn0param==1&&btn0==0){
	//move
		shotTz1();
		btn0param=0;
	}
	else if(btn1param==1&&btn1==0){
	//move
		shotTz2();
		btn1param=0;
	}
	else if(btn2param==1&&btn2==0){
	//move
		shotTz3();
		btn2param=0;
	}
	else if(btn3param==1&&btn3==0){
	//move
		set_front();
		btn3param=0;
	}
	else if(btn4param==-1&&btn4==0){
	//move
		set_back();
		btn4param=0;
	}
	else if(btn5param==1&&btn5==0){
	//move
		free();
		prepare();
		btn5param=0;
	}
	else if(btn6param==1&&btn6==0){
	//move
		close();
		btn6param=0;
	}
	
  }



// ==================== main workflow ====================
int main(int argc, char **argv)
{
ros::init(argc, argv, "main");
ros::NodeHandle n;

  servo_pub = n.advertise<std_msgs::Int8>("servo_task", 100);
  num_pub = n.advertise<std_msgs::Int8>("num", 100);
  joy = n.subscribe("joy",1000,joyCallback);
  
ros::Rate loop_rate(10);
  

   while (ros::ok())
  {
    task();
    ros::spinOnce();
    loop_rate.sleep();
  }


  return 0;
}
