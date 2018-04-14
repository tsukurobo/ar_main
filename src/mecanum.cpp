#include "ros/ros.h"
#include <stdlib.h>
#include <math.h>
#include "std_msgs/Int8.h"
#include "sensor_msgs/Joy.h"
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Int8MultiArray.h"
#include <sstream>
#include "conf.h"




// publishers

ros::Subscriber state_sub;
ros::Subscriber line_sub;
ros::Subscriber lrf_sub;
ros::Publisher pub;
ros::Publisher state_pub;
std_msgs::Int8MultiArray array;
std_msgs::Int8 motorState;
int w[5]; 
int line_param[4];
float lrfparam[6]; 

int delayCounter = 0;
bool delaying = false;
const int hz = 10;

float calcdt(int s){
	float caleddt =0;
	if(s==USE_FRONT_LRF){
		caleddt=lrfparam[0]/90;
	}
	else if(s==USE_LEFT_LRF){
		caleddt=lrfparam[1]/90;
	}
	else{
	}
	return caleddt;
}

float move_line(float line_degree){
	float caled_degree, line_gravity, calpw=5;//correct degree max fabs(calpw)
	
	if(line_degree==RIGHT){
		line_gravity=(line_param[1]+line_param[3])/8;
		caled_degree=line_degree+line_gravity*calpw;
	}
	else if(line_degree==FRONT){
		line_gravity=(line_param[0]+line_param[2])/8;
		caled_degree=line_degree-line_gravity*calpw;
	}
	else if(line_degree==LEFT){
		line_gravity=(line_param[1]+line_param[3])/8;
		caled_degree=line_degree-line_gravity*calpw;
	}
	else if(line_degree==BACK){
		line_gravity=(line_param[0]+line_param[2])/8;
		caled_degree=line_degree+line_gravity*calpw;
	}
	return caled_degree;
}

float calcset_degree(float vertical, float horizonal){
	float degree, lg;
	lg=sqrt(pow(vertical,2)+pow(horizonal,2));
	vertical=vertical/lg;
	horizonal=horizonal/lg;
	degree = atan2(vertical,horizonal);
	return degree;
}

float calcset_length(float vertical, float horizonal){
	float length;
	length = sqrt(pow(vertical,2)+pow(horizonal,2));
	return length;
}

void calcMotorPower(float degree,float length,int lrf,int line) {
  /**
   * @param degree: -180~180
   * @param length: 0~1
   * @param lrf: 0 or 1 0:flont 1:left
   *@oparam line 0 or 1 1:use
   */
	float x,y,tn,dt,radian,c=48,t=25;
	
	//ラインに沿って進むとき
	if(line == USE_LINE){
		degree=move_line(degree);
	}
	//ラインに沿わないとき
	else{
	}
	
	radian=degree*M_PI/180.0;
	
	//回転調整
	dt=calcdt(lrf);
	
	tn=dt*t;	
	
	x=cos(radian)*length*c;
	
	y=sin(radian)*length*c;
	
	w[1]=x+y-tn;
	w[2]=-x+y+tn;
	w[3]=-x+y-tn;
	w[4]=x+y+tn;
	
	for(int i=1;i<5;i++){
		if(abs(w[i])>100){
			w[i]=100*w[i]/abs(w[i]);
		}
		else{
		}
	}
	array.data.clear();
	array.data.push_back(w[1]);
	array.data.push_back(w[2]);
	array.data.push_back(w[3]);
	array.data.push_back(w[4]);
	pub.publish(array);
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

void set(){
	float set_length, set_degree, vertical_gravity, horizonal_gravity;
	int n=8;//powar of set 
	delay(500);
	while(delaying){
		vertical_gravity=(line_param[0]+line_param[2])/n;
		horizonal_gravity=(line_param[1]+line_param[3])/n;
		set_degree=calcset_degree(vertical_gravity,horizonal_gravity);
		set_length=calcset_length(vertical_gravity,horizonal_gravity);
		calcMotorPower(set_degree,set_length,USE_FRONT_LRF,DO_NOT_USE_LINE);
	  	ros::spinOnce();
	  	ros::Duration(0.1).sleep();
  		delayCount();
	}
}




//Callback
void stateCallback(const std_msgs::Int8::ConstPtr& move_state){
	motorState.data=move_state->data;
}

void lineCallback(const std_msgs::Int8MultiArray::ConstPtr& linepm)
{
    int ln = 0;
    for(std::vector<int8_t>::const_iterator it = linepm->data.begin(); it != linepm->data.end(); ++it)
    { 
        line_param[ln] = *it;
        ln++;
    }
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

void startToPass1(){
  for(int i=0;i<10;i++){
	  calcMotorPower(-80, i*0.1, USE_LEFT_LRF, DO_NOT_USE_LINE);
	  ros::Duration(3).sleep();
  }
  delay(1000);
  while(delaying){
  	calcMotorPower(-80, 1, USE_LEFT_LRF, DO_NOT_USE_LINE);
  	ros::spinOnce();
  	ros::Duration(0.1).sleep();
  	delayCount();
  }
  for(int i=0;i<7;i++){
	  calcMotorPower(-80, 1-i*0.1, USE_LEFT_LRF, DO_NOT_USE_LINE);
	  ros::Duration(3).sleep();
  }
  while(line_param[1]!=0){
  	calcMotorPower(-80, 0.3, USE_LEFT_LRF, DO_NOT_USE_LINE);
  	ros::spinOnce();
  }
} 

void pass1ToShot1(){
  for(int i=0;i<10;i++){\
	  calcMotorPower(RIGHT, i*0.1, DO_NOT_USE_LRF, USE_LINE);
	  ros::Duration(3).sleep();
  }
  delay(1000);
  while(delaying){
  	calcMotorPower(RIGHT, 1, DO_NOT_USE_LRF, USE_LINE);
  	ros::spinOnce();
  	ros::Duration(0.1).sleep();
  	delayCount();
  }
  for(int i=0;i<7;i++){
	  calcMotorPower(RIGHT, 1-i*0.1, DO_NOT_USE_LRF, USE_LINE);
	  ros::Duration(3).sleep();
  }
  while(line_param[0]!=0){
  	calcMotorPower(RIGHT, 0.3, USE_FRONT_LRF, USE_LINE);
  	ros::spinOnce();
  }
  
  set();
  	
}

void shot1ToPass2(){
 for(int i=0;i<10;i++){
	  calcMotorPower(LEFT, i*0.1, DO_NOT_USE_LRF, USE_LINE);
	  ros::Duration(3).sleep();
  }
  delay(1000);
  while(delaying){
  	calcMotorPower(LEFT, 1, DO_NOT_USE_LRF, USE_LINE);
  	ros::spinOnce();
  	ros::Duration(0.1).sleep();
  	delayCount();
  }
  for(int i=0;i<7;i++){
	  calcMotorPower(LEFT, 1-i*0.1, DO_NOT_USE_LRF, USE_LINE);
	  ros::Duration(3).sleep();
  }
  while(line_param[3]!=0){
  	calcMotorPower(LEFT, 0.3, DO_NOT_USE_LRF, USE_LINE);
  	ros::spinOnce();
  }
}

void pass2ToShot2(){
  for(int i=0;i<10;i++){
	  calcMotorPower(-40, i*0.1, USE_LEFT_LRF, DO_NOT_USE_LINE);
	  ros::Duration(3).sleep();
  }
  delay(1000);
  while(delaying){
  	calcMotorPower(-40, 1, USE_LEFT_LRF, DO_NOT_USE_LINE);
  	ros::spinOnce();
  	ros::Duration(0.1).sleep();
  	delayCount();
  }
  for(int i=0;i<7;i++){
	  calcMotorPower(-40, 1-i*0.1, USE_LEFT_LRF, DO_NOT_USE_LINE);
	  ros::Duration(3).sleep();
  }
  
  while(line_param[1]!=0){
  	calcMotorPower(-40, 0.3, USE_LEFT_LRF, DO_NOT_USE_LINE);
  	ros::spinOnce();
  }
  
  for(int i=0;i<10;i++){
	  calcMotorPower(RIGHT, i*0.1, DO_NOT_USE_LRF, USE_LINE);
	  ros::Duration(3).sleep();
  }
  delay(1000);
  while(delaying){
  	calcMotorPower(RIGHT, 1, DO_NOT_USE_LRF, USE_LINE);
  	ros::spinOnce();
  	ros::Duration(0.1).sleep();
  	delayCount();
  }
  for(int i=0;i<7;i++){
	  calcMotorPower(RIGHT, 1-i*0.1, DO_NOT_USE_LRF, USE_LINE);
	  ros::Duration(3).sleep();
  }
  while(line_param[0]!=0){
  	calcMotorPower(RIGHT, 0.3, USE_FRONT_LRF, USE_LINE);
  	ros::spinOnce();
  }
  set();
}

void shot2ToPass3(){
  for(int i=0;i<10;i++){
	  calcMotorPower(LEFT, i*0.1, DO_NOT_USE_LRF, USE_LINE);
	  ros::Duration(3).sleep();
  }
  delay(1000);
  while(delaying){
  	calcMotorPower(LEFT, 1, DO_NOT_USE_LRF, USE_LINE);
  	ros::spinOnce();
  	ros::Duration(0.1).sleep();
  	delayCount();
  }
  for(int i=0;i<7;i++){
	  calcMotorPower(LEFT, 1-i*0.1, DO_NOT_USE_LRF, USE_LINE);
	  ros::Duration(3).sleep();
  }
  while(line_param[0]!=0){
  	calcMotorPower(LEFT, 0.3, USE_FRONT_LRF, USE_LINE);
  	ros::spinOnce();
  }
}

void pass3ToShot3(){
  for(int i=0;i<10;i++){
	  calcMotorPower(RIGHT, i*0.1, USE_LEFT_LRF, USE_LINE);
	  ros::Duration(3).sleep();
  }
  delay(1000);
  while(delaying){
  	calcMotorPower(RIGHT, 1, USE_LEFT_LRF, USE_LINE);
  	ros::spinOnce();
  	ros::Duration(0.1).sleep();
  	delayCount();
  }
  for(int i=0;i<7;i++){
	  calcMotorPower(RIGHT, 1-i*0.1, USE_LEFT_LRF, USE_LINE);
	  ros::Duration(3).sleep();
  }
  while(line_param[1]!=0){
  	calcMotorPower(RIGHT, 0.3, USE_FRONT_LRF, USE_LINE);
  	ros::spinOnce();
  }
	set();
}
void shot3ToPass3(){
  for(int i=0;i<10;i++){
	  calcMotorPower(LEFT, i*0.1, DO_NOT_USE_LRF, USE_LINE);
	  ros::Duration(3).sleep();
  }
  delay(1000);
  while(delaying){
  	calcMotorPower(LEFT, 1, DO_NOT_USE_LRF, USE_LINE);
  	ros::spinOnce();
  	ros::Duration(0.1).sleep();
  	delayCount();
  }
  for(int i=0;i<7;i++){
	  calcMotorPower(LEFT, 1-i*0.1, DO_NOT_USE_LRF, USE_LINE);
	  ros::Duration(3).sleep();
  }
  while(line_param[0]!=0){
  	calcMotorPower(LEFT, 0.3, DO_NOT_USE_LRF, USE_LINE);
  	ros::spinOnce();
  }
}
void pass3ToShot2(){
  for(int i=0;i<10;i++){
	  calcMotorPower(RIGHT, i*0.1, DO_NOT_USE_LRF, USE_LINE);
	  ros::Duration(3).sleep();
  }
  delay(1000);
  while(delaying){
  	calcMotorPower(RIGHT, 1, DO_NOT_USE_LRF, USE_LINE);
  	ros::spinOnce();
  	ros::Duration(0.1).sleep();
  	delayCount();
  }
  for(int i=0;i<7;i++){
	  calcMotorPower(RIGHT, 1-i*0.1, DO_NOT_USE_LRF, USE_LINE);
	  ros::Duration(3).sleep();
  }
  while(line_param[1]!=0){
  	calcMotorPower(RIGHT, 0.3, USE_FRONT_LRF, USE_LINE);
  	ros::spinOnce();
  }
	set();
}



void move(){
	if(motorState.data==STARTTOPASS1){
		startToPass1();
		motorState.data=PREPARE;
		state_pub.publish(motorState);
	}
	else if(motorState.data==PASS1TOSHOT1){
		pass1ToShot1();
		motorState.data=PREPARE;
		state_pub.publish(motorState);
	}
	else if(motorState.data==SHOT1TOPASS2){
		shot1ToPass2();
		motorState.data=PREPARE;
		state_pub.publish(motorState);
	}
	else if(motorState.data==PASS2TOSHOT2){
		pass2ToShot2();
		motorState.data=PREPARE;
		state_pub.publish(motorState);
	}
	else if(motorState.data==SHOT2TOPASS3){
		shot2ToPass3();
		motorState.data=PREPARE;
		state_pub.publish(motorState);
	}
	else if(motorState.data==PASS3TOSHOT3){
		pass3ToShot3();
		motorState.data=PREPARE;
		state_pub.publish(motorState);
	}
	else if(motorState.data==SHOT3TOPASS3){
		shot3ToPass3();
		motorState.data=PREPARE;
		state_pub.publish(motorState);
	}
	else if(motorState.data==PASS3TOSHOT2){
		pass3ToShot2();
		motorState.data=PREPARE;
		state_pub.publish(motorState);
	}
	else{
	}
}



int main(int argc, char **argv)
{
  ros::init(argc, argv, "mecanum_auto");
  ros::NodeHandle n;
  pub = n.advertise<std_msgs::Int8MultiArray>("mecanum_motors", 100);
  state_pub = n.advertise<std_msgs::Int8>("state", 100);
  state_sub = n.subscribe("state",100,stateCallback);
  line_sub = n.subscribe("lineparam",100,lineCallback);
  lrf_sub = n.subscribe("lrfparam",100,lrfCallback);
  ros::Rate loop_rate(10);
  
  while (ros::ok())
    {
      move();
      ros::spinOnce();
      loop_rate.sleep();
    }

  return 0;
}
