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

ros::Subscriber joy;
ros::Publisher pub;

std_msgs::Int8MultiArray array;
std_msgs::Int8 motorState;
int w[5]; 
float hor,ver,tur,speed_fast,speed_slow;



void joyCallback(const sensor_msgs::Joy::ConstPtr& joy){
	hor=joy->axes[0];
	ver=joy->axes[1];
	tur=joy->axes[3];
	speed_slow=joy->buttons[6];
	speed_fast=joy->buttons[7];
}

void calcMotorPower(float horizonal,float vertical,float turn) {
  /**
   * @param horizonal: -1~1
   * @param vertical: -1~1
   * @param turn: -1~1
   */
	float x,y,tn,length,c=10,tg=0.1,t=5,maxw,gain_wheel2=1.5,gain_wheel4=1.1,slow=0.5,fast=1.5,weaker=0.7;
		
	length=sqrt(pow(horizonal,2)+pow(vertical,2));
	x=horizonal*length*c;
	y=vertical*length*c;
	for(int i=1;i<5;i++){
		if(abs(w[i])>maxw){
			maxw=abs(w[i]);
		}
		else{
		}	
	
	}

	if(maxw<90){
		tn=turn*t;
		w[1]=-x+y-tn;
		w[2]=(x+y+tn)*gain_wheel2;
		w[3]=x+y-tn;
		w[4]=(-x*weaker+y+tn)*gain_wheel4;
	
	
	}
	else{
		tn=turn*tg;
		w[1]=-x+y*(1-tn);
		w[2]=x+y*(1+tn);
		w[3]=x+y*(1-tn);
		w[4]=-x+y*(1+tn);
	}
	
	for(int i=1;i<5;i++){
		if(abs(w[i])>100){
			w[i]=100*w[i]/abs(w[i]);
		}
		else{
		}
	}
	
	if(speed_slow==1){
		for(int i=1;i<5;i++){
			w[i]=w[i]*slow;
		}
	}
	else if(speed_fast==1){
		for(int i=1;i<5;i++){
			w[i]=w[i]*fast;
		}
	}
	array.data.clear();
	array.data.push_back(w[1]);
	array.data.push_back(w[2]);
	array.data.push_back(w[3]);
	array.data.push_back(w[4]);
	pub.publish(array);
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "mecanum_auto");
  ros::NodeHandle n;
  joy = n.subscribe("joy",1000,joyCallback);
  pub = n.advertise<std_msgs::Int8MultiArray>("array", 100);
  ros::Rate loop_rate(10);
  
  while (ros::ok())
    {
      calcMotorPower(hor,ver,tur);
      ros::spinOnce();
      loop_rate.sleep();
      ROS_INFO("%d %d %d %d\n",w[1],w[2],w[3],w[4]);
    }

  return 0;
}
