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
ros::Subscriber lrf_sub;
ros::Publisher mecanum_pub;
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

void calcMotorPower(float degree,float length,int lrf) {
  /**
   * @param degree: -180~180
   * @param length: 0~1
   * @param lrf: 0 or 1 0:flont 1:left
   *@oparam line 0 or 1 1:use
   */
	float x,y,tn,dt,radian,c=15,t=10,maxw,gain_wheel2=1.5,gain_wheel4=1.1;
	//c<100-t
	
	
	
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
	
	w[2]=w[2]*gain_wheel2;
	w[4]=w[4]*gain_wheel4;
	
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
	mecanum_pub.publish(array);
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


void lrfCallback(const std_msgs::Float32MultiArray::ConstPtr& lrfpm)
{
    int lr = 0;
    for(std::vector<float>::const_iterator it = lrfpm->data.begin(); it != lrfpm->data.end(); ++it)
    { 
        lrfparam[lr] = *it;
        lr++;
    }
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "mecanum_auto");
  ros::NodeHandle n;
  mecanum_pub = n.advertise<std_msgs::Int8MultiArray>("mecanum_motors", 100);
  lrf_sub = n.subscribe("lrfparam",100,lrfCallback);
  ros::Rate loop_rate(10);
  
  while (ros::ok())
    {
      calcMotorPower(0,0,USE_FRONT_LRF);
      ros::spinOnce();
      loop_rate.sleep();
    }

  return 0;
}
