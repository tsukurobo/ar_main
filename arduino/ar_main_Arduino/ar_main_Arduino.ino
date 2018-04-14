/*
 * rosserial Publisher Example
 *test of Publishing Multi Array
 */
#include <stdio.h>
#include <ros.h>
#include <std_msgs/Int8MultiArray.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Int64MultiArray.h>
#include <std_msgs/Int64.h>
#include <stdlib.h>
#include <std_msgs/MultiArrayLayout.h>
#include <std_msgs/MultiArrayDimension.h>
#include <Wire.h>
#include <stdlib.h>
#include "ti2c.h"
#include "ise_motor_driver.h"
#define datalength 4294967295


#define DC2_INIT 0
#define DC2_PASS 500
#define DC1_INIT 0
#define DC1_PICK1 -1700
#define DC1_PICK2 -6000
#define DC1_PASS  -6000


volatile int pw[5]={0,0,0,0,0},w[6]={0,0,0,0,0,0};
long enc[5]={},Enc[5]={},Enc_a[5]={},Enc_b[5]={},n=14,armgoal[2];///18000
double p_gain=1.5,i_gain=0.0,d_gain=0.01,e1[5]={},e2[5]={},e3[5]={},e4[5]={},pic_p=0.01,pic_i=0.00001,pic_d=0.0,roll_p=0.02,roll_i=0.00055,roll_d=0.01,a1[2]={},a2[2]={},a3[2]={},a4[2]={};
//0.95 2.4 0.4
static int h=0,l=0;
long plus=2000000000;
long minus=-2000000000;
int state=0,armpw[2];

uint8_t addr1 = 0x22;
uint8_t addr2 = 0x20;
uint8_t addr3 = 0x23;
uint8_t addr4 = 0x21;
uint8_t addr5 = 0x24;
uint8_t addr6 = 0x25;
volatile int triger = 0;


IseMotorDriver m1 = IseMotorDriver(addr1);
IseMotorDriver m2 = IseMotorDriver(addr2);
IseMotorDriver m3 = IseMotorDriver(addr3);
IseMotorDriver m4 = IseMotorDriver(addr4);
IseMotorDriver m5 = IseMotorDriver(addr5);
IseMotorDriver m6 = IseMotorDriver(addr6);


ros::NodeHandle  nh;
std_msgs::Int64MultiArray arrayp;
std_msgs::Int8MultiArray array;
std_msgs::Int8MultiArray armp;
std_msgs::Int64MultiArray armenc;


//callback
void messageCb(const std_msgs::Int8MultiArray& array)
{
  for(int i=1;i<5;i++){
    w[i]=array.data[i-1];
  }
}

void Callback(const std_msgs::Int8& a){
      triger = a.data;
      for(int a=0;a<2;a++){
        a1[a]=0;
        a2[a]=0;
        a3[a]=0;
        a4[a]=0;
      }
      if(triger==2){
        armgoal[1]=DC2_INIT;
      }
      else if(triger==3){
        armgoal[0]=DC1_PICK1;
      }
      else if(triger==13){
        armgoal[0]=DC1_PICK2;
      }
      else if(triger==5){
        armgoal[0]=DC1_INIT;
      }
      else if(triger==6){
        armgoal[0]=DC1_PASS;
        armgoal[1]=DC2_PASS;
      }
      else if(triger=8){
        armgoal[0]=DC1_INIT;
        armgoal[1]=DC2_INIT;
      }
}

//Publisher/Subscriber
ros::Publisher wparam("wparam", &arrayp);
ros::Publisher arm_pub("armp", &armp);
ros::Publisher enc_pub("armenc", &armenc);
ros::Subscriber<std_msgs::Int8MultiArray>sub("array",messageCb);
ros::Subscriber<std_msgs::Int8> num("num",Callback);
//////////////////////////////////////////////

//arm

void calArms(){
  long arm[2];
  
  arm[0]=m5.encorder(); 
  arm[1]=m6.encorder();
  armenc.data[0]=arm[0];
  armenc.data[1]=arm[1];

  for(int i=0;i<2;i++){
    a1[i]=armgoal[i]-arm[i]; 
    a2[i]=a2[i]+a1[i];
    a3[i]=a1[i]-a4[i];
    a4[i]=a1[i];
  }
    armpw[0]=-(pic_p*a1[0]+pic_i*a2[0]+pic_d*a3[0]);
    armpw[1]=-(roll_p*a1[1]+roll_i*a2[1]+roll_d*a3[1]);
  for(int i=0;i<2;i++){
    if(abs(armpw[i])>100){
      armpw[i]=100*armpw[i]/abs(armpw[i]);
    }
    else{
    }
  }
  m5.setSpeed(armpw[0]);
  m6.setSpeed(armpw[1]);
  enc_pub.publish(&armenc);
}





//mecanum
void encorder(){
      
    Enc_b[1] = m1.encorder();
    Enc_b[2] = -m2.encorder();
    Enc_b[3] = m3.encorder();
    Enc_b[4] = -m4.encorder();
    delay(15);
    //-2,147,483,648 ～ 2,147,483,647
    //encorder foward +4-
    Enc_a[1] = m1.encorder();
    Enc_a[2] = -m2.encorder();
    Enc_a[3] = m3.encorder();
    Enc_a[4] = -m4.encorder();
  for(int i=1;i<5;i++){ 

    if(Enc_b[i]>plus){
      if(Enc_a[i]<=0&&Enc_b[i]>0){
        enc[i]=(datalength+Enc_a[i]-Enc_b[i])/n;
      }
      else{
        enc[i]=(Enc_a[i]-Enc_b[i])/n;
      }
    }
      
    //encorder back -4+ 
    else if(Enc_b[i]<minus){
      if(Enc_a[i]>=0&&Enc_b[i]<0){
        enc[i]=(-datalength+Enc_a[i]-Enc_b[i])/n;
      }
      else{
        enc[i]=(Enc_a[i]-Enc_b[i])/n;
      }
    }
    
    else{
      enc[i]=(Enc_a[i]-Enc_b[i])/n;
    }
    enc[i]=enc[i]/3;
  }
}
void cal(){
  for(int i=1;i<5;i++){
    int pid;
    e1[i]=w[i]-enc[i]; 
    e2[i]=e2[i]+e1[i];
    e3[i]=e1[i]-e4[i];
    e4[i]=e1[i];
    pid=p_gain*e1[i]+i_gain*e2[i]+d_gain*e3[i];
    pw[i]=enc[i]+pid;
    if(abs(pw[i])>100){
      pw[i]=100*pw[i]/abs(pw[i]);
    }
    else{
    }
    // pw[i]=w[i];
  }
}

void setup()
{
  arrayp.data_length=12;
  arrayp.data=(int64_t*)malloc(sizeof(int64_t*)*12);
  armp.data_length=2;
  armp.data=(int8_t*)malloc(sizeof(int8_t*)*2);
  armenc.data_length=2;
  armenc.data=(int64_t*)malloc(sizeof(int64_t*)*2);
  Wire.begin();
  //Serial.begin(9600);
  nh.initNode();
  //delay(1000);
  nh.subscribe(sub);
  nh.subscribe(num);
  //delay(1000);
  nh.advertise(arm_pub);
  nh.advertise(enc_pub);
  nh.advertise(wparam);
  //delay(1000);
}


void loop()
{
  encorder();
  cal();
  m1.setSpeed(pw[1]);
  m2.setSpeed(pw[2]);
  m3.setSpeed(pw[3]);
  m4.setSpeed(pw[4]);
  calArms();
  
  armp.data[0]=armpw[0];
  armp.data[1]=armpw[1];
  arm_pub.publish(&armp);
  
  wparam.publish( &arrayp );
  nh.spinOnce();
  delay(10);
}
