#include "ros/ros.h"
#include <stdlib.h>
#include <math.h>
#include "std_msgs/Int8.h"
#include "sensor_msgs/Joy.h"
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Int16MultiArray.h"
#include "conf.h"


// params
const int servoSum = 7;
const int closeDeg = 0; // close deg
const int openDeg = 90;   // open deg
const int hz = 10;
const int b = 0, c = 1, d = 2, e = 3, f = 4, g = 5, h = 6,pick_up=7,pick_down=8,flag=9;
const int delaySmall = 1000, delayMedium = 3000, delayLong = 1500, delayShot = 5000, delayReset = 3000, delay2000 = 2000;

// inner values
bool delaying = false; // for delay
int delayCounter = 0;
int state = SERVO_WAIT;
int wait;

// ros values
ros::Publisher pub;
ros::Publisher servoPub;
ros::Subscriber joy;
ros::Subscriber servoSub;
std_msgs::Int16MultiArray array;
std_msgs::Int8 servop;

// =========== callback ==========
void servoTaskCallback(const std_msgs::Int8::ConstPtr& m){
  //ROS_INFO("stcb");
  state = m->data;
}

void joyCallback(const sensor_msgs::Joy::ConstPtr& joy){
	wait=joy->buttons[1];
}

// ===========sub func==========
void sendArr(int servoNo, int degree) {
  array.data.clear();
  array.data.push_back(servoNo);
  array.data.push_back(degree);
  pub.publish(array);
}

void sOpen(int servoNo) {
  sendArr(servoNo, openDeg);  
}

void sClose(int servoNo) {
  sendArr(servoNo, closeDeg);  
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

// =========== routine ==========
void setup() {
  ROS_INFO("setup");
  int i = 0;
  for (i = 0; i < servoSum; i++) {
    sClose(i);
  }
}

void prepare() {
  // add pressure
  static int mode = -1;
   if(mode==-1){
   sendArr(7,30);
   sendArr(8,30);
   delay(delaySmall);
   mode=0;
   }
   else if (mode == 0) {
    ROS_INFO("prepare");
    sOpen(e);
    delay(delaySmall);
    mode = 1;
  } else if (mode == 1) {
    sOpen(b);
    delay(delaySmall);
    mode = 2;
  } else if (mode == 2) {
    sOpen(f);
    delay(delayMedium);
    mode = 3;
  } else if (mode == 3) {
    sClose(e);
    sClose(f);
    delay(delaySmall);
    mode=4;
  }else if(mode==4){
    sOpen(h);
    mode = -1;
    state = SERVO_WAIT;
    servop.data=state;
    servoPub.publish(servop);
  } 
  
}

void tz1(){
    static int mode = 0;
    if (mode == 0) {
    //flagup
      sOpen(flag);
      delay(delaySmall);
      
      ROS_INFO("tz1");
      // shot
      sOpen(d);
      delay(delayShot);
      //flagdown
      sClose(flag);
      //wait4user
      while(wait==0){
      	ros::spinOnce();
      }
      mode = 1;
    } else if (mode ==1) {
      // shot done
      sClose(d);
      sClose(h);
      delay(delay2000);
      mode = 2;
    } else if (mode == 2) {
      // prepare exhaust
      sOpen(e);
      delay(delaySmall);
      mode = 3;
    } else if ( mode == 3) {
      // reset the arm
      sOpen(f);
      delay(5000);
      mode = 4;
    } else if (mode == 4) {
      // reset done
      sClose(f);
      sClose(e);
      delay(delaySmall);
      mode = 5;
    } else if (mode == 5) {
      // prepare exhausting. close 0.45mp
      sOpen(h);
      delay(500);
      mode = 6;
    } else if (mode == 6) {
      sClose(b);
      delay(delay2000);
      mode = 7;
    } else if (mode == 7) {
      // prepare 0.55mp.
      sOpen(c);
      mode = 0;
      state = SERVO_WAIT;
      servop.data=state;
      servoPub.publish(servop);
    }
}

void tz2(){
    static int mode = 0;
    if (mode == 0) {
     //flagup
      sOpen(flag);
      delay(delaySmall);
      
      ROS_INFO("tz2");
      // shot
      sOpen(d);
      delay(delayShot);
      //flagdown
      sClose(flag);
      //wait4user
      while(wait==0){
      	ros::spinOnce();
      }
      mode = 1;
    } else if (mode ==1) {
      // shot done
      sClose(d);
      sClose(h);
      delay(delay2000);
      mode = 2;
    } else if (mode == 2) {
      // prepare exhaust
      sOpen(e);
      delay(delaySmall);
      mode = 3;
    } else if ( mode == 3) {
      // reset the arm
      sOpen(f);
      delay(delayReset);
      mode = 4;
    } else if (mode == 4) {
      // reset done
      sClose(f);
      delay(delay2000);
      mode = 5;
    } else if (mode == 5) {
      sClose(e);
      sOpen(h);
      delay(delaySmall);
      mode = 6;
    } else if (mode == 6) {
      sClose(c);
      delay(delay2000);
      mode = 7;
    } else if (mode == 7) {
      // prepare 0.71
      sOpen(g);
      mode = 0;
      state = SERVO_WAIT;
      servop.data=state;
      servoPub.publish(servop);
    } 
}

void tz3() {
    static int mode = 0;
    if (mode == 0) {
      //flagup
      sOpen(flag);
      delay(delaySmall);
      
      ROS_INFO("tz3");
      // shot
      sOpen(d);
      delay(delayShot);
      //flagdown
      sClose(flag);
      //wait4user
      while(wait==0){
      	ros::spinOnce();
      }
      mode = 1;
    } else if (mode ==1) {
      // shot done
      sClose(d);
      delay(delaySmall);
      mode = 2;
    } else if (mode == 2) {
      // prepare exhaust
      sClose(h);
      sOpen(e);
      delay(delay2000);
      mode = 3;
    } else if ( mode == 3) {
      // reset the arm
      sOpen(f);
      delay(delayReset);
      mode = 4;
    } else if (mode == 4) {
      // reset done
      sClose(f);
      delay(delay2000);
      mode = 5;
    } else if (mode == 5) {
      sClose(e);
      sOpen(h);
      mode = 0;
      state = SERVO_WAIT;
      servop.data=state;
      servoPub.publish(servop);
    }

}

void close(){
	int dc =10;
	sClose(b);
	delay(dc);
	sClose(c);
	delay(dc);
	sClose(d);
	delay(dc);
	sClose(e);
	delay(dc);
	sClose(f);
	delay(dc);
	sClose(g);
	delay(dc);
	sClose(h);
	state = SERVO_WAIT;
        servop.data=state;
        servoPub.publish(servop);
}

void free(){
	sendArr(pick_up,30);
	sendArr(pick_down,30);
	state = SERVO_WAIT;
	servop.data=state;
        servoPub.publish(servop);
}

void pick(){
	sendArr(pick_up,0);
	sendArr(pick_down,0);
	state = SERVO_WAIT;
	servop.data=state;
        servoPub.publish(servop);
}



void task() {
  //  ROS_INFO("task");
  if (state == SERVO_WAIT) {
	ROS_INFO("Wait\n");
    //pass
  } else if (state == SERVO_PREPARE) {
    ROS_INFO("prepare\n");
    prepare();
  } else if (state == SERVO_TZ1SHOT) {
  ROS_INFO("Tz1\n");
    tz1();
  } else if (state == SERVO_TZ2SHOT) {
  ROS_INFO("Tz2\n");
    tz2();
  } else if (state == SERVO_TZ3SHOT) {
  ROS_INFO("Tz3\n");
    tz3();
  } else if (state == SERVO_PICK) {
  ROS_INFO("pick\n");
    pick();
  } else if (state == SERVO_FREE) {
  ROS_INFO("free\n");
    free();
  } else if (state == SERVO_CLOSE) {
  ROS_INFO("close\n");
    close();
  }
  
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "servo_handler");
  ros::NodeHandle n; 
  pub = n.advertise<std_msgs::Int16MultiArray>("servo_deg", 100);
  joy = n.subscribe("joy",1000,joyCallback);
  servoPub = n.advertise<std_msgs::Int8>("servo_task", 100);
  servoSub = n.subscribe("servo_task", 100, servoTaskCallback);
  ros::Rate loop_rate(hz);
  setup();
  
  
  while (ros::ok())
    {
      ros::spinOnce();
      if (delaying) {
	delayCount();
      } else {
	task();
      }
      loop_rate.sleep();
    }

  return 0;
}




/*

// =====圧力をかける。=====
// 圧力をかける
h open;
delay(500);
b open;
delay(500);
f open;
delay(1000);
h, f close;// 圧力を止める。
delay(500);
e open

// result, b,e is opening, other close. -> tz1へ。


// ===== tz1の射出 =====
//at tz1
d open;    // shot
delay(5000);
d, e close;  // shot done
delay(500);
h open;
delay(500);
f open; // 構える位置に戻す。
delay(3000);
f, h close; // 戻った
delay(500);
e open; b close; // 0.45mpの圧力を閉じて排気の用意
delay(500);
# a open;  // 排気
# delay(1500);
# a close; // 排気完了
# delay(500);
c open;  // 0.55mpの圧力を準備。 = tz2の準備完了。
// e, c open, other close; -> tz2

// ===== tz2の射出 =====
d open; //shot
delay(5000);
d close; e close; //shot done
delay(500);
h open; // エアシリンダの排気用意
delay(500);
f open; // エアシリンダを初期位置に戻す。
delay(3000);
f close; // エアシリンダを初期位置に戻すが完了
delay(500);
h close; e open; c close; // h closeで排気をやめる。 e openでエアシリンダの排気用意。 cは0.55mpのタンクを閉じる。
delay(500);
# a open; // 0.55mpになっているのをリセット。
# delay(500);
# a close; //リセット完了
# delay(500);
g open; // 0.71mp(tz3用に全体を調整)=tz3に投げる用意が完了。
delay(500);
// e, g open, other close -> tz3


// ===== tz3の射出 =====
d open; // shot
delay(5000);
d close; // shot done
delay(500);
e close; h open; // eで排気を止める。 hで腕を曲げる排気の用意。
delay(500);
f open; // 腕を初期位置に戻す。
delay(3000);
f close; // 腕の位置初期化完了
delay(500);
e open;

done;;;


 */
