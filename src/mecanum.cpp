#include "ros/ros.h"
#include <stdlib.h>
#include <math.h>
#include "std_msgs/Int8.h"
#include "sensor_msgs/Joy.h"
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Int32MultiArray.h"
#include "std_msgs/Int8MultiArray.h"
#include <sstream>
#include "conf.h"

enum Modes {
  JOY = 0,
  FB = 1,  
};


int mode = JOY;

// publishers
ros::NodeHandle n;
ros::Publisher pub = n.advertise<std_msgs::Int8MultiArray>("mecanum_motors", 100);


//このプログラムではxboxのコントローラをつかいました。よってjoyの配列の値が少々異なります。
//ジョイスティックの可動範囲が円になっているため、spの値をジョイスティックの縦方向と横方向の移動量の和から中心からの移動量になりました。






//コメントアウトはmultipi及びmultisを読んでいるものとして書く。
//下記は平行移動の際のホイールの回転方向（１は順回転−１は逆回転）

//前
//1 1
//1 1

//右
//1-1
//-11

//左
//-11
//1-1

//後ろ
//-1-1
//-1-1

//w[]はホイールの制御量 maxは|w[]|の最大数 spは平行移動の速度（0~1） tnは回転の速度 m及びtは下の計算式参照。これにより制御の平行移動(m)と回転(t)の比重を変える。
int w[5],max,sp,tn,m,t;
//std_msgs::Int8 mv; // 0静止　1動作
std_msgs::Int8MultiArray array;


void calcMotorPower(int vertical, int horizontal, int turn) {
  /**
   * @param vertical: -1~1
   * @param horizontal: -1~1
   * @param turn: -1~1
   */
  m=50;
  t=100;
  sp=0;
  //      このプログラムでは縦方向・横方向・回転方向の制御量の和によって制御量を求めている。
  //      spに平行移動ジョイスティックの値の和*100（max200）を入れる。この必要性は後ほど
  sp=abs(100*sqrt(pow(vertical,2)+pow(horizontal,2)));
  tn=0;
  //      tnに回転方向のジョイスティックの値*100を入れる
  tn=abs(100*turn);

  //      縦方向・横方向・回転の制御量を足す。要はベクトルの合成
  w[1]=-m*vertical+m*horizontal-t*turn;
  w[2]=m*vertical+m*horizontal+t*turn;
  w[3]=m*vertical+m*horizontal-t*turn;
  w[4]=-m*vertical+m*horizontal+t*turn;
  //mv.data = 1;
  max=0;
  for(int i=1;i<5;i++){
    //		|w[]|の最大値をmaxに入れる
    if(abs(w[i])>max){
      max=abs(w[i]);
    }
    else{
    }
  }
  for(int j=1;j<5;j++){
    //		平行移動のジョイスティックが動いている時
    if(sp>5){
      //		縦方向、横方向、回転の値を足すと100以上になることがあるため（平行移動の時できるだけ早くするには縦方向と横方向の合計を100にしておきたい）、|w[]|の最大値を100に補正する。その上で上記sp/200をかけてジョイステックの操作量（縦方向操作量と横方向操作量の和）に応じて制御量がなめらかに変わるようにする。これを行わないといずれかのホイールの制御量が必ず100になっていしまう・
      w[j]=w[j]*100/max*sp/110;
      //      ROS_INFO("RUNNING &===(/^q^)/");
    }
    //		回転方向のジョイスティックのみが動いている
    else if(tn>5){
      //			書くのめんどいから上記から察して
      w[j]=w[j]*100/max*tn/200;
      //      ROS_INFO("TURNING <<\(^q^ )/>><<\( ^)/>><<\( ^q^)/>>");
    }
    else{
      w[j] = 0;
      //      ROS_INFO("IDLE (:3[__] sleep...");
    }
  }
}

void joyCallback(const sensor_msgs::Joy::ConstPtr& joy){
  //joy_nodeはジョイスティックの値およびボタンのon/offを配列でパブしている。rosrun joy joy_node　してrostopic echo joy　すれば分かる。
  //joy->axes[0]が平行移動のジョイスティックの縦方向の値　joy->axes[1]平行移動のジョイスティックの横方向の値 joy->axes[2]が回転のジョイスティックの横方向の値
  
  if (mode != JOY)
    return;  
  calcMotorPower(joy->axes[0], joy->axes[1], joy->axes[2]);
  // 右手と左手の人差し指のボタンが押されている時
  for(int j=1;j<5;j++){
  
    if(joy->buttons[4]==1&&joy->buttons[5]==0){
      w[j]=w[j]*5/10;
    }
    // 左手ボタンのみが押されている時
    else if(joy->buttons[4]==1&&joy->buttons[5]==1){
      w[j]=w[j]*3/10;
    }
    else{
      //pass
    }
  }
  
  //publish
  array.data.clear();
  array.data.push_back(w[1]);
  array.data.push_back(w[2]);
  array.data.push_back(w[3]);
  array.data.push_back(w[4]);
  pub.publish(array);
  
}

void taskCallback(const std_msgs::Int8::ConstPtr& m) {
  int r = m->data;
  if (r == ODOMRUN_LOGGING_BEGIN) {
    mode = JOY;
  } else {
    mode = FB;
  }

}



// subscriptions
ros::Subscriber joy = n.subscribe("joy",1000,joyCallback);
ros::Subscriber task = n.subscribe("task",1000,taskCallback);


int main(int argc, char **argv)
{
  ros::init(argc, argv, "mecanum");
  ros::Rate loop_rate(10);
  while (ros::ok())
    {
      ros::spinOnce();
      loop_rate.sleep();
    }


  return 0;
}
