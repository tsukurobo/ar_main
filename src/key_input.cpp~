#include "ros/ros.h"
#include "std_msgs/Char.h"
#include <ctype.h>
using namespace std;

std_msgs::Char key_msg;
int main(int argc, char **argv)
{
  ros::init(argc, argv, "tr_key");
  ros::NodeHandle nh;
  ros::Publisher  tr_key_pub = nh.advertise<std_msgs::Char>("key_input",10);
  ros::Rate rate(10);
  std::cout << "tr_key setup" << std::endl;

  while (ros::ok())                              
    {
      char key;
      cin >> key;
      cout << key << endl;

      //alphabet only
      if (isalpha(key)){
	key_msg.data = key;
      } else {
	//pass
      }
      
      if (key_msg.data != '\0')
	{
	  tr_key_pub.publish(key_msg); // メッセージを送信
	}
      ros::spinOnce();
      key_msg.data = '\0';
      rate.sleep();     
    }

  return 0;
}

