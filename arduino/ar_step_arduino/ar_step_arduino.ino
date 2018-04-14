#include <ros.h>

#include <std_msgs/Int8.h>

#include <std_msgs/Int16.h>


#define WHEEL_DIAMETER 25//だいたい

#define STEP 1.8

#define STEPS 200// 360 / STEP (360度の回転に何パルス必要か

#define RPM 30 //分あたりの回転数

#define PULSE_ROLL (WHEEL_DIAMETER * 3.14 * STEP / 360)//１パルスあたり

#define STEPPING_MOTOR_SUM 2

#define DEFAULTPW 80

#define PULSEPIN 2

#define DIRPIN 3

volatile int input_pulse = 0;

ros::NodeHandle nh;

void stpaCallback(const std_msgs::Int16& a){
     
     input_pulse = a.data/PULSE_ROLL;
     
     
}

ros::Subscriber<std_msgs::Int16> stpa("stpa",stpaCallback);

void pulse(){
  
  static  boolean output = HIGH;
 
  
  digitalWrite(PULSEPIN,output);//パルスをHIGH
  
  output = !output;
  
     
    
    
  
  }







void test(int b){
     int i,j = 0;
     static int count = 0;

     if(b>=0){
      Serial.print("+");
      
      digitalWrite(DIRPIN,HIGH);
      
      
      pulse();

      count++;
  
     
      
      
      }

     if(b<0){
      Serial.print("-");
      b= -b;
      digitalWrite(DIRPIN,LOW);
      
      
      pulse();
  
      count++;
      
      
      }

     if(count>b*2){
      
      count = 0;

      while(1){
        nh.spinOnce();

        static int num2 = 0;
        int num1 = input_pulse;

        if(num1 != num2 ){
          num2 = num1;

          break;
          
          }
        
        num2 = num1;
        
        
        }
      
      
     
      }


}










void setup(){
  pinMode(DIRPIN,OUTPUT);
  pinMode(PULSEPIN,OUTPUT);
  digitalWrite(DIRPIN,HIGH);
  digitalWrite(PULSEPIN,LOW);
  Serial.begin(9600);

  
  
  nh.initNode();

  nh.subscribe(stpa);


}



void loop(){
  Serial.print(1);

  test(input_pulse);


  delayMicroseconds(1500);

  nh.spinOnce();

}
