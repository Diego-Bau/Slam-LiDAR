/*
 * rosserial Publisher Example
 * Prints "hello world!"
 */

#include <ros.h>
#include <std_msgs/String.h>

ros::NodeHandle  nh;

std_msgs::String str_msg;
//ros::Publisher chatter("chatter", &str_msg);

char hello[13] = "hello world!";

void messageCb( const std_msgs::String& msg){
  String target = msg.data;
  Serial.println(target);
  
  if (target =="H"){
     digitalWrite(2, HIGH);
  }
}

ros::Subscriber<std_msgs::String> sub("chatter", &messageCb);

void setup()
{
  nh.initNode();
  nh.subscribe(sub);
  //nh.advertise(chatter);
  Serial.begin(57600);
  pinMode(2, OUTPUT);
}

void loop()
{
  str_msg.data = hello;
  //chatter.publish( &str_msg );

  
  nh.spinOnce();
  delay(1000);
}
