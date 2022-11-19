#include <ros/ros.h>
#include "smb_highlevel_controller/SmbHighlevelController.hpp"
#include <iostream>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/String.h>
#include <string>
using namespace std;

/*void laser_callback(const std_msgs::String& msg)
{
	int num = 2;
	ROS_INFO_STREAM(msg.data << num);

}*/

int main(int argc, char** argv)
 {
  	int buffer_size;//Tamaño de buffer
  	string topico;//nombre de topic
	
	//Configuracion
  	ros::init(argc, argv, "smb_highlevel_controller");//Se da de alta el nodo ante el maestro
 	 ros::NodeHandle nodeHandle;//Se inicializan las comunicaciones
  	
	/*nodeHandle.getParam("smb_highlevel_controller/parameters/buffer_size", buffer_size);//tamaño del buffer
  	nodeHandle.getParam("smb_highlevel_controller/parameters/topic_name", topico);//se obtiene parametro del topico a subscribirse

	ros::Subscriber subscriber =  nodeHandle.subscribe(topico, buffer_size, laser_callback);*/

 	 smb_highlevel_controller::SmbHighlevelController smbHighlevelController(nodeHandle);//Se instancia clase personalizada para hacer el callback

  ros::spin();
  return 0;
}
