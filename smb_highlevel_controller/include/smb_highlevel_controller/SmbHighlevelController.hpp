//Header de declaracion de clase personalizada para atender callback
#pragma once

//Se incluyen librerias
#include <ros/ros.h>//Librerias de ROS
#include <sensor_msgs/LaserScan.h>//Librerias de mensaje de interes
#include <std_msgs/String.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <vector>

namespace smb_highlevel_controller {

/*!
 * Class containing the Husky Highlevel Controller
 */
class SmbHighlevelController { //Clase de controlador
public://--> metodos
	/*!
	 * Constructor.
	 */
	SmbHighlevelController(ros::NodeHandle& nodeHandle);//Constructor, recibe como entrada el objeto de clase nodehandle

	/*!
	 * Destructor.
	 */
	virtual ~SmbHighlevelController();//Destructor

private: //--> atributos 

	ros::NodeHandle nodeHandle;//objeto de clase nodehandle
	ros::Subscriber subscriber1;//objeto de clase subscriber1 (pos)
	ros::Subscriber subscriber2;//objeto de clase subscriber2 (scan)

	void Laser_Callback(const sensor_msgs::LaserScan &msg);//funcion callback para recibir mensaje de tipo Laser

	void pose_Callback(const geometry_msgs::PoseWithCovarianceStamped &msg);//funcion callback para recibir mensaje de tipo Laser

	void Laser_Callback_vector(const std::vector<float> &msg);//funcion callback para recibir mensaje de tipo String
};

} /* namespace */