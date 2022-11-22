//Header de declaracion de clase personalizada para atender callback
#pragma once

//Se incluyen librerias
#include <ros/ros.h>//Librerias de ROS
#include <sensor_msgs/LaserScan.h>//Librerias de mensaje de interes
#include <std_msgs/Int16.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <vector>
#define indice_0 210//190
#define indice_180 530//550
#define dist_s 0.65 //distancia de seguridad (m)
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
	ros::Publisher publisher;

	std_msgs::Int16 msg_s;//Se instancia clase de tipo de mensaje a enviar 

	void Laser_Callback(const sensor_msgs::LaserScan &msg);//funcion callback para recibir mensaje de tipo Laser

	void pose_Callback(const geometry_msgs::PoseWithCovarianceStamped &msg);//funcion callback para recibir mensaje de tipo Laser

	void Laser_Callback_vector(const std::vector<float> &msg);//funcion callback para recibir mensaje de tipo String
};

} /* namespace */
