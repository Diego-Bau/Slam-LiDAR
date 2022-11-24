//Header de declaracion de clase personalizada para atender callback
#pragma once

//Se incluyen librerias
#include <ros/ros.h>//Librerias de ROS
#include <sensor_msgs/LaserScan.h>//Librerias de mensaje de interes
#include <std_msgs/Int16.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Twist.h>
#include <gazebo_msgs/ModelStates.h>
#include <vector>
#include <cmath>
#define indice_0 200//250   //210 //190
#define indice_180 510//500  //530//550
#define dist_s .4 //distancia de seguridad (m)
#define P_v 1 //ganancia proporcional de velocidad traslacional
#define P_w 1 //ganancia proporcional de velocidad angular
#define pi 3.14159265359 //ganancia proporcional de velocidad
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

	//Funcion para especificar velocidad tralacional y angular del robot
	void set_vel(const float &vel, const std::string &dof); //función para calculos de control

	//Funcion para publicar mensaje a topico /cmd_vel --> twist vector de velocidades traslacionales y angulares en cada eje
	void Drive_robot();			//función para publicar


	//Funcion para visualizar pilar con un marcador en Rviz
	void viz_marker();

private: //--> atributos 

	ros::NodeHandle nodeHandle;//objeto de clase nodehandle
	ros::Subscriber subscriber1;//objeto de clase subscriber1 (pos)
	ros::Subscriber subscriber2;//objeto de clase subscriber2 (scan)
	ros::Subscriber subscriber3;//objeto de clase subscriber2 (pos_marker)
	ros::Publisher publisher, viz_pub;
	visualization_msgs::Marker marker;//mensaje de tipo marker
	//std_msgs::Int16 msg_s;//Se instancia clase de tipo de mensaje a enviar 
	geometry_msgs::Twist msg_v;

	float marker_pos[2];//posicion del pilar 


	void Laser_Callback(const sensor_msgs::LaserScan &msg);//funcion callback para recibir mensaje de tipo Laser

	void pose_Callback(const geometry_msgs::PoseWithCovarianceStamped &msg);//funcion callback para recibir mensaje de tipo Laser

	void marker_Callback(const gazebo_msgs::ModelStates &msg);//funcion callback para recibir mensaje de tipo marker position

	void Laser_Callback_vector(const std::vector<float> &msg);//funcion callback para recibir mensaje de tipo String

	void init_Marker();//inicializador del marker en rviz

};

} /* namespace */
