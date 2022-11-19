//Header de definicion de clase personalizada 
#include <smb_highlevel_controller/SmbHighlevelController.hpp>//Se incluye header de declaracion de clase personalizada
#include <string>
using namespace std;

namespace smb_highlevel_controller 
{

SmbHighlevelController::SmbHighlevelController(ros::NodeHandle& nodeHandle) :
  nodeHandle(nodeHandle)//Definicion del constructor 
{
	//Configguracion de parametros
	string topico1, topico2;//nombre del topico
	int buffer_size1, buffer_size2;//tamaño del buffer

	//se obtienen parametros y se verifica el haberlos recibido
	if  ( !nodeHandle.getParam("smb_highlevel_controller/parameters/topic_name1", topico1) || !nodeHandle.getParam("smb_highlevel_controller/parameters/buffer_size1", buffer_size1) || !nodeHandle.getParam("smb_highlevel_controller/parameters/topic_name2", topico2) || !nodeHandle.getParam("smb_highlevel_controller/parameters/buffer_size2", buffer_size2))
	{
	//En caso de falla
	ROS_ERROR("no se pudieron cargar los parametros de forma exitosa");
	ros::requestShutdown();//Se finaliza el nodo
	}


	//Se crea subscriptor
	subscriber1 = nodeHandle.subscribe(topico1, buffer_size1, &SmbHighlevelController::pose_Callback, this);
	subscriber2 = nodeHandle.subscribe(topico2, buffer_size2, &SmbHighlevelController::Laser_Callback, this);
	
  	
	ROS_INFO("SmbHighlevelController cargado exitosamente");
}

SmbHighlevelController::~SmbHighlevelController()//Definicion del destructor
{
}


//Definicion de funcion callback
void SmbHighlevelController::Laser_Callback(const sensor_msgs::LaserScan &msg)
{	
	//int cuenta = 3;
	//ROS_INFO_STREAM(msg.data << cuenta);//se imprime dato recibidio

	auto min_dis = std::min_element(msg.ranges.cbegin(), msg.ranges.cend());
	ROS_INFO_STREAM("Distancia min (m): " << *min_dis);

	ROS_INFO_STREAM("SCAN: " << msg.angle_increment);
}

/*
void SmbHighlevelController::Laser_Callback_vector(const std::vector<float> &msg)
{	
	ROS_INFO_STREAM("SCAN: " << msg);
}
*/

void SmbHighlevelController::pose_Callback(const geometry_msgs::PoseWithCovarianceStamped &msg)
{
	ROS_INFO_STREAM("Posición en x: " << msg.pose.pose.position.x);//se imprime dato recibidio
	ROS_INFO_STREAM("Posición en y: " << msg.pose.pose.position.y);
	ROS_INFO_STREAM("Orientación: " << msg.pose.pose.orientation.z);

	//ROS_INFO_STREAM(msg.pose.position);
}


}
