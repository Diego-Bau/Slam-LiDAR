//Header de definicion de clase personalizada 
#include <smb_highlevel_controller/SmbHighlevelController.hpp>//Se incluye header de declaracion de clase personalizada
#include <string>
using namespace std;

namespace smb_highlevel_controller 
{

SmbHighlevelController::SmbHighlevelController(ros::NodeHandle& nodeHandle) :
  nodeHandle(nodeHandle),msg_v(), subscriber1(), subscriber2(), subscriber3(), publisher(), viz_pub()//Definicion del constructor 
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

	//Se crea publisher
  publisher = nodeHandle.advertise<geometry_msgs::Twist>("chatter", 1);
  viz_pub = nodeHandle.advertise<visualization_msgs::Marker>("visualization_marker", 0);//se crea publisher para

  	//Marker del pilar en Rviz
	init_Marker();//se inicializa el marker

	//Se crea subscriptor
	subscriber1 = nodeHandle.subscribe(topico1, buffer_size1, &SmbHighlevelController::pose_Callback, this);
	subscriber2 = nodeHandle.subscribe(topico2, buffer_size2, &SmbHighlevelController::Laser_Callback, this);
	subscriber3 = nodeHandle.subscribe("gazebo/model_states", 1, &SmbHighlevelController::marker_Callback, this);

  	
	ROS_INFO("SmbHighlevelController cargado exitosamente");
}

SmbHighlevelController::~SmbHighlevelController()//Definicion del destructor
{
}


//Definicion de funcion callback
void SmbHighlevelController::Laser_Callback(const sensor_msgs::LaserScan &msg)
{	

	/*
	//int cuenta = 3;
	//ROS_INFO_STREAM(msg.data << cuenta);//se imprime dato recibidio

	auto min_dis = std::min_element(msg.ranges.cbegin(), msg.ranges.cend());
	int indice = min_dis - msg.ranges.cbegin();
	//ROS_INFO_STREAM("Distancia min (m): " << *min_dis);
	//ROS_INFO_STREAM("INDICE: " << indice);

	if (indice>=indice_0 && indice<=indice_180) //es el rango de 180° frontal
	{
		if (*min_dis<=dist_s)
		{
			//ROS_INFO_STREAM("DETENTE");
			msg_s.data = 3;//Se especifica mensaje a enviar 

		}
		else{
			//ROS_INFO_STREAM("AVANZA");
			msg_s.data = 1;//Se especifica mensaje a enviar

					}
	}
	else
	{
		//ROS_INFO_STREAM("AVANZA");
		msg_s.data = 1;//Se especifica mensaje a enviar 
	}
	//publisher.publish(msg_s);
	
	//ROS_INFO_STREAM("SCAN: " << msg.angle_increment);
}
*/
}	

void SmbHighlevelController::pose_Callback(const geometry_msgs::PoseWithCovarianceStamped &msg)
{

	//ROS_INFO_STREAM("Posicion en x: " << msg.pose.pose.position.x);//se imprime dato recibidio
	//ROS_INFO_STREAM("Posicion en y: " << msg.pose.pose.position.y);
	//ROS_INFO_STREAM("Orientacion: " << msg.pose.pose.orientation.z);

	//ROS_INFO_STREAM(msg.pose.position);

	//variables de posición robot
	double posi_robot[2]= {msg.pose.pose.position.x, msg.pose.pose.position.y}; // posición del robot
 	double t= msg.pose.pose.orientation.z; //theta robot
	//Se calcula el error
	double d= sqrt(pow(marker_pos[0]- posi_robot[0],2)+pow(marker_pos[1]-posi_robot[1],2)); //distancia del error
	double td= atan2(marker_pos[1]-posi_robot[1], marker_pos[0]-posi_robot[0]);  //theta del error

	double te= t - td;

	//CONDICIONALES PARA TOMAR EL ANGULO MÁS PEQUEÑO. (SE ARREGLA LA DISCONTINUIDAD DE LA TANGENTE INVERSA).
	if (te > pi)  
	{
		te = te-2*pi;
	}

	else if (te < -pi)
	{
		te = te + 2*pi;
	}

	double v= P_v * d;     //control proporcional de la velocidad traslacional
	double omega= -P_w * te;     //control proporcional de la velocidad angular

	ROS_INFO_STREAM("velocidad: " << v);
	ROS_INFO_STREAM("omega: " << omega);

	msg_v.linear.x= v;
	msg_v.angular.z= omega;

	publisher.publish(msg_v);

}
void SmbHighlevelController::marker_Callback(const gazebo_msgs::ModelStates &msg)
{
	marker_pos[0]= msg.pose[1].position.x;
	marker_pos[1]= msg.pose[1].position.y;

	viz_marker();
	//ROS_INFO_STREAM("POSE: " << msg.pose[1].position.x);
}

void SmbHighlevelController::viz_marker()
{
	marker.header.frame_id = "map";
	marker.header.stamp = ros::Time();
	marker.ns = "pilar";
	marker.id = 1080;
	marker.type = visualization_msgs::Marker::SPHERE;
	marker.action = visualization_msgs::Marker::ADD;

	marker.pose.position.x = marker_pos[0];//posicion en x
	marker.pose.position.y = marker_pos[1];//posicion en y 
	marker.pose.position.z = 0.1;//posicion en z
	viz_pub.publish(marker);//se publica mensaje de posicion 
}


void SmbHighlevelController::init_Marker()
{
	marker.header.frame_id = "maps";
	marker.header.stamp = ros::Time();
	marker.ns = "pilar";
	marker.id = 1080;
	marker.type = visualization_msgs::Marker::SPHERE;
	marker.action = visualization_msgs::Marker::ADD;
	//posicion
	marker.pose.position.x = 5 ;//coordenada en x
	marker.pose.position.y = 0 ;//coordenada en y 
	marker.pose.position.z = 0.1;//coordenada en z
	
	//escala
	marker.scale.x = .2;//escala en x
	marker.scale.y = .2;//escala en y 
	marker.scale.z = .2;//escala en z
	
	//color
	marker.color.a = 1.0;//alpha
	marker.color.r = 1.0;//canal rojo
	marker.color.g = 1.0;//canal verde
	marker.color.b = 0.0;//canal azul	
	viz_pub.publish(marker);//se publica mensaje de posicion 	
}

}
