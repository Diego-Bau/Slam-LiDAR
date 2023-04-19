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
<<<<<<< HEAD
	
=======

	/*
>>>>>>> 7691c7499d51be87efc09bc17cc864b01d23615c
	//int cuenta = 3;
	//ROS_INFO_STREAM(msg.data << cuenta);//se imprime dato recibidio

	auto min_dis = std::min_element(msg.ranges.cbegin(), msg.ranges.cend());
	int indice = min_dis - msg.ranges.cbegin();
<<<<<<< HEAD
	ROS_INFO_STREAM("Distancia min (m): " << *min_dis);
	ROS_INFO_STREAM("INDICE: " << indice);
	//ROS_INFO_STREAM("promedioEnf: " << msg.ranges.size());

unsigned int longitudDeArreglo = RANGO_I * 2 + 1;
unsigned int longitudDeArregloE = RANGO_E * 2 + 1;

//Declaración de vectores de detección de distancias
float v_izquierda[longitudDeArreglo];
float v_derecha[longitudDeArreglo];
float v_enfrente[longitudDeArregloE];

//Declaración de indicadores de detección de paredes
bool p_der = false, p_izq = false, p_front = false;
=======
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
>>>>>>> 7691c7499d51be87efc09bc17cc864b01d23615c

//Adquisición de datos del vector de lidar
for (int i=0; i <= RANGO_I * 2 + 1; i++){
	v_izquierda[i] = msg.ranges[IZQUIERDA_I - RANGO_I + i];
	v_derecha[i] = msg.ranges[DERECHA_I - RANGO_I + i];

}
/*
for (int i=0; i <= RANGO_E * 2 + 1; i++){
		v_enfrente[i] = msg.ranges[ENFRENTE_I - RANGO_E + i];
	}
*/

// PROMEDIO DE DATOS
  double sumatoriaIzq, sumatoriaDer, sumatoriaEnf, promedioIzq, promedioDer, promedioEnf;
  int inf_Izq=0, inf_Der=0, inf_Enf= 0;

  for (int x = 0; x < longitudDeArreglo; x++) {
  	
  	if (v_izquierda[x]>=15 || v_izquierda[x]<=0.1){
  		v_izquierda[x]=0;
  		inf_Izq++;
  	}

  	if (v_derecha[x]>=15 || v_derecha[x]<=0.1){
  		v_derecha[x]=0;
  		inf_Der++;
  	}
  	sumatoriaIzq += v_izquierda[x];
    sumatoriaDer += v_derecha[x];

  }
  /*
  for (int x = 0; x < longitudDeArregloE; x++) {
	  	if (v_enfrente[x]>=15 || v_enfrente[x]<=0.1){
			//ROS_INFO_STREAM("si estoy entrando a la condicional");
	  		v_enfrente[x]=0;
	  		inf_Enf++;
	  	
	  	}
	  	sumatoriaEnf += v_enfrente[x];
  }
  */

  if((indice<=ENFRENTE_I + RANGO_E) && (indice>=ENFRENTE_I - RANGO_E ))
  {
  	
  	if (*min_dis<=dis_paredE)
  	{
  		p_front = true;
  	}
  	
  	else{
  		 p_front = false;
  	}

  }
  
  // El promedio es la sumatoria dividida entre el número de elementos
  promedioIzq = (longitudDeArreglo - inf_Izq == 0)? 0.12 : sumatoriaIzq / (longitudDeArreglo - inf_Izq);
  promedioDer = (longitudDeArreglo - inf_Der == 0)? 0.12: sumatoriaDer / (longitudDeArreglo - inf_Der);
  //promedioEnf = (longitudDeArregloE - inf_Enf == 0)? *min_dis: sumatoriaEnf / (longitudDeArregloE - inf_Enf);
  //ROS_INFO_STREAM("prom_enf: " << promedioEnf);
  ROS_INFO_STREAM("prom_izq: " << promedioIzq);
  ROS_INFO_STREAM("prom_der: " << promedioDer);


	if(promedioIzq <= dis_pared) p_izq = true;//se detecta pared izquierda
	if(promedioDer <= dis_pared) p_der = true;//se detecta pared derecha
	//if(promedioEnf <= dis_pared) p_front = true;//se detecta pared frontal obstaculo

	ROS_INFO_STREAM("pIzq: " << p_izq);
	ROS_INFO_STREAM("pDer: " << p_der);
	ROS_INFO_STREAM("pEnf: " << p_front);

	if (p_der && p_front==0) 
{
msg_s.data=1; //Avanza
//ROS_INFO_STREAM("AVANZA");
ROS_INFO_STREAM("1");
}

	if (p_der && p_front) 
{
msg_s.data=2; //Gira izquierda
//ROS_INFO_STREAM("IZQUIERDA");
ROS_INFO_STREAM("2");
}
	if (!p_der) 
{
msg_s.data=3; //gira Derecha
//ROS_INFO_STREAM("DERECHA");
ROS_INFO_STREAM("3");
}
	/*
	else
	{
		//ROS_INFO_STREAM("AVANZA");
<<<<<<< HEAD
		msg_s.data = 0;//Se especifica mensaje a enviar
		//ROS_INFO_STREAM("DETENTE");

	}
	*/
	publisher.publish(msg_s);
	//ROS_INFO_STREAM("SCAN: " << msg.angle_increment);

/*

/*
void SmbHighlevelController::Laser_Callback_vector(const std::vector<float> &msg)
{	
	ROS_INFO_STREAM("SCAN: " << msg);
}
*/
}
=======
		msg_s.data = 1;//Se especifica mensaje a enviar 
	}
	//publisher.publish(msg_s);
	
	//ROS_INFO_STREAM("SCAN: " << msg.angle_increment);
}
*/
}	

>>>>>>> 7691c7499d51be87efc09bc17cc864b01d23615c
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
