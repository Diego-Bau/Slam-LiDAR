//Header de definicion de clase personalizada 
#include <smb_highlevel_controller/SmbHighlevelController.hpp>//Se incluye header de declaracion de clase personalizada
#include <string>
using namespace std;

namespace smb_highlevel_controller 
{

SmbHighlevelController::SmbHighlevelController(ros::NodeHandle& nodeHandle) :
  nodeHandle(nodeHandle),msg_s(), subscriber1(), subscriber2(), publisher()//Definicion del constructor 
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
  publisher = nodeHandle.advertise<std_msgs::Int16>("chatter", 1);

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
	int indice = min_dis - msg.ranges.cbegin();
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
void SmbHighlevelController::pose_Callback(const geometry_msgs::PoseWithCovarianceStamped &msg)
{

	//ROS_INFO_STREAM("Posicion en x: " << msg.pose.pose.position.x);//se imprime dato recibidio
	//ROS_INFO_STREAM("Posicion en y: " << msg.pose.pose.position.y);
	//ROS_INFO_STREAM("Orientacion: " << msg.pose.pose.orientation.z);

	//ROS_INFO_STREAM(msg.pose.position);
}


}
