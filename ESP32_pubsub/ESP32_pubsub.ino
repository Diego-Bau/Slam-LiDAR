/*Código para comunicarse vía serial con un nodo de ROS
 * Este script es un subscriptor
 * Para que este nodo funcione tiene que ejecutarse el node serial subscriber en ROS a la par de cargarse el código en la ESP
 */

//Librerías para la comunicación serial
#include <ros.h>//Librerías de ROS
#include <std_msgs/String.h>//Librería del tipo de mensaje a recibir


ros::NodeHandle  nh;//Se inicializan comunicaciones del nodo

//Función de callback
void messageCb( const std_msgs::String& toggle_msg){
  digitalWrite(2, HIGH-digitalRead(2));   //Al recibr dato prende y apaga led --> acknowledge
}

ros::Subscriber<std_msgs::String> sub("chatter", messageCb );//Se configura el subscriptor

void setup()
{
  pinMode(2, OUTPUT);//Se configura led para acknowledge
  nh.initNode();//Se da de alta el nodo ante el maestro
  nh.subscribe(sub);//Se da de alta el subscriptor
}

void loop()
{
  nh.spinOnce();
}
