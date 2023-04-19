/*Header para la comunicación bluetooth con el control en telefono 
 * --> especifica la referencia vía bluetooth por medio de comandos en el control
*/
//Librerías
#include <Arduino.h>
#include "BluetoothSerial.h"
#include <stdlib.h>

//Parametros del sistema físico del robot 
#define rpm_max 209//rpm máx de los motores
#define rpm_min 19//rpm min de los motores
#define omegar_max 21.8//velocidad angular máxima de los motores rad/s
#define omegar_min 1.98//velocidad angular mínima de los motores rad/s 
#define v_max .875 //velocidad traslacional máxima del sistema (m/s) (absoluto)
#define v_min .0079//velocidad traslacional mínima del sistema (m/s) (absoluto)
#define omega_max 2.96 //velocidad angular máxima del sistema (rad/s) (absoluto)
#define omega_min .269 //velocidad angular mínima del sistema (rad/s) (absoluto)
#define l .1475 //distancia del centro geometrico del robot al centro de cada rueda en m
#define r .04 //radio de la rueda del motor en m
#define factor_limit 0.75//factor para limitar las velocidades traslacionales y angulares al girar
#define porcentaje_v 15 //20

//Definición de variables globales
char data;//datos recibidos por bluetooth
float omegad_m1;//velocidad angular deseada para el motor 1
float omegad_m2;//velocidad angular deseada para el motor 2
float vd;//velocidad traslacional deseada o de referencia
float omegad;//velocidad angular deseada o de referencia 

//Prototipo de funciones
float cinematica_rdiferencial(int, float, float);//función para el calculo de cinemática
