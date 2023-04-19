//Librerías para la comunicación serial
#include <ros.h>//Librerías de ROS
#include <std_msgs/Int16.h>//Librería del tipo de mensaje a recibir
#include "Control_PID.h"//Parámetros del control PID
#include "Bluetooth.h"//Parametros del sistema físico del robot 

ros::NodeHandle  nh;//Se inicializan comunicaciones del nodo

//Función de callback
void messageCb( const std_msgs::Int16& toggle_msg){
  digitalWrite(2, HIGH-digitalRead(2));   //Al recibr dato prende y apaga led --> acknowledge
  int mensaje=  toggle_msg.data;

  //switch case control remoto
  
  switch (mensaje)
  {
    case 0://Stop el robot no avanza --> no se presiona ningún botón 
      vd = 0;//velocidad traslacional deseada
      omegad = 0;//velocidad angular deseada
      break;
  
    //Casos de rotar 
    case 2://El robot gira hacia la derecha --> no avanza solo rota en sentido horario (positivo)
      vd = 0;//velocidad traslacional deseada
      omegad = (porcentaje_v * omega_max)/100;;//velocidad angular deseada
      break;
  
     case 3://El robot gira hacia la izquierda --> no avanza solo rota en sentido antihorario (negativo)
      vd = 0;//velocidad traslacional deseada
      omegad = -(porcentaje_v * omega_max)/100;;//velocidad angular deseada
      break;
  
    //Referencias positivas
    case 1://Forward el robot avanza conforme al %de velocidad especificado, solo avanza no rota
      vd = (porcentaje_v * v_max)/100;//velocidad traslacional deseada
      omegad = 0;//velocidad angular deseada
      break;
    default:
      vd = 0;//velocidad traslacional deseada
      omegad = 0;//velocidad angular deseada 
      break;
  }
}

ros::Subscriber<std_msgs::Int16> sub("chatter", messageCb );//Se configura el subscriptor

//Configuración de ESP32
void setup()
{
  //ros
  pinMode(2, OUTPUT);//Se configura led para acknowledge
  nh.initNode();//Se da de alta el nodo ante el maestro
  nh.subscribe(sub);//Se da de alta el subscriptor
  
  //Motor1
  //Pin para canal A encoder 1
  pinMode(pin_cA_e1, INPUT); //se configura pin como entrada
  attachInterrupt(pin_cA_e1, Conteo_flancosM1CA, RISING);//se configura interrupción en pin para flancos de subida
  // al ocurrir se llama a la función de conteo de flancos

  //Pin para canal B encoder 1
  pinMode(pin_cB_e1, INPUT); //se configura pin como entrada
  attachInterrupt(pin_cB_e1, Conteo_flancosM1CB, RISING);//se configura interrupción en pin para flancos de subida
  //al ocurrir se llama a la función de conteo de flancos

  //Configuración de PWM
  ledcSetup(canal_PWM1m1, frecuencia_PWM , resolucion_PWM);//se configura canal de PWM1 del motor 1 a la frecuencia y resolución especificada
  ledcSetup(canal_PWM2m1, frecuencia_PWM, resolucion_PWM);//se configura canal de PWM2 del motor 1 a la frecuencia y resolución especificada
  ledcAttachPin(pin_PWM1m1, canal_PWM1m1);//se carga configuración al pin del PWM1 del motor 1 (cable azul)
  ledcAttachPin(pin_PWM2m1, canal_PWM2m1);//se carga configuración al pin del PWM2 del motor 1 (cable blanco)

  //Motor 2
  //Pin para canal A encoder 2
  pinMode(pin_cA_e2, INPUT); //se configura pin como entrada
  attachInterrupt(pin_cA_e2, Conteo_flancosM2CA, RISING);//se configura interrupción en pin para flancos de subida
  // al ocurrir se llama a la función de conteo de flancos

  //Pin para canal B encoder 1
  pinMode(pin_cB_e2, INPUT); //se configura pin como entrada
  attachInterrupt(pin_cB_e2, Conteo_flancosM2CB, RISING);//se configura interrupción en pin para flancos de subida
  //al ocurrir se llama a la función de conteo de flancos

  //Configuración de PWM
  ledcSetup(canal_PWM1m2, frecuencia_PWM , resolucion_PWM);//se configura canal de PWM1 del motor 2 a la frecuencia y resolución especificada
  ledcSetup(canal_PWM2m2, frecuencia_PWM, resolucion_PWM);//se configura canal de PWM2 del motor 2 a la frecuencia y resolución especificada
  ledcAttachPin(pin_PWM1m2, canal_PWM1m2);//se carga configuración al pin del PWM1 del motor 2 (cable azul)
  ledcAttachPin(pin_PWM2m2, canal_PWM2m2);//se carga configuración al pin del PWM2 del motor 2 (cable blanco)
}

void loop()
{
  //Declaración de variables locales
  
  unsigned long tiempo_actual = millis();//tiempo actual de ejecución del programa en ms -->timer

    //Serial.println(data);
    //Serial.print("\t");

    //Conforme al dato de entrada se modifica la referencia para los motores
    //Se revisa la velocidad de referencia 
    
  
  if ((tiempo_actual - tiempo_previo) >= intervalo_ms)//Se aplica control en cada tiempo de muestreo --> activa bandera
  {    
    //----Referencia----
    float omegad_m1 = cinematica_rdiferencial(1, vd, omegad);//velocidad angular deseada para el motor derecho (Rad/s)
    float omegad_m2 = cinematica_rdiferencial(2, vd, omegad);//velocidad angular deseada para el motor izquierdo (Rad/s)
    
    //Motor 1 --> derecho        
    float rpmd1 = omegad_m1 / (Pi / 30.);
    frecuenciad1 = (rpmd1 / (60.0 / 630.0)) / 1000.;

    //Motor 2 --> izquierdo
    float rpmd2 = omegad_m2 / (Pi / 30.);
    frecuenciad2 = (rpmd2 / (60.0 / 630.0)) /1000;
           
    //----Retroalimentación----
    /*Se muestra frecuencia, pulsos detectados en el tiempo de muestreo --> se utilizan 2 canales
      --> se mide el doble de la frecuencia real del motor
      El conteo de pulsos detectado en el intervalo se multiplica por un factor para obtener los pulsos por segundo    //Motor de 630 PPR (por canal) --> se checan ambos canales*/

    //motor 1
    double frecuencia1 = retro_frec(1);//frecuencia actual del motor 1
    //double rpm1 = frecuencia1 * (60.0 / 1260.0);//Se calculan las rpm del motor 1
    //double omega1 = rpm1 * (Pi / 30.); //Se calcula la velocidad angular del motor 1 (rad/s)
    
    //motor 2
    double frecuencia2 = retro_frec(2);//frecuencia actual del motor 2 
    //double rpm2 = frecuencia2 * (60.0 / 1260.0);//Se calculan las rpm del motor 2
    //double omega2 = rpm2 * (Pi / 30.); //Se calcula la velocidad angular del motor 2 (rad/s)
    
    //----Control----
    //checar si la referencia es negativa
    double frecuenciad1_ = (frecuenciad1 >= 0)? frecuenciad1: -frecuenciad1;//en caso de ser negativa la vuelvo positiva
    double frecuenciad2_ = (frecuenciad2 >= 0)? frecuenciad2: -frecuenciad2;//en caso de ser negativa la vuelvo positiva
    frecuenciad1_ = (frecuenciad1_ >= 2.2)? 2.2 : frecuenciad1_;//se satura referencia
    frecuenciad2_ = (frecuenciad2_ >= 2.2)? 2.2 : frecuenciad2_;//se satura referencia
    control_PID(2, frecuencia2, frecuenciad2_);//Se aplica el control a motor 2
    control_PID(1, frecuencia1, frecuenciad1_);//Se aplica el control a motor 1


    //----Recursividad----
    tiempo_previo = tiempo_actual;//Se actualiza el tiempo de activación de la última bandera
    contador_flancos_m1 = 0;//se reinicia conteo de flancos
    frecuencia1_anterior = frecuencia1;
    frecuenciad1_anterior = frecuenciad1;
    contador_flancos_m2 = 0;//se reinicia conteo de flancos
    frecuencia2_anterior = frecuencia2;
    frecuenciad2_anterior = frecuenciad2;    

    //----Monitoreo----
    //Serial.print("Referencia =");

    //Serial.print("\t");
    //Serial.print(", frecuencia del motor 1 (hz) = ");
    //Serial.print(frecuenciad1 * 1000);
    //Serial.print("\t");
    //Serial.print(frecuencia1 * 1000 / 2);
    //Serial.print("\t");
    //Serial.print(", Señal de control ");
    //Serial.print(valor_PWM1);
    //Serial.print("\t");
    //Serial.print("error ");Serial.print("\t");
    //Serial.print("\t");
    //Serial.print(iedt_m1);
    //Serial.print(e);
    //Serial.print("\n");
    //Serial.print(", frecuencia del motor 1 (hz) = ");
    //Serial.print(frecuenciad2 * 1000);
    //Serial.print("\t");
    //Serial.print(frecuencia2 * 1000 / 2);
    //Serial.print("\t");
    //Serial.print(", Señal de control ");
    //Serial.print(valor_PWM2);
    //Serial.print("\t");
    //Serial.print("error ");Serial.print("\t");
    //Serial.print("\t");
    //Serial.print(iedt_m2);
    //Serial.print(e);
    //Serial.print("\n");
  }

  frecuenciad1_anterior = (frecuenciad1 >=0) ? frecuenciad1 : -frecuenciad1;//La frecuencia deseada para el motor 1 actual pasa a ser la anterior
  frecuenciad2_anterior = (frecuenciad2 >=0) ? frecuenciad2 : -frecuenciad2;//La frecuencia deseada para el motor 2 actual pasa a ser la anterior
nh.spinOnce();
}

//Definición de funciones
//----Retroalimentación----
double retro_frec(int motor)
{
  /*Función que calcula la frecuencia del motor especificado en la iteración especificada, recibe como entrada
   * el número del motor a calcular su frecuencia, retorna el valor de la frecuencia actual para dicho motor*/

  double frecuencia = (motor == 1) ? ((contador_flancos_m1) * (1000.0 / intervalo_ms)) / 1000 : ((contador_flancos_m2) * (1000.0 / intervalo_ms)) / 1000;//frecuencia actual del motor
  return frecuencia;
}

//Motor 1
void Conteo_flancosM1CA(void)
{
  /*Función de conteo de flancos de subida, --> canal A del encoder 1
    se llama por la interrupción al detectar un flanco de subida en un pin ya sea del canal A O B del encoder1
    Mofica el valor del conteo de flancos conforme al sentido de giro*/
    //Serial.print("hola");
  int dir = (frecuenciad1 >= 0)? 1: -1;//se define la dirección a girar conforme al signo de la referencia
  
  if(dir == 1)//el motor debe girar en sentido horario
  {
    int b = digitalRead(pin_cB_e1); //se lee salida del canal B para verificar sentido de giro del motor
    if (b > 0) { //giro antihorario
      contador_flancos_m1--;
    } 
    else {      //giro horario
      contador_flancos_m1++;
    }
  }
  
  else//el motor debe girar en sentido antihorario
  {
    int b = digitalRead(pin_cB_e1); //se lee salida del canal B para verificar sentido de giro del motor
    if (b > 0) { //giro antihorario
      contador_flancos_m1++;
    } 
    else {      //giro horario
      contador_flancos_m1--;
    }
  }
}

void Conteo_flancosM1CB(void)
{
  /*Función de conteo de flancos de subida,--> canal B del encoder 1
    se llama por la interrupción al detectar un flanco de subida en un pin ya sea del canal A O B del encoder1
    Mofica el valor del conteo de flancos conforme al sentido de giro*/
  int dir = (frecuenciad1 >= 0)? 1: -1;//se define la dirección a girar conforme al signo de la referencia

  if(dir == 1)//motor tiene que girar en sentido horario
  {
    int a = digitalRead(pin_cA_e1); //se lee salida del canal A para verificar sentido de giro del motor
    if (a > 0) { //giro horario
      contador_flancos_m1++;
    }
    else {      //giro antihorario
      contador_flancos_m1--;
    }
  }
  
  else//motor tiene que girar en sentido antihorario
  {
    int a = digitalRead(pin_cA_e1); //se lee salida del canal A para verificar sentido de giro del motor
    if (a > 0) { //giro horario
      contador_flancos_m1--;
    }
    else {      //giro antihorario
      contador_flancos_m1++;
    }
  }
}

//Motor 2
void Conteo_flancosM2CA(void)
{
  /*Función de conteo de flancos de subida, --> canal A del encoder 1
    se llama por la interrupción al detectar un flanco de subida en un pin ya sea del canal A O B del encoder1
    Mofica el valor del conteo de flancos conforme al sentido de giro*/
  int dir = (frecuenciad2 >= 0)? 2: -2;//se define la dirección a girar conforme al signo de la referencia
  int b = digitalRead(pin_cB_e2); //se lee salida del canal B para verificar sentido de giro del motor
  if (dir == 2) //el motor tiene que girar en sentido horario
  {
    if (b > 0) { //giro antihorario
      contador_flancos_m2++;
    }
    else {      //giro horario
      contador_flancos_m2--;
    }
  }
  else //el motor tiene que girar en sentido antihorario
  {
    if (b > 0) { //giro antihorario
      contador_flancos_m2--;
    }
    else {      //giro horario
      contador_flancos_m2++;
    }
  }
  //Serial.print("hola1");
}

void Conteo_flancosM2CB(void)
{
  /*Función de conteo de flancos de subida,--> canal B del encoder 1
    se llama por la interrupción al detectar un flanco de subida en un pin ya sea del canal A O B del encoder1
    Mofica el valor del conteo de flancos conforme al sentido de giro*/
  int dir = (frecuenciad2 >= 0)? 2: -2;//se define la dirección a girar conforme al signo de la referencia

  if(dir == 2) //el motor tiene que girar en sentido horario
  {
    int a = digitalRead(pin_cA_e2); //se lee salida del canal A para verificar sentido de giro del motor
    if (a > 0) { //giro horario
      contador_flancos_m2--;
    }
    else {      //giro antihorario
      contador_flancos_m2++;
    }
  }
  else //el motor tiene que girar en sentido antihorario
  {
    int a = digitalRead(pin_cA_e2); //se lee salida del canal A para verificar sentido de giro del motor
    if (a > 0) { //giro horario
      contador_flancos_m2++;
    }
    else {      //giro antihorario
      contador_flancos_m2--;
    }
  }
  //Serial.print("hola2");
  //Serial.print(contador_flancos_m2);
  //Serial.print("\n");
}

//----Control----
void control_Motor(int dir, int ciclo_trabajo, int canal_PWM1, int canal_PWM2)
{
  /*Esta función manda la señal de PWM al motor conforme
     a la dirección y el ciclo de trabajo especificado por el control*/
  if (dir == 1) //giro en sentido horario --> motor 1
  {
    //Serial.print("hola1");
    ledcWrite(canal_PWM1, 0);
    ledcWrite(canal_PWM2, ciclo_trabajo);
  }

  else if (dir == -1) //giro en sentido antihorario --> motor 1
  {
    //Serial.print("hola1");
    ledcWrite(canal_PWM1, ciclo_trabajo);
    ledcWrite(canal_PWM2, 0);
  }
  
  else if (dir == 2) //giro en sentido antihorario --> motor 2
  {
    //Serial.print("hola1");
    ledcWrite(canal_PWM1, ciclo_trabajo);
    ledcWrite(canal_PWM2, 0);
  }

  else if (dir == -2) //giro en sentido horario --> motor 2
  {
    //Serial.print("hola1");
    ledcWrite(canal_PWM1, 0);
    ledcWrite(canal_PWM2, ciclo_trabajo);
  }
  else //no se mueve motor --> se detectó una perturbación
  {
    ledcWrite(canal_PWM1, 0);
    ledcWrite(canal_PWM2, 0);
  }
}

void control_PID(int motor, double frecuencia, double frecuenciad)
{
 /*Función de control PID, recibe como entrada el motor a aplicar el control,  la frecuencia actual del motor, así como la deseada,
  con ello se determina el error y el control aplicar, retorna el valor de ancho de pulso para el PWM del motor
  correspondiente, además de que retorna el valor del error*/

  //Se verifica el motor a aplicar el control
  if(motor == 1) //se aplica PID para el motor 1
  {
    //----Control----

    float e = (frecuenciad * 2) - frecuencia; //Se calcula el error

    //Derivada del error
    double dedt = (e - e_anterior_m1) / dt;

    //Integral del error
    iedt_m1 = (iedt_m1 + (e * dt));//sumatoria del error
    iedt_m1 = (iedt_m1 <= lim_min_iedt)? lim_min_iedt : iedt_m1;//se limita negativamente la sumatoria del error
    //Serial.print(iedt_m1);

    //Señal de control
    double u;
    float frecuencia_p;//frecuencia de perturbaciones 
    
    //Se determina que controlador utilizar y que frecuencia de perturbación, así como el valor máximo de iedt
    if( (frecuenciad <= 2.2) && (frecuenciad >= 1.7) ) //se utiliza el controlador de alta frecuencia
    {
      //Serial.print("af");
      //Ajustes del controlador
      u = Kp_af * e + Kd_af * dedt + Ki_af * iedt_m1;//señal de control
      frecuencia_p = frec_per_af;//frecuencia de perturbaciones
      iedt_m1 = (iedt_m1 >= lim_max_iedt_af)? lim_max_iedt_af : iedt_m1;//se limita positivamente la sumatoria del error
      //Serial.print(frecuencia_p);
    }
    
    else if( (frecuenciad < 1.7) && (frecuenciad >= 1.3) ) //se utiliza el controlador de  media alta frecuencia 
    {
      //Serial.print("maf");
      //Ajustes del controlador
      u = Kp_maf * e + Kd_maf * dedt + Ki_maf * iedt_m1;//señal de control
      frecuencia_p = frec_per_maf;//frecuencia de perturbaciones
      iedt_m1 = (iedt_m1 >= lim_max_iedt_maf)? lim_max_iedt_maf : iedt_m1;//se limita positivamente la sumatoria del error
      //Serial.print(frecuencia_p);
      
    }

    else if( (frecuenciad < 1.3) && (frecuenciad >= 0.7) ) //se utiliza el controlador de  media baja frecuencia 
    {
      //Serial.print("mbf");
      //Ajustes del controlador
      u = Kp_mbf * e + Kd_mbf * dedt + Ki_mbf * iedt_m1;//señal de control
      frecuencia_p = (frecuenciad < 1.3) && (frecuenciad >= 1.0)? frec_per_mbf1: frec_per_mbf2;//frecuencia de perturbaciones
      iedt_m1 = (iedt_m1 >= lim_max_iedt_mbf)? lim_max_iedt_mbf : iedt_m1;//se limita positivamente la sumatoria del error
      //Serial.print(frecuencia_p);
      
    }
    else //se utiliza el controlador de baja frecuencia
    {
      //Serial.print("bf");
      //Ajustes del controlador
      u = Kp_bf * e + Kd_bf * dedt + Ki_bf * iedt_m1;//señal de control
      frecuencia_p = frec_per_bf;//frecuencia de perturbaciones
      iedt_m1 = (iedt_m1 >= lim_max_iedt_bf)? lim_max_iedt_bf : iedt_m1;//se limita positivamente la sumatoria del error
      //Serial.print(frecuencia_p);
    }
    
    //Serial.print(u);
    int valor_PWM1 = (int) fabs(u);

    //se define la dirección conforme al signo de la referencia
    int dir = (frecuenciad1 >= 0)? 1: -1;
    //int dir = 1;
    valor_PWM1 = valor_PWM1 >= 1023 ? 1023 : valor_PWM1; //se satura el ciclo de trabajo a 255
    //Serial.print(valor_PWM1);

    /*Se verifica si la frecuencia medida corresponde a una perturbación
    se mide la diferencia de frecuencias entre la actual y anterior, si esta por arriba de una frecuencia de corte conforme al controlador
    y la referencia no ha cambiado se considera que la frecuencia medida es de una perturbación y se frenan los motores*/
    if( ( ( (fabs(frecuencia*500 - frecuencia1_anterior*500) >= frecuencia_p) ) && ( fabs(frecuenciad - frecuenciad1_anterior) < .05) ) || (frecuenciad == 0))
    {      
      //Serial.print("si");
      valor_PWM1 = 0;
      dir = 0;
      iedt_m1 = 0;
    }

    if ((frecuenciad - frecuenciad1_anterior) <= -.600)//en caso de caídas de referencia bruscas --> se previene inestabilidad.
    {
      //Serial.print("si");
      valor_PWM1 = 0;
      dir = 0;
    }
    
    control_Motor(dir , valor_PWM1, canal_PWM1m1, canal_PWM2m1);//Se envía señal de control a los motores

    //Recursividad
    e_anterior_m1 = e;
    valor_PWM1_anterior = valor_PWM1;

    //Serial.print(valor_PWM1);
    //Serial.print("\t");
  }
  else//Se aplica PID para el motor 2
  {
    //----Control----

    float e = (frecuenciad * 2) - frecuencia; //Se calcula el error

    //Derivada del error
    double dedt = (e - e_anterior_m2) / dt;

    //Integral del error
    iedt_m2 = (iedt_m2 + (e * dt));//sumatoria del error
    iedt_m2 = (iedt_m2 <= lim_min_iedt)? lim_min_iedt : iedt_m2;//se limita negativamente la sumatoria del error
    //Serial.print(iedt_m1);

    //Señal de control
    double u;
    float frecuencia_p;
    
    //Se determina que controlador utilizar y que frecuencia de perturbación, así como el valor máximo de iedt
    if( (frecuenciad <= 2.2) && (frecuenciad >= 1.7) ) //se utiliza el controlador de alta frecuencia
    {
      //Serial.print("af");
      //Ajustes del controlador
      u = Kp_af * e + Kd_af * dedt + Ki_af * iedt_m2;//señal de control
      frecuencia_p = frec_per_af;//frecuencia de perturbaciones
      iedt_m2 = (iedt_m2 >= lim_max_iedt_af)? lim_max_iedt_af : iedt_m2;//se limita positivamente la sumatoria del error
      //Serial.print(frecuencia_p);
    }
    
    else if( (frecuenciad < 1.7) && (frecuenciad >= 1.3) ) //se utiliza el controlador de  media alta frecuencia 
    {
      //Serial.print("maf");
      //Ajustes del controlador
      u = Kp_maf * e + Kd_maf * dedt + Ki_maf * iedt_m2;//señal de control
      frecuencia_p = frec_per_maf;//frecuencia de perturbaciones
      iedt_m2 = (iedt_m2 >= lim_max_iedt_maf)? lim_max_iedt_maf : iedt_m2;//se limita positivamente la sumatoria del error
      //Serial.print(frecuencia_p);
      
    }

    else if( (frecuenciad < 1.3) && (frecuenciad >= 0.7) ) //se utiliza el controlador de  media baja frecuencia 
    {
      //Serial.print("mbf");
      //Ajustes del controlador
      u = Kp_mbf * e + Kd_mbf * dedt + Ki_mbf * iedt_m2;//señal de control
      frecuencia_p = (frecuenciad < 1.3) && (frecuenciad >= 1.0)? frec_per_mbf1: frec_per_mbf2;//frecuencia de perturbaciones
      iedt_m2 = (iedt_m2 >= lim_max_iedt_mbf)? lim_max_iedt_mbf : iedt_m2;//se limita positivamente la sumatoria del error
      //Serial.print(frecuencia_p);
      
    }
    else //se utiliza el controlador de baja frecuencia
    {
      //Serial.print("bf");
      //Ajustes del controlador
      u = Kp_bf * e + Kd_bf * dedt + Ki_bf * iedt_m2;//señal de control
      frecuencia_p = frec_per_bf;//frecuencia de perturbaciones
      iedt_m2 = (iedt_m2 >= lim_max_iedt_bf)? lim_max_iedt_bf : iedt_m2;//se limita positivamente la sumatoria del error
      //Serial.print(frecuencia_p);
    }
    
    //Serial.print(u);
    int valor_PWM2 = (int) fabs(u);

    int dir = (frecuenciad2 >= 0)? 2: -2;//se define la dirección a girar conforme al signo de la referencia
    //int dir = 2;
    valor_PWM2 = valor_PWM2 >= 1023 ? 1023 : valor_PWM2; //se satura el ciclo de trabajo a 255
    //Serial.print(valor_PWM2);

    /*Se verifica si la frecuencia medida corresponde a una perturbación
    se mide la diferencia de frecuencias entre la actual y anterior, si esta por arriba de una frecuencia de corte conforme al controlador
    y la referencia no ha cambiado se considera que la frecuencia medida es de una perturbación y se frenan los motores*/
    if( ( ( (fabs(frecuencia*500 - frecuencia2_anterior*500) >= frecuencia_p) ) && ( fabs(frecuenciad - frecuenciad2_anterior) < .05) ) || (frecuenciad == 0))
    {      
      //Serial.print("si");
      valor_PWM2 = 0;
      dir = 0;
      iedt_m2 = 0;
    }

    if ((frecuenciad - frecuenciad2_anterior) <= -.600)//en caso de caídas de referencia bruscas --> se previene inestabilidad.
    {
      //Serial.print("si");
      valor_PWM2 = 0;
      dir = 0;
    }
    
    control_Motor(dir , valor_PWM2, canal_PWM1m2, canal_PWM2m2);//Se envía señal de control a los motores

    //Recursividad
    e_anterior_m2 = e;
    valor_PWM2_anterior = valor_PWM2;

    //Serial.print(valor_PWM2);
    //Serial.print("\t");    
  }
}


float cinematica_rdiferencial(int rueda, float vel_tras, float omega)
{
  /*Esta función calcula las velocidades angulares de cada rueda conforme a una velocidad traslacional y 
   * angular deseada, dependiendo de que rueda se especifique*/


   //declaración de variables
   float v;//velocidad traslacional de la rueda
   float omegar;//velocidad angular de la rueda
   
   //Se verifica que que rueda se va a calcular la velocidad
   if (rueda == 1) // se trata de la rueda derecha
   {
      v = vel_tras - (l * omega);//velocidad traslacional de la rueda derecha(m/s)
      omegar = v / r; //velocidad angular de la rueda derecha (rad/s)
   }
   else //se trata de la rueda izquierda
   {
      v = vel_tras + (l * omega);//velocidad traslacional de la rueda izquierda(m/s)
      omegar = v / r; //velocidad angular de la rueda izquierda (rad/s)
      //Serial.print("\t");
      //Serial.print("hola");
      //Serial.print("\t");
   }
   return omegar;//se retorna valor 
}
