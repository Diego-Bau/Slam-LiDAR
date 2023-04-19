/*Header de librería para el control PID del robot diferencial, 
contiene todos los parámetros de ajuste, prototipos de función así como variables globales utilizadas en 
el código del control*/
//Macros
//----Configuración---- 
//General
#define frecuencia_PWM  5000//Frecuencia para el PWM en hz
#define resolucion_PWM 10 //#de bits para el control del ancho de pulso o ciclo de trabajo

//Motor 1
#define pin_cA_e1 26//Pin del canal A del encoder 1 cable rojo
#define pin_cB_e1 27//Pin del canal B del encoder 1 cable negro 
#define canal_PWM1m1  0//Canal del pwm1 del motor 1
#define canal_PWM2m1  1//Canal del pwm2 del motor 1
#define pin_PWM1m1 33//Pin para el PWM1 del motor 1 cable azul --> sentido antihorario
#define pin_PWM2m1 32//Pin para el PWM2 del motor 1 cable blanco --> sentido horario

//Motor 2
#define pin_cA_e2 16//Pin del canal A del encoder 2 cable rojo
#define pin_cB_e2 17//Pin del canal B del encoder 2 cable negro 
#define canal_PWM1m2  2//Canal del pwm1 del motor 2
#define canal_PWM2m2  3//Canal del pwm2 del motor 2
#define pin_PWM1m2 18//Pin para el PWM1 del motor 2 cable azul --> sentido antihorario
#define pin_PWM2m2 19//Pin para el PWM2 del motor 2 cable blanco --> sentido horario

//----Control----
//constantes
#define Pi  3.14159265358979323846//valor aprox de Pi
#define intervalo_ms 100.0//tiempo de muestreo para la retroalimentación en ms --> tiempo para activar la bandera del timer
#define intervalo2_ms 300.0
#define dt 0.1//tiempo de muestreo en segundos
#define lim_min_iedt -.00001 //Límite inferior de la sumatoria del error

//Control alta frecuencia --> entre 1.7 a 2.2 khz
#define Kp_af 480//Ganancia proporcional para el controlador
#define Kd_af 10.0//Ganancia derivativa para el controlador
#define Ki_af 750//Ganancia integral para el controlador
#define frec_per_af 850//frecuencia de perturbación en alta frecuencia (hz)
#define lim_max_iedt_af 2.0//Límite superior de la sumatoria del error

//Control media alta frecuencia --> entre 1.3 a 1.7 khz
#define Kp_maf 480//Ganancia proporcional para el controlador
#define Kd_maf 10.0//Ganancia derivativa para el controlador
#define Ki_maf 700//Ganancia integral para el controlador
#define frec_per_maf 750//frecuencia de perturbación en media alta frecuencia (hz)
#define lim_max_iedt_maf 2.7//Límite superior de la sumatoria del error

//Control media baja frecuencia --> entre 0.7 a 1.3 khz
#define Kp_mbf 500//Ganancia proporcional para el controlador
#define Kd_mbf 8.0//Ganancia derivativa para el controlador
#define Ki_mbf 700//Ganancia integral para el controlador
#define frec_per_mbf1 650//frecuencia de perturbación en media baja frecuencia (hz) --> entre 1.0 a 1.3 khz
#define frec_per_mbf2 450//frecuencia de perturbación en media baja frecuencia (hz) --> entre 0.7 a 1.0 khz
#define lim_max_iedt_mbf 2.7//Límite superior de la sumatoria del error

//Control baja frecuencia --> entre 0.2 a 0.7 khz
#define Kp_bf 450//Ganancia proporcional para el controlador
#define Kd_bf 8.0//Ganancia derivativa para el controlador
#define Ki_bf 750//Ganancia integral para el controlador
#define frec_per_bf 150//frecuencia de perturbación baja frecuencia (hz)
#define lim_max_iedt_bf 2.7//Límite superior de la sumatoria del error

//Declaración de variables globales
unsigned long tiempo_previo = 0;//tiempo de activación inicial de la última bandera en ms

//Motor 1
volatile int contador_flancos_m1 = 0;//Conteo inicial  de flancos de subida del motor 1
unsigned int valor_PWM1_anterior = 1023;//valor inicial para el ancho de pulso de PWM del motor 1 anterior
double frecuenciad1 = 0.0;
double frecuenciad1_anterior = 0; //frecuencia deseada anterior del motor 1
double frecuencia1_anterior = 0; //frecuencia anterior de motor 1
double iedt_m1 = 0, e_anterior_m1 = .1;//sumatoria del erro y error anterior


//Motor 2
volatile int contador_flancos_m2 = 0;//Conteo inicial  de flancos de subida del motor 2
unsigned int valor_PWM2_anterior = 1023;//valor inicial para el ancho de pulso de PWM del motor 2 anterior
double frecuenciad2 = 0.0;
double frecuenciad2_anterior = 0; //frecuencia deseada anterior del motor 2
double frecuencia2_anterior = 0; //frecuencia anterior de motor 2
double iedt_m2 = 0, e_anterior_m2 = .1;//sumatoria del error y error anterior

//Prototipos de función
double retro_frec(int);//Función para medir la frecuencia de los motores
void Conteo_flancosM1CA(void);//Función para el conteo de flancos del canal A del encoder 1
void Conteo_flancosM1CB(void);//Función para el conteo de flancos del canal B del encoder 1
void Conteo_flancosM2CA(void);//Función para el conteo de flancos del canal A del encoder 1
void Conteo_flancosM2CB(void);//Función para el conteo de flancos del canal B del encoder 1
void control_PID(int, double, double);//Función para el control PID de los motores
void control_Motor(int, int, int, int);//Función para el control del motor con un ancho de pulso dado por un PWM
float cinematica_rdiferencial(int, float, float);
