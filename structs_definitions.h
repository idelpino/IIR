#ifndef _structs_definitions_h_
#define _structs_definitions_h_

#include "MeOrion.h"

struct Hardware
{
  // Sensores
  MeUltrasonicSensor ultraSensor;

  // Actuadores
  MeDCMotor left_motor;
  MeDCMotor right_motor;
};

// En esta cabecera (fichero .h) declararemos structs que nos ayudarán a hacer genérico el programa, 
// de esta forma por ejemplo podremos declarar un "objeto" (los structs son parecidos a los objetos de C++, pero más sencillos)
// de tipo Config en el que guardaremos los valores de tiempo que hay que esperar entre lecturas de un determinado sensor. 
// En el caso de que queramos añadir un sensor nuevo, tendremos que añadir también un campo al struct Config
struct Config
{
  // Valores para la temporización
  int ultrasonic_sensor_reading_period_in_millis;
  int user_interface_refresh_period_in_millis;
  //... añadir aquí otros valores de tiempo cuando se añadan nuevos sensores...

  // Valores para control de motores
  int max_speed_pwm_value;         // hacia adelante
  int max_reverse_speed_pwm_value; // hacia atrás
  
  // Estas variables tomarán valor 1 o -1 para invertir el sentido de giro de los motores sin tener que cambiar el cableado 
  int left_motor_polarity;
  int right_motor_polarity;
};

// Definiremos un struct "Data", en el cual guardaremos los datos de todos los sensores que tengamos
// instalados en el robot. En el caso concreto de la práctica 2, sólo tenemos un sensor de ultrasonidos, pero para la siguiente
// práctica podremos tener un siguelineas y sensores de final de carrera  
struct Data
{
  float detected_distance_in_centimeters;
  
  //...añadir más campos para guardar datos en caso de añadir nuevos sensores...
  
};

struct Information
{
  int   obstacle_presence;       // Para dar valor a esta variable usaremos las "etiquetas" 
                                 // declaradas al principio del código, (NO_OBSTACLE_DETECTED, 
                                 // FAR_OBSTACLE_DETECTED...). Esta variable se usará para 
                                 // determinar las transiciones entre estados.
                                 
  float obstacle_distance_in_cm; // En el caso del sensor de ultrasonidos se da la circunstancia de que los datos que nos
                                 // devuelve en realidad ya están preprocesados, dado que el sensor en realidad lo que mide
                                 // es la diferencia de tiempos entre que envía un pulso de ultrasonido y le llega el eco,
                                 // a esta diferencia se le aplica después la velocidad de propagación del sonido para ESTIMAR
                                 // la distancia a la que se encuentra el objeto que ha provocado la reflexión. Hay que decir 
                                 // que la velocidad de propagación depende de distintas variables, tales como la presión
                                 // o la temperatura, y que la reflexión dependerá entre otras cosas del tamaño, la forma y el 
                                 // material de que esté hecho el objeto sobre el que rebota la onda (por ejemplo, un material blando 
                                 // como la lana puede absorber el ultrasonido y ser "invisible" para el sensor),
                                 // por lo que cuando el sensor nos devuelva centímetros tenemos que ser
                                 // conscientes de que las medidas tienen una incertidumbre--> la precisión de los sensores no es
                                 // infinita!! y el error cero no existe!!
                                 //
                                 // A lo que iba, que cuando usamos la función de MakeBlock distanceCm(); para leer el ultrasonido
                                 // lo que nos devuelve ya es información útil (no sólo datos) y por este motivo la vamos a incluir
                                 // en el struct "Information". 
  
  // ...añadir aquí variables para guardar la información extraída por los procesadores
  // a partir de la información de los nuevos sensores que se vayan a instalar.
};

// Este struct contendrá las salidas que hay que aplicar a cada motor
struct Actions
{
  int left_motor_pwm;
  int right_motor_pwm;
  //...añadir otros en caso de añadir motores... 
};

#endif
