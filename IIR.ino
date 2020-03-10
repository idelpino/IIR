//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// Ejemplo de solución de la práctica 2 de la asignatura de Iniciación a la Ingeniería Robótica. Curso 2019/2020.
//
// Para cualquier duda, podéis contactar con nosotros a través de los siguientes correos:
// ivan.delpino@ua.es 
// miguelangel.munoz@ua.es
//
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////// 
//
// Este programa implementa un esquema de funcionamiento genérico que podría servir para desarrollar otros programas similares. 
// Por lo tanto este ejemplo podría servir también como "plantilla" para el desarrollo
// de la práctica 3, en la que el robot debe resolver un circuito de navegación autónoma.
//
// De forma genérica, lo que debe hacer un programa de control de un robot es: (en el contexto de las prácticas de esta
// asignatura)
// 
// 1) Leer sensores:
//      De esta forma se actualizan los datos que se tienen acerca del entorno (con ultrasonidos, o cámaras de visión por ejemplo)
//      o del propio robot (p.e. número de pulsos de un encoder acoplado a las ruedas motrices...)
//
// 2) Procesar los datos para extraer información:
//      Una vez que se han capturado los datos empleando los sensores debemos procesarlos para extraer información útil para 
//      las tareas que queramos que el robot sea capaz de llevar a cabo. Por ejemplo, en una imagen podríamos detectar un carril de 
//      una carretera, o procesando una lectura de distancia de ultrasonidos podríamos decidir si existe algún obstáculo que nos
//      debe hacer parar el vehículo. Lo más importante conceptualmente en este punto es diferenciar DATOS de INFORMACIÓN: los sensores
//      nos dan datos, pero por sí mismos esos datos no significan nada, resulta necesario procesarlos de manera adecuada para extraer
//      la información que será empleada por el robot.
//
// 3) Actualizar máquina de estados:
//      En función de la información recibida y de su propio estado, el robot debe decidir si continuar con la tarea que está realizando
//      o darla por finalizada y cambiar a la siguiente tarea (otro estado).
//
// 4) Calcular la acción adecuada: (ésta es la tarea del controlador) 
//      En función de la tarea que esté ejecutando y las lecturas de los sensores 
//      decidir qué acción debe llevar a cabo (frenar, acelerar, cambiar de sentido, parar motores...)
//
// 5) Actuar:
//      Enviar la acción a los actuadores (generalmente motores)
//
// 6) Refrescar interfaz de usuario:
//      Enviar la información relevante al exterior para facilitar el depurado de los programas y la supervisión
//      del funcionamiento. (p.e. tarea que se está ejecutando, lecturas del sensor, comandos que se están enviando a 
//      motores, códigos de error etc.).
// 
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////// 
// 
//  Concretamente en cada iteración del loop se realizan las siguientes acciones:
//  --------------------------------------------------------------------------------
//    1) Leer sensores: Se llama a una función genérica ("readSensors") dentro de la cual se irán llamando a cada una de las 
//                      funciones encargadas de leer cada sensor individual. En este momento sólo está implementada la función
//                      readUltraSensor, ya que es el único sensor instalado, pero en caso de añadir algún sensor extra habrá
//                      que añadir una función específica para leerlo. Estas funciones van escribiendo los datos recogidos por
//                      cada sensor en los campos adecuados del struct de tipo "Data", de tal modo que al salir de la
//                      función "readSensors" la información de todos los sensores quedará dentro del struct, y se podrá utilizar
//                      en las siguientes partes del programa.
//                      Cabe destacar también que las funciones individuales (como readUltraSensor) se deben encargar por sí mismas 
//                      de controlar el timing para realizar las consultas a los sensores en los intervalos de tiempo previstos.
//
//    2) Procesar los datos para extraer información: Se ejecuta una función genérica llamada "processData" la cual va llamando a los
//                                                    distintos procesadores específicos. Por el momento sólo está implementado el 
//                                                    procesador ligado al sensor de ultrasonidos ("processUltrasonicSensorData")
//                                                    el cual toma una variable numérica continua como es la distancia en centímetros
//                                                    obtenida en la función readUltraSensor y devuelve una variable de tipo categórico
//                                                    a la que llamamos "obstacle_presence" y que puede tomar uno de los siguientes tres valores 
//                                                    discretos "NO_OBSTACLE_DETECTED" "FAR_OBSTACLE_DETECTED" o "CLOSE_OBSTACLE_DETECTED".
//
//    3) Actualizar la máquina de estados: Se llama a la función genérica "updateFiniteStateMachine", esta función está preparada
//                                         para hacer transiciones entre tareas utilizando la información extraída de los sensores 
//                                         y el estado en el que se encuentra el robot.
//
//    4) Calcular la acción adecuada: Para esto llamamos a una función genérica "controller" que en función del estado o tarea
//                                    que se esté llevando a cabo, llamará a una u otra función específica para calcular la salida adecuada
//                                    para cada motor. Por ejemplo tenemos "controllerNoObstacleDetected" que implementa lo que hay que
//                                    hacer con los motores en caso de que no se detecten obstáculos. En caso de querer añadir nuevos
//                                    comportamientos o tareas, habrá que programar un controlador específico, como por ejemplo si
//                                    queremos hacer un siguelíneas, haríamos algo así como "controllerLineFollower" y allí, en función
//                                    de los datos extraídos del sensor de IR decidiríamos qué hacer con cada motor.
//
//    5) Actuar: Se llama a la función "execute" que envía la velocidad deseada a cada motor.
//
//    6) Refrescar interfaz de usuario: Esta función ("refreshUserInterface") simplemente comprueba el tiempo, y cuando han transcurrido
//                                      los milisegundos especificados en el struct de configuración, pasa a enviar la información 
//                                      más importante por el puerto serie para poder monitorizar el sistema y ayudarnos en los procesos
//                                      de depuración del código.
//
//   Una vez terminados estos 6 puntos, el loop vuelve a iterar, y así indefinidamente! 
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


// ATENCIÓN!: Comienza el código!

// Incluimos el fichero con las etiquetas que vayamos a emplear
#include "labels.h"

// Incluimos el fichero con la declaración de los structs
#include "structs_definitions.h"

// Incluimos el fichero con las declaraciones de funciones de lectura de sensores
#include "sensors_reading.h"

// Incluimos el fichero con las declaraciones de funciones de procesado de datos
#include "processors.h"

// Incluimos el fichero con la declaración de la máquina de estados
#include "state_machine.h"

// Incluimos el fichero con las declaraciones de los distintos controladores
#include "controllers.h"

// Incluimos el fichero con las declaraciones de las funciones de escritura en actuadores
#include "actuators_writing.h"

// Incluimos el fichero con la función de interfaz con el usuario
#include "user_interface.h"

// Una vez declarados los structs, pasamos a instanciarlos en "objetos" globales
// (los hacemos de ámbito global para poder darles el valor en la función "setup").

Hardware st_hardware_;

Config st_config_; //añadimos prefijo st_ para que se sepa a simple vista que es 
                   //un struct, añadimos un guión bajo al final para indicar que es
                   //global (el guión bajo en C++ se suele usar para indicar que la 
                   //variable en cuestión es un atributo de la clase, pero aquí lo
                   //usaremos de una forma más flexible). 
                  
Data st_data_;

Information st_information_;

Actions st_actions_;

void setup() {

  // Inicializamos el puerto serie a un baudrate determinado 
  // (nos hará falta para enviar la información de monitorización del sistema).
  Serial.begin(115200);

  // Inicializamos los valores de los structs
  st_hardware_.ultraSensor.setpin(PORT_3);

  // Esto lo hacemos porque la clase MeDCMotor de makeblock no ofrece un método para cambiar el puerto dinamicamente
  MeDCMotor left_motor(M1);
  MeDCMotor right_motor(M2);
  st_hardware_.left_motor = left_motor;
  st_hardware_.right_motor = right_motor;

  // Nos aseguramos de comenzar con todos los actuadores parados
  st_hardware_.left_motor.stop();
  st_hardware_.right_motor.stop();

  // Configuración
  st_config_.ultrasonic_sensor_reading_period_in_millis = 150;
  st_config_.user_interface_refresh_period_in_millis = 500;
  st_config_.max_speed_pwm_value = 255;
  st_config_.max_reverse_speed_pwm_value = -255;
  st_config_.left_motor_polarity = 1.0;
  st_config_.right_motor_polarity = 1.0; // Poner -1.0 si algún motor está cableado al revés.
   
  // Medidas de los sensores
  st_data_.detected_distance_in_centimeters = IMPOSSIBLE_DISTANCE; // Comenzaremos con valor inválido, para detectar cuándo hemos tenido una lectura válida

  // Información extraída por los procesadores
  st_information_.obstacle_presence =  NOT_KNOWN;                // Inicializamos en valor inválido.
  st_information_.obstacle_distance_in_cm = IMPOSSIBLE_DISTANCE;

  // Valor para pasar a motores (actuación)
  st_actions_.left_motor_pwm = 0;  // Inicializamos a cero por seguridad, aunque hay otros puntos del programa en los que se checkea todo para no enviar acciones
  st_actions_.right_motor_pwm = 0; // no deseadas a motores.
}


// Loop principal: Esta función tiene que iterar lo más rápido posible, normalmente cientos de veces por segundo!
// Inportante --> ¡¡¡Queda terminantemente prohibido el uso de delays!!!
void loop() {

  // Leemos los sensores
  st_data_ = readSensors(st_config_, st_hardware_);

  // Extraemos la información a partir de los datos
  st_information_ = processData(st_data_);

  // Actualizamos la máquina de estados a partir de la información recibida por los sensores 
  int current_state = updateFiniteStateMachine(st_information_);

  // Calculamos las acciones que tenemos que aplicar a los distintos motores, en función del
  // estado y las lecturas de los sensores
  st_actions_ = controller(current_state, st_information_, st_config_);
  
  // Pasamos a motores las acciones calculadas
  int error = execute(st_actions_, st_config_, st_hardware_);

  // Publicamos info importante para el debug
  refreshUserInterface(st_information_, current_state, st_actions_, error, st_config_);

 // ... y volvemos a iterar!
}

// Eso es todo! Si hay dudas pedid tutorías! --> ivan.delpino@ua.es, miguelangel.munoz@ua.es
// Saludos!
// Iván y Miguel Ángel
