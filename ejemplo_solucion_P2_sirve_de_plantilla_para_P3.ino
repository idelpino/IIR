//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// Ejemplo de solución de la práctica 2 de la asignatura de Iniciación a la Ingeniería Robótica. Curso 2018/2019.
//
// Para cualquier duda, podéis contactar conmigo a través del correo:
// ivan.delpino@ua.es 
//
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////// 
//
// Este programa implementa un esquema de funcionamiento genérico que podría servir para desarrollar otros programas similares. 
// Por lo tanto este ejemplo podría servir también como "plantilla" para el desarrollo
// de la práctica 4, en la que el robot debe resolver un circuito de navegación autónoma.
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
//      o darla por finalizada y cambiar a la siguiente tarea.
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
// 7) Mantener sincronía:
//      Necesitaremos mantener una cierta sincronización, para lo cual será necesario ir monitorizando el 
//      tiempo transcurrido entre determinados eventos (p.e. querremos leer los sensores en intervalos de tiempo adecuados, o
//      refrescar la interfaz de usuario a menudo pero no constantemente para no saturar las comunicaciones ni repetir los 
//      mismos datos cientos de veces...
// 
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////// 
// 
// Concretamente, el código que contiene este archivo hace lo siguiente:
//
// - En primer lugar incluye las librerías necesarias y declara los elementos hardware que vamos a emplear
//
// - Después se crean las constantes que se utilizan a lo largo del programa, generalmente son etiquetas para mejorar la 
//   legibilidad del mismo.
//
// - A continuación se declaran unas estructuras de datos o "structs" para guardar las distintas variables agrupadas de forma 
//   lógica. Por ejemplo todas las variables que tienen que ver con configuración (velocidades máximas de los motores, 
//   tiempos que hay que esperar hasta poder volver a leer un sensor etc) se agrupan en una estructura de tipo "Config", 
//   mientras que las distintas medidas de los sensores (como la distancia en centímetros detectada por el sensor de ultrasonidos)
//   se guardan en una estructura de tipo "Data". Este planteamiento resulta muy ventajoso, ya que nos permitirá añadir
//   elementos hardware de forma fácil sin tener que reescribir código, como se verá más adelante.
//
// - Una vez creadas las estructuras, se instancian y se pasa a la función "setup" para inicializar los valores.
//
// - Lo más recomendable es una vez entendido el código hasta este punto, pasar a leer la función "loop" e ir 'tirando del hilo'
//   y entendiendo el resto de funciones en el orden en el que se ejecutan.
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
//    5) Refrescar interfaz de usuario: Esta función ("refreshUserInterface") simplemente comprueba el tiempo, y cuando han transcurrido
//                                      los milisegundos especificados en el struct de configuración, pasa a enviar la información 
//                                      más importante por el puerto serie para poder monitorizar el sistema y ayudarnos en los procesos
//                                      de depuración del código.
//
//   Una vez terminados estos 5 puntos, el loop vuelve a iterar, y así indefinidamente! 
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


// ATENCIÓN!: Comienza el código!

// Incluimos librerías, en este caso sólo necesitamos las propias del
// microcontrolador del robot (Orion)
#include "MeOrion.h"

// Declaramos los elementos hardware conectados al microcontrolador
// Sensores
MeUltrasonicSensor ultraSensor(PORT_3);
// ...añadir aquí otros sensores si es necesario...

// Actuadores
MeDCMotor left_motor(M1);
MeDCMotor right_motor(M2);
// ... añadir aquí más actuadores si es necesario ...


// A continuación, se suelen definir los valores constantes
// que se van a usar en el programa, como por ejemplo, umbrales a partir de los cuales cambiar de tarea o
// también se pueden declarar como constantes determinadas "etiquetas" que luego se usan en el código, esto resulta muy útil
// para mejorar la legibilidad.

// Umbrales de distancia:
const float FAR_OBJECT_DISTANCE_THRESHOLD_CM   = 40.0; // Cualquier obstáculo a una distancia mayor de la definida aquí se considera que no
                                                       // es un obstáculo, y por lo tanto el vehículo puede continuar recto.

const float CLOSE_OBJECT_DISTANCE_THRESHOLD_CM = 20.0; // Si la distancia detectada está en el intervalo [CLOSE_OBJECT_DISTANCE_CM, FAR_OBJECT_DISTANCE_CM]
                                                       // el vehículo girará mientras avanza.
                                                       // Cualquier distancia por debajo del unbral CLOSE_OBJECT_DISTANCE_CM, provocará que el 
                                                       // vehículo retroceda mientras gira.

// Valores que puede tomar la información extraída a partir de los datos del sensor de ultrasonidos:
const int NO_OBSTACLE_DETECTED    =  0; // En estos casos, al tratarse de un valor categorial (no númerico) los valores de las etiquetas no son importantes, 
const int FAR_OBSTACLE_DETECTED   =  1; // basta con que sean diferentes entre sí, para que se puedan distinguir unos de otros!
const int CLOSE_OBSTACLE_DETECTED =  2;
const int NOT_KNOWN               = -1; // Esta etiqueta la ponemos en negativo para que resulte muy llamativa, se usará en caso de que el sensor de ultrasonidos
                                        // haya dado una lectura errónea.

// States
const int STOP                              = 0;
const int MOVING_FORWARD_MAX                = 1;
const int MOVING_FORWARD_PROPORTIONAL       = 2;
const int MOVING_BACKWARD_LEFT_PROPORTIONAL = 3;

const float IMPOSSIBLE_DISTANCE = -1.0;

const int EXECUTION_ERROR = -1;
const int EXECUTION_SUCCESSFUL = 0;

// A continuación declararemos structs que nos ayudarán a hacer genérico el programa, 
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

// Definiremos un struct de medidas o "Data", en el cual guardaremos los datos de todos los sensores que tengamos
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

// Una vez declarados los structs, pasamos a instanciarlos en "objetos" globales
// (los hacemos de ámbito global para poder darles el valor en la función "setup").

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

  // Nos aseguramos de comenzar con todos los actuadores parados
  left_motor.stop();
  right_motor.stop();

  // Inicializamos los valores de los structs

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

// ATENCIÓN!! --> En este punto lo recomendable es pasar a leer la función "loop" e ir buscando en orden las funciones que se van ejecutando!





// Función específica para leer el sensor de ultrasonidos.
float readUltraSensor(Config st_config)
{
  static unsigned long int previous_time = millis(); // al declarar la variable de tipo static, lo que hacemos es que sólo se inicialice la primera vez
                                                     // que se ejecuta la función. Las siguientes veces que se llame a la función esta variable conservará
                                                     // el valor de la ejecución anterior.
                                                     
  static float detected_distance_in_centimeters = IMPOSSIBLE_DISTANCE; // Inicializamos con un valor imposible, de esta forma 
                                                                       // si hay cualquier problema con el sensor nos daremos cuenta 
                                                                       // al leer la información de depurado.

  unsigned long int current_time = millis(); // Consultamos el tiempo en milisegundos desde que se inició el programa

  // y comprobamos si se ha cumplido el tiempo mínimo para consultar el sensor
  if ( current_time - previous_time > st_config.ultrasonic_sensor_reading_period_in_millis )
  {
    // en caso de que sea momento de leer el sensor actualizamos la medida anterior
    detected_distance_in_centimeters = ultraSensor.distanceCm();

    // y actualizamos el contador con el valor de tiempo en el que se ha realizado la última lectura del sensor.
    previous_time = millis();
  }
  else
  {
    // en caso contrario en realidad no tenemos que hacer nada, simplemente devolveremos el 
    // valor que tenga la variable, que será el de la última lectura válida, o la IMPOSSIBLE_DISTANCE,
    // en caso de que todavía no se haya producido ninguna medida.
  }
                                                              
  return (detected_distance_in_centimeters);
}

// ...aquí habría que añadir las funciones para leer cada uno de los demás sensores que instalemos...




// Función genérica que debe ir llamando a cada una de las funciones específicas para rellenar el struct de "Data" con 
// los datos de todos los sensores instalados.
Data readSensors(Config st_config)
{
  // Declaramos un struct de tipo Measurement para guardar las medidas de todos los sensores
  Data st_meas;
  st_meas.detected_distance_in_centimeters = IMPOSSIBLE_DISTANCE; // lo inicializamos con valor inválido para detectar fallos en la
                                                                  // lectura del sensor
  
  // Llamamos a la función que lee el sensor de ultrasonidos
  st_meas.detected_distance_in_centimeters = readUltraSensor(st_config);

  // Si tuvieramos más sensores, habría que añadir las funciones para leerlos y guardar los
  // datos en otros campos del struct "st_meas"

  // Una vez que hemos leido todos los sensores retornamos las medidas
  return(st_meas);
}


// Procesador específico para extaer información acerca de la presencia de obstáculos en base a los datos del 
// sensor de ultrasonidos.
int processUltrasonicSensorData(float distance_in_cm)
{
  int obstacle_presence = NOT_KNOWN;

  if ( distance_in_cm == IMPOSSIBLE_DISTANCE )
  {
    obstacle_presence = NOT_KNOWN; // En caso de que no dispongamos de lecturas válidas del sensor no podemos extraer
                                   // información, esto (como se verá más adelante) hará que el vehículo se detenga.
                                   
  }
  else
  {
    if ( distance_in_cm > FAR_OBJECT_DISTANCE_THRESHOLD_CM ) // Si el obstáculo más proximo está por encima del
                                                   // umbral, consideramos que tenemos vía libre.
    {
      obstacle_presence = NO_OBSTACLE_DETECTED; 
    }
    else
    {
      if ( distance_in_cm > CLOSE_OBJECT_DISTANCE_THRESHOLD_CM ) // En caso contrario, si la distancia es mayor que
                                                       // la mínima permitida para seguir avanzando, consideramos
                                                       // que se trata de un obstáculo lejano.                                                                          
      {
        obstacle_presence = FAR_OBSTACLE_DETECTED;
      }
      else
      {
        obstacle_presence = CLOSE_OBSTACLE_DETECTED; // En caso contrario el obstáculo está en la zona cercana.
      }
    }
  }
   
  return(obstacle_presence);
}

// Función para extraer la información a partir de los datos 'en crudo'.
Information processData(Data st_data)
{
  Information st_info;

  ///////////////////////////////////////
  // Procesamos el sensor de ultrasonidos
  ///////////////////////////////////////
  // Primero copiamos la distancia en centímetros del struct de datos al de información, porque 
  // como ya se ha comentado anteriormente, el driver de MakeBlock nos hace una parte del procesado, 
  // por lo que la distancia en centímetros que retorna el sensor se puede considerar "información" 
  // y no sólo "datos".
  st_info.obstacle_distance_in_cm = st_data.detected_distance_in_centimeters;
  //                                                                                                                                                           
  // A continuación generamos la información de tipo categorial (hay via libre, existe un obstáculo, 
  // está lejos, está cerca...) a partir de la distancia detectada por el sensor.                                                                          
  st_info.obstacle_presence = processUltrasonicSensorData(st_data.detected_distance_in_centimeters);
  /////////////////////////////////////////

  // ...Aquí habría que añadir las distintas llamadas a los procesadores específicos para los datos de 
  // cada sensor
  
  return(st_info);
}


// Función para determinar qué tarea se va a llevar a cabo a partir de la información extraída a partir
// de los datos de los sensores.
int updateFiniteStateMachine(Information st_information)
{
  static int state = STOP; // Inicializamos esta variable estática con un valor inicial por defecto para
                           // poder detectar errores en la fase de debug
                           // En este caso la variable no necesitaría ser declarada como estática ya que
                           // el estado siguiente no depende del estado anterior (solo depende de la lectura
                           // del sensor) sin embargo habrá muchas ocasiones en las que se transitará a uno 
                           // u otro estado dependiendo tanto de las informaciones de los sensores como del 
                           // estado concreto en el que el robot se encuentre.

  switch(st_information.obstacle_presence)
  {
    case NOT_KNOWN:
      state = STOP;  // Si hay error en el sensor pararemos motores.
    break;

    case NO_OBSTACLE_DETECTED:
      state = MOVING_FORWARD_MAX; // Si no se detectan obstáculos nos moveremos hacia adelante a la 
                                  // velocidad máxima permitida por el struct de configuración
    break;

    case FAR_OBSTACLE_DETECTED:
      state = MOVING_FORWARD_PROPORTIONAL; // En caso de que se detecte un obstáculo lejano, continuaremos
                                           // la marcha hacia adelante usando un controlador proporcional
                                           // con la distancia (más detalles en las implementaciones 
                                           // de los controladores específicos).
    break;

    case CLOSE_OBSTACLE_DETECTED:
      state = MOVING_BACKWARD_LEFT_PROPORTIONAL; // Si el obstáculo se encuentra en la zona cercana se hará
                                                 // retroceder al vehículo mientras gira. También implementaremos
                                                 // un control proporcional a la distancia, como se verá en la sección
                                                 // de controladores específicos.
    break;
    default:
      state = STOP; // En caso de que venga algún valor extraño pararíamos el vehículo.
    break;
  }
  
  return(state);
}

// Controlador específico para parar motores
Actions controllerStop(void)
{
  Actions st_actions;

  // Símplemente pondremos ambos motores a cero. Por este motivo no necesitamos que la función
  // reciba como inputs las medidas de los sensores por esto aparece "void" los parámetros
  // de entrada.

  st_actions.left_motor_pwm = 0;
  st_actions.right_motor_pwm = 0;
  
  return(st_actions);
}


// Controlador específico que se utiliza para avanzar a la máxima velocidad permitida por configuración
Actions controllerMovingForwardMax(Config st_config)
{
  Actions st_actions;
 
  // Al usar este controlador avanzaremos empleando la máxima velocidad permitida por la
  // configuración (los valores se ponen en la función "setup")
  st_actions.left_motor_pwm = st_config.max_speed_pwm_value;
  st_actions.right_motor_pwm = st_config.max_speed_pwm_value;
  
  return(st_actions);
  
}

// Controlador específico para girar mientras se avanza.
Actions controllerForwardProportional(Config st_config, Information st_info)
{
  Actions st_actions;

  // En este caso vamos a emplear la distancia medida por el ultrasonidos
  // para ajustar la velocidad del vehículo.
  // Siempre giraremos hacia la derecha (esto se podría hacer random o alternando, o de 
  // otro modo, pero para que sirva de ejemplo creo que es suficiente así)
  // por lo tanto el motor izquierdo se pondrá a la velocidad máxima: 

  st_actions.left_motor_pwm = st_config.max_speed_pwm_value;
  
  // y lo que haremos será reducir
  // la velocidad del motor derecho en función de la distancia al obstáculo.
  //
  // Sabemos que este controlador se va a usar cuando los obstáculos estén a una
  // distancia en el intervalo [CLOSE_OBJECT_DISTANCE_CM, FAR_OBJECT_DISTANCE_CM]
  // por lo que escalaremos la velocidad del motor derecho 
  // para que quede en el intervalo [0, max_speed_pwm_value]

  // como el PWM tiene que ser un número entero, primero hacemos un casting a flotante "(float)max_speed_pwm_value"
  float right_pwm = (float)st_config.max_speed_pwm_value * (st_info.obstacle_distance_in_cm - CLOSE_OBJECT_DISTANCE_THRESHOLD_CM) / (FAR_OBJECT_DISTANCE_THRESHOLD_CM - CLOSE_OBJECT_DISTANCE_THRESHOLD_CM);

  // y luego redondeamos y pasamos a tipo int
  st_actions.right_motor_pwm = (int)round(right_pwm);

  // Y retornamos las acciones calculadas.
  return(st_actions);
}


// Controlador específico para girar mientras retrocedemos.
Actions controllerBackwardLeftProportional(Config st_config, Information st_info)
{
  Actions st_actions;

  // En este caso también vamos a emplear la distancia medida por el ultrasonidos.
  // Siempre giraremos hacia atrás a la izquierda 
  // por lo tanto el motor derecho se pondrá a la velocidad máxima hacia atrás: 

  st_actions.right_motor_pwm = st_config.max_reverse_speed_pwm_value;
  
  // y aumentaremos la velocidad reverse del motor izquierdo en función de 
  // la distancia hasta el obstáculo. De esta forma cuanto más cerca esté el objeto, más
  // retrocederemos, y cuanto más lejos, más giramos.
  //
  // Sabemos que este controlador se va a usar cuando los obstáculos estén a una
  // distancia en el intervalo [0, CLOSE_OBJECT_DISTANCE_CM]
  // por lo que escalaremos la velocidad del motor izquierdo 
  // para que quede en el intervalo [max_speed_pwm_value, 0]

  // al igual que antes, primero hacemos un casting a flotante
  float left_pwm = (float)st_config.max_reverse_speed_pwm_value * (1.0 - (st_info.obstacle_distance_in_cm / CLOSE_OBJECT_DISTANCE_THRESHOLD_CM ) );

  // redondeamos y pasamos a tipo int
  st_actions.left_motor_pwm = (int)round(left_pwm);

  // Y retornamos las acciones calculadas.
  return(st_actions);
}

// ... Aquí habrá que añadir otros controladores específicos, como por ejemplo un siguelíneas, o uno para poner el vehículo perpendicular a una pared... // 


// Función genérica que llama al controlador específico adecuado en función de la tarea
// que se deba realizar. 
Actions controller(int current_state, Information st_info, Config st_config)
{
  Actions st_actions;
  
  switch(current_state)
  {
    case STOP:
      st_actions = controllerStop();
    break;
    
    case MOVING_FORWARD_MAX:
      st_actions = controllerMovingForwardMax(st_config);

    break;
    
    case MOVING_FORWARD_PROPORTIONAL:
      st_actions = controllerForwardProportional(st_config, st_info);
    break;

    case MOVING_BACKWARD_LEFT_PROPORTIONAL:
      st_actions = controllerBackwardLeftProportional(st_config, st_info);
    break;

    //... si hubiesen otros controladores habría que llamarlos desde otros casos de este switch...//
    
    default:
      // En el caso de que nos llegase un valor extraño en la variable current_state
      // utilizaremos el controlador del estado inicial (que símplemente mantiene
      // parado el vehículo).
      st_actions = controllerStop();
    break;
  }
  
  return(st_actions);
}

// Función encargada de pasar a motores los comandos calculados por los controladores
// este es el último punto antes de actuar sobre los motores,por lo que tenemos que 
// ser cuidadosos de no enviar valores indeseados... En este caso la máquina es pequeña
// pero imaginemos que vamos a acelerar un coche o un camión... hay que pensar bien
// antes de actuar!!
int execute(Actions st_actions, Config st_config)
{
  // Usaremos un código de error para detectar posibles anomalías en los controladores
  int error = EXECUTION_SUCCESSFUL;

  // Calculamos la acción a aplicar al motor izquierdo
  int left_motor_action = st_config.left_motor_polarity * st_actions.left_motor_pwm;

  if (left_motor_action < st_config.max_reverse_speed_pwm_value || left_motor_action > st_config.max_speed_pwm_value)
  {
    // En caso de que el valor esté fuera de rango activaremos el código de error
    error = EXECUTION_ERROR; 
  }

  // en el caso del motor derecho haremos lo mismo
  int right_motor_action = st_config.right_motor_polarity * st_actions.right_motor_pwm;

  if (right_motor_action < st_config.max_reverse_speed_pwm_value || right_motor_action > st_config.max_speed_pwm_value)
  {
    error = EXECUTION_ERROR;
  }

  if( error == EXECUTION_ERROR ) 
  {
   // En caso de error paramos motores por seguridad.
   left_motor_action = 0;
   right_motor_action = 0;
  }

  // Finalmente aplicamos a motor los PWM calculados.
  left_motor.run(left_motor_action);
  right_motor.run(right_motor_action);

  // Y retornamos el código de error.
  return(error);
}

// Función encargada de refrescar la info para el usuario a intervalos de tiempo correctos
void refreshUserInterface (Information st_info, int current_state, Actions st_actions, int error, Config st_config)
{    
  // Procedemos con la parte de sincronización, esta funciona igual que la que hemos empleado antes en la función readUltraSensor, 
  // (en esa función están todos los detalles comentados, por lo que ahora no los repetiremos)
  static unsigned long int previous_time = millis();

  unsigned long int current_time = millis(); // Consultamos el tiempo en milisegundos desde que se inició el programa

  // y comprobamos si se ha cumplido el tiempo mínimo para publicar los datos por el puerto serie
  if ( current_time - previous_time > st_config.user_interface_refresh_period_in_millis )
  {
    // en caso de que sea momento de refrescar la info, pasamos a escribir en el puerto serie:

    // Primero enviamos la información de distancia captada mediante el sensor de ultrasonidos.
    Serial.print("Distance information = ");
    Serial.println(st_info.obstacle_distance_in_cm);

    // Después enviamos la información categorial (no numérica) sobre la detección de obstáculos.
    // al no ser numérica, lo que hacemos es interpretarla con un switch y luego enviar un mensaje 
    // en texto con la explicación.
    Serial.print("Categorical information based on distance: ");
    switch(st_info.obstacle_presence)
    {
      case NOT_KNOWN:
        Serial.println("Warning: It was impossible to read the sensor!!");
      break;
        
      case NO_OBSTACLE_DETECTED:
        Serial.println("No obstacle detected...");
      break;

      case FAR_OBSTACLE_DETECTED:
        Serial.println("Far obstacle detected!");
      break;

      case CLOSE_OBSTACLE_DETECTED:
        Serial.println("Close obstacle detected!");
      break;
      
      // El caso default es el que se lanza si ninguna de las anteriores se cumplen, 
      // si esto ocurriera es que hay un valor raro y nos interesará verlo durante el depurado. 
      default:
        Serial.println("Warning: in variable st_info.obstacle_presence --> unexpected value!!");
      break;
    }


    // A continuación informamos sobre el estado, como esta también es una variable categórica
    // lo que hacemos es poner un switch y enviar un mensaje descriptivo del estado actual,
    // esto es importante para el depurado, ya que leer "State = 1" no es muy informativo, mientras
    // que leer "State = moving forward at maximum speed" sí que aporta información.    
    Serial.print("State = ");
    switch(current_state)
    {
      case STOP:
        Serial.println("Vehicle stopped!");
      break;

      case MOVING_FORWARD_MAX:
        Serial.println("moving forward at maximum speed!");
      break;

      case MOVING_FORWARD_PROPORTIONAL:
        Serial.println("moving forward and turning!");
      break;

      case MOVING_BACKWARD_LEFT_PROPORTIONAL:
        Serial.println("moving backward and turning!");
      break;

      default:
        Serial.println("Warning: in variable current_state --> unexpected value!!");
      break;
    }
    
    Serial.print("Left motor pwm = ");
    Serial.println(st_actions.left_motor_pwm);

    Serial.print("Right motor pwm = ");
    Serial.println(st_actions.right_motor_pwm); 

    Serial.print("Error code = ");
    Serial.println(error);

    // y actualizamos el contador con el valor de tiempo en el que se ha realizado la última lectura del sensor.
    previous_time = millis();
  }
  return;
}

// Loop principal: Esta función tiene que iterar lo más rápido posible, normalmente cientos de veces por segundo!
// Consejo --> ¡Hay que evitar el uso de delays!!!
void loop() {

  // Leemos los sensores
  st_data_ = readSensors(st_config_);

  // Extraemos la información a partir de los datos
  st_information_ = processData(st_data_);

  // Actualizamos la máquina de estados a partir de la información recibida por los sensores 
  int current_state = updateFiniteStateMachine(st_information_);

  // Calculamos las acciones que tenemos que aplicar a los distintos motores, en función del
  // estado y las lecturas de los sensores
  st_actions_ = controller(current_state, st_information_, st_config_);
  
  // Pasamos a motores las acciones calculadas
  int error = execute(st_actions_, st_config_);

  // Publicamos info importante para el debug
 refreshUserInterface(st_information_, current_state, st_actions_, error, st_config_);

 // ... y volvemos a iterar!
}

// Eso es todo! Si hay dudas pedid tutorías! --> ivan.delpino@ua.es
// Saludos!
// Iván.
