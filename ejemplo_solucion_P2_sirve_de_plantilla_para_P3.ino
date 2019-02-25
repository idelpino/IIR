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
//      De esta forma se actualiza la información que se tiene del entorno (como por ejemplo presencia de obstáculos)
//      o del propio robot (p.e. velocidad de giro de las ruedas o nivel de batería)
//
// 2) Actualizar máquina de estados:
//      En función de la información recibida y de su propio estado el robot debe decidir si continuar con la tarea que está realizando
//      o darla por finalizada y cambiar a la siguiente tarea.
//
// 3) Calcular la acción adecuada: (ésta es la tarea del controlador) 
//      En función de la tarea que esté ejecutando y las lecturas de los sensores 
//      decidir qué acción debe llevar a cabo (frenar, acelerar, cambiar de sentido, parar motores...)
//
// 4) Actuar:
//      Enviar la acción a los actuadores (generalmente motores)
//
// 5) Refrescar interfaz de usuario:
//      Enviar la información relevante al exterior para facilitar el depurado de los programas y la supervisión
//      del funcionamiento. (p.e. tarea que se está ejecutando, lecturas del sensor, comandos que se están enviando a 
//      motores, códigos de error etc.).
//
// 6) Mantener sincronía:
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
// - Después se crean las constantes que se utilizan generalmente como etiquetas para mejorar la legibilidad del programa
//
// - A continuación se declaran unas estructuras de datos "structs" para guardar las distintas variables agrupadas de forma 
//   lógica. Por ejemplo todas las variables que tienen que ver con configuración (velocidades máximas de los motores, 
//   tiempos que hay que esperar hasta poder volver a leer un sensor etc.) se agrupan en una estructura de tipo "Config", 
//   mientras que las distintas medidas de los sensores (como la distancia en centímetros detectada por el sensor de ultrasonidos)
//   se guardan en una estructura de tipo "Measurements". Este planteamiento resulta muy ventajoso, ya que nos permitirá añadir
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
//                      cada sensor en los campos adecuados del struct de tipo "Measurements", de tal modo que al salir de la
//                      función "readSensors" la información de todos los sensores quedará dentro del struct, y se podrá utilizar
//                      en las siguientes partes del programa.
//                      Cabe destacar también que las funciones individuales (como readUltraSensor) se deben encargar por sí mismas 
//                      de controlar el timing para realizar las consultas a los sensores en los intervalos de tiempo previstos.
//
//    2) Actualizar la máquina de estados: Se llama a la función genérica "updateFiniteStateMachine", esta función está preparada
//                                         para hacer transiciones entre tareas utilizando la info de los sensores y el estado
//                                         en el que se encuentra el robot, aunque en esta versión de la práctica, sólo se tiene
//                                         en cuenta la info del sensor de ultrasonidos, pero la plantilla está preparada para 
//                                         implementar lógicas más complejas, que probablemente harán falta en la práctica siguiente.
//
//    3) Calcular la acción adecuada: Para esto llamamos a una función genérica "controller" que en función del estado o tarea
//                                    que se esté llevando a cabo, llama a una u otra función específica para calcular la salida adecuada
//                                    para cada motor. Por ejemplo tenemos "controllerNoObstacleDetected" que implementa lo que hay que
//                                    hacer con los motores en caso de que no se detecten obstáculos. En caso de querer añadir nuevos
//                                    comportamientos o tareas, habrá que programar un controlador específico, como por ejemplo si
//                                    queremos hacer un siguelíneas, haríamos algo así como "controllerLineFollower" y allí, en función
//                                    de los datos del sensor de IR decidiríamos qué hacer con cada motor.
//
//   5) Refrescar interfaz de usuario: El sistema de timing es idéntico al implementado en la función para leer el sensor de ultrasonidos
//                                     esta función ("refreshUserInterface") simplemente comprueba el tiempo, y cuando han transcurrido
//                                     los milisegundos especificados en el struct de configuración, pasa a enviar la información 
//                                     más importante por el puerto serie para poder monitorizar el sistema y ayudarnos en los procesos
//                                     de depuración del código.  
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

// Umbrales de distancia
const float FAR_OBJECT_DISTANCE_CM   = 40.0; // Cualquier obstáculo a una distancia mayor de la definida aquí se considera que no
                                             // es un obstáculo, y por lo tanto el vehículo puede continuar recto.

const float CLOSE_OBJECT_DISTANCE_CM = 20.0; // Si la distancia detectada está en el intervalo [CLOSE_OBJECT_DISTANCE_CM, FAR_OBJECT_DISTANCE_CM]
                                             // el vehículo girará mientras avanza.

                                             // Cualquier distancia por debajo del unbral CLOSE_OBJECT_DISTANCE_CM, provocará que el 
                                             // vehículo retroceda mientras gira.

// States
const int INITIAL_STATE                  = 0;
const int NO_OBSTACLE_DETECTED           = 1;
const int OBSTACLE_DETECTED_IN_SAFE_ZONE = 2;
const int OBSTACLE_DETECTED_TOO_CLOSE    = 3;

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
  int max_reverse_speed_pwm_value; //hacia atrás
  
  // Estas variables tomarán valor 1 o -1 para invertir el sentido de giro de los motores sin tener que cambiar el cableado 
  int left_motor_polarity;
  int right_motor_polarity;
};

// Definiremos un struct de medidas o "Measurements", en el cual guardaremos los datos de todos los sensores que tengamos
// instalados en el robot. En el caso concreto de la práctica 2, sólo tenemos un sensor de ultrasonidos, pero para la siguiente
// práctica podremos tener un siguelineas y sensores de final de carrera  
struct Measurements
{
  float detected_distance_in_centimeters;
  //...añadir más campos para guardar datos en caso de añadir nuevos sensores...
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
                  
Measurements st_measurements_;

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
  st_measurements_.detected_distance_in_centimeters = IMPOSSIBLE_DISTANCE; // Comenzaremos con valor inválido, para detectar cuándo hemos tenido una lectura válida

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




// Función genérica que debe ir llamando a cada una de las funciones específicas para rellenar el struct de "Measurements" con 
// los datos de todos los sensores instalados.
Measurements readSensors(Config st_config)
{
  // Declaramos un struct de tipo Measurement para guardar las medidas de todos los sensores
  Measurements st_meas;
  st_meas.detected_distance_in_centimeters = IMPOSSIBLE_DISTANCE; // lo inicializamos con valor inválido para detectar fallos en la
                                                                  // lectura del sensor
  
  // Llamamos a la función que lee el sensor de ultrasonidos
  st_meas.detected_distance_in_centimeters = readUltraSensor(st_config);

  // Si tuvieramos más sensores, habría que añadir las funciones para leerlos y guardar los
  // datos en otros campos del struct "st_meas"

  // Una vez que hemos leido todos los sensores retornamos las medidas
  return(st_meas);
}


// Función para determinar qué tarea se va a llevar a cabo
int updateFiniteStateMachine(Measurements st_meas)
{
  static int state = INITIAL_STATE; // Inicializamos esta variable estática con un valor inicial por defecto para
                                    // poder detectar errores en la fase de debug
                                    // En este caso la variable no necesitaría ser declarada como estática ya que
                                    // el estado siguiente no depende del estado anterior (solo depende de la lectura
                                    // del sensor) sin embargo habrá muchas ocasiones en las que se transitará a uno 
                                    // u otro estado dependiendo tanto de las informaciones de los sensores como del 
                                    // estado concreto en el que el robot se encuentre.

  if ( st_meas.detected_distance_in_centimeters == IMPOSSIBLE_DISTANCE )
  {
    state = INITIAL_STATE; // En caso de que no dispongamos de lecturas válidas del sensor iremos al estado inicial, 
                           // lo cual implica como se verá en la parte de control, que pararemos motores. Esto 
                           // es bueno desde el punto de vista de la seguridad para detener el vehículo en caso de 
                           // fallo del sensor.
  }
  else
  {
    if ( st_meas.detected_distance_in_centimeters > FAR_OBJECT_DISTANCE_CM ) // Si el obstáculo más proximo está por encima del
                                                                             // umbral, consideramos que tenemos vía libre.
    {
      state = NO_OBSTACLE_DETECTED; 
    }
    else
    {
      if ( st_meas.detected_distance_in_centimeters > CLOSE_OBJECT_DISTANCE_CM ) // en caso contrario, si la distancia es mayor que
                                                                                 // la mínima permitida para seguir avanzando, consideramos
                                                                                 // que hemos encontrado un obstáculo, pero que no es necesario
                                                                                 // retroceder, por lo que consideramos que lo hemos detectado en
                                                                                 // "zona segura".
      {
        state = OBSTACLE_DETECTED_IN_SAFE_ZONE;
      }
      else
      {
        state = OBSTACLE_DETECTED_TOO_CLOSE; // En caso contrario, el obstáculo está demasiado cerca, esto hará que el vehículo retroceda.
      }
    }
  }

  return(state);
}

// Controlador específico de estado inicial (simplemente para motores)
Actions controllerInitialState(void)
{
  Actions st_actions;

  // En el estado inicial no queremos que el robot se mueva, por lo que símplemente
  // pondremos ambos motores a cero. Por este motivo tampoco necesitamos que la función
  // reciba como inputs las medidas de los sensores, por esto ponemos como "void" los parámetros
  // de entrada.

  st_actions.left_motor_pwm = 0;
  st_actions.right_motor_pwm = 0;
  
  return(st_actions);
}


// Controlador específico que se utiliza en ausencia de obstáculos
Actions controllerNoObstacleDetected(Config st_config)
{
  Actions st_actions;

  // En el caso de que no detectemos obstáculos cercanos, 
  // avanzaremos empleando la máxima velocidad permitida por la
  // configuración (los valores se ponen en la función "setup")
  st_actions.left_motor_pwm = st_config.max_speed_pwm_value;
  st_actions.right_motor_pwm = st_config.max_speed_pwm_value;
  
  return(st_actions);
  
}

// Controlador específico para el caso en que detectemos un obstáculo lejano
// (giraremos mientras avanzamos).
Actions controllerObstacleDetectedInSafeZone(Config st_config, Measurements st_meas)
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
  float right_pwm = (float)st_config.max_speed_pwm_value * (st_meas.detected_distance_in_centimeters - CLOSE_OBJECT_DISTANCE_CM) / (FAR_OBJECT_DISTANCE_CM - CLOSE_OBJECT_DISTANCE_CM);

  // y luego redondeamos y pasamos a tipo int
  st_actions.right_motor_pwm = (int)round(right_pwm);

  // Y retornamos las acciones calculadas.
  return(st_actions);
}


// Controlador específico para los casos en los que se haya detectado un obstáculo 
// en la zona cercana (deberemos girar mientras retrocedemos).
Actions controllerObstacleDetectedTooClose(Config st_config, Measurements st_meas)
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
  float left_pwm = (float)st_config.max_reverse_speed_pwm_value * (1.0 - (st_meas.detected_distance_in_centimeters / CLOSE_OBJECT_DISTANCE_CM ) );

  // redondeamos y pasamos a tipo int
  st_actions.left_motor_pwm = (int)round(left_pwm);

  // Y retornamos las acciones calculadas.
  return(st_actions);
}

// ... Aquí habrá que añadir otros controladores específicos, como por ejemplo un siguelíneas, o uno para poner el vehículo perpendicular a una pared... // 


// Función genérica que llama al controlador específico adecuado en función de la tarea
// que se deba realizar. 
Actions controller(int current_state, Measurements st_meas, Config st_config)
{
  Actions st_actions;
  
  switch(current_state)
  {
    case INITIAL_STATE:
      st_actions = controllerInitialState();
    break;
    
    case NO_OBSTACLE_DETECTED:
      st_actions = controllerNoObstacleDetected(st_config);

    break;
    
    case OBSTACLE_DETECTED_IN_SAFE_ZONE:
      st_actions = controllerObstacleDetectedInSafeZone(st_config, st_meas);
    break;

    case OBSTACLE_DETECTED_TOO_CLOSE:
      st_actions = controllerObstacleDetectedTooClose(st_config, st_meas);
    break;

    //... si hubiesen otros controladores habría que llamarlos desde otros casos de este switch...//
    
    default:
      // En el caso de que nos llegase un valor extraño en la variable current_state
      // utilizaremos el controlador del estado inicial (que símplemente mantiene
      // parado el vehículo).
      st_actions = controllerInitialState();
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
void refreshUserInterface (Measurements st_meas, int current_state, Actions st_actions, int error, Config st_config)
{    
  // Procedemos con la parte de sincronización, esta funciona igual que la que hemos empleado antes en la función readUltraSensor, 
  // (en esa función están todos los detalles comentados, por lo que ahora no los repetiremos)
  static unsigned long int previous_time = millis();

  unsigned long int current_time = millis(); // Consultamos el tiempo en milisegundos desde que se inició el programa

  // y comprobamos si se ha cumplido el tiempo mínimo para publicar los datos por el puerto serie
  if ( current_time - previous_time > st_config.user_interface_refresh_period_in_millis )
  {
    // en caso de que sea momento de refrescar la info, pasamos a escribir en el puerto serie:
    Serial.print("Distance reading = ");
    Serial.println(st_meas.detected_distance_in_centimeters);
  
    Serial.print("State = ");
    Serial.println(current_state);
    
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
  st_measurements_ = readSensors(st_config_);

  // Actualizamos la máquina de estados a partir de la información recibida por los sensores 
  int current_state = updateFiniteStateMachine(st_measurements_);

  // Calculamos las acciones que tenemos que aplicar a los distintos motores, en función del
  // estado y las lecturas de los sensores
  st_actions_ = controller(current_state, st_measurements_, st_config_);
  
  // Pasamos a motores las acciones calculadas
  int error = execute(st_actions_, st_config_);

  // Publicamos info importante para el debug
 refreshUserInterface(st_measurements_, current_state, st_actions_, error, st_config_);

 // ... y volvemos a iterar!
}

// Eso es todo! Si hay dudas pedid tutorías! --> ivan.delpino@ua.es
// Saludos!
// Iván.
