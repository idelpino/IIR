#ifndef _labels_h_
#define _labels_h_

// A continuación, se suelen definir los valores constantes
// que se van a usar en el programa, como por ejemplo, umbrales a partir de los cuales cambiar de tarea o
// también se pueden declarar como constantes determinadas "etiquetas" que luego se usan en el código, esto resulta muy útil
// para mejorar la legibilidad.

// Umbrales de distancia: Se utilizarán para generar información de tipo categorial (clasificando los obstáculos como cercanos, lejanos o no-obstáculos)
// que se empleará para decidir las transiciones de la máquina de estados.
const float FAR_OBJECT_DISTANCE_THRESHOLD_CM   = 40.0; // Cualquier obstáculo a una distancia mayor de la definida aquí se considera que no
                                                       // es un obstáculo.

const float CLOSE_OBJECT_DISTANCE_THRESHOLD_CM = 20.0; // Si la distancia detectada está en el intervalo [CLOSE_OBJECT_DISTANCE_CM, FAR_OBJECT_DISTANCE_CM]
                                                       // el obstáculo se considerará en zona lejana.
                                                       // Cualquier obstáculo por debajo del unbral CLOSE_OBJECT_DISTANCE_CM se considerará en campo cercano.

// Valor para marcar como inválidos los datos leidos con el ultrasonidos
const float IMPOSSIBLE_DISTANCE = -1.0;

// Valores que puede tomar la información extraída a partir de los datos del sensor de ultrasonidos:
const int NO_OBSTACLE_DETECTED    =  0; // En estos casos, al tratarse de un valor categorial (no númerico) los valores de las etiquetas no son importantes, 
const int FAR_OBSTACLE_DETECTED   =  1; // basta con que sean diferentes entre sí, para que se puedan distinguir unos de otros!
const int CLOSE_OBSTACLE_DETECTED =  2;
const int NOT_KNOWN               = -1; // Esta etiqueta la ponemos en negativo para que resulte muy llamativa, se usará en caso de que el sensor de ultrasonidos
                                        // haya dado una lectura errónea.

// States
const int IMPOSSIBLE_STATE                  = -1; // Estos valores "imposibles" son muy útiles durante el depurado, ya que permiten lanzar 
                                                  // warnings en caso de que alguna variable no se haya inicializado correctamente.
const int STOP                              =  0;
const int MOVING_FORWARD_MAX                =  1;
const int MOVING_FORWARD_WHILE_TURNING      =  2;
const int MOVING_BACKWARD_WHILE_TURNING     =  3;

const int EXECUTION_ERROR = -1;
const int EXECUTION_SUCCESSFUL = 0;

#endif
