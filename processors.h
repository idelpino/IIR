#ifndef _processors_h_
#define _processors_h_

#include "labels.h"
#include "structs_definitions.h"


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

#endif
