#ifndef _sensors_reading_h_
#define _sensors_reading_h_

#include <Arduino.h>

#include "labels.h"
#include "structs_definitions.h"

// Función específica para leer el sensor de ultrasonidos.
float readUltraSensor(Config st_config, Hardware st_hardware)
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
    detected_distance_in_centimeters = st_hardware.ultraSensor.distanceCm();

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
Data readSensors(Config st_config, Hardware& st_hardware)
{
  
  // Declaramos un struct de tipo Measurement para guardar las medidas de todos los sensores
  Data st_meas;
  st_meas.detected_distance_in_centimeters = IMPOSSIBLE_DISTANCE; // lo inicializamos con valor inválido para detectar fallos en la
                                                                  // lectura del sensor
  
  // Llamamos a la función que lee el sensor de ultrasonidos
  //st_meas.detected_distance_in_centimeters = readUltraSensor(st_config, st_hardware);

  // Si tuvieramos más sensores, habría que añadir las funciones para leerlos y guardar los
  // datos en otros campos del struct "st_meas"

  // Una vez que hemos leido todos los sensores retornamos las medidas
  return(st_meas);
  
}

#endif
