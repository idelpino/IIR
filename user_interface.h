#ifndef _user_interface_h_
#define _user_interface_h_

#include "labels.h"
#include "structs_definitions.h"

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
      case IMPOSSIBLE_STATE:
        Serial.println("Warning: current_state variable not initialized!!");
      break;
      case STOP:
        Serial.println("Vehicle stopped!");
      break;

      case MOVING_FORWARD_MAX:
        Serial.println("moving forward at maximum speed!");
      break;

      case MOVING_FORWARD_WHILE_TURNING:
        Serial.println("moving forward while turning!");
      break;

      case MOVING_BACKWARD_WHILE_TURNING:
        Serial.println("moving backward while turning!");
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

#endif
