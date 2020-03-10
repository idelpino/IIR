#ifndef _actuators_writing_h_
#define _actuators_writing_h_

#include "labels.h"
#include "structs_definitions.h"

// Función encargada de pasar a motores los comandos calculados por los controladores
// este es el último punto antes de actuar sobre los motores,por lo que tenemos que 
// ser cuidadosos de no enviar valores indeseados... En este caso la máquina es pequeña
// pero imaginemos que vamos a acelerar un coche o un camión... hay que pensar bien
// antes de actuar!!
int execute(Actions st_actions, Config st_config, Hardware st_hardware)
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
  st_hardware.left_motor.run(left_motor_action);
  st_hardware.right_motor.run(right_motor_action);

  // Y retornamos el código de error.
  return(error);
}

#endif
