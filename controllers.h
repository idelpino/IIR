#ifndef _controllers_h_
#define _controllers_h_

#include "labels.h"
#include "structs_definitions.h"


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
Actions controllerForwardRightProportional(Config st_config, Information st_info)
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
    
    case MOVING_FORWARD_WHILE_TURNING:
      st_actions = controllerForwardRightProportional(st_config, st_info);
    break;

    case MOVING_BACKWARD_WHILE_TURNING:
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

#endif
