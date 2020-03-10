#ifndef _state_machine_h_
#define _state_machine_h_

#include "labels.h"
#include "structs_definitions.h"

// Función para determinar qué tarea se va a llevar a cabo a partir de la información extraída a partir
// de los datos de los sensores.
int updateFiniteStateMachine(Information st_information)
{
  static int previous_state = STOP; // Inicializamos esta variable estática con un valor inicial por defecto para
                                    // poder detectar errores en la fase de debug, al ser estática sólo tomará el
                                    // valor de inicialización en la primera ejecución de esta función. En las 
                                    // siguientes iteraciones conservará el valor anterior, aunque se salga y se
                                    // vuelva a entrar en la función. 
  
  int current_state =  IMPOSSIBLE_STATE; // Esta variable tomará valor en función de las entradas y del estado anterior, por
                                         // lo que no es necesario que sea de tipo "static". La inicializamos a valor
                                         // inválido para facilitar el depurado: si al salir de esta función la variable
                                         // continúa teniendo valor inválido esto querrá decir que ha habido algún 
                                         // problema!

  // En caso de que la lectura del sensor haya sido incorrecta nos iremos a estado "STOP" independientemente
  // del estado anterior, por lo que pondremos una estructura if-else antes del switch:
  if(st_information.obstacle_presence == NOT_KNOWN)
  {
    current_state =  STOP;
  }
  else
  {
    // En  caso de que se haya podido extraer la información a partir de los datos del sensor pasamos a la implementación
    // de la lógica de la máquina de estados:
     
    switch(previous_state) // En este switch nos vamos a fijar en el estado en el que se encontraba el vehículo
                           // en el momento en el que se recibe la nueva información, y en función tanto del estado
                           // anterior como de la información sensorial, decidiremos cual va a ser nuestro próximo estado.
    {
      case STOP:
        // En caso de que estuviéramos parados, esperaremos la señal de NO_OBSTACLE_DETECTED
        // para iniciar la marcha a velocidad máxima hacia adelante, es decir de este estado sólo
        // saldremos cuando no se detecten obstáculos.
        if(st_information.obstacle_presence == NO_OBSTACLE_DETECTED)
        {
          current_state = MOVING_FORWARD_MAX;
        }
        else
        {
          current_state = STOP;
        }
      break;

      case MOVING_FORWARD_MAX:
        // En caso que estuviéramos avanzando hacia adelante:
        
        // a) si no detectemos obstáculos continuaremos la marcha, por lo que no hará falta cambiar el estado.
        if(st_information.obstacle_presence == NO_OBSTACLE_DETECTED)
        {
          current_state = MOVING_FORWARD_MAX;
        }
        else
        {
          // b) En caso de que aparezca de repente un obstáculo cercano, nos iremos a STOP
          if(st_information.obstacle_presence == CLOSE_OBSTACLE_DETECTED)
          {
            current_state = STOP;
          }
          else
          {
            // c) En caso de que aparezca un obstáculo lejano pasamos a girar mientras avanzamos
            if(st_information.obstacle_presence == FAR_OBSTACLE_DETECTED)
            {
              current_state = MOVING_FORWARD_WHILE_TURNING;
            }
          }
        }
      break;

      case MOVING_FORWARD_WHILE_TURNING:
        // En caso de que estuviéramos girando mientras avanzamos:
        
        // a) Si detectamos un objeto lejano, mantendremos el estado en "girar mientras avanzamos"
        if(st_information.obstacle_presence == FAR_OBSTACLE_DETECTED)
        {
          current_state = MOVING_FORWARD_WHILE_TURNING;
        }
        else
        {
          // b) En caso de que desaparezca el obstáculo pasaremos a ir hacia adelante sin girar
          if(st_information.obstacle_presence == NO_OBSTACLE_DETECTED)
          {
            current_state = MOVING_FORWARD_MAX;
          }
          else
          {
            // c) En caso de que el obstáculo entre a campo cercano pasamos a retroceder mientras giramos
            if(st_information.obstacle_presence == CLOSE_OBSTACLE_DETECTED)
            {
              current_state = MOVING_BACKWARD_WHILE_TURNING;
            }
          }
        }
          
      break;

      case MOVING_BACKWARD_WHILE_TURNING:
        // En caso de que estuviéramos girando mientras retrocedemos:
        
        // a) Si detectemos el objeto en campo cercano, mantendremos el estado de "girar mientras 
        // retrocedemos"
        if(st_information.obstacle_presence == CLOSE_OBSTACLE_DETECTED)
        {
          current_state = MOVING_BACKWARD_WHILE_TURNING;
        }
        else
        {
          // b) En caso de que desaparezca el obstáculo pasaremos a ir hacia adelante sin girar
          if(st_information.obstacle_presence == NO_OBSTACLE_DETECTED)
          {
            current_state = MOVING_FORWARD_MAX;
          }
          else
          {
            // c) En caso de que el obstáculo pase a campo lejano pasamos a avanzar mientras giramos
            if(st_information.obstacle_presence == FAR_OBSTACLE_DETECTED)
            {
              current_state = MOVING_FORWARD_WHILE_TURNING;
            }
          }
        }

      break;
    }
  }

  // Una vez que hemos calculado el estado pasamos a guardar el resultado en la variable estática, 
  // ya que el estado actual pasará a ser el estado anterior en la siguiente iteración:
  previous_state = current_state;

  // Y retornamos el valor calculado!
  return(current_state);
}

#endif
