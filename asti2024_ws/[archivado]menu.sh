#!/bin/bash

# Preguntar por el parametro
echo -n "Por favor, introduzca un número (1,2,3,4): "
read parametro

# Ejecutar el comando dependiendo del parámetro
case $parametro in
  1) 
    echo "$parametro"
    # Inicia una nueva sesión de tmux en segundo plano
    tmux new-session -d -s my_session 

    # Divida la ventana de tmux en dos paneles
    tmux split-window
	
    # Envia las teclas al primer panel
    tmux send-keys -t my_session:0.0 'ros2 run bringup motor_controller' Enter
	
    # Envia las teclas al segundo panel
    tmux send-keys -t my_session:0.1 'ros2 run movement vel_controller' Enter
    
    # Divide la subventana en mas subventanas
    tmux split-window
    
    # Envia las teclas al segundo panel
    tmux send-keys -t my_session:0.2 'cd ../asti2024_ws' Enter
    tmux send-keys -t my_session:0.2 'ros2 run semifnal laberinto' Enter

    # Adjunta la sesión
    tmux attach -t my_session
    ;;
  2) 
    echo "$parametro"
    ;;
  3) 
    echo "$parametro"
    ;;
  4) 
    echo "$parametro"
    tmux send-keys -t my_session:0.1 'ros2 topic pub -1 /set_velocity custom_interfaces/SetVelocity "{id: 1, velocity: 0}" ' Enter
    tmux send-keys -t my_session:0.1 'ros2 topic pub -1 /set_velocity custom_interfaces/SetVelocity "{id: 2, velocity: 0}" ' Enter
    ;;
  *) 
    echo "Por favor, introduzca un número válido (1,2,3,4)"
    ;;
esac
