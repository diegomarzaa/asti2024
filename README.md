# UJI ROBOTICS - ASTI2024


## Compilación

(Solo hacer si no funciona el ros2 run de algo, acordarse también de hacer los sources después de cada colcon)

MOTORES

`cd asti2024/test_ws`

(Si están las carpetas build, install y log hay que borrarlas)  
`rm -r build`  
`rm -r install`  
`rm -r log`

`colcon build --packages-select custom_interfaces`  
`colcon build --packages-select dynamixel_sdk`  
`source install/setup.bash`  
`colcon build --symlink-install --packages-select bringup movement`  
`source install/setup.bash`



## Ejecución

MOTORES

Terminal 1 (Donde se ha compilado todo):  
`ros2 run bringup motor_vel_controller`

En caso de fallo:  
`sudo usermod -aG dialout <linux_account>`  
`sudo chmod 777 /dev/ttyUSB0`  
`ros2 run bringup motor_vel_controller /dev/ttyUSB0`   (usar `ls /dev/ttyUSB*` para verlo)  
Reiniciar la controladora de motores.  
Cambiar el usb de sitio.

Terminal 2:   
`source /opt/ros/foxy/setup.bash`  
`source install/setup.bash`  
`ros2 run semifinal siguelineas`  (o laberinto)




PRUEBAS

`cd ../retos_ws`

(Si están las carpetas build, install y log hay que borrarlas)  
`rm -r build`  
`rm -r install`  
`rm -r log`  

`colcon build --packages-select custom_interfaces`  
`source install/setup.bash`  
`colcon build --symlink-install --packages-select semifinal`  
`source install/setup.bash`




## Conexión wifi

- Conectarse al router PIROBOTNET
- conectar robot
- ssh pi@192.168.0.114
- qwerty
- estamos dentro



### Otros

`ros2 topic pub cmd_vel geometry_msgs/Twist "{linear: {x: 0.1, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"`

`ros2 topic pub -1 /set_velocity custom_interfaces/SetVelocity "{id: 1, velocity: 50}"`  
(Id puede ser 1 o 2, y velocity puede ser + o -)

`ros2 run movement keyboard_teleop`

