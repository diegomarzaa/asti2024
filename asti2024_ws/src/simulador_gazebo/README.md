# Trabajar con el simulador_gazebo

1. Para abrir el simulador de gazebo:
   - `ros2 launch simulador_gazebo launch_sim.launch.py`
   
2. El simulador te pedira que des el modelo del robot tienes dos opciones, (temporalmente hasta que se actualice el simulador):

3. Para usar un world especifico:
    - `ros2 launch simulador_gazebo launch_sim.launch.py world:=dirección_del_mundo`
    - Ejemplo: `ros2 launch simulador_gazebo launch_sim.launch.py world:=./src/simulador_gazebo/worlds/cuadricula.world`
   
   ### Nota:
    - Si se quiere usar un mundo que no esté en la carpeta `simulador_gazebo/worlds`, habrá que poner la dirección completa del mundo.
    - Se debe tener en cuenta que el mundo debe ser un archivo `.world` y que debe estar en la carpeta `simulador_gazebo/worlds`.
    - Tambien se debe de poner el ./src/ para que el simulador lo encuentre, (si estas ejecutando el codigo desde el ws).

4. Para mover el robot:
    - `ros2 run teleop_twist_keyboard teleop_twist_keyboard`

5. Modificaciones en el mundo:
    - Para modificar el mundo, se debe de modificar el archivo `.world` que se encuentra en la carpeta `simulador_gazebo/worlds`.
    - Para modificar el robot, se debe de modificar el archivo `.xacro` que se encuentra en la carpeta 
    - `simulador_gazebo/models/modelo a elegir/robot_core.xacro`.
    - Para modificar la camara, se debe de modificar el archivo `.xacro` que se encuentra en la carpeta
    - `simulador_gazebo/models/modelo a elegir/camera.xacro`.

## Errores comunes

1. Si launch_sim.launch.py da error, comprueba si esta instalado twis:
    - `sudo apt install ros-foxy-twist-mux`

2. Si no funciona cmd_vel, cuando agregas un nuevo modelo, comprueba que el modelo tenga el plugin de cmd_vel en 
   gazebo_control.xacro:
    - `<plugin name="diff_drive" filename="libgazebo_ros_diff_drive.so">`

   ### Si lo tiene y sigue sin funcionar, comprueba que el paquete al que esta llamando en ros2_control.xacro sea el correcto.
   ### Busca la siguiente linea en el archivo xacro, y comprueba que el nombre del paquete sea el correcto:
    - `<parameters>$(find simulador_gazebo)/config/my_controllers.yaml</parameters>`
    - `<parameters>$(find simulador_gazebo)/config/gaz_ros2_ctl_use_sim.yaml</parameters>`

3. Si el programa gazebo se queda pegado:
    - `Ctrl + C` para cerrar el programa.
    - Ve a la carpeta `~/.gazebo` y elimina la carpeta `models`.
    - Luego cambiala por la carpeta 'models' que esta en `simulador_gazebo/solo_modifica_gazebo`.
