# UJI ROBOTICS - ASTI2024

## ¿Cómo ejecutar un programa ya hecho en el CyberCrex o en el portátil?

### Pequeño tutorial de WIFI en la raspberry

- /etc/netplan/01-network-manager-all.yaml
  - La configuración de la red wifi está en este archivo.
  - Esta permite que se conecten automaticamente a la red wifi que se quiera. Si no se pone aquí, se tendría que usar una pantalla externa, iniciar sesión... De esta forma se puede conectar a la red wifi deseada sin necesidad de hacer nada en la raspberry, ni siquiera iniciar sesión.
  - Para cambiar esta configuración, habrá que conectarse a una pantalla, o entrar desde un wifi ya agregado

- Escribir wifis en el archivo de configuración de la red wifi, el nuestro está así:

```yaml
network:
  version: 2
  renderer: NetworkManager
  wifis:
    wlan0:
      optional: true
      access-points:
        "ego":    # Nombre de la red
          password: "lo_que_toque"  # Contraseña
        "PIROBOTNET": {}    # No necesita contraseña, es abierto
        # etc... poner todos los que haga falta
      dhcp4: true
```

- Aplicar los cambios:
  - `sudo netplan apply`



### Opción 100% Raspberry Pi

1. Conectar raspberry a la corriente / batería.
2. Entrar en la raspberry con ssh desde otro portátil conectado a la misma red wifi.
  - ADRIA: `ssh -X pi@192.168.224.104` 
  - `ssh -X pi@192.168.113.22`            `ssh -X pi@192.168.245.104`           (`ssh -X pi@192.168.0.114` antes era este pero ya no)
  - Contraseña: `qwerty`
  - La red wifi puede ser el router PIROBOTNET o los datos de cualquier móvil ya configurado.
  - Para conectarlo a los datos de un móvil o red no configurada, habría que hacerlo desde la interfaz gráfica de la raspberry, conectando un teclado, ratón y monitor... O conectando cable ethernet.

1. Sources.
  - `cd Documents/asti2024/retos_ws`
  - `source install/setup.bash`  (O usar el alias `src` para hacerlo más rápido)

2. Ejecutar el programa deseado.
  - `ros2 run bringup motor_vel_controller`     (Para activar los motores, no se debe cerrar esta terminal. Asegurarse de que está los motores están bloqueados girando las ruedas con la mano)
  - `ros2 run final dibuja_figura`  (En otra terminal, aquí sería donde se ejecutaría el programa deseado)

3. En caso de error:
  - Hacer sources:
    - `source /opt/ros/foxy/setup.bash`
    - `source install/setup.bash`
  - Compilar el código en la raspberry, en principio ya estará siempre hecho.
    - `colcon build --packages-select custom_interfaces`
    - `colcon build --packages-select dynamixel_sdk`
    - `source install/setup.bash`
    - `colcon build --symlink-install --packages-select bringup`
    - Compilar los programas restantes (final...)
    - `source install/setup.bash`
  - Si sigue sin funcionar
    - `sudo usermod -aG dialout pi`
    - `sudo chmod 777 /dev/ttyUSB0`  (o el puerto que sea, podría ser ttyUSB1, ttyUSB2, etc. Usar `ls /dev/ttyUSB*` para verlo)
    - Reiniciar la controladora de motores. Conectar USB + energia.
    - Cambiar el usb de sitio.

4. Para cerrar la raspberry:
  - `sudo shutdown now`  (Esperar a que se apague la luz roja del led de la raspberry, no desconectar la corriente antes de que se apague)

5. Para cerrar la conexión ssh:
  - `exit`


### Testeos en el portátil

1. Descargar repositorio.
2. Compilar el código
  - `colcon build --packages-select custom_interfaces`
  - `colcon build --packages-select dynamixel_sdk`
  - `source install/setup.bash`
  - `colcon build --symlink-install --packages-select bringup`
  - `source install/setup.bash`
  - `colcon build --symlink-install --packages-select pruebas final`   (etc... Todos los paquetes que se quieran usarS)
3. Ejecutar el programa deseado.
  - `ros2 run final dibuja_figura`  (En otra terminal, aquí sería donde se ejecutaría el programa deseado)

4. Si queremos probar los motores
  - Habrá que conectar la controladora de motores al portátil con un cable USB.
  - También habrá que dar energía a la controladora con otro USB o con una batería.  La opción del USB sería mediante un arduino que proporciona 5V.
  - (Falta explicar mejor)
  - Dar permisos al ordenador para controlar los motores
  - `sudo usermod -aG dialout pi`
  - `sudo chmod 777 /dev/ttyUSB0`  (o el puerto que sea, podría ser ttyUSB1, ttyUSB2, etc. Usar `ls /dev/ttyUSB*` para verlo)
  - `ros2 run bringup motor_vel_controller`  (Para activar los motores, no se debe cerrar esta terminal. Asegurarse de que está los motores están bloqueados girando las ruedas con la mano)
  - Si no funciona, cambiar el usb de sitio o ejecutar `ros2 run bringup motor_vel_controller /dev/ttyUSB0`  (o el puerto que sea, podría ser ttyUSB1, ttyUSB2, etc. Usar `ls /dev/ttyUSB*` para verlo)


## ¿Cómo hacer un nuevo programa?

1. Crear el programa en la carpeta `asti2024/retos_ws/src/pruebas/pruebas` (por ejemplo)
2. Si el programa es en python, ir a `asti2024/retos_ws/src/pruebas/setup.py` y añadir el nombre del programa en la lista de `entry_points`.
3. Ejemplo:
  ```python
  entry_points={
      'console_scripts': [
          'test_vision_gazebo = pruebas.test_vision_gazebo:main',
          'distance_sensor = pruebas.distance_sensor:main',
      ],
  },
  ```
4. Compilar el código
  - `colcon build --symlink-install --packages-select pruebas`
  - `source install/setup.bash`

5. No haría falta compilar cada cambio que hagamos al programa si habíamos usado --symlink-install para compilar, pero si se cambia el `setup.py` otra vez sí que habría que hacerlo.


## Otros comandos

`ros2 topic pub cmd_vel geometry_msgs/Twist "{linear: {x: 0.1, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"`

`ros2 topic pub -1 /set_velocity custom_interfaces/SetVelocity "{id: 1, velocity: 50}"`  
(Id puede ser 1 o 2, y velocity puede ser + o -)

`ros2 run teleop_twist_keyboard teleop_twist_keyboard`

