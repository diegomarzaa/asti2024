# ASTI 2024 - UJI ROBOTICS TEAM 🚀

## ¿Qué es esto?

Bienvenidos al repositorio usado para quedar en segunda posición en el ASTI2024. Esto contiene todos los paquetes de los programas usados para las diferentes pruebas. Entre ello tenemos:

- ROS2 Foxy con integración para TurtleBot3.
- Simulación en Gazebo.
- Paquetes ROS.
- Un Docker para que no corrompas tu portátil.

## Prerrequisitos

Sistema operativo:
- Cualquiera, mientras soporte Docker (En principio este repositorio necesitaría Ubuntu 20.04 con ROS2 Foxy, que es el que se usó para el desarrollo, pero como en las asignaturas de la carrera se recomienda instalar ubuntu 22.04 con Humble, el uso de docker ayuda a no tener problemas de compatibilidad). 

Primero asegúrate de tener instalado en tu sistema:
- [Git](https://git-scm.com/downloads) (para clonar este repositorio).
- [Docker](https://docs.docker.com/get-docker/) (es como una máquina virtual para no corromper tu portátil en las instalaciones, en [este tutorial](https://youtu.be/SAMPOK_lazw?si=vdhNN9obfBl0p4j3) y [este otro](https://youtu.be/XcJzOYe3E6M?si=rwoMkr18-vI-5y3D&t=765) podrás aprender más sobre como funciona docker y su importancia.)


## Como empezar?

### 1. Clona este repositorio

```bash
git clone https://github.com/diegomarzaa/asti2024.git
cd asti2024
```

### 2. Construye la imagen de Docker

Usa el `Dockerfile` incluido para construir tu entorno de trabajo, que ya viene con todo lo que necesitas:

```bash
sudo docker build -t asti2024 .
```

Esto te creará una imagen llamada `asti2024` con todo el caos controlado de ROS2 y los paquetes que necesitas. Puedes ver si se ha creado correctamente con:

```bash
sudo docker images
```

y deberías ver algo similar a esto:

```bash
REPOSITORY          TAG       IMAGE ID       CREATED            SIZE
asti2024            latest    535bcd5d8e02   14 seconds ago     900MB
```

### 3. Ejecuta el contenedor

```bash
chmod +x abrir-workspace-asti.sh
./abrir-workspace-asti.sh
```

Al ser la primera vez, se compilarán automáticamente todos los archivos del entorno de ROS2, y tras esto, estarás dentro del contenedor.

---

## Usar el entorno Docker 🐳

En el contenedor abierto con el script `abrir-workspace-asti.sh` cualquier cambio que hagas en la carpeta `asti2024_ws` también se hará en tu portátil. Así podrás hacer las modificaciones que quieras en tu código sin miedo a romper nada.

- [ ] Aún en fase beta, falta implementar:

También está la alternativa `abrir-workspace-asti-tests.sh` que abre el contenedor totalmente aislado, podrás modificar lo que quieras sin miedo a romper nada, ya que cuando salgas del contenedor, todo lo que hayas hecho se perderá.

### Ejemplo de uso 🎯

#### Corre la simulación de TurtleBot3

- [ ] Falta implementar el TurtleBot3 en la simulación, con el docker aún no se instalan todos los paquetes del turtlebot, solo van los programas hechos por nosotros que se encuentran en la carpeta

```bash
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

#### Corre el keyboard teleop para mover el robot

```bash
ros2 run final keyboard_teleop
```

---

## Problemas y Cosas que podrían salir mal 💥

- **Permisos denegados en dispositivos USB**:

  ```bash
  sudo chmod 777 /dev/ttyUSB0
  ```

- **Fallos al compilar**: asegúrate de que has sourceado todo bien:

  ```bash
  source /opt/ros/foxy/setup.bash
  source /root/asti2024/asti2024_ws/install/setup.bash
  ```
















## Comandos Comunes 🎮

- **Rosbags**

```bash
# Grabar movimientos
ros2 bag record /cmd_vel
```

```bash
# Reproducir movimientos
ros2 bag play rosbag2_<fecha>/rosbag2_<fecha>.db3
```

- **IP de la Raspberry** (en cualquier wifi, sin monitor)
    - [ ] Falta mirar mejor esto

```bash
sudo apt-get install arp-scan
sudo arp-scan --interface=wlo1 --localnet
```

- Si sigue sin funcionar
  - `sudo usermod -aG dialout pi`
  - `sudo chmod 777 /dev/ttyUSB0`  (o el puerto que sea, podría ser ttyUSB1, ttyUSB2, etc. Usar `ls /dev/ttyUSB*` para verlo)
  - Reiniciar la controladora de motores. Conectar USB + energia.
  - Cambiar el usb de sitio.

- **Testeo de motores**
    - `ros2 run final keyboard_teleop`
    - `ros2 run teleop_twist_keyboard teleop_twist_keyboard`
    - `ros2 topic pub cmd_vel geometry_msgs/Twist "{linear: {x: 0.1, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"`
    - `ros2 topic pub -1 /set_velocity custom_interfaces/SetVelocity "{id: 1, velocity: 50}"`  
    - (Id puede ser 1 o 2, y velocity puede ser + o -)

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


0. Para ver la IP de la raspberry en cualquier wifi sin usar un monitor:
  - sudo apt-get install arp-scan
  - sudo arp-scan --interface=wlo1 --localnet
1. Conectar raspberry a la corriente / batería.
2. Entrar en la raspberry con ssh desde otro portátil conectado a la misma red wifi.
  - ADRIA: `ssh -X pi@192.168.183.xxx` 
  - DIEGO: `ssh -X pi@192.168.54.22`

  - `ssh -X pi@192.168.113.22`            `ssh -X pi@192.168.245.104`           (`ssh -X pi@192.168.0.114` antes era este pero ya no)
  - Contraseña: `qwerty`
  - La red wifi puede ser el router PIROBOTNET o los datos de cualquier móvil ya configurado.
  - Para conectarlo a los datos de un móvil o red no configurada, habría que hacerlo desde la interfaz gráfica de la raspberry, conectando un teclado, ratón y monitor... O conectando cable ethernet.

3. Sources.
  - `cd Documents/asti2024/asti2024_ws`
  - `source install/setup.bash`  (O usar el alias `src` para hacerlo más rápido)

4. Ejecutar el programa deseado.
  - `ros2 run bringup motor_vel_controller`     (Para activar los motores, no se debe cerrar esta terminal. Asegurarse de que está los motores están bloqueados girando las ruedas con la mano)
  - `ros2 run final dibuja_figura`  (En otra terminal, aquí sería donde se ejecutaría el programa deseado)

5. En caso de error:
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

6. Para cerrar la raspberry:
  - `sudo shutdown now`  (Esperar a que se apague la luz roja del led de la raspberry, no desconectar la corriente antes de que se apague)

7. Para cerrar la conexión ssh:
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

1. Crear el programa en la carpeta `asti2024/asti2024_ws/src/pruebas/pruebas` (por ejemplo)
2. Si el programa es en python, ir a `asti2024/asti2024_ws/src/pruebas/setup.py` y añadir el nombre del programa en la lista de `entry_points`.
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


