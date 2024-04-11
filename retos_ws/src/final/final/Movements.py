import time
import math
from rclpy.node import Node
from geometry_msgs.msg import Twist
from final.Sensors import Sensors
try:
    import RPi.GPIO as GPIO
    GPIO_ENABLED = True
except ImportError:
    print("RPi.GPIO no encontrada en el sistema")
    GPIO_ENABLED = False

    
def get_movements():
    # TODO: Importante mantener actualizado get_movements() con las funciones que se vayan añadiendo
    # 1r argumento: Nombre de la función
    # 2o argumento: Lista de argumentos obligatorios
    # 3r argumento: Lista de argumentos opcionales
    
    actions = {
        "0": ('prueba_movimientos', [], []), 
        "w": ('avanzar', [], []),
        "x": ('retroceder', [], []),
        "a": ('girar_izquierda', [], []),
        "d": ('girar_derecha', [], []),
        "s": ('detener', [], []),
        "ww": ('avanzar_distancia', ['distancia_m'], []),          # Hay aceleración como opcional, pero se descarta por ahora, lo complica mucho
        "xx": ('retroceder_distancia', ['distancia_m'], []),
        "aa": ('girar_grados_izq', ['grados'], ['radio']),
        "dd": ('girar_grados_der', ['grados'], ['radio']),
        "zz": ('girar_grados_izq_atras', ['grados'], ['radio']),
        "cc": ('girar_grados_der_atras', ['grados'], ['radio']),
        "v": ('actualizar_vel_lineal', ['max_linear_vel'], []),
        "b": ('actualizar_vel_angular', ['max_angular_vel'], []),
        "vv": ('actualizar_acc_lineal', ['linear_acc'], []),
        "bb": ('actualizar_acc_angular', ['angular_acc'], []),
    }
    return actions
    
def get_servo_movements():
    actions_servo = {       # TODO
        "ñw": ('boli_subir', [], []),
        "ñs": ('boli_bajar', [], []),
        "lw": ('bolos_soltar', [], []),
        "ls": ('bolos_mantener', [], []),
        "pw": ('pale_subir', [], []),
        "ps": ('pale_bajar', [], []),
        "ca": ('ver_angulos_herramienta', [], []),
    }
    return actions_servo

class Servo():
    def __init__(self):
        # Grados
        self.servo_current_angle = 0.0
        
        self.grados_boli_alto = 15.0     # TODO: Cambiar a valor correcto para cada prueba
        self.grados_boli_bajo = 0.0     
        self.grados_bolos_soltar = 50.0
        self.grados_bolos_mantener = 0.0
        self.grados_pale_alto = 10.0       # TODO: Cambiar
        self.grados_pale_bajo = 0.0
    
        # Tiempos
        self.tiempo_boli_subir = 0.1
        self.tiempo_boli_bajar = 0.1
        
        self.tiempo_bolos_soltar = 0.1
        self.tiempo_bolos_mantener = 0.5
        
        self.tiempo_pale_subir = 2.5
        self.tiempo_pale_bajar = 2
        
        try:
            self.setup_servo()
        except:
            print("RPi.GPIO no conectada correctamente")
            global GPIO_ENABLED
            GPIO_ENABLED = False
                
    def setup_servo(self):
        self.PIN_SERVO = 11
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(self.PIN_SERVO, GPIO.OUT)
        self.servo = GPIO.PWM(self.PIN_SERVO, 50)  # 50Hz pulse for servo
        self.servo.start(0)
        

            
    # ╔═════════════════════════════╗
    # ║       TOOL FUNCTIONS        ║
    # ╚═════════════════════════════╝

    def herramienta_girar(self, grados_objetivo, tiempo=0):
        """
        tiempo: Tiempo que tarda en llegar al angulo objetivo (opcional)
        """
        
        if tiempo <= 0:
            step = abs(grados_objetivo - self.servo_current_angle)
            delay = 0
        else:
            step = abs(grados_objetivo - self.servo_current_angle) / (tiempo / 0.05)
            delay = 0.05
            
        direction = 1 if grados_objetivo > self.servo_current_angle else -1     # Evitamos que el servo gire en sentido contrario

        # Gradually move the servo to the target angle
        while abs(self.servo_current_angle - grados_objetivo) > step:
            self.servo_current_angle += direction * step
            duty_cycle = self.servo_angle_to_duty_cycle(self.servo_current_angle)
            self.servo.ChangeDutyCycle(duty_cycle)                                      if GPIO_ENABLED else None
            print(f"Moving angle: {self.servo_current_angle}")
            time.sleep(delay)
            
        self.servo_current_angle = grados_objetivo
        duty_cycle = self.servo_angle_to_duty_cycle(self.servo_current_angle)
        self.servo.ChangeDutyCycle(duty_cycle)                                        if GPIO_ENABLED else None
        print(f"Final angle: {self.servo_current_angle}")
        # time.sleep(0.5)
        # self.servo.ChangeDutyCycle(0)                                                 if GPIO_ENABLED else None

    
    def ver_angulos_herramienta(self):
        """
        Funcion que va pidiendo angulos.
        Permite que el usuario introduzca un ángulo para observar en la herramienta.
        El ángulo debe estar entre 0 y 180 grados.
        El usuario también puede introducir un tiempo para llegar al ángulo.
        Si no se especifica un tiempo, se utilizará un valor predeterminado de 2 segundos.
        """

        while True:
            print(f"Ángulo actual: {self.servo_current_angle}")
            angle_sent = input("Introduce un ángulo para observar en la herramienta (Entre 0 y 180), Enter para salir: ")
            if angle_sent == "":
                break
            angle_sent = float(angle_sent)
            if angle_sent < 0 or angle_sent > 180:
                print("El ángulo debe estar entre 0 y 180 grados.")
                continue
            tiempo_sent = input("Introduce un tiempo para llegar al ángulo (0 o enter para llegar directamente): ")
            if tiempo_sent == "":
                tiempo_sent = 2   # 2 segundos por seguridad
            else:
                tiempo_sent = float(tiempo_sent)
            self.herramienta_girar(angle_sent, tiempo=tiempo_sent)     # Dar tiempo a reaccionar y que no se rompa algo otra vez
            self.servo_current_angle = angle_sent



    def servo_angle_to_duty_cycle(self, angle):
        """Convert an angle to the suitable duty cycle."""
        return 2 + (angle / 18)
    
    def change_duty_cycle(self, duty_cycle, prints=False):
        """
        Change the duty cycle of the servo.
        Args:
            duty_cycle (float): The duty cycle value.
            prints (bool, optional): If True, only print the duty cycle value without making any changes. Defaults to False.
        """
        if prints:
            print(f"Changing duty cycle to: {duty_cycle}")
        else:
            # Uncomment the following line to actually change the duty cycle
            # self.servo.ChangeDutyCycle(duty_cycle)
            print(f"Changing duty cycle to: {duty_cycle}")


    # Dibujar la figura
    def boli_subir(self, prints=False):
        if prints:
            print("Boli subido")
        self.herramienta_girar(self.grados_boli_alto, self.tiempo_boli_subir)

    def boli_bajar(self, prints=False):
        if prints:
            print("Boli bajado")
        self.herramienta_girar(self.grados_boli_bajo, self.tiempo_boli_bajar)
    
    # Bolos
    def bolos_soltar(self, prints=False):
        if prints:
            print("Bolos soltados")
        self.herramienta_girar(self.grados_bolos_soltar, self.tiempo_bolos_soltar)
        
    def bolos_mantener(self, prints=False):
        if prints:
            print("Bolos mantenidos")
        self.herramienta_girar(self.grados_bolos_mantener, self.tiempo_bolos_mantener)
        
    # Mini-fábrica
    def pale_subir(self, prints=False):
        if prints:
            print("Pale subido")
        self.herramienta_girar(self.grados_pale_alto, self.tiempo_pale_subir)
        
    def pale_bajar(self, prints=False):
        if prints:
            print("Pale bajado")
        self.herramienta_girar(self.grados_pale_bajo, self.tiempo_pale_bajar)






class Movements(Node):
    def __init__(self):
        """
        Clase Personalizada para el manejo de movimientos del robot.
        
        SETUP INICIAL:
        from .Movements import Movements      (Falta ver si este tipo de importación en la raspberry funciona bien)
        mov = Movements()
        
        EJEMPLO RUEDAS BÁSICOS:
        mov.avanzar()
        mov.retroceder()
        mov.girar_izquierda()
        mov.girar_derecha()
        mov.detener()
        mov.prueba_movimientos()      (Avanzar, retroceder, girar izquierda, girar derecha, detenerse -> Para probar que todo funciona bien)
        
        EJEMPLO RUEDAS POR DISTANCIAS, RADIOS, GRADOS:
        mov.avanzar_distancia(1)        (Avanzar 1 metro)
        mov.retroceder_distancia(1)
        mov.girar_grados_izq(90)        (Radio 0.0 por defecto)
        mov.girar_grados_der(75)
        mov.girar_grados_izq(90, radio=1.5)
        
        EJEMPLO MODIFICAR VELOCIDADES:
        mov.actualizar_vel_lineal(0.1)
        mov.actualizar_vel_angular(1)
        mov.actualizar_acc_lineal(0.01)
        mov.actualizar_acc_angular(0.1)
        
        EJEMPLO TOOLS:
        mov.herramienta_girar(90)
        mov.boli_subir()
        mov.boli_bajar()
        mov.bolos_soltar()
        mov.bolos_mantener()
        mov.pale_subir()
        mov.pale_bajar()
        """
        
        super().__init__('movement_publisher')
        self.wheel_publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)   # cmd_vel has (m/s , rad/s)

        # WHEELS
        self.obj_linear_vel = 0.2       # TODO: CAMBIAR A VALORES QUE SEAN BUENOS POR DEFECTO
        self.obj_angular_vel = 0.7
        self.linear_acc = 0.01
        self.angular_acc = 0.1
        self.last_vel = (0.0, 0.0)
        
        # TOOL
        # self.servo_current_angle = 0.0

        # self.grados_boli_alto = 20.0     # TODO: Cambiar a valor correcto para cada prueba
        # self.grados_boli_bajo = -20.0     
        # self.grados_bolos_soltar = 50.0
        # self.grados_bolos_mantener = 0.0
        # self.grados_pale_alto = 10.0        # TODO: Cambiar
        # self.grados_pale_bajo = -10.0   
        
        # # Tiempos
        # self.tiempo_boli_subir = 1.5
        # self.tiempo_boli_bajar = 1.5
        
        # self.tiempo_bolos_soltar = 0.1
        # self.tiempo_bolos_mantener = 0.5
        
        # self.tiempo_pale_subir = 2.5
        # self.tiempo_pale_bajar = 2
        
        # # Activar pin del servo (si se puede)
        # if GPIO_ENABLED:
        #     self.PIN_SERVO = 11
        #     self.servo = self.setup_servo()  
        
        # SENSORS
        sensors = Sensors()     # TODO

    # ╔═════════════════════════════════════════╗
    # ║ MOVIMIENTOS BÁSICOS RUEDAS CON SENSORES ║
    # ╚═════════════════════════════════════════╝
    
    def detectar_pared(self, sensors, dist = 20):
        print(f'Distancia delante: {sensors.distancia_delante} cm')
        distancias = sensors.get_distancias()
        if (distancias < dist).any():
            print("Muy cerca de un obstáculo")
            return True
        else:
            print("No hay obstáculos")
            return False

    def avanzar_hasta_pared(self, sensors):
        while True:
            if self.detectar_pared(sensors):
                break
            self.avanzar()
        self.detener()

    def girar_derecha_hasta_despejar(self, sensors):
        while True:
            if not self.detectar_pared(sensors):
                break
            self.girar_derecha()
        self.detener()

    def girar_izquierda_hasta_despejar(self, sensors):
        while True:
            if not self.detectar_pared(sensors):
                break
            self.girar_izquierda()
        self.detener()

    def avanzar_derecha(self, sensors):
        """
        Se mantiene a la derecha
        -1 en caso de obstaculo
        """
        distancias = sensors.distancias()
        compar_der_e = self.comparar_distancias(sensors.get_distancia_derecha(), distancias[3], 0.05)
        compar_izq_e = self.comparar_distancias(sensors.get_distancia_izquierda(), distancias[0], 0.05)
        compar_der = self.comparar_distancias(sensors.get_distancia_derecha(), distancias[3], 0.2)
        while self.detectar_pared(sensors, 0.05):
            # Enderezar el robot para mantenerlo paralelo
            if compar_der_e:  # TODO: Mirar el radio de curvatura
                if compar_der_e == 2: # Nos vamos para la derecha
                    grados = sensors.get_distancia_delante_der() / sensors.get_distancia_derecha() 
                    self.girar_grados_izq(grados)
                else: # Nos vamos para la izquierda
                    grados = sensors.get_distancia_delante_izq() / sensors.get_distancia_izquierda()
                    self.girar_grados_der(grados)   
            elif compar_izq_e:    # TODO No es necesario del todo, pero sirve para asegurar
                if compar_izq_e == 2: # Nos vamos para la izquierda
                    grados = sensors.get_distancia_delante_izq() /sensors.get_distancia_izquierda()
                    self.girar_grados_der(grados) 
                elif compar_der == 1:
                    grados = sensors.get_distancia_delante_der() / sensors.get_distancia_derecha() 
                    self.girar_grados_izq(grados)
            
            # Busca los caminos por la derecha
            
            if compar_der:
                if compar_der == 2: # No hay nada en el lado
                    self.girar_grados(90, 0.3)  # Girar 90 grados con un radio de 30 cm
                    if sensors.get_distancia_delante < 0.5 and sensors.get_distancia_derecha() < 0.1: # TODO Comprobar valores
                        self.girar_grados(90, 0.2)
            elif distancias[1] < 0.3 and distancia[2] < 0.3:
                if distancias[0] < 0.25:    # Suponemos que para la derecha no hay camino
                    self.girar_grados(180)
                else:
                    self.girar_grados_izq(90)
            else:
                self.avanzar()
            distancias = sensors.distancias()
            compar_der = self.comparar_distancias(sensors.get_distancia_derecha(), distancias[3], 0.5)
        self.detener()
        return -1
        
    def comparar_distancias(self, distancia1, distancia2, error=0.1):
        """
        Sirve para comparar distancias con un límite de error
        Return: 0 si son iguales, 1 si distancia1 es mayor, 2 si distancia2 es mayor
        """
        diferencia = distancia2 - distancia1
        if diferencia > error:
            return 2
        elif abs(diferencia) < error:
            return 0
        else:
            return 1
    
    # SCALLBACK
        
    def callback_derecha(self, msg):
        self.distancia_der = msg.data

    def callback_izquierda(self, msg):
        self.distancia_izq = msg.data

    # WHEELS
    
    def publish_wheel_velocity(self, vel_x, vel_y):
        msg = Twist()
        msg.linear.x = vel_x
        msg.angular.z = vel_y
        self.wheel_publisher_.publish(msg)
        
        self.last_vel = (vel_x, vel_y)
        # self.show_current_vel()       # Comentar si no se quiere mostrar la velocidad actual

    def show_current_vel(self):
        self.get_logger().info(f'Current velocity: {self.last_vel}')
        
    def actualizar_vel_lineal(self, max_linear_vel):
        self.obj_linear_vel = max_linear_vel
    
    def actualizar_vel_angular(self, max_angular_vel):
        self.obj_angular_vel = max_angular_vel
    
    def actualizar_acc_lineal(self, linear_acc):
        self.linear_acc = linear_acc
    
    def actualizar_acc_angular(self, angular_acc):
        self.angular_acc = angular_acc

    # ╔═════════════════════════════╗
    # ║ MOVIMIENTOS BÁSICOS RUEDAS  ║
    # ╚═════════════════════════════╝
    
    def avanzar(self):
        self.publish_wheel_velocity(self.obj_linear_vel, 0.0)
        
    def retroceder(self):
        self.publish_wheel_velocity(-self.obj_linear_vel, 0.0)
        
    def girar_izquierda(self):
        self.publish_wheel_velocity(0.0, self.obj_angular_vel)
        
    def girar_derecha(self):
        self.publish_wheel_velocity(0.0, -self.obj_angular_vel)
        
    def detener(self):
        self.publish_wheel_velocity(0.0, 0.0)

    # ╔═══════════════════════╗
    # ║ PRUEBA DE MOVIMIENTOS ║
    # ╚═══════════════════════╝
    
    def prueba_movimientos(self):
        print(f'Vel lin: {self.obj_linear_vel} m/s')
        print(f'Vel ang: {self.obj_angular_vel} rad/s')
        print(f'Acc lin: {self.linear_acc} m/s^2')
        print(f'Acc ang: {self.angular_acc} rad/s^2')
        print("Realizando prueba de movimiento...")
        
        # Move forward
        self.avanzar()
        time.sleep(1.0)
        
        # Move backward
        self.retroceder()
        time.sleep(1.0)
        
        # Turn left
        self.girar_derecha()
        time.sleep(1.0)
        
        # Turn right
        self.girar_izquierda()
        time.sleep(1.0)
        
        # Stop
        self.detener()

    # ╔═══════════════════════════════════════════╗
    # ║ FUNCIONES POR DISTANCIAS, RADIOS O GRADOS ║
    # ╚═══════════════════════════════════════════╝
    
    def avanzar_distancia(self, distancia_m, aceleracion=False):
        ## TODO: Opción de cancelar el movimiento a mitad de camino.
        vel_linear = 0.0
        while(distancia_m >= 0):
            if aceleracion:
                if (vel_linear < self.obj_linear_vel):
                    vel_linear += self.linear_acc
            else:
                vel_linear = self.obj_linear_vel
            self.publish_wheel_velocity(vel_linear, 0.0)
            time.sleep(0.1)
            distancia_recorrida = vel_linear*0.1
            distancia_m -= distancia_recorrida
        self.detener()

    def retroceder_distancia(self, distancia_m, aceleracion=False):
        vel_linear = 0.0
        while(distancia_m >= 0):
            if aceleracion:
                if (vel_linear < self.obj_linear_vel):
                    vel_linear += self.linear_acc
            else:
                vel_linear = self.obj_linear_vel
            self.publish_wheel_velocity(-vel_linear, 0.0)
            time.sleep(0.1)
            distancia_recorrida = vel_linear*0.1
            distancia_m -= distancia_recorrida
        self.detener()


    def girar_grados_izq(self, grados, radio=0.0):
        self.girar_grados(grados, radio)
        
    def girar_grados_der(self, grados, radio=0.0):
        self.girar_grados(-grados, radio)
        
    def girar_grados_izq_atras(self, grados, radio=0.0):
        self.girar_grados(grados, radio, avanzar=False)

    def girar_grados_der_atras(self, grados, radio=0.0):
        self.girar_grados(-grados, radio, avanzar=False)

    def girar_avanzar(self, angulo, distancia_m):

        # GIRAMOS EL ROBOT
        if angulo < 0:
            self.girar_grados_izq(abs(angulo))
        elif angulo > 0:
            self.girar_grados_der(abs(angulo))

        # AVANZAMOS LA DISTANCIA
        self.avanzar_distancia(distancia_m)
        self.detener()


    def girar_grados(self, grados, radio=0.0, avanzar=True):
        # TODO: Testear que está bien, ya que no lo he probado
        """Rotar el robot en un angulo determinado
        Args:
            grados (int): Grados a rotar (en decimal), izquierda+ o derecha-
            radio (float, optional): Si se quiere realizar el movimiento con un radio. Defaults to 0.0
            avanzar (bool, optional): Si se quiere que el giro se haga avanzando o retrocediendo. Defaults to True.
        """
        if grados == 0:
            return
        
        # Grados -> Radianes
        radian = abs(grados*math.pi/180)
        
        # Determinar velocidades lineal y angular
        if radio == 0.0:
            vel_lin = 0.0
            ang_vel = self.obj_angular_vel
        else:
            vel_lin = self.obj_linear_vel if avanzar else -self.obj_linear_vel
            ang_vel = vel_lin/radio
        
        # Sentido de giro?
        if grados < 0:
            ang_vel *= -1
        else:
            ang_vel *= 1
        
        # MOVIMIENTO
        radians_done = 0.0
        '''start_time = time.time()'''

        while radians_done < radian:
            self.publish_wheel_velocity(vel_lin, ang_vel)
            time.sleep(0.1)
            radian_increment = abs(ang_vel) * 0.1
            radians_done += radian_increment
            '''
            current_time = time.time()
            elapsed = current_time - start_time
            radians_done = abs(ang_vel) * elapsed
            '''
            # TODO: Odometria para saber cuánto ha girado??
        
        self.detener()



    # # ╔═════════════════════════════╗
    # # ║       TOOL FUNCTIONS        ║
    # # ╚═════════════════════════════╝

    # def herramienta_girar(self, grados_objetivo, tiempo=0):
    #     """
    #     tiempo: Tiempo que tarda en llegar al angulo objetivo (opcional)
    #     """
        
    #     if tiempo <= 0:
    #         step = abs(grados_objetivo - self.servo_current_angle)
    #         delay = 0
    #     else:
    #         step = abs(grados_objetivo - self.servo_current_angle) / (tiempo / 0.05)
    #         delay = 0.05
            
    #     direction = 1 if grados_objetivo > self.servo_current_angle else -1     # Evitamos que el servo gire en sentido contrario

    #     # Gradually move the servo to the target angle
    #     while abs(self.servo_current_angle - grados_objetivo) > step:
    #         self.servo_current_angle += direction * step
    #         duty_cycle = self.servo_angle_to_duty_cycle(self.servo_current_angle)
    #         self.servo.ChangeDutyCycle(duty_cycle)                                      if GPIO_ENABLED else None
    #         print(f"Moving angle: {self.servo_current_angle}")
    #         time.sleep(delay)

    #     # Ensure the servo reaches the exact target angle
    #     self.servo_current_angle = grados_objetivo
    #     duty_cycle = self.servo_angle_to_duty_cycle(self.servo_current_angle)
    #     self.servo.ChangeDutyCycle(duty_cycle)                                        if GPIO_ENABLED else None
    #     print(f"Final angle: {self.servo_current_angle}")
    #     time.sleep(0.5)
    #     self.servo.ChangeDutyCycle(0)                                                 if GPIO_ENABLED else None

    
    # def ver_angulos_herramienta(self):
    #     """
    #     Funcion que va pidiendo angulos.
    #     Permite que el usuario introduzca un ángulo para observar en la herramienta.
    #     El ángulo debe estar entre 0 y 180 grados.
    #     El usuario también puede introducir un tiempo para llegar al ángulo.
    #     Si no se especifica un tiempo, se utilizará un valor predeterminado de 2 segundos.
    #     """

    #     while True:
    #         print(f"Ángulo actual: {self.servo_current_angle}")
    #         angle_sent = input("Introduce un ángulo para observar en la herramienta (Entre 0 y 180), Enter para salir: ")
    #         if angle_sent == "":
    #             break
    #         angle_sent = float(angle_sent)
    #         if angle_sent < 0 or angle_sent > 180:
    #             print("El ángulo debe estar entre 0 y 180 grados.")
    #             continue
    #         tiempo_sent = input("Introduce un tiempo para llegar al ángulo (0 o enter para llegar directamente): ")
    #         if tiempo_sent == "":
    #             tiempo_sent = 2   # 2 segundos por seguridad
    #         else:
    #             tiempo_sent = float(tiempo_sent)
    #         self.herramienta_girar(angle_sent, tiempo=tiempo_sent)     # Dar tiempo a reaccionar y que no se rompa algo otra vez
    #         self.servo_current_angle = angle_sent


    # def setup_servo(self):
    #     GPIO.setmode(GPIO.BOARD)  # Use physical pin numbering
    #     GPIO.setup(self.PIN_SERVO, GPIO.OUT)
    #     servo = GPIO.PWM(self.PIN_SERVO, 50)  # 50Hz pulse for servo
    #     servo.start(0)  # DEFAULT POSITION
    #     return servo

    # def servo_angle_to_duty_cycle(self, angle):
    #     """Convert an angle to the suitable duty cycle."""
    #     return 2 + (angle / 18)
    
    # def change_duty_cycle(self, duty_cycle, prints=False):
    #     """
    #     Change the duty cycle of the servo.
    #     Args:
    #         duty_cycle (float): The duty cycle value.
    #         prints (bool, optional): If True, only print the duty cycle value without making any changes. Defaults to False.
    #     """
    #     if prints:
    #         print(f"Changing duty cycle to: {duty_cycle}")
    #     else:
    #         # Uncomment the following line to actually change the duty cycle
    #         # self.servo.ChangeDutyCycle(duty_cycle)
    #         print(f"Changing duty cycle to: {duty_cycle}")


    # # Dibujar la figura
    # def boli_subir(self, prints=False):
    #     if prints:
    #         print("Boli subido")
    #     self.herramienta_girar(self.grados_boli_alto, self.tiempo_boli_subir)

    # def boli_bajar(self, prints=False):
    #     if prints:
    #         print("Boli bajado")
    #     self.herramienta_girar(self.grados_boli_bajo, self.tiempo_boli_bajar)
    
    # # Bolos
    # # def bolos_soltar(self, prints=False):
    # #     if prints:
    # #         print("Bolos soltados")
    # #     self.herramienta_girar(self.grados_bolos_soltar, self.tiempo_bolos_soltar)
        
    # # def bolos_mantener(self, prints=False):
    # #     if prints:
    # #         print("Bolos mantenidos")
    # #     self.herramienta_girar(self.grados_bolos_mantener, self.tiempo_bolos_mantener)
        
    # # Mini-fábrica
    # def pale_subir(self, prints=False):
    #     if prints:
    #         print("Pale subido")
    #     self.herramienta_girar(self.grados_pale_alto, self.tiempo_pale_subir)
        
    # def pale_bajar(self, prints=False):
    #     if prints:
    #         print("Pale bajado")
    #     self.herramienta_girar(self.grados_pale_bajo, self.tiempo_pale_bajar)
