import time
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
# from custom_interfaces.msg import SetPosition
from final.Sensors import Sensors

try:
    import RPi.GPIO as GPIO
except ImportError:
    print("RPi.GPIO not available")

# TODO: Importante mantener actualizado get_movements() con las funciones que se vayan añadiendo
# TODO: Cambiar a cm en vez de m, más rapido
    
def get_movements():
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
        "12": ('herramienta_girar', ['grados'], []),
        "13": ('boli_subir', [], []),
        "14": ('boli_bajar', [], []),
        "15": ('bolos_soltar', [], []),
        "16": ('bolos_mantener', [], []),
        "17": ('pale_subir', [], []),
        "18": ('pale_bajar', [], []),
        "v": ('actualizar_vel_lineal', ['max_linear_vel'], []),
        "b": ('actualizar_vel_angular', ['max_angular_vel'], []),
        "vv": ('actualizar_acc_lineal', ['linear_acc'], []),
        "bb": ('actualizar_acc_angular', ['angular_acc'], []),
    }
    return actions


class Movements(Node):
    def __init__(self, usar_herramienta=False):
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
        self.obj_linear_vel = 0.1       # TODO: CAMBIAR A VALORES QUE SEAN BUENOS POR DEFECTO
        self.obj_angular_vel = 0.7
        self.linear_acc = 0.01
        self.angular_acc = 0.1
        self.last_vel = (0.0, 0.0)
        
        # TOOL
        if usar_herramienta:
            
            self.PIN_SERVO = 11
            self.servo_current_angle = 0.0
            self.servo = self.setup_servo()  
            
            self.grados_boli_alto = 20.0     # TODO: Cambiar a valor correcto para cada prueba
            self.grados_boli_bajo = 0.0     
            
            self.grados_bolos_soltar = 0.0  
            self.grados_bolos_mantener = 0.0
            self.grados_pale_alto = 0.0     
            self.grados_pale_bajo = 0.0   
        
        # SENSORS
        sensors = Sensors()

    # ╔═════════════════════════════════════════╗
    # ║ MOVIMIENTOS BÁSICOS RUEDAS CON SENSORES ║
    # ╚═════════════════════════════════════════╝
    
    def detectar_pared(self, sensors):
        print(f'Distancia delante: {sensors.distancia_delante} cm')
        if sensors.get_distancia_delante() < 20:
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

    def avanzar_paralelo_paredes(self, sensors):
        distancia_der_principio = sensors.get_distancia_derecha()
        distancia_izq_principio = sensors.get_distancia_izquierda()
        while True:
            if sensors.get_distancia_derecha() < distancia_der_principio:
                self.girar_grados_izq(10)
            elif sensors.get_distancia_izquierda() < distancia_izq_principio:
                self.girar_grados_der(10)
            else:
                self.avanzar()
    
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



    # ╔═════════════════════════════╗
    # ║       TOOL FUNCTIONS        ║
    # ╚═════════════════════════════╝

    def herramienta_girar(self, grados_objetivo, tiempo=0):
        """
        tiempo: Tiempo que tarda en llegar al objetivo
        """
        #TODO: REVISAR
        
        if tiempo <= 0:
            step = abs(grados_objetivo - self.servo_current_angle)
            delay = 0
        else:
            step = abs(grados_objetivo - self.servo_current_angle) / (tiempo / 0.05)
            delay = 0.05
            
        direction = 1 if grados_objetivo > self.servo_current_angle else -1

        # Gradually move the servo to the target angle
        while abs(self.servo_current_angle - grados_objetivo) > step:
            self.servo_current_angle += direction * step
            duty_cycle = self.servo_angle_to_duty_cycle(self.servo_current_angle)
            self.servo.ChangeDutyCycle(duty_cycle)
            time.sleep(delay)

        # Ensure the servo reaches the exact target angle
        self.servo_current_angle = grados_objetivo
        duty_cycle = self.servo_angle_to_duty_cycle(self.servo_current_angle)
        self.servo.ChangeDutyCycle(duty_cycle)
        time.sleep(0.5)
        self.servo.ChangeDutyCycle(0)  # Stop sending signal to hold position

        
    def pedir_angulo_tool(self):
        angle = float(input("Introduce el ángulo para la herramienta (Entre 0 y 180): "))
        if 0 <= angle <= 180:
            return angle
        else:
            print("El ángulo debe estar entre 0 y 180 grados.")
            
    
            
    
    
    
    
    
    def setup_servo(self):
        GPIO.setmode(GPIO.BOARD)  # Use physical pin numbering
        GPIO.setup(self.PIN_SERVO, GPIO.OUT)
        servo = GPIO.PWM(self.PIN_SERVO, 50)  # 50Hz pulse for servo
        servo.start(0)  # DEFAULT POSITION
        return servo

    def servo_angle_to_duty_cycle(self, angle):
        """Convert an angle to the suitable duty cycle."""
        return 2 + (angle / 18)
    





    # Dibujar la figura
    
    def boli_subir(self, prints=False):
        if prints:
            print("Boli subido")
        tiempo = 1.5
        self.herramienta_girar(self.grados_boli_alto, tiempo)
        
    def boli_bajar(self, prints=False):
        if prints:
            print("Boli bajado")
        self.herramienta_girar(self.grados_boli_bajo)
    
    # Bolos
    
    def bolos_soltar(self):
        self.herramienta_girar(self.grados_bolos_soltar)
        
    def bolos_mantener(self):
        self.herramienta_girar(self.grados_bolos_mantener)
        
    # Mini-fábrica
    
    def pale_subir(self):
        self.herramienta_girar(self.grados_pale_alto)
        
    def pale_bajar(self):
        self.herramienta_girar(self.grados_pale_bajo)
    
    # def herramienta_girar_dynamixel(self, grados):      # NO
    #     msg = SetPosition()
    #     msg.position = int(grados)
    #     msg.id = self.tool_id
    #     self.tool_publisher_.publish(msg)
