import rclpy
import time
from final.Movements import Movements, get_movements
import os
import glob

# DISTANCIAS INTRODUCIDAS: EN CM
# GRADOS: EN GRADOS

SLEEP = 0.05       # Tiempo de espera entre movimientos

class MovementSaver():
    def __init__(self):
        print("MovementSaver")
        self.movements = []     # (movement (w,a,s,d,q,e)
        self.parentDir = os.path.join(os.path.dirname(__file__), 'recordings')       # Todo: add date by default so it doesnt duplicate
        self.fileName = f"movements_{time.strftime('%Y%m%d_%H%M%S')}.txt"
        self.pathName = self.get_path(self.fileName)
        self.mover = Movements()
        
    def get_path(self, fileName):
        return os.path.join(self.parentDir, fileName)
    
    def get_valid_input(self, input):
        # TODO: Careful with Booleans for example
        try:
            return float(input.replace(",", "."))
        except ValueError:
            print("Estás introduciendo un valor incorrecto. Introduce un número.")


    def save_inscript(self):
        print(f"Guardando movimientos en {self.pathName}")
        
        with open(self.pathName, "w") as fichero:
            for datos in self.movements:
                cadena = f'{datos[0]},{datos[1]},{datos[2]}\n'
                fichero.write(cadena)
                
        print("Movimientos guardados.")
                
    def load_inscript(self):
        self.movements = []
        with open(self.pathName, "r") as fichero:
            for linea in fichero:
                lista = linea.split(",")
                self.movements.append((lista[0], float(lista[1]), float(lista[2])))
                
    def execute_movement(self, movement, data1, data2):
        if movement == "w":
            self.mover.avanzar_distancia(data1)
        elif movement == "s":
            self.mover.retroceder_distancia(data1)
        elif movement == "a":
            self.mover.girar_grados_izq(data1)
        elif movement == "d":
            self.mover.girar_grados_der(data1)
        elif movement == "q":
            self.mover.girar_grados_izq(data1, data2)
        elif movement == "e":
            self.mover.girar_grados_der(data1, data2)
        
    def execute_movements(self):
        for movement in self.movements:
            self.execute_movement(movement[0], movement[1], movement[2])
            time.sleep(SLEEP)

def main(args=None):
    rclpy.init(args=args)
    
    saver = MovementSaver()
    
    # Programa principal
    opcion = input("1 - Guardar movimientos\n2 - Ejecutar movimientos\n3- TEST\n")
    
    if opcion == "1":       # GUARDAR MOVIMIENTOS
        saver.fileName = input("El nombre del fichero (Enter para poner la fecha actual): ")
        if saver.fileName == "":
            saver.fileName = f"movements_{time.strftime('%Y%m%d_%H%M%S')}.txt"
        saver.pathName = saver.get_path(saver.fileName)

        movement = ""
        while movement != "x":
            # GET MOVEMENT AND DATA
            movement = input("Introduce movimiento (w,a,s,d: Básicos) y (q,e): Giro con radio: ")
            if movement == "w" or movement == "s":
                try:
                    data1 = abs(float(input("Distancia (cm): ")) / 100)
                    data2 = 0
                except ValueError:
                    print("Invalid input. Cancelling movement.")
                    continue
            elif movement == "a" or movement == "d":
                try:
                    data1 = abs(float(input("Giro (grados): ")))
                    data2 = 0
                except ValueError:
                    print("Invalid input. Cancelling movement.")
                    continue
            elif movement == "q" or movement == "e":
                try:
                    data1 = abs(float(input("Giro (grados): ")))
                    data2 = abs(float(input("Radio (cm): "))) / 100
                except ValueError:
                    print("Invalid input. Cancelling movement.")
                    continue
            elif movement == "x":
                saver.save_inscript()
                break
            else:
                print("ERROR MOVIMIENTO")
                continue
            
            # MOVE THE ROBOT WITH ITS DATA
            saver.execute_movement(movement, data1, data2)

            # SAVE THE MOVEMENT
            saver.movements.append((movement, data1, data2))
            
        

    elif opcion == "2":     # EJECUTAR MOVIMIENTOS
        
        list_of_files = glob.glob(os.path.join(saver.parentDir, '*'))
        if not list_of_files:
            print("No hay ficheros en la carpeta.")
            return
        print("Ficheros disponibles:")
        for i, file in enumerate(list_of_files):
            file_to_print = file.split("/")[-1]
            print(f'{i+1} - {file_to_print}')
        try:
            saver.fileName = list_of_files[int(input("Introduce el número del fichero: "))-1]
            saver.pathName = saver.get_path(saver.fileName)
        except:
            print("ERROR FICHERO")

        print(f"Cargando movimientos de {saver.pathName.split('/')[-1]}")
        saver.load_inscript()
        saver.execute_movements()
        
        
        
        
        
        
        
        
        
        
    elif opcion == "3":
        # TODO: Implementar mejor el hecho de pasar parametros, con parametros opcionales (pulsando enter para los que están por defecto...)
        actions = get_movements()

        while True:
            print("\nAvailable commands:")
            for key, value in actions.items():
                print(f"{key} - {value[0]}({', '.join(value[1])})")  # Display method signature
            choice = input("Introduce the number of a command (or 'exit' to quit): ")
            
            if choice.lower() == 'exit':
                print("Exiting program.")
                break  # Exit the program

            if not choice.isdigit() or int(choice) not in actions:
                print("Invalid choice. Try again.")
                continue
            
            choice = int(choice)
            action, params = actions[choice]
            
            args = []
            for param in params:
                user_input = input(f"Enter {param}: ")
                if param == 'aceleracion':  # Convert specific parameters correctly
                    user_input = user_input.lower() in ['true', '1', 't', 'y', 'yes']
                else:
                    user_input = float(user_input)  # Assuming distances and angles are floats for simplicity
                args.append(user_input)

            # Execute the chosen action
            getattr(saver.mover, action)(*args)
            print(f"Executed {action}.")

    else:
        print("ERROR OPCION")
        return
    
    
    saver.mover.detener()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
