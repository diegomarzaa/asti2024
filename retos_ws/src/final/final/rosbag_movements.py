import rclpy
import time
from final.Movements import Movements, get_movements
import os
import glob
import inspect

# DISTANCIAS INTRODUCIDAS: EN CM
# GRADOS: EN GRADOS

################################################
# TODO: Mejorar grabado de movimientos a partir de las acciones
################################################

SLEEP = 0.05       # Tiempo de espera entre movimientos

# Colorines para la terminal ejjejejeje

RED = '\033[91m'
GREEN = '\033[92m'
YELLOW = '\033[93m'
BLUE = '\033[94m'
MAGENTA = '\033[95m'
CYAN = '\033[96m'
RESET = '\033[0m'


class MovementSaver():
    def __init__(self):
        self.actions_to_do = []         # list with the name of the functions movements + data
        self.parentDir = os.path.join(os.path.dirname(__file__), 'recordings')       # Todo: add date by default so it doesnt duplicate
        self.fileName = f"movements_{time.strftime('%Y%m%d_%H%M%S')}.txt"
        self.pathName = self.get_path(self.fileName)
        self.mover = Movements()
        self.actions = get_movements()
        
        self.action_playing = None
        self.last_action = None
        self.recording = False
        self.selecting = False
        self.playing = False
        
        self.rosbag_numbers = []
        
    def get_valid_input(self, input):
        # TODO: Careful with Booleans for example
        try:
            return float(input.replace(",", "."))
        except ValueError:
            print("Estás introduciendo un valor incorrecto. Introduce un número.")
            
        
    def get_path(self, fileName):
        return os.path.join(self.parentDir, fileName)

    def save_inscript(self):
        # TODO: Implement correctly (Use args and kwargs instead of data1 and data2)
        
        print(f"Guardando movimientos en {self.pathName}")
        
        with open(self.pathName, "w") as fichero:
            for datos in self.actions_to_do:
                cadena = f'{datos[0]},{datos[1]},{datos[2]}\n'
                fichero.write(cadena)
                
        print("Movimientos guardados.")
                
    def load_inscript(self):
        # TODO: Implement correctly (Use args and kwargs instead of data1 and data2)
        self.actions_to_do = []
        with open(self.pathName, "r") as fichero:
            for linea in fichero:
                lista = linea.split(",")
                self.actions_to_do.append((lista[0], float(lista[1]), float(lista[2])))
                
    def execute_movement(self, movement, *args, **kwargs):
        # TODO: Update
        
        # if movement == "w":
        #     self.mover.avanzar_distancia(data1)
        # elif movement == "s":
        #     self.mover.retroceder_distancia(data1)
        # elif movement == "a":
        #     self.mover.girar_grados_izq(data1)
        # elif movement == "d":
        #     self.mover.girar_grados_der(data1)
        # elif movement == "q":
        #     self.mover.girar_grados_izq(data1, data2)
        # elif movement == "e":
        #     self.mover.girar_grados_der(data1, data2)
        
        getattr(self.mover, movement)(*args, **kwargs)
        
        
    def execute_movements(self):
        for action in self.actions_to_do:
            print("wdjhWJDL hAWlkjd hawlkdgawkdg")      # TODO: Print the entire menu to show the user which movement is being executed of all the movements (with colors)
            self.action_playing = action[0]
            self.execute_movement(action[0], action[1], action[2])
            time.sleep(SLEEP)
        self.action_playing = None
        
    def show_menu(self, ask_input=False, param=None, action=None):
        
        print("\n==============================================\
            \n\t\t\tMENU\
            \n==============================================")
        
        # RECORDING
        if not self.recording:
            print(GREEN + "\t1\tStart recording movements" + RESET)
        else:
            print(RED + "\t1\tStop recording movements" + RESET)
        
        
        # EXECUTING MOVEMENTS
        print(YELLOW + "\t2\tExecute recorded movements" + RESET)
        # Show possible rosbag files to execute
        if self.selecting:
            self.rosbag_numbers = []
            self.list_of_files = glob.glob(os.path.join(self.parentDir, '*'))
            if not self.list_of_files:
                print("No hay ficheros en la carpeta.")
                return
            for i, file in enumerate(self.list_of_files):
                file_to_print = file.split("/")[-1]
                print(f'\t\t\t{i+100}: {file_to_print}')
                self.rosbag_numbers.append(str(i+100))
           
             
        # SHOW REST OF ACTIONS
        for key, value in self.actions.items():
            if value[0] == self.action_playing:
                print(BLUE + f"\t{key}\t{value[0]}({', '.join(value[1])})" + RESET)
                if ask_input:
                    print(MAGENTA + f"\t\t- Introduce el parámetro: '{param}'" + RESET)
                # if additional_info_executing_option:
                #     print(f" ({', '.join(additional_info_executing_option)})")
            if value[0] == self.last_action:
                print(CYAN + f"\t{key}\t{value[0]}({', '.join(value[1])})" + RESET)
            else:
                print(f"\t{key}\t{value[0]}({', '.join(value[1])})")





def main(args=None):
    rclpy.init(args=args)
    
    saver = MovementSaver()
    
    while rclpy.ok():
        
        ############################
        # SHOW OPTIONS MENU
        ############################

        # RECORDING
        
        saver.show_menu()
                        
        # SELECT AN OPTION
        
        choice = input("\nIntroduce the number of a command: ")
        
        
        ############################
        # HANDLE CHOICE
        ############################
        
        # RECORDING
        
        if choice == "1":
            saver.recording = not saver.recording
            saver.selecting = False
            if saver.recording:
                print("Recording movements...")
            else:
                print("Recording stopped.")
            continue
        
        # TODO: Implement correctly
        # if choice == "1":       # GUARDAR MOVIMIENTOS
        #     saver.fileName = input("El nombre del fichero (Enter para poner la fecha actual): ")
        #     if saver.fileName == "":
        #         saver.fileName = f"movements_{time.strftime('%Y%m%d_%H%M%S')}.txt"
        #     saver.pathName = saver.get_path(saver.fileName)

        #     movement = ""
        #     while movement != "x":
        #         # GET MOVEMENT AND DATA
        #         movement = input("Introduce movimiento (w,a,s,d: Básicos) y (q,e): Giro con radio: ")
        #         if movement == "w" or movement == "s":
        #             try:
        #                 data1 = abs(float(input("Distancia (cm): ")) / 100)
        #                 data2 = 0
        #             except ValueError:
        #                 print("Invalid input. Cancelling movement.")
        #                 continue
        #         elif movement == "a" or movement == "d":
        #             try:
        #                 data1 = abs(float(input("Giro (grados): ")))
        #                 data2 = 0
        #             except ValueError:
        #                 print("Invalid input. Cancelling movement.")
        #                 continue
        #         elif movement == "q" or movement == "e":
        #             try:
        #                 data1 = abs(float(input("Giro (grados): ")))
        #                 data2 = abs(float(input("Radio (cm): "))) / 100
        #             except ValueError:
        #                 print("Invalid input. Cancelling movement.")
        #                 continue
        #         elif movement == "x":
        #             saver.save_inscript()
        #             break
        #         else:
        #             print("ERROR MOVIMIENTO")
        #             continue
                
        #         # MOVE THE ROBOT WITH ITS DATA
        #         saver.execute_movement(movement, data1, data2)

        #         # SAVE THE MOVEMENT
        #         saver.movements.append((movement, data1, data2))
                
            

        # EJECUCIÓN DE MOVIMIENTOS

        if choice == "2":
            if saver.recording:
                continue
            # TODO: Save movements in saver.movements_to_do()
            saver.selecting = not saver.selecting
        
        if saver.selecting and choice in saver.rosbag_numbers:
            try:
                saver.fileName = saver.list_of_files[int(choice)-100]
                saver.pathName = saver.get_path(saver.fileName)
            except:
                print("ERROR FICHERO")
                
            print(f"Cargando movimientos de {saver.pathName.split('/')[-1]}")
            saver.load_inscript()
            saver.execute_movements()

            continue
        
            
        
        # EJECUCIÓN DE ACCIONES

        elif choice not in saver.actions:
            print(f"Invalid choice. Try again.")
            continue
        
        action_name, params_required, params_optional = saver.actions[choice]
            
        args = []       # Required parameters
        kwargs = {}     # Optional parameters
        
        saver.action_playing = action_name
        saver.last_action = None
        saver.show_menu()
        
        # Ask for required parameters
        
        for param in params_required:
            while True:
                saver.show_menu(ask_input=True, param=param, action=action_name)
                user_input = input(f"\nRespuesta: ")
                if user_input == '':
                    print(RED + f"{param} is required." + RESET)
                    continue
                break
            # if param == 'aceleracion':
            #     user_input = user_input.lower() in ['true', '1', 't', 'y', 'yes']
            # else:
            #     user_input = float(user_input)
            
            
            user_input = float(user_input)
            args.append(user_input)
            
            
        # Ask for optional parameters
        
        for param in params_optional:
            # Get the default value
            default_value = inspect.signature(getattr(saver.mover, action_name)).parameters[param].default
            user_input = input(f"Enter {param} ('Enter' to default {default_value}): ")
            if user_input == '':
                print("Using default value.")
                continue
            else:
                user_input = saver.get_valid_input(user_input)
                kwargs[param] = user_input
            
        # TODO: Add the action to the list of actions to do
        saver.action_playing = action_name
        if saver.recording:
            saver.actions_to_do.append([action_name, args, kwargs])
        print('MOVEMENTS:', saver.actions_to_do)
        
        # Execute the chosen action
        getattr(saver.mover, action_name)(*args, **kwargs)
        # print(GREEN + f"Action {action_name} executed." + RESET)
        saver.last_action = action_name
        saver.action_playing = None
        
    saver.mover.detener()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
