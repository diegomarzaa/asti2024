import rclpy
import time
from final.Movements import Movements, get_movements
import os
import glob
import inspect
import json

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
        self.fileName = f"movements_{time.strftime('%Y-%m-%d_%H-%M-%S')}.txt"
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
        user_input = input("Introduce el nombre del fichero, ENTER para generar uno automáticamente, 'x' para cancelar: ")
        if user_input == 'x':
            print("Cancelando grabación.")
            return
        
        if user_input:
            self.fileName = f"{user_input}.json"
        else:
            self.fileName = f"{time.strftime('%Y-%m-%d_%H-%M-%S')}.json"
        self.pathName = self.get_path(self.fileName)

        print(f"Guardando movimientos en {self.pathName}")
        with open(self.pathName, "w") as fichero:
            json.dump(self.actions_to_do, fichero, indent=4)
            
        print("Movimientos guardados.")
        self.actions_to_do = []         # Reset the list of actions to do


    def load_inscript(self):
        self.actions_to_do = []
        with open(self.pathName, "r") as fichero:
            self.actions_to_do = json.load(fichero)
                
    def execute_movement(self, movement, *args, **kwargs):
        getattr(self.mover, movement)(*args, **kwargs)
        
        
    def execute_movements(self):
        for action_name, params, kwargs in self.actions_to_do:
            print(f"\033[1;34;40mExecuting movement: {action_name} with parameters: {params} and keyword parameters: {kwargs}\033[0m")
            self.action_playing = action_name
            self.execute_movement(action_name, *params, **kwargs)
            time.sleep(SLEEP)
        self.action_playing = None


    def show_menu(self, param=None, action=None):          # TODO: Quitar parametro ask_input
        
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
                saver.save_inscript()
            continue


        # EJECUCIÓN DE MOVIMIENTOS

        if choice == "2":
            if saver.recording:
                continue
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
            avisar_requerido = False
            while True:
                saver.show_menu(param=param, action=action_name)
                if avisar_requerido:
                    print(RED + f"\nEl parámetro {param} es obligatorio ponerlo." + RESET, end=' ')
                user_input = input(f"\nIntroduce el parámetro obligatorio {param}: ")
                if user_input == '':
                    avisar_requerido = True
                    continue
                break
            
            user_input = float(user_input)
            args.append(user_input)
            
        
        # Ask for optional parameters
        for param in params_optional:
            # Get the default value
            default_value = inspect.signature(getattr(saver.mover, action_name)).parameters[param].default
            user_input = input(f"Intruduce el parámetro opcional {param} (default: {default_value}): ")
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
        saver.last_action = action_name
        saver.action_playing = None
        
    saver.mover.detener()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
