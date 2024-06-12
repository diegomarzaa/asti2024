import os
import json
import tkinter as tk
import subprocess
import sounddevice as sd
from scipy.io.wavfile import write
import numpy as np
from openai import OpenAI
from dotenv import load_dotenv
import threading


# TODO: IMPLEMENTAR
# - Boton de audio a texto, usando whisper
# - Implementar un programa estilo "Movements" que pueda recibir parametros, avanzar, distancia, velocidad...


# OPENAI SETUP

load_dotenv()     # Load environment variables from .env file  (Crear un archivo .env en el mismo directorio que este script, y poner dentro OPENAI_API_KEY='your_key')
OPENAI_API_KEY = os.getenv('OPENAI_API_KEY')
client = OpenAI(api_key=OPENAI_API_KEY)

class RosButtonApp:
    def __init__(self, master):
        # WINDOW INTERFACE
        self.master = master
        self.master.title("ROS 2 Button Application")
        
        # input for the WORKSPACE
        self.input_label = tk.Label(self.master, text="Carpeta del workspace:")
        self.input_label.pack(pady=5)
        self.input_box = tk.Entry(self.master)
        self.input_box.insert(tk.END, "~/Documents/10.UJI/UJIROBOTICS-ASTI/asti2024_ws/retos_ws/install/setup.bash")  # Default input
        self.input_box.pack(pady=5)
        
        # division line
        self.div_line = tk.Label(self.master, text="----------------------------------------")
        self.div_line.pack(pady=5)
        

        # ROS 2 command mapping
        self.ros_commands = [
            ("Lanzamiento de Bolos!", "ros2 run final bolos"),
            ("Dibuja la Figura!", "ros2 run final dibuja_figura"),
        ]
        number_of_buttons = len(self.ros_commands)

        # Frame for buttons
        self.button_frame = tk.Frame(self.master)
        self.button_frame.pack(pady=10)

        # Create buttons dynamically
        self.buttons = []
        for i, (button_text, _) in enumerate(self.ros_commands):
            button = tk.Button(self.button_frame, text=button_text, command=lambda i=i: self.execute_command_from_index(i))
            button.grid(row=i // number_of_buttons, column=i % number_of_buttons, padx=5, pady=5)
            self.buttons.append(button)


        
        # CHATGPT INTEGRATION
        self.label_chat = tk.Label(self.master, text="Chat:")
        self.label_chat.pack(pady=5)
        
        self.chat_box = tk.Text(self.master, height=5, width=40)
        self.chat_box.pack(pady=5)
        
        
        self.chat_button = tk.Button(self.master, text="Enviar", command=lambda: self.execute_command_from_natural_input(self.chat_box.get("1.0", tk.END)))
        self.chat_button.pack(pady=5)
        
        # Audio recording button
        self.record_audio_button = tk.Button(self.master, text="Record Audio", command=self.toggle_recording)
        self.record_audio_button.pack(pady=5)
        
        # Recording state
        self.is_recording = False
        self.fs = 16000  # Sample rate
        self.recording_data = []  # List to hold the recording chunks
        self.recording_thread = None



        # Label to display the status
        self.label = tk.Label(self.master, text="No command executed yet")
        self.label.pack(pady=10)
        
        
        
        
        
        
    def toggle_recording(self):
        if self.is_recording:
            self.stop_recording()
        else:
            self.start_recording()

    def start_recording(self):
        self.is_recording = True
        self.record_audio_button.config(text="Stop Recording")
        self.recording_data = []  # Clear any previous recording data
        self.recording_thread = threading.Thread(target=self.record_audio)
        self.recording_thread.start()

    def stop_recording(self):
        if self.is_recording:
            self.is_recording = False
            self.record_audio_button.config(text="Record Audio")
            if self.recording_thread is not None:
                self.recording_thread.join()  # Ensure recording thread completes
            if self.recording_data:  # Save the recorded data to 'output.wav'
                output = np.concatenate(self.recording_data, axis=0)
                write('output.wav', self.fs, output)
                self.convert_speech_to_text('output.wav')
            else:
                print("No recording data found.")
        else:
            print("Recording is not active.")
            
    def record_audio(self):
        try:
            print("Starting recording...")
            while self.is_recording:
                chunk = sd.rec(1024, samplerate=self.fs, channels=1, dtype='int16')
                sd.wait()
                self.recording_data.append(chunk)
            print("Recording stopped.")
        except Exception as e:
            print(f"An error occurred during recording: {e}")
            self.is_recording = False

    def convert_speech_to_text(self, audio_file):
        try:
            response = client.audio.transcriptions.create(
                model="whisper-1",
                file=open(audio_file, 'rb'),
                response_format="verbose_json"
            )

            print(f"API Response: {response}")
            transcription = response.text.strip()
            print(f"Transcription: {transcription}")
            self.chat_box.insert(tk.END, transcription + "\n")
            return transcription
        except Exception as e:
            print(f"An error occurred during transcription: {e}")
            self.chat_box.insert(tk.END, "Error during transcription.\n")
            return None

        
    def input_togpt_toaction(self, input):
        """
        Send the input to GPT-3.5 and return the action returned or None if no action is found
        """

        self.chat_box.delete("1.0", tk.END)
        
        system_description = f"You are a helpful assistant designed to output JSON.\
        The user will provide the description of the action to do, and you will understand what he wants, and return an appropiate ros command to execute.\n\
        The possible actions already implemented are: {', '.join([x[1] for x in self.ros_commands])}\
        Just return: command: <command_name>"       #  (being <None> if you think that you shouldn't execute any of the commands given the input)
        
        # input = self.chat_box.get("1.0", "end-1c")
        self.chat_box.delete("1.0", tk.END)
        
        response = client.chat.completions.create(
            model="gpt-3.5-turbo",
            response_format={ "type": "json_object" },
            messages=[
                {"role": "system", "content": system_description},
                {"role": "user", "content": input}
            ]
        )
        
        response = json.loads(response.choices[0].message.content)
        print(response)     # {'command': 'ros2 run final bolos'}
        command = response.get("command")
        if command is not None:
            for i, (_, command_name) in enumerate(self.ros_commands):
                if command_name == command:
                    return command_name
        else:
            print("No command found")
            self.label.config(text="No command found")
            return None


    def execute_command_from_natural_input(self, input):
        command_name = self.input_togpt_toaction(input)
        self.directly_execute_command(command_name)


    def directly_execute_command(self, cmd):
        """
        Open a new terminal and run the selected ROS 2 command.
        The terminal will be closed after the command finishes.
        """
        
        local_source_folder = self.input_box.get()

        env = os.environ.copy()
        env["PYTHONUNBUFFERED"] = "1"
        
        bashrc = os.path.expanduser("~/.bashrc")
        ros_distro = "foxy"  # Replace with your ROS 2 distribution
        ros_setup = f"/opt/ros/{ros_distro}/setup.bash"
        
        if os.path.exists(ros_setup) and os.path.exists(bashrc):
            command = f"source {bashrc} && source {ros_setup} && source {local_source_folder} && {cmd} && exit; exec bash"
            terminal_command = f"gnome-terminal -- bash -c '{command}'"
        else:
            print("Error: ROS 2 setup file not found.")
            return
        
        print(f"Running command: {terminal_command}")
        subprocess.Popen(terminal_command, shell=True, env=env)
        
        
    def execute_command_from_index(self, cmd_index):
        """
        This is to use the buttons in the interface to fastly open the programs
        """
        command_name = self.ros_commands[cmd_index][0]
        cmd = self.ros_commands[cmd_index][1]
        
        print(f"Executing command: {command_name}")
        self.label.config(text=f"Executing: {command_name}")

        self.directly_execute_command(cmd)

if __name__ == "__main__":
    root = tk.Tk()
    app = RosButtonApp(root)
    root.mainloop()
