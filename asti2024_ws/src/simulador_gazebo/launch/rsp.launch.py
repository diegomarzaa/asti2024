import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, Command
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

import xacro


def generate_launch_description():

    # Check if we're told to use sim time
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_ros2_control = LaunchConfiguration('use_ros2_control')

    # Process the URDF file
    pkg_path = os.path.join(get_package_share_directory('simulador_gazebo'))
    opcion = input("Ingrese el modelo del robot: \n"
                   "1. Cybercrex \n"
                   "2. Cybercrex_frontal \n"
                   "3. Cybercrex_camara_movil (aun no hay modelo)\n"
                   "4. Cybercrex_industrial (aun no hay modelo) \n"
                   "5. Cybercrex_bolos (aun no hay modelo) \n"
                   "6. Cybercrex_dibujos (aun no hay modelo) \n"
                   "7. Cybercrex_sumo (aun no hay modelo) \n"
                   "8. Cybercrex_figuras (aun no hay modelo) \n"
                   )
    if opcion == '1':
        modelo = 'Cybercrex'
    elif opcion == '2':
        modelo = 'Cybercrex_frontal'
    elif opcion == '3':
        modelo = 'Cybercrex_camara_movil'
    elif opcion == '4':
        modelo = 'Cybercrex_industrial'
    elif opcion == '5':
        modelo = 'Cybercrex_bolos'
    elif opcion == '6':
        modelo = 'Cybercrex_dibujos'
    elif opcion == '7':
        modelo = 'Cybercrex_sumo'
    elif opcion == '8':
        modelo = 'Cybercrex_figuras'



    xacro_file = os.path.join(pkg_path,'models/' + modelo,'robot.urdf.xacro')
    # robot_description_config = xacro.process_file(xacro_file).toxml()
    robot_description_config = Command(['xacro ', xacro_file, ' use_ros2_control:=', use_ros2_control, ' sim_mode:=', use_sim_time])
    
    # Create a robot_state_publisher node
    params = {'robot_description': robot_description_config, 'use_sim_time': use_sim_time}
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )


    # Launch!
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use sim time if true'),
        DeclareLaunchArgument(
            'use_ros2_control',
            default_value='true',
            description='Use ros2_control if true'),

        node_robot_state_publisher
    ])
