from launch import LaunchDescription
from launch_ros.actions import Node
import launch.actions

def generate_launch_description():
  return LaunchDescription([
    Node(
      package='pruebas',
      executable='detector',
      namespace='front_left',
      parameters=[
        {"echo_pin": 27},
        {"trigger_pin": 22},
        {"distancia_limite": 0.35},
      ]
    ),
    Node(
      package='pruebas',
      executable='detector',
      namespace='front_right',
      parameters=[
        {"echo_pin": 23},
        {"trigger_pin": 24},
        {"distancia_limite": 0.35},
      ]
    ),
    Node(
      package='pruebas',
      executable='detector',
      namespace='right',
      parameters=[
        {"echo_pin": 20},
        {"trigger_pin": 21},
        {"distancia_limite": 0.25},
      ]
    ),
    Node(
      package='pruebas',
      executable='detector',
      namespace='left',
      parameters=[
        {"echo_pin": 5},
        {"trigger_pin": 6},
        {"distancia_limite": 0.25},
      ]
    ),
    Node(
      package='pruebas',
      executable='avoidance',
    ),
  ])
