"""
Arranca el Agent de micro-ROS (puente serial ESP32 <-> ROS 2).

Requiere el paquete micro_ros_agent instalado/sourced. Equivale a:
    ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0 -b 115200

Argumentos:
    dev:=/dev/ttyUSB0     puerto serie del ESP32.
    baud:=115200          baudios (debe coincidir con el firmware).
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    dev = DeclareLaunchArgument("dev", default_value="/dev/ttyUSB0")
    baud = DeclareLaunchArgument("baud", default_value="115200")

    agent = Node(
        package="micro_ros_agent", executable="micro_ros_agent", output="screen",
        arguments=["serial", "--dev", LaunchConfiguration("dev"),
                   "-b", LaunchConfiguration("baud")],
    )
    return LaunchDescription([dev, baud, agent])
