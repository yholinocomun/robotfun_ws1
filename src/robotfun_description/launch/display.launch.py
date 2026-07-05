"""
Visualiza el brazo en RViz con sliders (joint_state_publisher_gui).

SOLO para inspección manual del modelo. Para IK/simulación/hardware usa
``robotfun_bringup bringup.launch.py`` (garantiza UNA sola fuente de
/joint_states; no lances ambos a la vez).

Argumentos:
  model:=primitives | meshes   modelo canónico (primitivas, DH-exacto) o piezas
                               reales del CAD. Comparten interfaz de juntas.
  gui:=true | false            sliders manuales (true) o joint_state_publisher.

Ejemplos:
  ros2 launch robotfun_description display.launch.py
  ros2 launch robotfun_description display.launch.py model:=meshes
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import (
    Command, LaunchConfiguration, PathJoinSubstitution, PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = FindPackageShare("robotfun_description")

    model = DeclareLaunchArgument(
        "model", default_value="primitives",
        description="'primitives' (DH-exacto) o 'meshes' (piezas reales del CAD).")
    gui = DeclareLaunchArgument(
        "gui", default_value="true",
        description="true ⇒ joint_state_publisher_gui (sliders).")

    model_cfg = LaunchConfiguration("model")
    gui_cfg = LaunchConfiguration("gui")

    xacro_file = PythonExpression(
        ["'robotfun_meshes.urdf.xacro' if '", model_cfg,
         "' == 'meshes' else 'robotfun.urdf.xacro'"])
    robot_description = {
        "robot_description": ParameterValue(
            Command(["xacro ", PathJoinSubstitution([pkg, "urdf", xacro_file])]),
            value_type=str)
    }

    rsp = Node(
        package="robot_state_publisher", executable="robot_state_publisher",
        output="screen", parameters=[robot_description])

    jsp_gui = Node(
        package="joint_state_publisher_gui", executable="joint_state_publisher_gui",
        condition=IfCondition(gui_cfg))
    jsp = Node(
        package="joint_state_publisher", executable="joint_state_publisher",
        condition=UnlessCondition(gui_cfg))

    rviz = Node(
        package="rviz2", executable="rviz2", name="rviz2", output="screen",
        arguments=["-d", PathJoinSubstitution([pkg, "rviz", "robotfun.rviz"])])

    return LaunchDescription([model, gui, rsp, jsp_gui, jsp, rviz])
