"""
bringup.launch.py — composition root con UNA sola fuente de /joint_states.

El "temblor/bucle" en RViz ocurre cuando dos nodos publican /joint_states a la
vez (sliders + relay, sliders + ESP32, …). El argumento ``mode`` garantiza una
única fuente:

  mode:=sim       (defecto) IK/trayectoria → /joint_command → joint_state_relay
                  → /joint_states → RViz. El robot sigue a la IK SIN hardware.
                  (Fuente única: el relay.)
  mode:=gui       sliders manuales (joint_state_publisher_gui). SIN cinemática.
                  (Fuente única: los sliders.)
  mode:=hardware  el ESP32 publica /joint_states (arranca aparte el micro-ROS
                  Agent). Corre la cinemática, NO el relay ni los sliders.
                  (Fuente única: el ESP32.)

Otros argumentos:
  model:=primitives|meshes       modelo a visualizar (primitives = DH-exacto).
  controller:=ik|trajectory      controlador (modos sim/hardware).
  lock_joint_4:=true|false       modo A (roll bloqueado) / modo B (roll activo).
                                 DEBE coincidir con JOINT4_LOCKED del firmware.
  method:=analytic|dls|newton|gradient ; approach_deg:=-90

IMPORTANTE: no lances display.launch.py a la vez que este bringup (duplicarías
/joint_states). Usa SOLO este bringup cambiando el modo.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command, LaunchConfiguration, PathJoinSubstitution, PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    desc_pkg = FindPackageShare("robotfun_description")
    kin_pkg = FindPackageShare("robotfun_kinematics")

    model = DeclareLaunchArgument("model", default_value="primitives")
    mode = DeclareLaunchArgument("mode", default_value="sim")
    controller = DeclareLaunchArgument("controller", default_value="ik")
    lock_joint_4 = DeclareLaunchArgument("lock_joint_4", default_value="true")
    method = DeclareLaunchArgument("method", default_value="analytic")
    approach_deg = DeclareLaunchArgument("approach_deg", default_value="-90.0")

    model_cfg = LaunchConfiguration("model")
    mode_cfg = LaunchConfiguration("mode")

    is_sim = IfCondition(PythonExpression(["'", mode_cfg, "' == 'sim'"]))
    is_gui = IfCondition(PythonExpression(["'", mode_cfg, "' == 'gui'"]))
    not_gui = IfCondition(PythonExpression(["'", mode_cfg, "' != 'gui'"]))

    xacro_file = PythonExpression(
        ["'robotfun_meshes.urdf.xacro' if '", model_cfg,
         "' == 'meshes' else 'robotfun.urdf.xacro'"])
    robot_description = {
        "robot_description": ParameterValue(
            Command(["xacro ", PathJoinSubstitution([desc_pkg, "urdf", xacro_file])]),
            value_type=str)
    }

    rsp = Node(package="robot_state_publisher", executable="robot_state_publisher",
               output="screen", parameters=[robot_description])
    rviz = Node(package="rviz2", executable="rviz2", name="rviz2", output="screen",
                arguments=["-d", PathJoinSubstitution([desc_pkg, "rviz", "robotfun.rviz"])])

    # Fuente de /joint_states según el modo (exactamente UNA activa)
    relay = Node(package="robotfun_kinematics", executable="joint_state_relay",
                 name="joint_state_relay", output="screen", condition=is_sim)
    jsp_gui = Node(package="joint_state_publisher_gui",
                   executable="joint_state_publisher_gui", condition=is_gui)

    # Cinemática (IK/trayectoria + fk_check) en sim y hardware, NO en gui
    kinematics = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([kin_pkg, "launch", "kinematics.launch.py"])),
        launch_arguments={
            "controller": LaunchConfiguration("controller"),
            "lock_joint_4": LaunchConfiguration("lock_joint_4"),
            "method": LaunchConfiguration("method"),
            "approach_deg": LaunchConfiguration("approach_deg"),
        }.items(),
        condition=not_gui,
    )

    return LaunchDescription([
        model, mode, controller, lock_joint_4, method, approach_deg,
        rsp, rviz, relay, jsp_gui, kinematics,
    ])
