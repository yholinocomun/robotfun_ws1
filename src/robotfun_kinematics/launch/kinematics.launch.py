"""
Lanza la cinemática de alto nivel (robot 4 GDL + gripper).

Un único controlador publica en /joint_command a la vez (``controller``):
  * controller:=ik          → IK directa (el firmware del ESP32 suaviza el
                              movimiento con su perfil trapezoidal). Recomendado.
  * controller:=trajectory  → perfil trapezoidal en el lado ROS (útil para
                              rectas cartesianas o en simulación).
fk_check_node siempre se ejecuta (valida TF ↔ FK publicando /fk_pose).

Argumentos:
  method:=analytic|dls|newton|gradient   método de IK (analytic recomendado).
  approach_deg:=-90          ángulo de aproximación de la pinza (grados).
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    controller = DeclareLaunchArgument(
        "controller", default_value="ik",
        description="Controlador de alto nivel: 'ik' (directo, el ESP32 suaviza) "
                    "o 'trajectory' (perfil en ROS).")
    method = DeclareLaunchArgument(
        "method", default_value="analytic",
        description="Método de IK: analytic | dls | newton | gradient.")
    approach_deg = DeclareLaunchArgument(
        "approach_deg", default_value="-90.0",
        description="Ángulo de aproximación de la pinza (grados). -90 = hacia abajo.")

    ctrl = LaunchConfiguration("controller")
    is_traj = IfCondition(PythonExpression(["'", ctrl, "' == 'trajectory'"]))
    is_ik = IfCondition(PythonExpression(["'", ctrl, "' == 'ik'"]))

    fk_check_node = Node(
        package="robotfun_kinematics", executable="fk_check_node",
        name="fk_check_node", output="screen")
    trajectory_node = Node(
        package="robotfun_kinematics", executable="trajectory_node",
        name="trajectory_node", output="screen", condition=is_traj)
    ik_node = Node(
        package="robotfun_kinematics", executable="ik_node", name="ik_node",
        output="screen", condition=is_ik,
        parameters=[{
            "method": LaunchConfiguration("method"),
            "approach_deg": LaunchConfiguration("approach_deg"),
        }])

    return LaunchDescription([
        controller, method, approach_deg,
        fk_check_node, trajectory_node, ik_node,
    ])
