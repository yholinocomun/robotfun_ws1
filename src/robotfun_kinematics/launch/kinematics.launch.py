"""
Lanza la cinemática de alto nivel (5 juntas + gripper, modos bloqueado/roll).

Un único controlador publica en /joint_command a la vez (``controller``):
  * controller:=trajectory  → control suave con perfil trapezoidal (recom. HW).
  * controller:=ik          → IK directa "salto a la pose".
fk_check_node siempre se ejecuta (valida TF ↔ FK publicando /fk_pose).

Argumentos:
  lock_joint_4:=true|false   modo A (roll bloqueado, defecto) o modo B (activo).
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
        description="Controlador de alto nivel: 'ik' (directo) o 'trajectory' (suave).")
    lock_joint_4 = DeclareLaunchArgument(
        "lock_joint_4", default_value="true",
        description="true: roll bloqueado (4 GDL efectivos, IK analítica); "
                    "false: roll activo (DLS).")
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
        name="trajectory_node", output="screen", condition=is_traj,
        parameters=[{"lock_joint_4": LaunchConfiguration("lock_joint_4")}])
    ik_node = Node(
        package="robotfun_kinematics", executable="ik_node", name="ik_node",
        output="screen", condition=is_ik,
        parameters=[{
            "lock_joint_4": LaunchConfiguration("lock_joint_4"),
            "method": LaunchConfiguration("method"),
            "approach_deg": LaunchConfiguration("approach_deg"),
        }])

    return LaunchDescription([
        controller, lock_joint_4, method, approach_deg,
        fk_check_node, trajectory_node, ik_node,
    ])
