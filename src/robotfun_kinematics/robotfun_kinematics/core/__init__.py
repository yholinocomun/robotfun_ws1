"""Núcleo de cinemática puro (sin ROS): DH 4 GDL (yaw + 3 pitch), FK,
Jacobiano, IK analítica cerrada + numérica (DLS/Newton/gradiente),
trayectorias trapezoidales y área de trabajo."""

from .dh_model import (
    A3, HAND, L0, L1, L2, L3, L4, L5,
    ARM_JOINT_NAMES,
    GRIPPER_JOINT_NAME,
    N_JOINTS,
    ROBOT,
    DHChain,
    build_default_robot,
    dh,
    fkine,
    jacobian_geometric,
    jacobian_position,
)
from .ik_solver import (
    IKResult,
    NumericConfig,
    approach_angle,
    ik_analytic,
    rot_to_quat,
    solve_analytic,
    solve_ik,
    solve_numeric,
)
from .trajectory import joint_trajectory, trapezoidal_profile
from .workspace import (
    R_MAX_PHYSICAL, WorkspaceLimits, clamp_target, reach, validate_target,
)

__all__ = [
    "A3", "HAND", "L0", "L1", "L2", "L3", "L4", "L5",
    "ARM_JOINT_NAMES", "GRIPPER_JOINT_NAME", "N_JOINTS", "ROBOT",
    "DHChain", "build_default_robot", "dh", "fkine", "jacobian_geometric",
    "jacobian_position",
    "IKResult", "NumericConfig", "approach_angle", "ik_analytic",
    "rot_to_quat", "solve_analytic", "solve_ik", "solve_numeric",
    "joint_trajectory", "trapezoidal_profile",
    "R_MAX_PHYSICAL", "WorkspaceLimits", "clamp_target", "reach", "validate_target",
]
