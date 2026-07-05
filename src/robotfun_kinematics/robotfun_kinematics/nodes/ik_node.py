#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ik_node.py
==========

Nodo ROS 2 de cinemática inversa (5 juntas + gripper, modos bloqueado/roll).

    /target_pose      (geometry_msgs/Pose)      — posición deseada del TCP; si
                                                  la orientación NO es identidad,
                                                  su eje X se usa como dirección
                                                  de apuntado (modo roll).
    /gripper_command  (std_msgs/Float32)        — apertura del gripper (rad)
    /joint_states     (sensor_msgs/JointState)  — realimentación (semilla IK)
              |
              v   IK (analítica cerrada en modo bloqueado; DLS en modo roll)
              |
    /joint_command    (std_msgs/Float32MultiArray, RADIANES)
                      [joint_1, joint_2, joint_3, joint_4, joint_5, gripper] → ESP32

Parámetros
----------
lock_joint_4  : true (defecto) → modo A, roll bloqueado a 0 (4 GDL efectivos,
                IK analítica). false → modo B, roll activo (DLS, posición +
                dirección de apuntado 3D).
method        : "analytic" (recom. en modo bloqueado) | "dls" | "newton" | "gradient".
approach_deg  : ángulo de aproximación φ en grados (−90 = pinza hacia abajo).
damping       : λ del DLS (rad).
enforce_workspace, ws_x/ws_y/ws_z : área de trabajo segura (ver workspace.py).
"""

import numpy as np
import rclpy
from geometry_msgs.msg import Pose
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32, Float32MultiArray

from robotfun_kinematics.core import (
    ARM_JOINT_NAMES, N_JOINTS, ROLL_INDEX, WorkspaceLimits, clamp_target,
    fkine, quat_to_rot, solve_ik, validate_target,
)


class IKNode(Node):
    def __init__(self):
        super().__init__("ik_node")

        self.declare_parameter("lock_joint_4", True)
        self.declare_parameter("method", "analytic")
        self.declare_parameter("approach_deg", -90.0)
        self.declare_parameter("damping", 0.05)
        self.declare_parameter("enforce_workspace", True)
        # límites del área de trabajo (metros)
        self.declare_parameter("ws_x", [-0.30, 0.30])
        self.declare_parameter("ws_y", [-0.30, 0.30])
        self.declare_parameter("ws_z", [0.02, 0.42])

        self.lock_j4 = bool(self.get_parameter("lock_joint_4").value)
        self.method = str(self.get_parameter("method").value)
        self.approach = np.radians(float(self.get_parameter("approach_deg").value))
        self.damping = float(self.get_parameter("damping").value)
        self.enforce_ws = bool(self.get_parameter("enforce_workspace").value)
        xr = self.get_parameter("ws_x").value
        yr = self.get_parameter("ws_y").value
        zr = self.get_parameter("ws_z").value
        self.limits = WorkspaceLimits(x_min=xr[0], x_max=xr[1], y_min=yr[0],
                                      y_max=yr[1], z_min=zr[0], z_max=zr[1])

        self.q_current = np.zeros(N_JOINTS)
        self.gripper_cmd = 0.0

        self.cmd_pub = self.create_publisher(Float32MultiArray, "/joint_command", 10)
        self.create_subscription(Pose, "/target_pose", self.target_cb, 10)
        self.create_subscription(Float32, "/gripper_command", self.gripper_cb, 10)
        self.create_subscription(JointState, "/joint_states", self.joint_state_cb, 10)

        modo = "A: joint_4 BLOQUEADO (4 GDL efectivos)" if self.lock_j4 \
            else "B: joint_4 ACTIVO (roll)"
        self.get_logger().info(
            f"ik_node listo | modo {modo} | método={self.method} "
            f"φ={np.degrees(self.approach):.0f}° "
            f"workspace={'ON' if self.enforce_ws else 'OFF'}. "
            "Publica una posición en /target_pose.")

    def joint_state_cb(self, msg: JointState):
        name_to_pos = dict(zip(msg.name, msg.position))
        for i, jn in enumerate(ARM_JOINT_NAMES):
            if jn in name_to_pos:
                self.q_current[i] = name_to_pos[jn]

    def gripper_cb(self, msg: Float32):
        self.gripper_cmd = float(msg.data)
        self.publish_command(self.q_current)

    def target_cb(self, msg: Pose):
        x_des = np.array([msg.position.x, msg.position.y, msg.position.z])

        ok, reason = validate_target(x_des, self.limits)
        if not ok:
            if self.enforce_ws:
                x_clamped = clamp_target(x_des, self.limits)
                self.get_logger().warn(
                    f"objetivo fuera del área de trabajo ({reason}); "
                    f"recortado a {np.round(x_clamped, 3)}.")
                x_des = x_clamped
            else:
                self.get_logger().warn(f"objetivo fuera del área de trabajo ({reason}).")

        # Si la Pose trae orientación explícita (no identidad), su eje X define
        # la dirección de apuntado (requiere modo roll para salir del plano).
        o = msg.orientation
        direction = None
        if abs(o.x) + abs(o.y) + abs(o.z) > 1e-6:
            direction = quat_to_rot(o.x, o.y, o.z, o.w)[:, 0]
            if self.lock_j4:
                self.get_logger().warn(
                    "orientación recibida con joint_4 bloqueado: solo se podrá "
                    "apuntar dentro del plano del brazo (lanza con "
                    "lock_joint_4:=false para dirección 3D).")

        res = solve_ik(x_des, self.q_current, approach=self.approach,
                       direction=direction, method=self.method,
                       lock_joint_4=self.lock_j4, damping=self.damping)
        if not res.ok:
            self.get_logger().warn(
                f"IK no resolvió ({res.reason}); se publica la mejor solución.")

        self.q_current = res.q
        self.publish_command(res.q)

        x_chk = fkine(res.q)[0:3, 3]
        self.get_logger().info(
            f"objetivo {np.round(x_des, 4)} → q(deg) {np.round(np.degrees(res.q), 1)} "
            f"| FK={np.round(x_chk, 4)} err={res.error:.2e} "
            f"({res.method}/{res.mode}, q4={np.degrees(res.q[ROLL_INDEX]):.1f}°)")

    def publish_command(self, q):
        out = Float32MultiArray()
        out.data = [float(v) for v in q[:N_JOINTS]] + [float(self.gripper_cmd)]
        self.cmd_pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = IKNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
