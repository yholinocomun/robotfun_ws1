#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
trajectory_node.py
==================

Control cinemático con PERFIL TRAPEZOIDAL (movimiento suave en lazo abierto)
para el brazo de 5 juntas + gripper.

Dos modos de objetivo:
  1) ARTICULAR  : /joint_goal (Float32MultiArray [q1..q5, gripper], RADIANES)
                  → interpolación trapezoidal sincronizada en espacio articular.
  2) CARTESIANO : /target_pose (Pose) → recta cartesiana con perfil trapezoidal;
                  en cada paso integra  q += J⁺(q)·dx  (control diferencial con
                  pseudo-inversa amortiguada). Respeta el área de trabajo.

Con ``lock_joint_4:=true`` (defecto) la columna del roll se congela y joint_4
se mantiene en 0, igual que en la IK.

Salida: /joint_command (Float32MultiArray, RADIANES) — stream fino
[q1..q5, gripper] a ``control_rate`` Hz hacia el ESP32 (o el relay en sim).
"""

import numpy as np
import rclpy
from geometry_msgs.msg import Pose
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32MultiArray

from robotfun_kinematics.core import (
    ARM_JOINT_NAMES, GRIPPER_JOINT_NAME, N_JOINTS, ROBOT, ROLL_INDEX,
    WorkspaceLimits, clamp_target,
)
from robotfun_kinematics.core.trajectory import joint_trajectory, trapezoidal_profile


class TrajectoryNode(Node):
    def __init__(self):
        super().__init__("trajectory_node")

        self.declare_parameter("control_rate", 50.0)
        self.declare_parameter("v_max", 0.6)        # rad/s articular
        self.declare_parameter("a_max", 1.2)        # rad/s²
        self.declare_parameter("v_max_cart", 0.08)  # m/s cartesiano
        self.declare_parameter("a_max_cart", 0.15)  # m/s²
        self.declare_parameter("damping", 0.06)     # λ del J⁺ amortiguado
        self.declare_parameter("lock_joint_4", True)

        self.rate = float(self.get_parameter("control_rate").value)
        self.lock_j4 = bool(self.get_parameter("lock_joint_4").value)
        self.limits = WorkspaceLimits()

        self.q_arm = np.zeros(N_JOINTS)
        self.gripper = 0.0
        self.have_feedback = False
        self.traj: list[np.ndarray] = []
        self.traj_idx = 0

        self.cmd_pub = self.create_publisher(Float32MultiArray, "/joint_command", 10)
        self.create_subscription(Float32MultiArray, "/joint_goal", self.joint_goal_cb, 10)
        self.create_subscription(Pose, "/target_pose", self.pose_goal_cb, 10)
        self.create_subscription(JointState, "/joint_states", self.joint_state_cb, 10)
        self.timer = self.create_timer(1.0 / self.rate, self.stream_cb)

        self.get_logger().info(
            f"trajectory_node listo (trapezoidal, lock_joint_4={self.lock_j4}).\n"
            "  articular : ros2 topic pub /joint_goal std_msgs/msg/Float32MultiArray "
            "\"{data: [q1,q2,q3,q4,q5,gripper]}\"\n"
            "  cartesiano: ros2 topic pub /target_pose geometry_msgs/msg/Pose ...")

    def joint_state_cb(self, msg: JointState):
        name_to_pos = dict(zip(msg.name, msg.position))
        if not self.have_feedback:
            for i, jn in enumerate(ARM_JOINT_NAMES):
                if jn in name_to_pos:
                    self.q_arm[i] = name_to_pos[jn]
            if GRIPPER_JOINT_NAME in name_to_pos:
                self.gripper = name_to_pos[GRIPPER_JOINT_NAME]
            self.have_feedback = True

    def joint_goal_cb(self, msg: Float32MultiArray):
        data = list(msg.data)
        if len(data) < N_JOINTS:
            self.get_logger().warn(
                f"Se requieren al menos {N_JOINTS} ángulos [q1..q5(,gripper)].")
            return
        q_goal = ROBOT.clamp(np.array(data[:N_JOINTS]))
        if self.lock_j4:
            q_goal[ROLL_INDEX] = 0.0
        grip_goal = float(data[N_JOINTS]) if len(data) > N_JOINTS else self.gripper
        v_max = float(self.get_parameter("v_max").value)
        a_max = float(self.get_parameter("a_max").value)
        q0 = np.hstack((self.q_arm, self.gripper))
        qg = np.hstack((q_goal, grip_goal))
        self.traj = joint_trajectory(q0, qg, v_max, a_max, self.rate)
        self.traj_idx = 0
        self.get_logger().info(
            f"[ARTICULAR] {len(self.traj)} pasos, "
            f"duración={len(self.traj) / self.rate:.2f} s")

    def pose_goal_cb(self, msg: Pose):
        x_goal = clamp_target([msg.position.x, msg.position.y, msg.position.z],
                              self.limits)
        v_max = float(self.get_parameter("v_max_cart").value)
        a_max = float(self.get_parameter("a_max_cart").value)
        lam2 = float(self.get_parameter("damping").value) ** 2

        active = ([i for i in range(N_JOINTS) if i != ROLL_INDEX]
                  if self.lock_j4 else list(range(N_JOINTS)))
        q = self.q_arm.copy()
        if self.lock_j4:
            q[ROLL_INDEX] = 0.0
        x0 = ROBOT.fkine(q)[0:3, 3]
        dist = float(np.linalg.norm(x_goal - x0))
        s_list = trapezoidal_profile(dist, v_max, a_max, self.rate)

        traj = []
        s_prev = 0.0
        for s in s_list:
            dx = (s - s_prev) * (x_goal - x0)
            J = ROBOT.jacobian_position(q)[:, active]
            dq = J.T @ np.linalg.solve(J @ J.T + lam2 * np.eye(3), dx)
            q[active] = q[active] + dq
            q = ROBOT.clamp(q)
            traj.append(np.hstack((q, self.gripper)))
            s_prev = s
        self.traj = traj
        self.traj_idx = 0
        self.get_logger().info(
            f"[CARTESIANO] {len(traj)} pasos, recta={dist * 100:.1f} cm, "
            f"duración={len(traj) / self.rate:.2f} s")

    def stream_cb(self):
        if self.traj_idx >= len(self.traj):
            return
        point = self.traj[self.traj_idx]
        self.traj_idx += 1
        self.q_arm = point[:N_JOINTS].copy()
        self.gripper = float(point[N_JOINTS])
        out = Float32MultiArray()
        out.data = [float(v) for v in point]
        self.cmd_pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryNode()
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
