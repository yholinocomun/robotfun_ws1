#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
fk_check_node.py
================

Nodo de COMPROBACIÓN de la cinemática DIRECTA.

    /joint_states (sensor_msgs/JointState) → FK → /fk_pose (geometry_msgs/Pose)

Sirve para validar de forma independiente que la tabla DH y el URDF coinciden:
la posición de /fk_pose debe ser idéntica a la del frame ``tool0`` en TF
(RViz). Adaptador delgado sobre ``robotfun_kinematics.core``.
"""

import numpy as np
import rclpy
from geometry_msgs.msg import Pose
from rclpy.node import Node
from sensor_msgs.msg import JointState

from robotfun_kinematics.core import ARM_JOINT_NAMES, ROBOT, rot_to_quat


class FKCheckNode(Node):
    def __init__(self):
        super().__init__("fk_check_node")
        self.pose_pub = self.create_publisher(Pose, "/fk_pose", 10)
        self.create_subscription(JointState, "/joint_states", self.cb, 10)
        self.get_logger().info("fk_check_node listo. Publicando FK en /fk_pose.")

    def cb(self, msg: JointState):
        name_to_pos = dict(zip(msg.name, msg.position))
        q = np.array([name_to_pos.get(jn, 0.0) for jn in ARM_JOINT_NAMES])

        T = ROBOT.fkine(q)
        p = T[0:3, 3]
        qx, qy, qz, qw = rot_to_quat(T[0:3, 0:3])

        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = map(float, p)
        pose.orientation.x = float(qx)
        pose.orientation.y = float(qy)
        pose.orientation.z = float(qz)
        pose.orientation.w = float(qw)
        self.pose_pub.publish(pose)

        self.get_logger().info(
            f"q(deg)={np.round(np.degrees(q), 1)} → TCP xyz={np.round(p, 4)}",
            throttle_duration_sec=1.0)


def main(args=None):
    rclpy.init(args=args)
    node = FKCheckNode()
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
