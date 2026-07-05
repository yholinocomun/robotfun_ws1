#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
joint_state_relay.py
====================

Puente de SIMULACIÓN: hace de "ESP32 virtual". Reenvía los comandos
``/joint_command`` (Float32MultiArray [q1..q4, gripper], RADIANES) como
``/joint_states`` (JointState), de modo que en RViz el robot siga a la IK SIN
hardware.

IMPORTANTE: en cada modo debe haber UNA sola fuente de /joint_states (este
relay, los sliders o el ESP32) — nunca dos a la vez, o RViz "salta" entre
ambas. ``bringup.launch.py`` garantiza esto con el argumento ``mode``.

También publica el dedo espejado ``gripper_right = −gripper`` (junta mimic del
URDF) para que el TF del dedo derecho exista en simulación.

Republica el último comando a ``rate`` Hz para mantener vivo el árbol TF.
Arranca en HOME (todo a 0).
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32MultiArray

from robotfun_kinematics.core import ARM_JOINT_NAMES, GRIPPER_JOINT_NAME

MIMIC_JOINT = "gripper_right"      # espejo del gripper en el URDF (multiplier=-1)


class JointStateRelay(Node):
    def __init__(self):
        super().__init__("joint_state_relay")
        self.declare_parameter("rate", 30.0)
        rate = float(self.get_parameter("rate").value)

        self.names = ARM_JOINT_NAMES + [GRIPPER_JOINT_NAME]
        self.positions = [0.0] * len(self.names)        # HOME

        self.pub = self.create_publisher(JointState, "/joint_states", 10)
        self.create_subscription(Float32MultiArray, "/joint_command", self.cmd_cb, 10)
        self.timer = self.create_timer(1.0 / rate, self.publish_state)
        self.get_logger().info(
            "joint_state_relay (ESP32 virtual) listo: /joint_command → /joint_states. "
            "Úsalo SOLO en simulación (no junto a sliders ni al ESP32).")

    def cmd_cb(self, msg: Float32MultiArray):
        data = list(msg.data)
        for i in range(len(self.names)):
            if i < len(data):
                self.positions[i] = float(data[i])

    def publish_state(self):
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = self.names + [MIMIC_JOINT]
        js.position = self.positions + [-self.positions[-1]]
        self.pub.publish(js)


def main(args=None):
    rclpy.init(args=args)
    node = JointStateRelay()
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
