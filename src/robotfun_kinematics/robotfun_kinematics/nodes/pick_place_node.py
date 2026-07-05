#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
pick_place_node.py — Secuencia de PICK & PLACE automática (un solo nodo).
========================================================================

Publica objetivos ARTICULARES [q1, q2, q3, q4, gripper] (RADIANES) en
``/joint_command``. El firmware del ESP32 hace el movimiento SUAVE hacia cada
objetivo (perfil trapezoidal en su tarea de tiempo real); este nodo solo manda
el siguiente punto y ESPERA (dwell) a que el brazo llegue. Control directo por
juntas (NO usa cinemática inversa): son ángulos probados a mano.

En SIMULACIÓN funciona igual: lanza el bringup en mode:=sim y el
joint_state_relay hará de ESP32 virtual (sin suavizado, salta al objetivo).

CÓMO AÑADIR / EDITAR PUNTOS
---------------------------
1. Define el waypoint en la sección WAYPOINTS como una lista de 5 valores en
   radianes: [joint_1, joint_2, joint_3, joint_4, gripper].
2. Añádelo a SECUENCIA como una tupla:  ("nombre", WAYPOINT, espera_seg).
   'espera_seg' es el tiempo que se espera tras mandarlo (deja margen para que
   el brazo llegue; los giros grandes de joint_1 tardan más).
3. Para una PAUSA (esperar sin mover, p. ej. antes de agarrar/soltar) usa None
   como waypoint:  ("PAUSA", None, 2.0).

Parámetros ROS
--------------
  cycles       : nº de ciclos de la secuencia (0 = infinito).      [def. 1]
  speed_scale  : multiplica TODOS los tiempos de espera (>1 = más
                 lento/seguro; <1 = más rápido).                    [def. 1.0]
  go_home_end  : volver a HOME al terminar / al cortar con Ctrl+C.  [def. True]

Uso
---
  ros2 run robotfun_kinematics pick_place_node
  ros2 run robotfun_kinematics pick_place_node --ros-args -p cycles:=0 -p speed_scale:=1.5
"""

import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

# ===========================================================================
#  GRIPPER (canal 5). Rango útil [0.0 .. 1.22] rad (0..70°).
# ===========================================================================
GRIP_OPEN = 0.90     # pinza ABIERTA  (suelta)
GRIP_CLOSE = 0.20    # pinza CERRADA  (agarra)

# ===========================================================================
#  WAYPOINTS  =  [joint_1, joint_2, joint_3, joint_4, gripper]  (radianes)
#  (ángulos calibrados a mano por el usuario; edita aquí las posiciones)
# ===========================================================================
HOME = [0.00, 0.00, 0.00, 0.00, GRIP_OPEN]

# --- Zona de PICK (recoger): brazo girado a +1.5 (a la derecha) ---
PICK_OVER = [1.50, 1.00, 0.50, -1.00, GRIP_OPEN]    # encima del objeto, abierta
PICK_DOWN = [1.50, 1.30, 0.50, -1.00, GRIP_OPEN]    # baja hasta el objeto
PICK_GRASP = [1.50, 1.30, 0.50, -1.00, GRIP_CLOSE]  # cierra la pinza (agarra)
PICK_LIFT = [1.00, 1.00, 0.50, -1.00, GRIP_CLOSE]   # levanta con el objeto

# --- Zona de PLACE (soltar): brazo girado a -1.57 (a la izquierda) ---
PLACE_OVER = [-1.57, 0.80, 0.50, -1.00, GRIP_CLOSE]  # encima del destino
PLACE_DOWN = [-1.57, 1.00, 1.00, -1.00, GRIP_CLOSE]  # baja hasta el destino
PLACE_REL = [-1.57, 1.00, 1.00, -1.00, GRIP_OPEN]    # abre la pinza (suelta)
PLACE_UP = [-1.57, 0.70, 0.90, -1.00, GRIP_OPEN]     # sube ya sin el objeto

# ===========================================================================
#  SECUENCIA  =  ("nombre", waypoint, espera_segundos)
#  El orden es el que se ejecuta. Añade/quita filas a tu gusto.
#  Un waypoint = None es una PAUSA (solo espera, no mueve el brazo).
# ===========================================================================
SECUENCIA = [
    ("HOME",                  HOME,       2.0),
    ("PICK · encima",         PICK_OVER,  2.5),
    ("PICK · bajar",          PICK_DOWN,  2.0),
    ("PAUSA · antes agarrar", None,       2.0),
    ("PICK · AGARRAR",        PICK_GRASP, 1.5),
    ("PICK · levantar",       PICK_LIFT,  2.0),
    ("PLACE · encima",        PLACE_OVER, 4.0),   # giro grande de joint_1
    ("PLACE · bajar",         PLACE_DOWN, 2.0),
    ("PAUSA · antes soltar",  None,       2.0),
    ("PLACE · SOLTAR",        PLACE_REL,  1.5),
    ("PLACE · subir",         PLACE_UP,   2.0),
    ("HOME",                  HOME,       3.0),
]

N_CHANNELS = 5      # [q1, q2, q3, q4, gripper]


class PickPlaceNode(Node):
    def __init__(self):
        super().__init__("pick_place_node")
        self.declare_parameter("cycles", 1)
        self.declare_parameter("speed_scale", 1.0)
        self.declare_parameter("go_home_end", True)

        self.cycles = int(self.get_parameter("cycles").value)
        self.speed = float(self.get_parameter("speed_scale").value)
        self.go_home_end = bool(self.get_parameter("go_home_end").value)

        self.pub = self.create_publisher(Float32MultiArray, "/joint_command", 10)

    def send(self, q):
        msg = Float32MultiArray()
        msg.data = [float(v) for v in q[:N_CHANNELS]]
        self.pub.publish(msg)

    def wait_for_subscriber(self, timeout=5.0):
        """Espera a que alguien (ESP32/relay) escuche /joint_command."""
        t0 = time.time()
        while rclpy.ok() and self.pub.get_subscription_count() == 0:
            if time.time() - t0 > timeout:
                self.get_logger().warn(
                    "Nadie escucha /joint_command todavía; envío igualmente. "
                    "¿Está el agente micro-ROS / ESP32 conectado?")
                return
            rclpy.spin_once(self, timeout_sec=0.1)

    def run(self):
        self.wait_for_subscriber()
        n = 0
        try:
            while rclpy.ok() and (self.cycles == 0 or n < self.cycles):
                n += 1
                self.get_logger().info(f"========== CICLO {n} ==========")
                for name, wp, dwell in SECUENCIA:
                    if not rclpy.ok():
                        break
                    if wp is None:                     # PAUSA: solo espera
                        self.get_logger().info(f"  ⏸  {name}  ({dwell:.1f}s)")
                    else:
                        self.get_logger().info(f"  →  {name:20s} {wp}")
                        self.send(wp)
                    self._sleep(dwell * self.speed)
            self.get_logger().info("Secuencia terminada.")
        except KeyboardInterrupt:
            self.get_logger().info("Interrumpido por el usuario.")
        finally:
            if self.go_home_end and rclpy.ok():
                self.get_logger().info("Volviendo a HOME…")
                self.send(HOME)
                self._sleep(3.0)

    def _sleep(self, seconds):
        """Dormir procesando ROS (permite Ctrl+C limpio)."""
        end = time.time() + max(0.1, seconds)
        while rclpy.ok() and time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.05)


def main(args=None):
    rclpy.init(args=args)
    node = PickPlaceNode()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
