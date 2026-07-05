#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
dh_model.py
===========

Modelo cinemático del brazo **5 juntas (yaw, pitch, pitch, roll, pitch) +
gripper** en convención **Denavit-Hartenberg estándar**. Capa de dominio
(Clean Architecture): NO depende de ROS y se ejecuta/testea aislada:

    python3 -m robotfun_kinematics.core.dh_model

Estructura física real
----------------------
    joint_1 : yaw de base
    joint_2 : pitch de hombro
    joint_3 : pitch de codo
    joint_4 : ROLL de muñeca (eje colineal con el antebrazo)
    joint_5 : pitch de pinza
    gripper : apertura/cierre (no afecta a la FK del TCP)

El espacio de TAREA útil tiene 4 dimensiones (posición + dirección de
aproximación en el plano). Por eso el robot se opera por defecto con
**joint_4 BLOQUEADO a 0 rad (servo a 90°) → 4 GDL efectivos** con IK
analítica cerrada, y opcionalmente con joint_4 ACTIVO (5 GDL) para
orientar la pinza fuera del plano (ver ``ik_solver``).

Medidas reales (tabla DH del usuario; metros):
    L0 = 0.010   offset radial de la base (a_1)
    L1 = 0.063   base → eje de pitch del hombro (d_1)
    L2 = 0.120   hombro → codo (brazo)
    L3 = 0.090   codo → servo de roll
    L4 = 0.030   servo de roll → eje de pitch de la pinza
    L5 = 0.090   eje de pitch de la pinza → punta del gripper (TCP)

Tabla DH estándar (validada numéricamente; θ_i = q_i + θoff_i):

    i | d_i   | θ_i      | α_i  | a_i
    --|-------|----------|------|-----
    1 | L1    | q1       | +90° | L0
    2 | 0     | q2 + 90° |   0° | L2
    3 | 0     | q3 + 90° | +90° | 0
    4 | L3+L4 | q4       | +90° | 0
    5 | 0     | q5 + 90° |   0° | L5

Propiedades verificadas (tests en ``test/test_kinematics.py``):
  * HOME (q=0, servos a 90°): brazo recto vertical,
    FK(HOME) → TCP = [0.010, 0, 0.393] m.
  * joint_4 es roll PURO: girarlo en HOME no traslada el TCP.
  * La pinza apunta por el **eje X del frame final** (en HOME, hacia +Z).
  * El URDF de primitivas reproduce esta FK exactamente (TF == FK) con ejes
    j1=(0,0,1)  j2=j3=(0,-1,0)  j4=(0,0,1)  j5=(0,+1,0).
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

cos = np.cos
sin = np.sin
pi = np.pi

# ---------------------------------------------------------------------------
# Medidas físicas reales (metros) — tabla DH del usuario. Ajustar si re-mides.
# ---------------------------------------------------------------------------
L0 = 0.010              # offset radial de la base (a_1)
L1 = 0.063              # base → eje de pitch del hombro (d_1)
L2 = 0.120              # brazo (hombro → codo)
L3 = 0.090              # codo → servo de roll
L4 = 0.030              # servo de roll → eje de pitch de la pinza
L5 = 0.090              # eje de pitch de la pinza → punta del gripper (TCP)
A3 = L3 + L4            # antebrazo completo (codo → muñeca) = d_4

# Offsets θ para que q=0 == HOME (brazo RECTO y vertical, servos a 90°).
THETA_OFFSET = np.array([0.0, pi / 2.0, pi / 2.0, 0.0, pi / 2.0])

#: Nombres canónicos. DEBEN coincidir con URDF, firmware y /joint_states.
ARM_JOINT_NAMES = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5"]
GRIPPER_JOINT_NAME = "gripper"

#: Índice del roll de muñeca dentro de q (para bloquearlo/activarlo).
ROLL_INDEX = 3


def dh(d: float, theta: float, a: float, alpha: float) -> np.ndarray:
    """Matriz homogénea DH estándar: ``Rz(theta)·Tz(d)·Tx(a)·Rx(alpha)``."""
    ct, st = cos(theta), sin(theta)
    ca, sa = cos(alpha), sin(alpha)
    return np.array([
        [ct, -st * ca,  st * sa, a * ct],
        [st,  ct * ca, -ct * sa, a * st],
        [0.0,      sa,       ca,      d],
        [0.0,     0.0,      0.0,    1.0],
    ])


@dataclass(frozen=True)
class DHChain:
    """Cadena cinemática serie parametrizada por su tabla DH constante."""

    d: np.ndarray
    a: np.ndarray
    alpha: np.ndarray
    theta_offset: np.ndarray
    q_min: np.ndarray
    q_max: np.ndarray

    @property
    def n_joints(self) -> int:
        return len(self.d)

    def fkine(self, q, upto: int | None = None, return_frames: bool = False):
        """Cinemática directa. q en radianes; ignora elementos extra (gripper)."""
        q = np.asarray(q, dtype=float).ravel()
        n = self.n_joints if upto is None else upto
        T = np.eye(4)
        frames = []
        for i in range(n):
            T = T @ dh(self.d[i], q[i] + self.theta_offset[i], self.a[i], self.alpha[i])
            frames.append(T.copy())
        return (T, frames) if return_frames else T

    def tool_position(self, q) -> np.ndarray:
        return self.fkine(q)[0:3, 3]

    def tool_axis(self, q) -> np.ndarray:
        """Dirección de apuntado de la pinza = eje X del frame final (unitario)."""
        return self.fkine(q)[0:3, 0]

    def jacobian_geometric(self, q) -> np.ndarray:
        """Jacobiano geométrico 6 x n (todas las juntas revolutas): [v; ω]."""
        _, frames = self.fkine(q, return_frames=True)
        p_e = frames[-1][0:3, 3]
        J = np.zeros((6, self.n_joints))
        z_prev = np.array([0.0, 0.0, 1.0])
        p_prev = np.array([0.0, 0.0, 0.0])
        for i in range(self.n_joints):
            J[0:3, i] = np.cross(z_prev, p_e - p_prev)
            J[3:6, i] = z_prev
            z_prev = frames[i][0:3, 2]
            p_prev = frames[i][0:3, 3]
        return J

    def jacobian_position(self, q) -> np.ndarray:
        return self.jacobian_geometric(q)[0:3, :]

    def clamp(self, q) -> np.ndarray:
        return np.clip(np.asarray(q, dtype=float), self.q_min, self.q_max)

    def in_limits(self, q, tol: float = 1e-9) -> bool:
        q = np.asarray(q, dtype=float)
        return bool(np.all(q >= self.q_min - tol) and np.all(q <= self.q_max + tol))

    # -- geometría para la IK analítica (estructura yaw + 2R + pitch de mano) --
    @property
    def base_offset(self) -> float:
        return float(self.a[0])          # L0 (offset radial del hombro)

    @property
    def shoulder_height(self) -> float:
        return float(self.d[0])          # L1

    @property
    def link_upper(self) -> float:
        return float(self.a[1])          # L2

    @property
    def link_fore(self) -> float:
        return float(self.d[3])          # A3 = L3 + L4

    @property
    def link_hand(self) -> float:
        return float(self.a[4])          # L5


def build_default_robot() -> DHChain:
    """Construye la cadena DH validada del robot (5 juntas)."""
    n = 5
    return DHChain(
        d=np.array([L1, 0.0, 0.0, A3, 0.0]),
        a=np.array([L0, L2, 0.0, 0.0, L5]),
        alpha=np.array([pi / 2.0, 0.0, pi / 2.0, pi / 2.0, 0.0]),
        theta_offset=THETA_OFFSET.copy(),
        q_min=np.array([-pi / 2.0] * n),   # servos saturados a ±90°
        q_max=np.array([pi / 2.0] * n),
    )


ROBOT: DHChain = build_default_robot()
N_JOINTS: int = ROBOT.n_joints           # 5 juntas de brazo (sin gripper)


def fkine(q, **kw):
    return ROBOT.fkine(q, **kw)


def jacobian_geometric(q):
    return ROBOT.jacobian_geometric(q)


def jacobian_position(q):
    return ROBOT.jacobian_position(q)


if __name__ == "__main__":
    np.set_printoptions(suppress=True, precision=5)
    _, fr = ROBOT.fkine(np.zeros(N_JOINTS), return_frames=True)
    print("FK(HOME) — brazo recto y vertical (tabla DH del usuario):")
    print("  hombro :", np.round(fr[0][:3, 3], 4), " esperado (0.010, 0, 0.063)")
    print("  codo   :", np.round(fr[1][:3, 3], 4), " esperado (0.010, 0, 0.183)")
    print("  muñeca :", np.round(fr[3][:3, 3], 4), " esperado (0.010, 0, 0.303)")
    print("  TCP    :", np.round(fr[4][:3, 3], 4), " esperado (0.010, 0, 0.393)")
    print("  pinza apunta (eje X final):", np.round(fr[4][:3, 0], 4), " esperado (0, 0, 1)")
    print("\njoint_4 es roll puro (TCP no se mueve en HOME):")
    for q4 in (-pi / 2, -pi / 4, pi / 4, pi / 2):
        p = ROBOT.tool_position([0, 0, 0, q4, 0])
        print(f"  q4={np.degrees(q4):6.1f}°  TCP={np.round(p, 5)}")
    print(f"\nalcance máx desde el hombro = L2+A3+L5 = {L2 + A3 + L5:.4f} m")
    print("θ offsets (deg):", np.round(np.degrees(ROBOT.theta_offset), 2))
