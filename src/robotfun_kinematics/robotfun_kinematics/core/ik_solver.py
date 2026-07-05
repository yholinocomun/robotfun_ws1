#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ik_solver.py
============

Cinemática **inversa** del brazo (5 juntas + gripper). Capa de dominio (sin ROS).

Con 5 GDL NO se controla una orientación SO(3) completa (harían falta 6). La
tarea realista para pick & place es:

    posición del TCP (3)  +  dirección de apuntado de la pinza (eje X del
    frame final; 2 GDL de dirección como máximo).

Dos MODOS de operación (requisito del proyecto):

  A) ``lock_joint_4=True``  (por defecto) — joint_4 (roll) BLOQUEADO a 0 rad
     (servo a 90°). El brazo queda yaw + 3 pitch coplanares = **4 GDL
     efectivos** y la tarea es posición (3) + ángulo de aproximación φ en el
     plano vertical del brazo (1) → sistema cuadrado con **IK ANALÍTICA
     CERRADA** (exacta, instantánea, global). Es el modo recomendado para
     empezar el pick & place de pastillas.

  B) ``lock_joint_4=False`` — joint_4 ACTIVO. La tarea es posición (3) +
     dirección de apuntado 3D (vector unitario; 2 independientes) → 5
     ecuaciones efectivas con 5 juntas. Se resuelve numéricamente con **DLS
     (Levenberg-Marquardt amortiguado)**. Permite inclinar/orientar la pinza
     FUERA del plano del brazo (imposible en modo A) y mejora la continuidad
     en trayectorias; es el modo para la versión final con visión.

Métodos numéricos disponibles (también para el modo A, como verificación):
  * ``dls``      dq = Jᵀ(JJᵀ + λ²I)⁻¹ e   — amortiguado, robusto en singularidades.
  * ``newton``   dq = J⁺ e                — Gauss-Newton, rápido pero frágil.
  * ``gradient`` dq = α Jᵀ e              — didáctico, converge lento.

Parámetros numéricos (documentados por requisito):
  damping   λ = 0.05 rad (amortiguamiento DLS; sube a 0.1 cerca de singularidades)
  tol       1e-5 (norma del error de tarea combinado; posición en m)
  max_iter  200
  step_clip 0.4 rad por iteración (evita saltos)
  límites   q se recorta a [q_min, q_max] en cada iteración (respect_limits)
  dir_weight 0.05 m (peso que convierte el error de dirección, adimensional,
             a metros para mezclarlo con el error de posición)

Convención del ángulo de aproximación φ (modo A, plano vertical del brazo):
    φ = +90° → pinza hacia ARRIBA (HOME)
    φ =   0° → pinza horizontal hacia afuera
    φ = −90° → pinza hacia ABAJO (pick & place top-down)

Identidad exacta (validada en tests, q4=0):  φ(q) = π/2 + q2 + q3 − q5.
(El signo −q5 viene de la tabla DH: el eje del pitch de pinza queda en +Y.)
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .dh_model import ROBOT, ROLL_INDEX, DHChain

cos = np.cos
sin = np.sin
pi = np.pi

#: Signos con los que cada junta contribuye a φ (q4=0):  φ = π/2 + q2 + q3 − q5
PHI_SIGNS = np.array([0.0, 1.0, 1.0, 0.0, -1.0])


# ---------------------------------------------------------------------------
# Resultado de la IK
# ---------------------------------------------------------------------------
@dataclass
class IKResult:
    q: np.ndarray            # solución articular (rad), n_joints
    ok: bool                 # True si es válida (alcanzable y dentro de límites)
    error: float             # norma del error cartesiano final (m)
    method: str              # método usado
    mode: str = "locked"     # "locked" (joint_4=0) | "roll" (joint_4 activo)
    iterations: int = 0      # iteraciones (0 para analítica)
    reason: str = ""         # motivo si ok=False


def _wrap(a: float) -> float:
    return (a + pi) % (2.0 * pi) - pi


def approach_angle(q) -> float:
    """φ absoluto de la pinza en el plano del brazo (rad). Exacto si q4=0."""
    q = np.asarray(q, dtype=float).ravel()
    return pi / 2.0 + float(PHI_SIGNS @ q[:5])


def approach_direction(x: float, y: float, phi: float) -> np.ndarray:
    """Vector unitario de apuntado para φ en el plano vertical hacia (x, y)."""
    psi = np.arctan2(y, x)
    return np.array([cos(phi) * cos(psi), cos(phi) * sin(psi), sin(phi)])


# ---------------------------------------------------------------------------
# A) IK ANALÍTICA CERRADA — modo joint_4 BLOQUEADO (q4 = 0)
# ---------------------------------------------------------------------------
def _planar_2r(rw: float, zw: float, sr: float, d1: float, a2: float, a3: float,
               elbow_up: bool):
    """2R planar: hombro (sr, d1) → muñeca (rw, zw). (ang2_abs, ang3_abs) o None."""
    dr, dz = rw - sr, zw - d1
    c3 = (dr * dr + dz * dz - a2 * a2 - a3 * a3) / (2.0 * a2 * a3)
    if c3 < -1.0 or c3 > 1.0:
        return None
    s3 = np.sqrt(max(0.0, 1.0 - c3 * c3))
    if elbow_up:
        s3 = -s3
    ang_elbow = np.arctan2(s3, c3)
    ang2 = np.arctan2(dz, dr) - np.arctan2(a3 * sin(ang_elbow), a2 + a3 * cos(ang_elbow))
    return ang2, ang2 + ang_elbow


def ik_analytic(x: float, y: float, z: float, phi: float,
                robot: DHChain = ROBOT, elbow_up: bool = False):
    """
    IK cerrada con joint_4=0: TCP (x,y,z) + ángulo de aproximación φ (rad).

    q1 = atan2(y, x) (el robot opera al frente, q1 ∈ ±90°); la muñeca se
    retrocede L5 a lo largo de φ y el 2R (L2, L3+L4) da hombro/codo por ley de
    cosenos; el pitch de pinza cierra φ. ``elbow_up`` elige la rama del codo.
    Devuelve q = [q1, q2, q3, 0, q5] o None si esa rama no alcanza.

    Mapeo DH (ángulos absolutos en el plano, medidos desde el eje radial +r):
        q2 = ang2 − π/2 ;  q3 = ang3 − ang2 ;  q5 = ang3 − φ   (q4 = 0)
    """
    d1, a2, a3, hand = (robot.shoulder_height, robot.link_upper,
                        robot.link_fore, robot.link_hand)
    sr = robot.base_offset                       # offset radial del hombro (L0)
    q1 = np.arctan2(y, x)
    r = np.hypot(x, y)
    rw = r - hand * cos(phi)                     # muñeca en el plano (r, z)
    zw = z - hand * sin(phi)
    planar = _planar_2r(rw, zw, sr, d1, a2, a3, elbow_up)
    if planar is None:
        return None
    ang2, ang3 = planar
    q2 = _wrap(ang2 - pi / 2.0)
    q3 = _wrap(ang3 - ang2)
    q5 = _wrap(ang3 - phi)
    return np.array([q1, q2, q3, 0.0, q5])


def solve_analytic(x, y, z, phi, robot: DHChain = ROBOT, seed=None) -> IKResult:
    """
    IK analítica (modo bloqueado): prueba las dos ramas del codo, filtra por
    límites articulares y elige la válida más cercana a ``seed`` (continuidad).
    """
    target = np.array([x, y, z])
    candidates = []   # (q, in_limits, err)
    for up in (False, True):
        q = ik_analytic(x, y, z, phi, robot, elbow_up=up)
        if q is None:
            continue
        err = float(np.linalg.norm(robot.tool_position(q) - target))
        candidates.append((q, robot.in_limits(q), err))

    valid = [c for c in candidates if c[1] and c[2] < 1e-4]
    if valid:
        if seed is not None:
            s = np.asarray(seed, dtype=float).ravel()[:robot.n_joints]
            q = min(valid, key=lambda c: np.linalg.norm(c[0] - s))[0]
        else:
            q = valid[0][0]
        return IKResult(q=q, ok=True, method="analytic", mode="locked",
                        error=float(np.linalg.norm(robot.tool_position(q) - target)))

    if not candidates:
        return IKResult(q=np.zeros(robot.n_joints), ok=False, error=float("inf"),
                        method="analytic", mode="locked",
                        reason="objetivo fuera del espacio de trabajo")
    q = robot.clamp(min(candidates, key=lambda c: c[2])[0])
    return IKResult(q=q, ok=False, method="analytic", mode="locked",
                    error=float(np.linalg.norm(robot.tool_position(q) - target)),
                    reason="solución fuera de límites articulares "
                           "(¿q1>90°? ¿objetivo detrás o demasiado bajo?)")


# ---------------------------------------------------------------------------
# B) IK NUMÉRICA (DLS / Newton / gradiente) — modo bloqueado o roll activo
# ---------------------------------------------------------------------------
@dataclass
class NumericConfig:
    """Parámetros del solver numérico (ver docstring del módulo)."""

    method: str = "dls"          # "dls" | "newton" | "gradient"
    damping: float = 0.05        # λ del DLS (rad)
    alpha: float = 0.3           # paso del método de gradiente
    max_iter: int = 200
    tol: float = 1e-5            # norma del error de tarea para converger
    step_clip: float = 0.4       # máximo |dq| por iteración (rad)
    dir_weight: float = 0.05     # peso (m) del error de dirección
    respect_limits: bool = True


def _skew(v: np.ndarray) -> np.ndarray:
    return np.array([[0.0, -v[2], v[1]],
                     [v[2], 0.0, -v[0]],
                     [-v[1], v[0], 0.0]])


def _dq_step(J: np.ndarray, e: np.ndarray, cfg: NumericConfig) -> np.ndarray:
    m = J.shape[0]
    if cfg.method == "gradient":                # Jacobiano transpuesto
        return cfg.alpha * (J.T @ e)
    if cfg.method == "newton":                  # pseudo-inversa (Gauss-Newton)
        return J.T @ np.linalg.solve(J @ J.T + 1e-12 * np.eye(m), e)
    lam2 = cfg.damping * cfg.damping            # "dls" / Levenberg-Marquardt
    return J.T @ np.linalg.solve(J @ J.T + lam2 * np.eye(m), e)


def solve_numeric(x_des, q0, phi_des=None, dir_des=None, lock_joint_4=True,
                  robot: DHChain = ROBOT, cfg: NumericConfig | None = None,
                  restarts: int = 3, **overrides) -> IKResult:
    """
    IK numérica con warm-start + reintentos desde semillas alternativas.

    Tarea = posición (3) + UNA de estas orientaciones (o ninguna):
      * ``phi_des``  ángulo de aproximación en el plano del brazo (solo tiene
                     sentido con ``lock_joint_4=True``; con roll activo se
                     convierte internamente en un ``dir_des``).
      * ``dir_des``  vector unitario 3D de apuntado de la pinza (eje X final).

    Con ``lock_joint_4=True`` la columna de joint_4 se elimina del Jacobiano y
    q4 se mantiene en 0 (mismo comportamiento que el hardware con el servo de
    roll fijo a 90°).

    Los métodos numéricos son LOCALES: si no convergen desde ``q0`` (postura
    actual), se reintenta desde una semilla analítica coherente con la tarea y
    desde perturbaciones deterministas (``restarts``).
    """
    cfg = cfg or NumericConfig()
    for k, v in overrides.items():
        if hasattr(cfg, k):
            setattr(cfg, k, v)

    x_des = np.asarray(x_des, dtype=float).ravel()
    q0 = np.asarray(q0, dtype=float).ravel().copy()[:robot.n_joints]
    if len(q0) < robot.n_joints:
        q0 = np.pad(q0, (0, robot.n_joints - len(q0)))

    seeds = [q0]
    if restarts > 0:
        # semilla analítica coherente con la tarea (elevación de dir_des o φ)
        if dir_des is not None:
            d = np.asarray(dir_des, dtype=float).ravel()
            phi_seed = np.arctan2(d[2], np.hypot(d[0], d[1]))
        else:
            phi_seed = phi_des if phi_des is not None else -pi / 2.0
        res_a = solve_analytic(x_des[0], x_des[1], x_des[2], phi_seed, robot)
        seeds.append(res_a.q)
        rng = np.random.default_rng(12345)
        for _ in range(max(0, restarts - 1)):
            seeds.append(robot.clamp(res_a.q + rng.uniform(-0.5, 0.5, robot.n_joints)))

    best = None
    for seed in seeds:
        res = _solve_numeric_once(x_des, seed, phi_des, dir_des, lock_joint_4,
                                  robot, cfg)
        if res.ok:
            return res
        if best is None or res.error < best.error:
            best = res
    return best


def _solve_numeric_once(x_des, q0, phi_des, dir_des, lock_joint_4,
                        robot: DHChain, cfg: NumericConfig) -> IKResult:
    """Una pasada del solver iterativo desde una semilla concreta."""
    q = np.asarray(q0, dtype=float).ravel().copy()[:robot.n_joints]
    mode = "locked" if lock_joint_4 else "roll"
    if lock_joint_4:
        q[ROLL_INDEX] = 0.0
        active = [i for i in range(robot.n_joints) if i != ROLL_INDEX]
    else:
        active = list(range(robot.n_joints))
        if phi_des is not None and dir_des is None:
            # con roll activo la tarea angular se expresa como dirección 3D
            dir_des = approach_direction(x_des[0], x_des[1], phi_des)
            phi_des = None
    if dir_des is not None:
        dir_des = np.asarray(dir_des, dtype=float).ravel()
        n = np.linalg.norm(dir_des)
        if n < 1e-9:
            return IKResult(q=q, ok=False, error=float("inf"), method=cfg.method,
                            mode=mode, reason="dir_des nulo")
        dir_des = dir_des / n

    err = np.inf
    for it in range(1, cfg.max_iter + 1):
        T = robot.fkine(q)
        p, a = T[0:3, 3], T[0:3, 0]
        Jg = robot.jacobian_geometric(q)
        e_parts = [x_des - p]
        J_parts = [Jg[0:3, :]]
        if dir_des is not None:
            e_parts.append(cfg.dir_weight * (dir_des - a))
            J_parts.append(cfg.dir_weight * (-_skew(a) @ Jg[3:6, :]))
        elif phi_des is not None:
            e_parts.append(np.array([_wrap(phi_des - approach_angle(q))]))
            J_parts.append(PHI_SIGNS[np.newaxis, :].copy())
        e = np.hstack(e_parts)
        J = np.vstack(J_parts)[:, active]

        err = float(np.linalg.norm(e))
        if err < cfg.tol:
            return IKResult(q=q, ok=True, error=float(np.linalg.norm(x_des - p)),
                            method=cfg.method, mode=mode, iterations=it)

        dq = _dq_step(J, e, cfg)
        nrm = np.linalg.norm(dq)
        if nrm > cfg.step_clip:
            dq *= cfg.step_clip / nrm
        q[active] = q[active] + dq
        if cfg.respect_limits:
            q = robot.clamp(q)
        if lock_joint_4:
            q[ROLL_INDEX] = 0.0

    return IKResult(q=q, ok=False, error=err, method=cfg.method, mode=mode,
                    iterations=cfg.max_iter,
                    reason="no convergió (¿fuera de alcance o singularidad?)")


# ---------------------------------------------------------------------------
# Interfaz unificada
# ---------------------------------------------------------------------------
def solve_ik(x_des, q0, *, approach=None, direction=None, method="analytic",
             lock_joint_4=True, robot: DHChain = ROBOT, **kw) -> IKResult:
    """
    Punto de entrada único de la IK.

    x_des       : posición cartesiana deseada del TCP [x, y, z] (m).
    q0          : semilla articular (postura actual → continuidad/warm-start).
    approach    : φ (rad) en el plano del brazo. None → −90° (pinza abajo).
    direction   : vector 3D de apuntado; si se da, tiene prioridad sobre
                  ``approach`` y fuerza el solver numérico.
    method      : "analytic" (solo modo bloqueado) | "dls" | "newton" | "gradient".
    lock_joint_4: True → modo A (roll bloqueado); False → modo B (roll activo).
    """
    x_des = np.asarray(x_des, dtype=float).ravel()

    if direction is None and method == "analytic" and lock_joint_4:
        phi = -pi / 2.0 if approach is None else approach
        return solve_analytic(x_des[0], x_des[1], x_des[2], phi, robot, seed=q0)

    numeric = method if method != "analytic" else "dls"
    if direction is not None:
        return solve_numeric(x_des, q0, dir_des=direction,
                             lock_joint_4=lock_joint_4, robot=robot,
                             method=numeric, **kw)
    phi = -pi / 2.0 if approach is None else approach
    return solve_numeric(x_des, q0, phi_des=phi, lock_joint_4=lock_joint_4,
                         robot=robot, method=numeric, **kw)


# ---------------------------------------------------------------------------
# Utilidades de orientación (reutilizadas por los nodos)
# ---------------------------------------------------------------------------
def rot_to_quat(R: np.ndarray):
    """Matriz de rotación → cuaternión (x, y, z, w)."""
    tr = np.trace(R)
    if tr > 0:
        s = np.sqrt(tr + 1.0) * 2
        qw = 0.25 * s; qx = (R[2, 1] - R[1, 2]) / s
        qy = (R[0, 2] - R[2, 0]) / s; qz = (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        qw = (R[2, 1] - R[1, 2]) / s; qx = 0.25 * s
        qy = (R[0, 1] + R[1, 0]) / s; qz = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        qw = (R[0, 2] - R[2, 0]) / s; qx = (R[0, 1] + R[1, 0]) / s
        qy = 0.25 * s; qz = (R[1, 2] + R[2, 1]) / s
    else:
        s = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
        qw = (R[1, 0] - R[0, 1]) / s; qx = (R[0, 2] + R[2, 0]) / s
        qy = (R[1, 2] + R[2, 1]) / s; qz = 0.25 * s
    return qx, qy, qz, qw


def quat_to_rot(qx: float, qy: float, qz: float, qw: float) -> np.ndarray:
    """Cuaternión (x, y, z, w) → matriz de rotación 3x3."""
    n = np.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if n < 1e-12:
        return np.eye(3)
    qx, qy, qz, qw = qx / n, qy / n, qz / n, qw / n
    return np.array([
        [1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qz * qw), 2 * (qx * qz + qy * qw)],
        [2 * (qx * qy + qz * qw), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qx * qw)],
        [2 * (qx * qz - qy * qw), 2 * (qy * qz + qx * qw), 1 - 2 * (qx * qx + qy * qy)],
    ])


if __name__ == "__main__":
    np.set_printoptions(suppress=True, precision=5)
    rng = np.random.default_rng(0)
    phi = -pi / 2.0                        # pinza hacia abajo (pick top-down)
    print("Pick & place sobre la mesa, pinza ABAJO (φ=−90°).")
    print("Comparación de métodos — modo joint_4 BLOQUEADO:\n")
    seed = solve_analytic(0.18, 0.0, 0.10, phi).q
    stats = {m: [0, 0.0] for m in ["analytic", "dls", "newton", "gradient"]}
    reachable = 0
    N = 200
    for _ in range(N):
        x = rng.uniform(0.10, 0.28); y = rng.uniform(-0.15, 0.15); z = rng.uniform(0.04, 0.22)
        target = np.array([x, y, z])
        if not solve_analytic(x, y, z, phi).ok:
            continue                       # no alcanzable apuntando abajo
        reachable += 1
        for m in stats:
            res = solve_ik(target, seed, approach=phi, method=m, lock_joint_4=True)
            d = np.linalg.norm(ROBOT.tool_position(res.q) - target)
            dphi = abs(_wrap(approach_angle(res.q) - phi))
            stats[m][0] += int(d < 1e-3 and dphi < 1e-2)
            stats[m][1] += res.iterations
    print(f"  objetivos alcanzables (de {N}): {reachable}\n")
    print(f"  {'método':9s}  aciertos   iters_prom")
    for m, (ok, its) in stats.items():
        print(f"  {m:9s}  {ok:3d}/{reachable:<3d}   {its / max(reachable, 1):6.1f}")

    print("\nModo joint_4 ACTIVO (roll): posición + dirección FUERA del plano.")
    q_true = np.array([0.30, 0.45, 0.50, 0.80, -0.70])   # postura factible
    target = ROBOT.tool_position(q_true)
    d_des = ROBOT.tool_axis(q_true)
    seed = np.zeros(5)
    res = solve_numeric(target, seed, dir_des=d_des, lock_joint_4=False)
    a = ROBOT.tool_axis(res.q)
    print(f"  ok={res.ok} iters={res.iterations} err_pos={res.error:.2e} "
          f"ángulo(a, a_des)={np.degrees(np.arccos(np.clip(a @ d_des, -1, 1))):.3f}° "
          f"q4={np.degrees(res.q[ROLL_INDEX]):.1f}°")
    res_locked = solve_numeric(target, seed, dir_des=d_des, lock_joint_4=True)
    a_l = ROBOT.tool_axis(res_locked.q)
    print(f"  (bloqueado no puede): ángulo residual "
          f"{np.degrees(np.arccos(np.clip(a_l @ d_des, -1, 1))):.1f}°")
