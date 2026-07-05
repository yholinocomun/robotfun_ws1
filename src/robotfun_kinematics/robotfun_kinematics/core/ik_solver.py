#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ik_solver.py
============

Cinemática **inversa** del robot de 4 GDL (yaw + 3 pitch) + gripper. Capa de
dominio (sin ROS).

La tarea de un brazo de 4 GDL es **posición (3) + ángulo de aproximación φ
(1)** = 4 coordenadas con 4 juntas → sistema cuadrado. Con 4 GDL NO se
controla una orientación SO(3) completa (harían falta 6); no se promete.

Métodos, del más eficiente al más general:

  1) ANALÍTICA cerrada  (``method="analytic"``, RECOMENDADA)
     yaw directo + 2R planar por ley de cosenos + pitch de mano. Exacta,
     instantánea, global; da las dos ramas (codo arriba/abajo) y elige la
     más cercana a la semilla (continuidad de movimiento).

  2) DLS / Levenberg-Marquardt (``method="dls"``, la mejor numérica)
     dq = Jᵀ(JJᵀ + λ²I)⁻¹ e. Amortiguada → estable cerca de singularidades.

  3) Newton / Gauss-Newton (``method="newton"``)
     dq = J⁺ e. Convergencia rápida pero frágil sin amortiguamiento.

  4) Gradiente / Jacobiano transpuesto (``method="gradient"``)
     dq = α Jᵀ e. Didáctico; converge lento.

Parámetros numéricos (documentados por requisito):
  damping   λ = 0.05 rad (amortiguamiento DLS)
  tol       1e-5 (norma del error de tarea; posición en m)
  max_iter  200
  step_clip 0.4 rad por iteración (evita saltos)
  límites   q se recorta a [q_min, q_max] en cada iteración
  restarts  si no converge desde la postura actual, reintenta desde la
            semilla analítica y desde perturbaciones deterministas

Convención del ángulo de aproximación φ (plano vertical del brazo):
    φ = +90° → pinza hacia ARRIBA (HOME)
    φ =   0° → pinza horizontal hacia afuera
    φ = −90° → pinza hacia ABAJO (pick & place top-down)

Identidad exacta (validada en tests):  φ(q) = π/2 + q2 + q3 + q4.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .dh_model import ROBOT, DHChain

cos = np.cos
sin = np.sin
pi = np.pi

#: Contribución de cada junta a φ:  φ = π/2 + q2 + q3 + q4.
PHI_SIGNS = np.array([0.0, 1.0, 1.0, 1.0])


# ---------------------------------------------------------------------------
# Resultado de la IK
# ---------------------------------------------------------------------------
@dataclass
class IKResult:
    q: np.ndarray          # solución articular (rad), n_joints
    ok: bool               # True si es válida (alcanzable y dentro de límites)
    error: float           # norma del error cartesiano final (m)
    method: str            # método usado
    iterations: int = 0    # iteraciones (0 para analítica)
    reason: str = ""       # motivo si ok=False


def _wrap(a: float) -> float:
    return (a + pi) % (2.0 * pi) - pi


def approach_angle(q) -> float:
    """φ absoluto de la pinza en el plano del brazo (rad)."""
    q = np.asarray(q, dtype=float).ravel()
    return pi / 2.0 + float(PHI_SIGNS @ q[:4])


# ---------------------------------------------------------------------------
# 1) IK ANALÍTICA CERRADA
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
    IK cerrada: TCP (x,y,z) + ángulo de aproximación φ (rad).

    q1 = atan2(y, x) (el robot opera al frente, q1 ∈ ±90°); la muñeca se
    retrocede HAND a lo largo de φ y el 2R (L2, A3) da hombro/codo por ley de
    cosenos; el pitch de muñeca cierra φ. ``elbow_up`` elige la rama del codo.
    Devuelve q = [q1, q2, q3, q4] o None si esa rama no alcanza.

    Mapeo DH (ángulos absolutos en el plano, medidos desde el eje radial +r):
        q2 = ang2 − π/2 ;  q3 = ang3 − ang2 ;  q4 = φ − ang3
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
    q4 = _wrap(phi - ang3)
    return np.array([q1, q2, q3, q4])


def solve_analytic(x, y, z, phi, robot: DHChain = ROBOT, seed=None) -> IKResult:
    """
    IK analítica: prueba las dos ramas del codo, filtra por límites articulares
    y elige la válida más cercana a ``seed`` (continuidad de movimiento).
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
        return IKResult(q=q, ok=True, method="analytic",
                        error=float(np.linalg.norm(robot.tool_position(q) - target)))

    if not candidates:
        return IKResult(q=np.zeros(robot.n_joints), ok=False, error=float("inf"),
                        method="analytic",
                        reason="objetivo fuera del espacio de trabajo")
    q = robot.clamp(min(candidates, key=lambda c: c[2])[0])
    return IKResult(q=q, ok=False, method="analytic",
                    error=float(np.linalg.norm(robot.tool_position(q) - target)),
                    reason="solución fuera de límites articulares "
                           "(¿q1>90°? ¿objetivo detrás o demasiado bajo?)")


# ---------------------------------------------------------------------------
# 2-4) IK NUMÉRICA (DLS / Newton / gradiente) sobre [posición(3); φ(1)]
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
    respect_limits: bool = True


def _dq_step(J: np.ndarray, e: np.ndarray, cfg: NumericConfig) -> np.ndarray:
    m = J.shape[0]
    if cfg.method == "gradient":                # Jacobiano transpuesto
        return cfg.alpha * (J.T @ e)
    if cfg.method == "newton":                  # pseudo-inversa (Gauss-Newton)
        return J.T @ np.linalg.solve(J @ J.T + 1e-12 * np.eye(m), e)
    lam2 = cfg.damping * cfg.damping            # "dls" / Levenberg-Marquardt
    return J.T @ np.linalg.solve(J @ J.T + lam2 * np.eye(m), e)


def solve_numeric(x_des, q0, phi_des=None, robot: DHChain = ROBOT,
                  cfg: NumericConfig | None = None, restarts: int = 3,
                  **overrides) -> IKResult:
    """
    IK numérica con warm-start + reintentos desde semillas alternativas.

    Tarea = posición (3) + φ (1) si ``phi_des`` no es None (si es None,
    resuelve solo posición y la redundancia la fija el método).

    Los métodos numéricos son LOCALES: si no convergen desde ``q0`` (postura
    actual), se reintenta desde la semilla analítica coherente con la tarea y
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
        phi_seed = phi_des if phi_des is not None else -pi / 2.0
        res_a = solve_analytic(x_des[0], x_des[1], x_des[2], phi_seed, robot)
        seeds.append(res_a.q)
        rng = np.random.default_rng(12345)
        for _ in range(max(0, restarts - 1)):
            seeds.append(robot.clamp(res_a.q + rng.uniform(-0.5, 0.5, robot.n_joints)))

    best = None
    for seed in seeds:
        res = _solve_numeric_once(x_des, seed, phi_des, robot, cfg)
        if res.ok:
            return res
        if best is None or res.error < best.error:
            best = res
    return best


def _solve_numeric_once(x_des, q0, phi_des, robot: DHChain,
                        cfg: NumericConfig) -> IKResult:
    """Una pasada del solver iterativo desde una semilla concreta."""
    q = np.asarray(q0, dtype=float).ravel().copy()[:robot.n_joints]
    err = np.inf
    for it in range(1, cfg.max_iter + 1):
        T = robot.fkine(q)
        p = T[0:3, 3]
        e_parts = [x_des - p]
        J_parts = [robot.jacobian_position(q)]
        if phi_des is not None:
            e_parts.append(np.array([_wrap(phi_des - approach_angle(q))]))
            J_parts.append(PHI_SIGNS[np.newaxis, :].copy())
        e = np.hstack(e_parts)
        J = np.vstack(J_parts)

        err = float(np.linalg.norm(e))
        if err < cfg.tol:
            return IKResult(q=q, ok=True, error=float(np.linalg.norm(x_des - p)),
                            method=cfg.method, iterations=it)

        dq = _dq_step(J, e, cfg)
        nrm = np.linalg.norm(dq)
        if nrm > cfg.step_clip:
            dq *= cfg.step_clip / nrm
        q = q + dq
        if cfg.respect_limits:
            q = robot.clamp(q)

    return IKResult(q=q, ok=False, error=err, method=cfg.method,
                    iterations=cfg.max_iter,
                    reason="no convergió (¿fuera de alcance o singularidad?)")


# ---------------------------------------------------------------------------
# Interfaz unificada
# ---------------------------------------------------------------------------
def solve_ik(x_des, q0, *, approach=None, method="analytic",
             robot: DHChain = ROBOT, **kw) -> IKResult:
    """
    Punto de entrada único de la IK.

    x_des    : posición cartesiana deseada del TCP [x, y, z] (m).
    q0       : semilla articular (postura actual → continuidad/warm-start).
    approach : φ (rad) en el plano del brazo. None → −90° (pinza abajo).
    method   : "analytic" (recomendado) | "dls" | "newton" | "gradient".
    """
    x_des = np.asarray(x_des, dtype=float).ravel()
    phi = -pi / 2.0 if approach is None else approach
    if method == "analytic":
        return solve_analytic(x_des[0], x_des[1], x_des[2], phi, robot, seed=q0)
    return solve_numeric(x_des, q0, phi_des=phi, robot=robot, method=method, **kw)


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


if __name__ == "__main__":
    np.set_printoptions(suppress=True, precision=5)
    rng = np.random.default_rng(0)
    phi = -pi / 2.0                        # pinza hacia abajo (pick top-down)
    print("Pick & place sobre la mesa, pinza ABAJO (φ=−90°).")
    print("Comparación de métodos de IK (4 GDL):\n")
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
            res = solve_ik(target, seed, approach=phi, method=m)
            d = np.linalg.norm(ROBOT.tool_position(res.q) - target)
            dphi = abs(_wrap(approach_angle(res.q) - phi))
            stats[m][0] += int(d < 1e-3 and dphi < 1e-2)
            stats[m][1] += res.iterations
    print(f"  objetivos alcanzables (de {N}): {reachable}\n")
    print(f"  {'método':9s}  aciertos   iters_prom   nota")
    notas = {"analytic": "exacta, 0 iteraciones (RECOMENDADA)",
             "dls": "robusta cerca de singularidades",
             "newton": "rápida; sin amortiguar es frágil",
             "gradient": "simple/barata, converge lento"}
    for m, (ok, its) in stats.items():
        print(f"  {m:9s}  {ok:3d}/{reachable:<3d}   {its / max(reachable, 1):6.1f}     {notas[m]}")
