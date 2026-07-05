#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
trajectory.py
=============

Generación de trayectorias suaves (capa de dominio, sin ROS).

Por qué un perfil de POSICIÓN y no un lazo de velocidad
-------------------------------------------------------
Los servos (MG995R) son de POSICIÓN: solo aceptan un ángulo objetivo, no una
velocidad. Por eso NO se cierra un lazo de velocidad. Se genera, en lazo
abierto, una secuencia MUY FINA de setpoints q(t) publicados a tasa fija
(p. ej. 50 Hz). Como el incremento dq entre pasos es pequeño, el servo "persigue"
la referencia y el movimiento resulta suave.

Perfil TRAPEZOIDAL de velocidad:
    - Arranca lento  → pasos dq pequeños  (aceleración)
    - Va más rápido  → pasos dq grandes   (crucero a v_max)
    - Frena suave    → pasos dq pequeños  (desaceleración)
    Si el recorrido es corto, el perfil se vuelve TRIANGULAR (no alcanza v_max).
"""

from __future__ import annotations

import numpy as np


def trapezoidal_profile(dist_max: float, v_max: float, a_max: float,
                        rate: float) -> list[float]:
    """
    Devuelve una lista de fracciones ``s ∈ [0, 1]`` muestreadas a ``rate`` Hz que
    recorren un perfil de velocidad trapezoidal (o triangular si el tramo es
    corto) sobre la distancia ``dist_max``.

    ``s`` escala TODO el movimiento (sincronizado):  ``q(t) = q0 + s(t)·(q_goal - q0)``.

    dist_max : desplazamiento de la junta que MÁS se mueve (rad) o distancia
               cartesiana total (m).
    v_max    : velocidad máxima (rad/s o m/s).
    a_max    : aceleración (rad/s² o m/s²).
    rate     : frecuencia de publicación (Hz) → dt = 1/rate (fineza del paso).
    """
    if dist_max < 1e-9:
        return [1.0]

    t_acc = v_max / a_max
    d_acc = 0.5 * a_max * t_acc * t_acc

    if 2.0 * d_acc >= dist_max:                 # tramo corto → TRIANGULAR
        t_acc = np.sqrt(dist_max / a_max)
        t_flat = 0.0
        v_peak = a_max * t_acc
        d_acc = 0.5 * a_max * t_acc * t_acc
    else:                                       # TRAPEZOIDAL completo
        v_peak = v_max
        t_flat = (dist_max - 2.0 * d_acc) / v_max

    total_t = 2.0 * t_acc + t_flat
    n = max(int(np.ceil(total_t * rate)), 1)

    s_list = []
    for k in range(1, n + 1):
        t = k / rate
        if t < t_acc:                                   # aceleración
            d = 0.5 * a_max * t * t
        elif t < t_acc + t_flat:                        # crucero
            d = d_acc + v_peak * (t - t_acc)
        elif t < total_t:                               # desaceleración
            td = total_t - t
            d = dist_max - 0.5 * a_max * td * td
        else:
            d = dist_max
        s_list.append(min(d / dist_max, 1.0))

    s_list[-1] = 1.0                                    # llegada exacta
    return s_list


def joint_trajectory(q0: np.ndarray, q_goal: np.ndarray, v_max: float,
                     a_max: float, rate: float) -> list[np.ndarray]:
    """Interpolación articular sincronizada con perfil trapezoidal."""
    q0 = np.asarray(q0, dtype=float)
    q_goal = np.asarray(q_goal, dtype=float)
    dist_max = float(np.max(np.abs(q_goal - q0)))
    return [q0 + s * (q_goal - q0) for s in trapezoidal_profile(dist_max, v_max, a_max, rate)]
