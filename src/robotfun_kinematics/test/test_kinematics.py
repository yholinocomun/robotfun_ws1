#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Tests del núcleo de cinemática 4 GDL (yaw + 3 pitch) + gripper.

Cubren las condiciones de validación del proyecto:
  * FK(HOME) = [0.010, 0, 0.393] con hombro/codo/muñeca en su sitio.
  * La pinza apunta por el eje X del frame final.
  * El URDF de primitivas reproduce la FK DH exactamente (TF == FK).
  * Identidad del ángulo de aproximación: φ = π/2 + q2 + q3 + q4.
  * IK analítica exacta e IK numérica (dls/newton) con warm-start.
  * Límites articulares, área de trabajo y trayectorias trapezoidales.
"""

import numpy as np
import pytest

from robotfun_kinematics.core import (
    A3, HAND, L0, L1, L2, N_JOINTS, ROBOT, WorkspaceLimits, approach_angle,
    clamp_target, joint_trajectory, solve_analytic, solve_ik,
    trapezoidal_profile, validate_target,
)
from robotfun_kinematics.core.ik_solver import _wrap

PHI_DOWN = -np.pi / 2.0


# ---------------------------------------------------------------------------
# Cinemática directa
# ---------------------------------------------------------------------------
def test_n_joints_is_four():
    assert N_JOINTS == 4


def test_fk_home_matches_expected():
    """FK(HOME=q0): brazo recto vertical (tabla DH del usuario, HAND=0.090)."""
    _, fr = ROBOT.fkine(np.zeros(N_JOINTS), return_frames=True)
    np.testing.assert_allclose(fr[0][:3, 3], [0.010, 0.0, 0.063], atol=1e-9)  # hombro
    np.testing.assert_allclose(fr[1][:3, 3], [0.010, 0.0, 0.183], atol=1e-9)  # codo
    np.testing.assert_allclose(fr[2][:3, 3], [0.010, 0.0, 0.303], atol=1e-9)  # muñeca
    np.testing.assert_allclose(fr[3][:3, 3], [0.010, 0.0, 0.393], atol=1e-9)  # TCP


def test_gripper_points_along_tool_x():
    """La pinza apunta por el eje X del frame final; en HOME hacia +Z."""
    np.testing.assert_allclose(ROBOT.tool_axis(np.zeros(N_JOINTS)),
                               [0.0, 0.0, 1.0], atol=1e-12)


def test_approach_angle_identity():
    """φ(q) = π/2 + q2 + q3 + q4 exacto (comprobado contra la geometría)."""
    rng = np.random.default_rng(3)
    for _ in range(50):
        q = rng.uniform(-1.2, 1.2, N_JOINTS)
        a = ROBOT.tool_axis(q)
        c1, s1 = np.cos(q[0]), np.sin(q[0])
        phi_geo = np.arctan2(a[2], a[0] * c1 + a[1] * s1)
        assert abs(_wrap(approach_angle(q) - phi_geo)) < 1e-9


# ---------------------------------------------------------------------------
# URDF == FK (el esqueleto del modelo de primitivas reproduce la tabla DH)
# ---------------------------------------------------------------------------
def _T(xyz):
    M = np.eye(4)
    M[:3, 3] = xyz
    return M


def _R(axis, ang):
    x, y, z = axis
    c, s = np.cos(ang), np.sin(ang)
    C = 1 - c
    M = np.eye(4)
    M[:3, :3] = [[x * x * C + c, x * y * C - z * s, x * z * C + y * s],
                 [y * x * C + z * s, y * y * C + c, y * z * C - x * s],
                 [z * x * C - y * s, z * y * C + x * s, z * z * C + c]]
    return M


def _fk_urdf(q):
    """Esqueleto del URDF de primitivas (mismos orígenes y ejes que el xacro)."""
    return (_R((0, 0, 1), q[0])                              # joint_1 yaw
            @ _T([L0, 0, L1]) @ _R((0, -1, 0), q[1])         # joint_2 hombro
            @ _T([0, 0, L2]) @ _R((0, -1, 0), q[2])          # joint_3 codo
            @ _T([0, 0, A3]) @ _R((0, -1, 0), q[3])          # joint_4 muñeca
            @ _T([0, 0, HAND]))                              # tool0 (TCP)


def test_urdf_skeleton_equals_dh_fk():
    rng = np.random.default_rng(7)
    for _ in range(100):
        q = rng.uniform(-np.pi / 2, np.pi / 2, N_JOINTS)
        p_dh = ROBOT.tool_position(q)
        p_urdf = _fk_urdf(q)[:3, 3]
        np.testing.assert_allclose(p_urdf, p_dh, atol=1e-12)


# ---------------------------------------------------------------------------
# IK analítica y numérica
# ---------------------------------------------------------------------------
def _reachable_front_targets(n, seed_rng=0, phi=PHI_DOWN):
    rng = np.random.default_rng(seed_rng)
    out = []
    while len(out) < n:
        x = rng.uniform(0.10, 0.26)
        y = rng.uniform(-0.14, 0.14)
        z = rng.uniform(0.04, 0.20)
        if solve_analytic(x, y, z, phi).ok:
            out.append((x, y, z))
    return out


def test_analytic_ik_exact():
    """La IK analítica reproduce posición y φ exactos."""
    for (x, y, z) in _reachable_front_targets(40):
        res = solve_analytic(x, y, z, PHI_DOWN)
        assert res.ok
        assert np.linalg.norm(ROBOT.tool_position(res.q) - [x, y, z]) < 1e-9
        assert abs(_wrap(approach_angle(res.q) - PHI_DOWN)) < 1e-9


def test_analytic_ik_multiple_phis():
    for phi_deg in (0.0, -45.0, -90.0):
        phi = np.radians(phi_deg)
        for (x, y, z) in _reachable_front_targets(10, seed_rng=int(phi_deg) + 90, phi=phi):
            res = solve_analytic(x, y, z, phi)
            assert res.ok
            assert np.linalg.norm(ROBOT.tool_position(res.q) - [x, y, z]) < 1e-9
            assert abs(_wrap(approach_angle(res.q) - phi)) < 1e-9


@pytest.mark.parametrize("method", ["dls", "newton"])
def test_numeric_ik_with_warm_seed(method):
    """DLS y Newton convergen a posición+φ con semilla pick-ready (warm-start)."""
    seed = solve_analytic(0.18, 0.0, 0.10, PHI_DOWN).q
    ok = 0
    targets = _reachable_front_targets(30, seed_rng=1)
    for (x, y, z) in targets:
        res = solve_ik([x, y, z], seed, approach=PHI_DOWN, method=method)
        p = ROBOT.tool_position(res.q)
        if (np.linalg.norm(p - [x, y, z]) < 1e-3
                and abs(_wrap(approach_angle(res.q) - PHI_DOWN)) < 1e-2):
            ok += 1
    assert ok >= 28      # tolera algún caso al borde del espacio de trabajo


def test_joint_limits_respected():
    res = solve_ik([0.20, 0.0, 0.10], np.zeros(N_JOINTS), approach=PHI_DOWN,
                   method="dls")
    assert np.all(res.q >= ROBOT.q_min - 1e-9)
    assert np.all(res.q <= ROBOT.q_max + 1e-9)


def test_unreachable_target_flagged():
    res = solve_analytic(0.6, 0.0, 0.4, PHI_DOWN)
    assert not res.ok and res.reason


# ---------------------------------------------------------------------------
# Área de trabajo y trayectorias
# ---------------------------------------------------------------------------
def test_workspace_validate_and_clamp():
    lim = WorkspaceLimits()
    ok, _ = validate_target([0.20, 0.0, 0.10], lim)
    assert ok
    bad, reason = validate_target([0.0, 0.0, 0.0], lim)
    assert not bad and reason
    c = clamp_target([0.9, 0.9, 0.9], lim)
    ok2, _ = validate_target(c, lim)
    assert ok2


def test_trapezoidal_profile_properties():
    s = trapezoidal_profile(1.0, v_max=0.5, a_max=1.0, rate=50.0)
    assert s[-1] == 1.0
    assert all(b >= a - 1e-12 for a, b in zip(s, s[1:]))       # monótono
    assert max(np.diff([0.0] + s)) <= 0.5 / 50.0 + 1e-6        # respeta v_max


def test_joint_trajectory_endpoints():
    q0 = np.zeros(N_JOINTS + 1)
    qg = np.array([0.4, -0.3, 0.2, 0.5, 0.3])
    traj = joint_trajectory(q0, qg, v_max=0.6, a_max=1.2, rate=50.0)
    np.testing.assert_allclose(traj[-1], qg, atol=1e-12)
