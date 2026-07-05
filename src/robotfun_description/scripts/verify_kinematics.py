#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
verify_kinematics.py
====================

Verifica que el URDF REAL (robotfun.urdf.xacro expandido) reproduce EXACTAMENTE
la FK del núcleo DH (``robotfun_kinematics.core``): TF == FK.

A diferencia de una copia manual del esqueleto, este script:
  1. Expande el xacro del modelo de primitivas (librería ``xacro``).
  2. Parsea el URDF resultante (xml.etree) y construye la cadena
     world → tool0 componiendo <origin> y rotaciones sobre <axis>.
  3. Compara la posición del TCP con la FK DH para q aleatorios y HOME.

Uso (no requiere ROS instalado, solo ``pip install xacro numpy``):
    python3 scripts/verify_kinematics.py
"""

import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np

# --- localizar el núcleo de cinemática (instalado o en el árbol de src) -----
try:
    from robotfun_kinematics.core import ARM_JOINT_NAMES, N_JOINTS, ROBOT
except ImportError:
    sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "robotfun_kinematics"))
    from robotfun_kinematics.core import ARM_JOINT_NAMES, N_JOINTS, ROBOT

XACRO_FILE = Path(__file__).resolve().parents[1] / "urdf" / "robotfun.urdf.xacro"


def rpy_to_rot(r, p, y):
    cr, sr = np.cos(r), np.sin(r)
    cp, sp = np.cos(p), np.sin(p)
    cy, sy = np.cos(y), np.sin(y)
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    return Rz @ Ry @ Rx


def axis_rot(axis, ang):
    x, y, z = axis / np.linalg.norm(axis)
    c, s = np.cos(ang), np.sin(ang)
    C = 1 - c
    return np.array([[x * x * C + c, x * y * C - z * s, x * z * C + y * s],
                     [y * x * C + z * s, y * y * C + c, y * z * C - x * s],
                     [z * x * C - y * s, z * y * C + x * s, z * z * C + c]])


def expand_xacro(path: Path) -> str:
    """Expande el xacro resolviendo $(find robotfun_description) localmente."""
    import re

    import xacro

    pkg_root = str(path.parents[1])
    raw = path.read_text()
    raw = re.sub(r"\$\(find robotfun_description\)", pkg_root, raw)
    tmp = path.parent / ".verify_expanded.urdf.xacro"
    tmp.write_text(raw)
    try:
        doc = xacro.process_file(str(tmp))
        return doc.toprettyxml(indent="  ")
    finally:
        tmp.unlink(missing_ok=True)


def parse_chain(urdf_xml: str):
    """Extrae la cadena world→tool0: [(joint_name, tipo, T_origin, axis), ...]."""
    root = ET.fromstring(urdf_xml)
    joints = {}
    for j in root.findall("joint"):
        parent = j.find("parent").attrib["link"]
        child = j.find("child").attrib["link"]
        o = j.find("origin")
        xyz = np.array([float(v) for v in (o.attrib.get("xyz", "0 0 0")).split()]) \
            if o is not None else np.zeros(3)
        rpy = np.array([float(v) for v in (o.attrib.get("rpy", "0 0 0")).split()]) \
            if o is not None else np.zeros(3)
        ax = j.find("axis")
        axis = np.array([float(v) for v in ax.attrib["xyz"].split()]) \
            if ax is not None else np.array([0.0, 0.0, 1.0])
        T = np.eye(4)
        T[:3, :3] = rpy_to_rot(*rpy)
        T[:3, 3] = xyz
        joints[child] = (j.attrib["name"], j.attrib["type"], T, axis, parent)

    chain = []
    link = "tool0"
    while link != "world":
        if link not in joints:
            raise RuntimeError(f"eslabón sin junta padre: {link}")
        name, jtype, T, axis, parent = joints[link]
        chain.append((name, jtype, T, axis))
        link = parent
    return list(reversed(chain))


def fk_urdf(chain, q_map):
    T = np.eye(4)
    for name, jtype, T_org, axis in chain:
        T = T @ T_org
        if jtype == "revolute":
            R = np.eye(4)
            R[:3, :3] = axis_rot(axis, q_map.get(name, 0.0))
            T = T @ R
    return T


def main() -> int:
    print(f"expandiendo {XACRO_FILE.name} …")
    urdf_xml = expand_xacro(XACRO_FILE)
    chain = parse_chain(urdf_xml)
    print("cadena world→tool0:", " → ".join(n for n, _, _, _ in chain))

    rng = np.random.default_rng(0)
    worst = 0.0
    for trial in range(200):
        q = (np.zeros(N_JOINTS) if trial == 0
             else rng.uniform(-np.pi / 2, np.pi / 2, N_JOINTS))
        q_map = dict(zip(ARM_JOINT_NAMES, q))
        p_urdf = fk_urdf(chain, q_map)[:3, 3]
        p_dh = ROBOT.tool_position(q)
        err = float(np.linalg.norm(p_urdf - p_dh))
        worst = max(worst, err)
        if trial == 0:
            print(f"HOME: URDF={np.round(p_urdf, 5)}  DH={np.round(p_dh, 5)}  "
                  f"err={err:.2e}  (esperado [0.010, 0, 0.393])")

    print(f"error máximo TCP URDF vs FK DH en 200 posturas: {worst:.2e} m")
    if worst < 1e-9:
        print("OK ✅  TF == FK: el URDF de primitivas reproduce la tabla DH.")
        return 0
    print("FALLO ❌  el URDF no coincide con la FK DH.")
    return 1


if __name__ == "__main__":
    sys.exit(main())
