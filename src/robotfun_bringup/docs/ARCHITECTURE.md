# Arquitectura — Brazo RobotFun 4 GDL + gripper (robotfun_ws1)

Brazo **Pick & Place de bajo costo** para manipular pastillas/productos
farmacéuticos no frágiles. **4 GDL (yaw + 3 pitch) + gripper**, servos MG995R
con realimentación por potenciómetros, **ESP32 + micro-ROS**, integrado en
ROS 2. Objetivo inmediato: cinemática directa/inversa correcta y coherente
(URDF == FK == firmware) + secuencia de pick & place. Objetivo final: pick &
place guiado por cámara (QR/color), con MoveIt2 y Gazebo como ampliaciones.

## Estructura física (DEFINITIVA, coincide con el firmware)

| Junta | Movimiento | Canal | Servo | Pot | Eje URDF |
|-------|-----------|:-----:|:-----:|:---:|----------|
| joint_1 | yaw de base | 0 | GPIO 2 | 32 | (0, 0, 1) |
| joint_2 | pitch de hombro | 1 | GPIO 4 | 33 | (0, −1, 0) |
| joint_3 | pitch de codo | 2 | GPIO 5 | 34 | (0, −1, 0) |
| joint_4 | pitch de muñeca | 3 | GPIO 18 | 35 | (0, −1, 0) |
| gripper | apertura/cierre | 4 | GPIO 19 | 27 | — |

> **Historia del roll:** el antiguo joint_4 de roll fue **retirado del
> hardware** y su canal (GPIO 18/35) se reutilizó para el pitch de muñeca.
> Con yaw + 3 pitch coplanares la IK es **analítica cerrada**. Si algún día
> se reintroduce el roll, el modelo de 5 juntas con modos bloqueado/activo
> está en el historial de git de esta rama.

## Principios de diseño

- **Clean Architecture**: núcleo de cinemática Python **puro, sin ROS**
  (`robotfun_kinematics/core/`); los nodos ROS son adaptadores delgados.
  Testeable sin ROS (`pytest`, 14 tests).
- **Una sola fuente de verdad**: la tabla DH vive en `core/dh_model.py`; el
  URDF de primitivas la reproduce (**TF == FK**, error < 1e-12, verificado por
  `scripts/verify_kinematics.py` que parsea el URDF real); límites coherentes
  entre `joint_limits.yaml`, el núcleo y el firmware.
- **Interfaz de juntas única**: `joint_1..joint_4`, `gripper` en descripción,
  cinemática, firmware y RViz. `/joint_command` posicional
  `[q1, q2, q3, q4, gripper]` en **radianes** (REP-103); grados/µs solo dentro
  del firmware.
- **Suavidad en el firmware**: el ESP32 ejecuta el perfil trapezoidal en una
  tarea FreeRTOS de tiempo real (ver `robotfun_firmware/README.md`). Los nodos
  ROS pueden mandar objetivos "en escalón" y el brazo se mueve fluido.

## Paquetes

```
robotfun_description/   URDF/xacro: primitivas (DH-exacto, canónico) + meshes (CAD, visual)
robotfun_kinematics/    núcleo puro (FK, Jacobiano, IK analítica + DLS/Newton/gradiente,
                        workspace, trayectorias) + nodos ROS (ik, trajectory, fk_check,
                        relay, pick_place)
robotfun_firmware/      ESP32 micro-ROS definitivo (5 canales, tarea de servo fluida) + Agent
robotfun_bringup/       composition root (bringup.launch.py) + esta documentación
```

## Modelo cinemático (tabla DH del usuario, validada numéricamente)

Longitudes (m): `L0=0.010  L1=0.063  L2=0.120  A3=L3+L4=0.120  HAND=L5=0.090`

| i | d_i | θ_i | α_i | a_i |
|---|-----|-----|-----|-----|
| 1 | L1 | q1 | +90° | L0 |
| 2 | 0 | q2+90° | 0° | L2 |
| 3 | 0 | q3 | 0° | A3 |
| 4 | 0 | q4 | 0° | HAND |

Propiedades verificadas (tests):
- HOME (q=0, servos a 90°): brazo recto vertical; hombro (0.010, 0, 0.063),
  codo (0.010, 0, 0.183), muñeca (0.010, 0, 0.303), **TCP (0.010, 0, 0.393)**.
- La pinza apunta por el **eje X del frame final** (`tool0`).
- Ángulo de aproximación: **φ = π/2 + q2 + q3 + q4**.
- Con 4 GDL la tarea es posición (3) + φ (1); **no** se promete orientación
  SO(3) completa.

## Cinemática inversa — métodos

| método | idea | carácter |
|--------|------|----------|
| **analytic** (recom.) | yaw=atan2 + 2R planar (ley de cosenos) + pitch de muñeca | exacta, 0 iteraciones, global; ramas codo arriba/abajo, elige la más cercana a la semilla |
| **dls** | `dq = Jᵀ(JJᵀ+λ²I)⁻¹e` | robusta cerca de singularidades; λ=0.05, tol=1e-5, ≤200 iter, paso ≤0.4 rad, límites por iteración, reintentos desde semilla analítica |
| **newton** | `dq = J⁺e` | rápida, frágil sin amortiguar |
| **gradient** | `dq = αJᵀe` | didáctica, converge lento |

## Área de trabajo (workspace)

`core/workspace.py`: `WorkspaceLimits` (caja x/y/z + alcance radial
r_min..r_max desde el hombro; r_max físico = L0+L2+A3+HAND = 0.340 m) con
`validate_target()` y `clamp_target()`. El `ik_node` la aplica
(`enforce_workspace`, `ws_x/ws_y/ws_z`).

## Flujo de datos

```
 /target_pose ─► ik_node (IK) ────────┐
 /joint_goal  ─► trajectory_node ─────┼─► /joint_command [q1..q4,gripper] ─► ESP32 ─► servos
 pick_place_node (secuencia) ─────────┘        (radianes)      (suaviza)      │
                                                                              │
 RViz ◄─ robot_state_publisher ◄──────────── /joint_states ◄──────────────────┘ (pots, 25 Hz)
              fk_check_node ─► /fk_pose (valida TF == FK en vivo)
```

`pick_place_node` publica directamente waypoints articulares probados a mano
(sin IK); el firmware hace el movimiento suave. No lo ejecutes a la vez que
mandas objetivos por `/target_pose` (ambos escriben en `/joint_command`).

## Regla de UNA sola fuente de /joint_states

RViz "salta"/tiembla si dos nodos publican `/joint_states` a la vez.
`bringup.launch.py` lo garantiza con `mode`:
- `sim` → solo `joint_state_relay` (ESP32 virtual)
- `gui` → solo `joint_state_publisher_gui` (sliders)
- `hardware` → solo el ESP32 (vía micro-ROS Agent)

Nunca lances `display.launch.py` junto al bringup.

## Roadmap

1. Verificar en hardware la secuencia `pick_place_node` (ajustar waypoints y
   tiempos de espera).
2. Pick & place por IK: secuencias de `/target_pose` + `/gripper_command`
   (aproximación → descenso → cierre → ascenso → traslado → apertura).
3. Nodo de visión (cámara + QR/color) que publique `/target_pose`.
4. MoveIt2 (los `joint_limits.yaml` ya son consumibles) y Gazebo moderno
   (añadir inercias/colisiones reales al URDF).
