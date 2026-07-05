# Arquitectura — Brazo RobotFun (robotfun_ws1)

Brazo **Pick & Place de bajo costo** para manipular pastillas/productos
farmacéuticos no frágiles. 5 juntas + gripper, servos MG995R con
realimentación por potenciómetros, **ESP32 + micro-ROS**, integrado en ROS 2.
Objetivo inmediato: **cinemática directa e inversa correctas y coherentes**
(URDF == FK == firmware). Objetivo final: pick & place guiado por cámara
(QR/color), con MoveIt2 y Gazebo como ampliaciones futuras.

## Estructura física

| Junta | Movimiento | Canal | Eje URDF (HOME) |
|-------|-----------|:-----:|------------------|
| joint_1 | yaw de base | 0 | (0, 0, 1) |
| joint_2 | pitch de hombro | 1 | (0, −1, 0) |
| joint_3 | pitch de codo | 2 | (0, −1, 0) |
| joint_4 | **ROLL de muñeca** (colineal con el antebrazo) | 3 | (0, 0, 1) |
| joint_5 | pitch de pinza | 4 | (0, +1, 0) |
| gripper | apertura/cierre | 5 | — |

> El eje de joint_5 es **+Y** (no −Y como hombro/codo): lo impone la tabla DH
> (los dos twists α=+90° de las filas 3 y 4 voltean el eje). El firmware lo
> compensa con `SERVO_DIRECTION`/`FEEDBACK_DIRECTION`.

## Los DOS modos de operación (decisión clave del proyecto)

El espacio de tarea útil de este brazo tiene **4 dimensiones** (posición del
TCP + dirección de aproximación): con 5 GDL **no** se controla una orientación
SO(3) completa (harían falta 6). De ahí los dos modos:

| | **Modo A: joint_4 BLOQUEADO** (defecto) | **Modo B: joint_4 ACTIVO** |
|---|---|---|
| GDL efectivos | 4 (yaw + 3 pitch coplanares) | 5 |
| Tarea | posición (3) + ángulo φ en el plano (1) | posición (3) + dirección de apuntado 3D (2) |
| Solver | **IK analítica cerrada** (exacta, 0 iteraciones, global) | **DLS / Levenberg-Marquardt** (numérica amortiguada) |
| ROS 2 | `lock_joint_4:=true` | `lock_joint_4:=false` |
| Firmware | `#define JOINT4_LOCKED 1` | `#define JOINT4_LOCKED 0` |
| Cuándo | arrancar el proyecto, validar IK, pick & place top-down simple | orientar la pinza fuera del plano, alinearse con pastillas giradas (visión), evitar límites, versión final |

**Por qué así:** para el pick & place inicial el roll no aporta posición (es
colineal con el antebrazo) y sí añade complejidad numérica y riesgo mecánico;
bloqueado, la IK es cerrada y estable. Para la versión final el roll permite
apuntar la pinza en direcciones fuera del plano del brazo (validado en tests:
el modo A deja >15° de error angular en esos casos) y orientar los dedos.
La interfaz de 6 canales **no cambia** entre modos: activar el roll no toca
nombres, tamaños de mensajes ni URDF.

## Principios de diseño

- **Clean Architecture**: núcleo de cinemática Python **puro, sin ROS**
  (`robotfun_kinematics/core/`); los nodos ROS son adaptadores delgados.
  Testeable sin ROS (`pytest`).
- **Una sola fuente de verdad**: la tabla DH vive en `core/dh_model.py`; el
  URDF de primitivas la reproduce (TF == FK, error < 1e-12, verificado por
  `scripts/verify_kinematics.py` que parsea el URDF real); los límites
  articulares coinciden con `config/joint_limits.yaml` y el firmware.
- **Interfaz de juntas única**: `joint_1..joint_5`, `gripper` en descripción,
  cinemática, firmware y RViz. Orden de canales fijo en `/joint_command`.
- **Radianes en el bus ROS** (REP-103); grados solo dentro del firmware al
  escribir el PWM.

## Paquetes

```
robotfun_description/   URDF/xacro: primitivas (DH-exacto, canónico) + meshes (CAD, visual)
robotfun_kinematics/    núcleo puro (FK, Jacobiano, IK analítica + DLS/Newton/gradiente,
                        workspace, trayectorias) + nodos ROS (ik, trajectory, fk_check, relay)
robotfun_firmware/      ESP32 micro-ROS (6 canales, JOINT4_LOCKED, suavizado FreeRTOS) + Agent
robotfun_bringup/       composition root (bringup.launch.py) + esta documentación
```

## Modelo cinemático (tabla DH del usuario, validada numéricamente)

Longitudes (m): `L0=0.010  L1=0.063  L2=0.120  L3=0.090  L4=0.030  L5=0.090`

| i | d_i | θ_i | α_i | a_i |
|---|-----|-----|-----|-----|
| 1 | L1 | q1 | +90° | L0 |
| 2 | 0 | q2+90° | 0° | L2 |
| 3 | 0 | q3+90° | +90° | 0 |
| 4 | L3+L4 | q4 | +90° | 0 |
| 5 | 0 | q5+90° | 0° | L5 |

Propiedades verificadas (tests):
- HOME (q=0, servos a 90°): brazo recto vertical; hombro (0.010, 0, 0.063),
  codo (0.010, 0, 0.183), muñeca (0.010, 0, 0.303), **TCP (0.010, 0, 0.393)**.
- joint_4 es **roll puro**: girarlo en HOME no traslada el TCP.
- La pinza apunta por el **eje X del frame final** (`tool0`), no por Z.
- Identidad del ángulo de aproximación (q4=0): **φ = π/2 + q2 + q3 − q5**.

## Cinemática inversa — métodos

| método | idea | carácter |
|--------|------|----------|
| **analytic** (modo A) | yaw=atan2 + 2R planar (ley de cosenos) + pitch de pinza | exacta, 0 iteraciones, global; da ramas codo arriba/abajo y elige la más cercana a la semilla |
| **dls** (modo B y respaldo) | `dq = Jᵀ(JJᵀ+λ²I)⁻¹e` | robusta cerca de singularidades; λ=0.05, tol=1e-5, ≤200 iter, paso ≤0.4 rad, límites en cada iteración, reintentos desde semilla analítica |
| **newton** | `dq = J⁺e` | rápida, frágil sin amortiguar |
| **gradient** | `dq = αJᵀe` | didáctica, converge lento |

En modo B el error de tarea es `[p_des − p ; k·(a_des − a)]` con `a` = eje X
del tool y `J = [Jp ; −k·[a]×·Jω]` (k=0.05 m). Los métodos numéricos son
locales: se siembran con la postura actual (warm-start) y reintentan desde la
semilla analítica si no convergen.

## Área de trabajo (workspace)

`core/workspace.py` define `WorkspaceLimits` (caja x/y/z + alcance radial
r_min..r_max desde el hombro; r_max físico = L0+L2+L3+L4+L5 = 0.340 m) con
`validate_target()` y `clamp_target()`. El `ik_node` la aplica
(`enforce_workspace`). Configura la mesa/bandeja de pastillas con los
parámetros `ws_x/ws_y/ws_z`.

## Flujo de datos

```
 /target_pose ─► ik_node (IK) ──────┐
 /joint_goal  ─► trajectory_node ───┼─► /joint_command [q1..q5,gripper] ─► ESP32 ─► servos
                                    │        (radianes)                     │
 RViz ◄─ robot_state_publisher ◄────┴────────── /joint_states ◄─────────────┘ (pots, 25 Hz)
              fk_check_node ─► /fk_pose (valida TF == FK en vivo)
```

## Regla de UNA sola fuente de /joint_states

RViz "salta"/tiembla si dos nodos publican `/joint_states` a la vez.
`bringup.launch.py` lo garantiza con `mode`:
- `sim` → solo `joint_state_relay` (ESP32 virtual)
- `gui` → solo `joint_state_publisher_gui` (sliders)
- `hardware` → solo el ESP32 (vía micro-ROS Agent)

Nunca lances `display.launch.py` junto al bringup.

## Qué se rescató de cada workspace de referencia

- **robotfun_ws** (desarrollo previo): núcleo DH/IK/trayectorias y estructura
  de paquetes — adaptados de 4 a 5 juntas; firmware — extendido de 5 a 6
  canales con los pines/ceros reales; meshes y RViz config — copiados.
  **NO** se copió: la eliminación total de joint_4 (contradice el requisito de
  conservar ambos modos) ni la longitud HAND=0.110 (la tabla validada dice
  L5=0.090 → TCP HOME z=0.393, no 0.413).
- **brazo_ws**: orígenes del CAD para el modelo de meshes (brazo.urdf
  corregido, eslabones unidos). El offset x=0.038 de la muñeca del CAD se
  mantiene documentado como pieza descentrada; el modelo canónico de
  primitivas no lo hereda.
- **twin_ws / lab_ws6 / frlabsyholi**: no accesibles en esta sesión; el
  gripper primitivo y el estilo matemático (DH/FK/Jacobiano) siguen el diseño
  ya rescatado en robotfun_ws.

## Roadmap

1. **Calibración en hardware** (ver `robotfun_firmware/README.md`): RAW_ZERO,
   direcciones de servo/pot, límites por canal.
2. Pick & place por secuencias de `/target_pose` + `/gripper_command`
   (aproximación → descenso → cierre → ascenso → traslado → apertura).
3. Nodo de visión (cámara + QR/color) que publique `/target_pose` con
   orientación (el `ik_node` ya acepta el eje X del quaternion como dirección
   de apuntado en modo B).
4. MoveIt2 (los `joint_limits.yaml` ya son consumibles) y Gazebo moderno
   (añadir inercias/colisiones reales al URDF).
