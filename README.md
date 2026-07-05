# robotfun_ws1 — Brazo robótico Pick & Place (ROS 2) · 4 GDL + gripper

Workspace **definitivo** del brazo Pick & Place de bajo costo para pastillas:
**4 GDL (yaw + 3 pitch) + gripper**, servos con realimentación por
potenciómetros y **ESP32 + micro-ROS** con **movimiento fluido** (perfil
trapezoidal en tarea FreeRTOS de tiempo real dentro del firmware).

| Paquete | Rol |
|---------|-----|
| `robotfun_description` | URDF/xacro: modelo de **medidas reales** (primitivas, TF==FK) y de **piezas reales** (meshes del CAD). |
| `robotfun_kinematics` | Núcleo **puro sin ROS** (DH, FK, Jacobiano, IK analítica + DLS/Newton/gradiente, workspace, trayectorias) + nodos ROS + **pick_place_node**. |
| `robotfun_firmware` | **Firmware definitivo** ESP32 micro-ROS (5 canales, movimiento fluido) + micro-ROS Agent. |
| `robotfun_bringup` | Composition root + `docs/ARCHITECTURE.md`. |

---

## 0. CLONAR el workspace en tu computadora

```bash
# clonar (primera vez) en tu home:
cd ~
git clone https://github.com/yholinocomun/robotfun_ws1.git
cd ~/robotfun_ws1
git checkout claude/robotfun-arm-kinematics-4dlipe

# si YA lo tenías clonado, solo actualiza:
cd ~/robotfun_ws1
git fetch origin claude/robotfun-arm-kinematics-4dlipe
git checkout claude/robotfun-arm-kinematics-4dlipe
git pull origin claude/robotfun-arm-kinematics-4dlipe
```

## 1. Compilar

```bash
cd ~/robotfun_ws1
colcon build --symlink-install
source install/setup.bash
# (añade "source ~/robotfun_ws1/install/setup.bash" a tu ~/.bashrc si quieres)
```

## 2. VERIFICAR la cinemática SIN ROS (rápido, en cualquier PC)

```bash
# tests unitarios del núcleo (FK, IK, URDF==FK, workspace, trayectorias)
cd ~/robotfun_ws1/src/robotfun_kinematics
python3 -m pytest test/ -v

# FK(HOME) → TCP = [0.010, 0, 0.393]
python3 -m robotfun_kinematics.core.dh_model

# comparación de métodos de IK (analytic/dls/newton/gradient)
python3 -m robotfun_kinematics.core.ik_solver

# TF == FK contra el URDF REAL (expande el xacro y lo compara con la FK DH)
cd ~/robotfun_ws1/src/robotfun_description
python3 scripts/verify_kinematics.py
```

## 3. SIMULACIÓN (sin hardware) — el robot se mueve en RViz

> ⚠️ Usa SIEMPRE `bringup` con un `mode`: garantiza UNA sola fuente de
> `/joint_states` (si hay dos, RViz tiembla). No lances `display.launch.py`
> a la vez.

```bash
# IK analítica + RViz (modelo de primitivas, DH-exacto):
ros2 launch robotfun_bringup bringup.launch.py mode:=sim model:=primitives

# en OTRA terminal: objetivo cartesiano (pinza hacia abajo por defecto)
ros2 topic pub /target_pose geometry_msgs/msg/Pose "{position: {x: 0.16, y: 0.0, z: 0.06}}" --once
ros2 topic echo /fk_pose --once        # debe coincidir con el objetivo

# abrir/cerrar gripper (radianes, 0..1.2)
ros2 topic pub /gripper_command std_msgs/msg/Float32 "{data: 0.9}" --once
ros2 topic pub /gripper_command std_msgs/msg/Float32 "{data: 0.2}" --once

# ver las piezas reales del CAD moviéndose con la misma IK:
ros2 launch robotfun_bringup bringup.launch.py mode:=sim model:=meshes

# sliders manuales (inspeccionar el modelo, sin cinemática):
ros2 launch robotfun_bringup bringup.launch.py mode:=gui model:=meshes
```

### Ángulo de aproximación

`approach_deg`: **−90 = pinza hacia abajo** (defecto), 0 = horizontal,
+90 = arriba. Si la IK avisa "fuera del espacio de trabajo", acerca el punto o
usa un ángulo mayor:

```bash
ros2 launch robotfun_bringup bringup.launch.py mode:=sim approach_deg:=-45.0
ros2 topic pub /target_pose geometry_msgs/msg/Pose "{position: {x: 0.20, y: 0.0, z: 0.15}}" --once
```

## 4. SECUENCIA PICK & PLACE (waypoints articulares probados a mano)

```bash
# en simulación (con el bringup mode:=sim corriendo):
ros2 run robotfun_kinematics pick_place_node

# con hardware (con el Agent + bringup mode:=hardware corriendo):
ros2 run robotfun_kinematics pick_place_node

# opciones: ciclos infinitos y 1.5x más lento:
ros2 run robotfun_kinematics pick_place_node --ros-args -p cycles:=0 -p speed_scale:=1.5
```

Los waypoints y la secuencia se editan arriba de
`src/robotfun_kinematics/robotfun_kinematics/nodes/pick_place_node.py`
(listas `[q1, q2, q3, q4, gripper]` en radianes + tiempos de espera).
El **firmware** hace el movimiento suave hacia cada punto.

## 5. Con HARDWARE (ESP32) — firmware definitivo

```bash
# 0) sube src/robotfun_firmware/firmware/robotfun_esp32_microros.ino con
#    Arduino IDE (placa ESP32 Dev Module; librerías micro_ros_arduino, ESP32Servo).
#    Pines fijos: SERVO {2,4,5,18,19}  POT {32,33,34,35,27}.

# 1) agente micro-ROS (el ESP32 será la fuente de /joint_states):
ros2 launch robotfun_firmware microros_agent.launch.py dev:=/dev/ttyUSB0

# 2) bringup en modo hardware (sin relay ni sliders):
ros2 launch robotfun_bringup bringup.launch.py mode:=hardware model:=meshes

# 3) VERIFICAR la cadena completa:
ros2 topic list                        # deben estar /joint_states y /joint_command
ros2 topic echo /joint_states          # pots del ESP32, radianes, 25 Hz
ros2 topic echo /fk_pose               # FK de la postura real (== TF de tool0)
ros2 topic hz /joint_states            # ~25 Hz

# mover una junta directa (radianes [q1,q2,q3,q4,gripper]) — el ESP32 suaviza:
ros2 topic pub /joint_command std_msgs/msg/Float32MultiArray "{data: [0.5, 0.3, 0.3, -0.5, 0.9]}" --once
# volver a HOME:
ros2 topic pub /joint_command std_msgs/msg/Float32MultiArray "{data: [0.0, 0.0, 0.0, 0.0, 0.9]}" --once

# objetivo cartesiano por IK:
ros2 topic pub /target_pose geometry_msgs/msg/Pose "{position: {x: 0.16, y: 0.0, z: 0.06}}" --once

# secuencia completa:
ros2 run robotfun_kinematics pick_place_node
```

### Calibración (detalle en `src/robotfun_firmware/README.md`)

1. Robot en **HOME** (brazo recto vertical, servos a 90°) → apunta las
   lecturas ADC en `RAW_ZERO[5]` del `.ino`
   (actuales: `{1267, 1232, 1265, 1405, 1302}`).
2. Junta invertida en RViz → signo en `FEEDBACK_DIRECTION[5]` (`{1,1,-1,-1,1}`).
3. Servo al revés de la cinemática → signo en `SERVO_DIRECTION[5]` (`{1,-1,1,-1,1}`).
4. Velocidad/suavidad del movimiento → `MAX_VEL` (°/s) y `MAX_ACC` (°/s²).
5. Si el brazo "rebota a HOME" solo: es **brownout** de la fuente de servos
   (fuente externa 5-6 V, GND común, condensador ≥1000 µF).

## 6. Restringir el ÁREA DE TRABAJO (mesa de pastillas)

```bash
ros2 run robotfun_kinematics ik_node --ros-args \
  -p method:=analytic -p approach_deg:=-90.0 -p enforce_workspace:=true \
  -p ws_x:="[0.10, 0.28]" -p ws_y:="[-0.15, 0.15]" -p ws_z:="[0.03, 0.20]"
```

Con `enforce_workspace:=true` un objetivo fuera de la zona se **recorta**;
con `false` solo avisa. Alcance físico: 0.08–0.34 m desde el hombro.

## 7. Referencia rápida de tópicos

| Tópico | Tipo | Unidades | Quién |
|--------|------|----------|-------|
| `/target_pose` | `geometry_msgs/Pose` | m | tú/visión → IK |
| `/joint_goal` | `std_msgs/Float32MultiArray` | rad `[q1..q4,grip]` | tú → trajectory |
| `/gripper_command` | `std_msgs/Float32` | rad | tú → IK |
| `/joint_command` | `std_msgs/Float32MultiArray` | rad `[q1..q4,grip]` | IK/traj/pick_place → ESP32 |
| `/joint_states` | `sensor_msgs/JointState` | rad | ESP32/relay → RViz |
| `/fk_pose` | `geometry_msgs/Pose` | m | fk_check (valida TF==FK) |

## Decisiones de diseño (resumen)

- **4 GDL definitivo**: el roll de muñeca fue retirado del hardware (su canal
  GPIO 18/35 es ahora el pitch de muñeca). Estructura yaw+3·pitch → **IK
  analítica cerrada**. (El modelo de 5 juntas con roll queda en el historial
  de git por si se reintroduce.)
- **Suavidad en el firmware**: perfil trapezoidal por junta en tarea FreeRTOS
  de tiempo real (núcleo 0, 50 Hz exactos, `writeMicroseconds`) — inmune al
  jitter de micro-ROS.
- **Tabla DH validada**: FK(HOME) = [0.010, 0, 0.393]; pinza por el eje X del
  frame final; φ = π/2 + q2 + q3 + q4.
- **Radianes en el bus ROS** (REP-103); grados/µs solo dentro del firmware.
- **TF == FK garantizado**: `verify_kinematics.py` parsea el URDF real
  (error < 1e-12 m).
- Único cambio al firmware del usuario: el canal 5 se publica como
  `"gripper"` (antes `"joint_5"`) para coincidir con el URDF.

Detalle completo en `src/robotfun_bringup/docs/ARCHITECTURE.md`.
