# robotfun_ws1 — Brazo robótico Pick & Place (ROS 2)

Workspace del brazo **Pick & Place** de bajo costo para pastillas: **5 juntas
(yaw, hombro, codo, ROLL de muñeca, pitch de pinza) + gripper**, servos con
realimentación por potenciómetros y **ESP32 + micro-ROS**.

El espacio de tarea útil es de **4 GDL** (posición + ángulo de aproximación).
Por eso hay dos modos, conmutables por parámetro:

- **Modo A (defecto)** — `lock_joint_4:=true`: roll bloqueado a 0 (servo a
  90°) → 4 GDL efectivos, **IK analítica cerrada**. Para validar y para el
  pick & place inicial.
- **Modo B** — `lock_joint_4:=false`: roll activo → posición + dirección de
  apuntado 3D con **IK DLS**. Para orientar la pinza fuera del plano (visión).

| Paquete | Rol |
|---------|-----|
| `robotfun_description` | URDF/xacro: modelo de **medidas reales** (primitivas, TF==FK) y de **piezas reales** (meshes del CAD). |
| `robotfun_kinematics` | Núcleo **puro sin ROS** (DH, FK, Jacobiano, IK analítica + DLS/Newton/gradiente, workspace, trayectorias) + nodos ROS. |
| `robotfun_firmware` | Firmware ESP32 micro-ROS (6 canales, `JOINT4_LOCKED`, suavizado FreeRTOS) + micro-ROS Agent. |
| `robotfun_bringup` | Composition root + `docs/ARCHITECTURE.md`. |

---

## 1. Compilar el workspace

```bash
cd ~/robotfun_ws1
colcon build --symlink-install
source install/setup.bash
```

## 2. Verificar la cinemática SIN ROS (rápido, en cualquier PC)

```bash
# tests unitarios del núcleo (FK, IK ambos modos, URDF==FK, workspace, trayectorias)
cd ~/robotfun_ws1/src/robotfun_kinematics
python3 -m pytest test/ -v

# FK(HOME) y demostración del roll puro
python3 -m robotfun_kinematics.core.dh_model

# comparación de métodos de IK (analytic/dls/newton/gradient) y demo del modo roll
python3 -m robotfun_kinematics.core.ik_solver

# TF == FK contra el URDF REAL (expande el xacro y lo compara con la FK DH)
cd ~/robotfun_ws1/src/robotfun_description
python3 scripts/verify_kinematics.py
```

## 3. Simulación (sin hardware) — el robot sigue a la IK en RViz

> ⚠️ Usa SIEMPRE `bringup` con un `mode`: garantiza UNA sola fuente de
> `/joint_states` (si hay dos, RViz tiembla/salta). No lances
> `display.launch.py` a la vez.

```bash
# (A) modo bloqueado (defecto): IK analítica, 4 GDL efectivos
ros2 launch robotfun_bringup bringup.launch.py mode:=sim model:=primitives

# en otra terminal: manda un objetivo ALCANZABLE (pinza hacia abajo por defecto)
ros2 topic pub /target_pose geometry_msgs/msg/Pose "{position: {x: 0.16, y: 0.0, z: 0.06}}" --once
ros2 topic echo /fk_pose --once        # debe coincidir con el objetivo

# abrir/cerrar gripper (radianes)
ros2 topic pub /gripper_command std_msgs/msg/Float32 "{data: 0.8}" --once
ros2 topic pub /gripper_command std_msgs/msg/Float32 "{data: 0.0}" --once

# ver las piezas reales del CAD moviéndose con la misma IK
ros2 launch robotfun_bringup bringup.launch.py mode:=sim model:=meshes

# (B) modo roll ACTIVO: posición + dirección de apuntado 3D (DLS)
ros2 launch robotfun_bringup bringup.launch.py mode:=sim lock_joint_4:=false method:=dls
# posición sola (usa φ=approach_deg en el plano):
ros2 topic pub /target_pose geometry_msgs/msg/Pose "{position: {x: 0.16, y: 0.05, z: 0.08}}" --once
# posición + dirección: el eje X del quaternion define hacia dónde apunta la pinza
ros2 topic pub /target_pose geometry_msgs/msg/Pose \
  "{position: {x: 0.16, y: 0.05, z: 0.10}, orientation: {x: 0.0, y: 0.7071, z: 0.0, w: 0.7071}}" --once
```

### Ángulo de aproximación (modo A)

`approach_deg`: **−90 = pinza hacia abajo** (pick top-down, defecto),
0 = horizontal, +90 = arriba. La pinza recta hacia abajo es exigente para un
brazo pequeño; si la IK avisa "fuera del espacio de trabajo", acerca el punto
o usa un ángulo mayor:

```bash
ros2 launch robotfun_bringup bringup.launch.py mode:=sim approach_deg:=0.0
ros2 topic pub /target_pose geometry_msgs/msg/Pose "{position: {x: 0.10, y: 0.0, z: 0.30}}" --once

ros2 launch robotfun_bringup bringup.launch.py mode:=sim approach_deg:=-45.0
ros2 topic pub /target_pose geometry_msgs/msg/Pose "{position: {x: 0.20, y: 0.0, z: 0.15}}" --once

# pinza abajo (−90°): solo puntos bajos y cercanos
ros2 topic pub /target_pose geometry_msgs/msg/Pose "{position: {x: 0.16, y: 0.0, z: 0.05}}" --once
```

### Movimiento suave (perfil trapezoidal)

```bash
ros2 launch robotfun_bringup bringup.launch.py mode:=sim controller:=trajectory
# articular [q1,q2,q3,q4,q5,gripper] en radianes:
ros2 topic pub /joint_goal std_msgs/msg/Float32MultiArray "{data: [0.5, 0.4, 0.6, 0.0, -0.5, 0.0]}" --once
# cartesiano en línea recta:
ros2 topic pub /target_pose geometry_msgs/msg/Pose "{position: {x: 0.16, y: 0.05, z: 0.10}}" --once
```

### Sliders manuales (inspeccionar el modelo, sin cinemática)

```bash
ros2 launch robotfun_bringup bringup.launch.py mode:=gui model:=primitives
ros2 launch robotfun_bringup bringup.launch.py mode:=gui model:=meshes
```

## 4. Con hardware (ESP32)

```bash
# 0) compila y sube el firmware con Arduino IDE (ver src/robotfun_firmware/README.md)
#    ¡Verifica que JOINT4_LOCKED del .ino coincida con lock_joint_4 del launch!

# 1) agente micro-ROS (el ESP32 será la fuente de /joint_states):
ros2 launch robotfun_firmware microros_agent.launch.py dev:=/dev/ttyUSB0

# 2) bringup en modo hardware (sin relay ni sliders):
ros2 launch robotfun_bringup bringup.launch.py mode:=hardware model:=meshes controller:=trajectory

# 3) comprobar la cadena completa:
ros2 topic echo /joint_states          # pots del ESP32, radianes, 25 Hz
ros2 topic echo /fk_pose               # FK de la postura real
ros2 topic pub /target_pose geometry_msgs/msg/Pose "{position: {x: 0.16, y: 0.0, z: 0.06}}" --once
```

### Calibración (resumen; detalle en `src/robotfun_firmware/README.md`)

1. Robot en **HOME** (brazo recto vertical, servos a 90°) → apuntar las
   lecturas ADC en `RAW_ZERO[6]` del `.ino`.
2. Junta invertida en RViz → signo en `FEEDBACK_DIRECTION[6]`.
3. Servo que gira al revés de la cinemática → signo en `SERVO_DIRECTION[6]`.
4. `joint_5` arranca invertido (−1) porque su eje DH es +Y; verifícalo.

## 5. Restringir el ÁREA DE TRABAJO (mesa de pastillas)

La zona segura vive en `core/workspace.py` y la aplica el `ik_node`
(caja x/y/z + alcance radial 0.08–0.34 m desde el hombro). Por ejemplo, una
mesa de 25×30 cm a 3 cm de altura:

```bash
ros2 run robotfun_kinematics ik_node --ros-args \
  -p lock_joint_4:=true -p method:=analytic -p approach_deg:=-90.0 \
  -p enforce_workspace:=true \
  -p ws_x:="[0.10, 0.28]" -p ws_y:="[-0.15, 0.15]" -p ws_z:="[0.03, 0.20]"
```

Con `enforce_workspace:=true` un objetivo fuera de la zona se **recorta** a
ella; con `false` solo avisa.

## 6. Referencia rápida de tópicos

| Tópico | Tipo | Unidades | Quién |
|--------|------|----------|-------|
| `/target_pose` | `geometry_msgs/Pose` | m (+ quaternion opcional en modo B) | tú/visión → IK |
| `/joint_goal` | `std_msgs/Float32MultiArray` | rad `[q1..q5,grip]` | tú → trajectory |
| `/gripper_command` | `std_msgs/Float32` | rad | tú → IK |
| `/joint_command` | `std_msgs/Float32MultiArray` | rad `[q1..q5,grip]` | IK/traj → ESP32 |
| `/joint_states` | `sensor_msgs/JointState` | rad | ESP32/relay → RViz |
| `/fk_pose` | `geometry_msgs/Pose` | m | fk_check (valida TF==FK) |

## Decisiones de diseño (resumen)

- **joint_4 (roll) NO se elimina**: se bloquea por defecto (modo A, IK
  analítica) y se activa por parámetro (modo B, DLS). Interfaz de 6 canales
  idéntica en ambos modos.
- **Tabla DH del usuario validada**: FK(HOME) = [0.010, 0, 0.393]; roll puro;
  pinza por el eje X del frame final; φ = π/2 + q2 + q3 − q5.
- **Radianes en el bus ROS**, grados solo dentro del firmware (REP-103).
- **TF == FK garantizado**: `verify_kinematics.py` parsea el URDF real y lo
  compara con la FK DH (error < 1e-12 m).
- Con 5 GDL **no se promete orientación SO(3) completa**: la tarea es
  posición + dirección de apuntado.

Detalle completo en `src/robotfun_bringup/docs/ARCHITECTURE.md`.
