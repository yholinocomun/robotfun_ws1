# robotfun_firmware — ESP32 (micro-ROS)

Bajo nivel del brazo de **5 juntas + gripper = 6 canales**. El ESP32 mueve 6
servos y publica la realimentación de 6 potenciómetros.

## Modos de joint_4 (roll de muñeca)

En `robotfun_esp32_microros.ino`:

```c
#define JOINT4_LOCKED 1   // 1 = bloqueado a 90° (modo A) | 0 = activo (modo B)
```

| | Firmware | ROS 2 (ik_node / bringup) |
|---|---|---|
| **Modo A** (recomendado para empezar) | `JOINT4_LOCKED 1` | `lock_joint_4:=true` |
| **Modo B** (roll activo) | `JOINT4_LOCKED 0` | `lock_joint_4:=false` |

En ambos modos la **interfaz no cambia**: `/joint_command` y `/joint_states`
siempre llevan 6 canales. Bloqueado, el firmware ignora el comando de joint_4 y
mantiene su servo a 90°; el pot sigue publicándose (verdad del hardware).

## Interfaz ROS

| Tópico | Tipo | Sentido | Unidades |
|--------|------|---------|----------|
| `/joint_command` | `std_msgs/Float32MultiArray` | PC → ESP32 | **radianes** `[q1,q2,q3,q4,q5, gripper]` |
| `/joint_states`  | `sensor_msgs/JointState`     | ESP32 → PC | **radianes**, 25 Hz |

> **Radianes en el bus, grados solo en el servo** (REP-103). La conversión
> rad→grados ocurre únicamente al escribir el PWM. La calibración se expresa
> en grados/cuentas ADC (lo intuitivo del hardware).

## Pines (fijos del hardware, 6 canales)

| Canal | Junta | Servo (PWM) | Pot (ADC) | RAW_ZERO |
|------:|-------|-------------|-----------|---------:|
| 0 | joint_1 (yaw)     | GPIO 2  | GPIO 32 | 1194 |
| 1 | joint_2 (hombro)  | GPIO 4  | GPIO 33 | 1165 |
| 2 | joint_3 (codo)    | GPIO 5  | GPIO 34 (solo IN) | 1191 |
| 3 | joint_4 (ROLL)    | GPIO 18 | GPIO 35 (solo IN) | 1415 |
| 4 | joint_5 (pinza)   | GPIO 19 | GPIO 27 (ADC2) | 1423 |
| 5 | gripper           | GPIO 21 | GPIO 26 (ADC2) | 1300 |

LED de estado: GPIO 13. Los pots en ADC2 (27/26) no dan conflicto porque el
transporte micro-ROS es **serial** (no WiFi).

## Movimiento suave (dos capas)

1. **`trajectory_node`** (ROS) genera el perfil **trapezoidal** fino a 50 Hz —
   es la capa principal de suavidad.
2. **Tarea FreeRTOS** en el firmware (50 Hz, núcleo 0) limita la **velocidad
   máxima** de cada servo (`MAX_SPEED_DPS`) como red de seguridad ante comandos
   escalón (p. ej. usando `ik_node` directo sin trayectoria).

## Calibración

1. Coloca el robot en **HOME** (brazo recto vertical, pinza arriba; juntas a 0).
2. Lee el ADC de cada pot y copia los valores en `RAW_ZERO[6]`.
3. Si una junta aparece **invertida en RViz** → cambia su signo en
   `FEEDBACK_DIRECTION[6]`.
4. Si un servo gira **al revés respecto a la cinemática** (compara con el
   modelo de primitivas en RViz) → signo en `SERVO_DIRECTION[6]`.
   *Nota:* `joint_5` arranca con signo invertido (−1) porque su eje DH es +Y,
   opuesto a los pitch de hombro/codo. **Verifícalo en tu hardware.**
5. Ajusta `JOINT_MIN_DEG/JOINT_MAX_DEG` por canal (el gripper usa 0..70°).

## Compilar / subir (Arduino IDE)

- Placa: **ESP32 Dev Module**. Librerías: `micro_ros_arduino`, `ESP32Servo`.
- Sube `firmware/robotfun_esp32_microros.ino` y arranca el Agent:

```bash
ros2 launch robotfun_firmware microros_agent.launch.py dev:=/dev/ttyUSB0
# equivale a: ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0 -b 115200
```
