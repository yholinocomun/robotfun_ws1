# robotfun_firmware — ESP32 (micro-ROS) · VERSIÓN DEFINITIVA

Bajo nivel del brazo de **4 GDL (yaw + 3 pitch) + gripper = 5 canales**, con
**movimiento FLUIDO**: el perfil trapezoidal corre en una **tarea FreeRTOS de
tiempo real** (núcleo 0, 50 Hz exactos con `vTaskDelayUntil`), aislada del
jitter de micro-ROS (núcleo 1), y escribe los servos en **microsegundos**
(resolución fina, sin pasos de 1°).

> El antiguo **roll de muñeca fue RETIRADO del hardware**; su canal
> (servo GPIO 18 / pot GPIO 35) se reutilizó para el **pitch de muñeca**
> (joint_4).

> **Único cambio respecto al código original del usuario:** el canal 5 se
> publica en `/joint_states` como `"gripper"` (antes `"joint_5"`) para que
> coincida con la junta del URDF y RViz anime la pinza. `/joint_command` es
> posicional y NO cambia: `[q1, q2, q3, q4, gripper]`.

## Interfaz ROS

| Tópico | Tipo | Sentido | Unidades |
|--------|------|---------|----------|
| `/joint_command` | `std_msgs/Float32MultiArray` | PC → ESP32 | **radianes** `[q1,q2,q3,q4, gripper]` (acepta ≥4) |
| `/joint_states`  | `sensor_msgs/JointState`     | ESP32 → PC | **radianes**, 25 Hz |

> **Radianes en el bus, grados/µs solo dentro del firmware** (REP-103).

## Pines (fijos del hardware, 5 canales)

| Canal | Junta | Servo (PWM) | Pot (ADC) | RAW_ZERO |
|------:|-------|-------------|-----------|---------:|
| 0 | joint_1 (yaw)      | GPIO 2  | GPIO 32 | 1267 |
| 1 | joint_2 (hombro)   | GPIO 4  | GPIO 33 | 1232 |
| 2 | joint_3 (codo)     | GPIO 5  | GPIO 34 (solo IN) | 1265 |
| 3 | joint_4 (muñeca)   | GPIO 18 | GPIO 35 (solo IN) | 1405 |
| 4 | gripper            | GPIO 19 | GPIO 27 (ADC2) | 1302 |

LED de estado: GPIO 13. `SERVO_DIRECTION = {1,-1,1,-1,1}` (confirmado),
`FEEDBACK_DIRECTION = {1,1,-1,-1,1}`. Gripper: 0..70°.
*(Si tu cableado de servos siguiera en {2,4,5,19,21}, ajusta `SERVO_PINS[]`.)*

## Movimiento fluido — cómo funciona

- `command_callback` (micro-ROS, núcleo 1) SOLO fija el objetivo
  (`target_deg[]`, float atómico compartido).
- `servo_task` (núcleo 0, 50 Hz exactos) persigue el objetivo con **perfil
  trapezoidal por junta**: `MAX_VEL` (°/s, crucero) y `MAX_ACC` (°/s²,
  suavidad de arranque/frenado), con frenado calculado para llegar con v=0
  (sin overshoot) y detección de cruce (sin micro-oscilación).
- Ajusta la "personalidad" del movimiento con `MAX_VEL`/`MAX_ACC` (más bajo =
  más lento/suave). El gripper va más ágil (120°/s).

Anti-reinicios: es lazo abierto (llega y se queda). Si el brazo "rebotara a
HOME", es **brownout** de la fuente de servos: usa fuente externa 5-6 V, GND
común con el ESP32 y condensador ≥1000 µF.

## Calibración

1. Robot en **HOME** (brazo recto vertical, servos a 90°, juntas a 0 rad) →
   lee el ADC de cada pot y copia los valores en `RAW_ZERO[5]`.
2. Junta **invertida en RViz** → cambia su signo en `FEEDBACK_DIRECTION[5]`.
3. Servo que gira **al revés de la cinemática** (compara con el modelo de
   primitivas en RViz) → signo en `SERVO_DIRECTION[5]`.
4. Ajusta `JOINT_MIN_DEG/JOINT_MAX_DEG` por canal (gripper 0..70°).

## Compilar / subir (Arduino IDE)

- Placa: **ESP32 Dev Module**. Librerías: `micro_ros_arduino`, `ESP32Servo`.
- Sube `firmware/robotfun_esp32_microros.ino` y arranca el Agent:

```bash
ros2 launch robotfun_firmware microros_agent.launch.py dev:=/dev/ttyUSB0
# equivale a: ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0 -b 115200
```
