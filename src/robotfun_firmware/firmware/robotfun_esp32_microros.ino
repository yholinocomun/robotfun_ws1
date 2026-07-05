/* ============================================================================
 *  robotfun_esp32_microros.ino
 *  ---------------------------------------------------------------------------
 *  BAJO NIVEL (micro-ROS) del brazo de 5 juntas + GRIPPER = 6 canales.
 *
 *  Canales (orden FIJO en todo el sistema — URDF, IK y firmware):
 *      idx :    0        1        2        3        4        5
 *      junta:  joint_1  joint_2  joint_3  joint_4  joint_5  gripper
 *                yaw     hombro   codo     ROLL     pinza    apertura
 *
 *  joint_4 es el ROLL de muñeca (eje colineal con el antebrazo). Se puede:
 *      JOINT4_LOCKED 1  → BLOQUEADO: ignora el comando y mantiene el servo en
 *                         su centro (0 rad = 90°). Modo A (4 GDL efectivos,
 *                         coherente con lock_joint_4:=true en ROS).
 *      JOINT4_LOCKED 0  → ACTIVO: obedece /joint_command. Modo B (roll),
 *                         coherente con lock_joint_4:=false en ROS.
 *  En AMBOS casos el canal existe en /joint_states y en /joint_command (el
 *  vector siempre tiene 6 elementos): cambiar de modo NO cambia la interfaz.
 *
 *  Pipeline:
 *    /joint_command (std_msgs/Float32MultiArray, RADIANES,
 *                    [q1, q2, q3, q4, q5, gripper])
 *        --> perfil de suavizado (tarea FreeRTOS, límite de velocidad)
 *        --> 6 servos
 *    6 potenciómetros --> /joint_states (sensor_msgs/JointState, RADIANES) @25 Hz
 *
 *  UNIDADES: el bus ROS va SIEMPRE en RADIANES (REP-103). La conversión a
 *  GRADOS de servo ocurre solo al escribir el PWM. La calibración se expresa
 *  en grados/cuentas ADC (lo intuitivo del hardware).
 *
 *  HOME: todas las juntas a 0 rad => servos a 90 grados (brazo recto vertical).
 *
 *  PINES (fijos del hardware):
 *      idx :    0    1    2    3    4    5
 *      SERVO:   2    4    5   18   19   21
 *      POT  :  32   33   34   35   27   26
 *      (34 y 35 son solo-entrada ADC1: perfectos para pots. 27/26 son ADC2:
 *       sin conflicto porque el transporte micro-ROS es SERIAL, no WiFi.)
 *      LED de estado: GPIO 13.
 *
 *  MOVIMIENTO SUAVE (dos capas):
 *    1. trajectory_node (ROS) genera el perfil trapezoidal fino a 50 Hz.
 *    2. Esta tarea FreeRTOS limita la velocidad de cada servo (MAX_SPEED_DPS)
 *       como red de seguridad ante comandos escalón (p. ej. ik_node directo).
 * ==========================================================================*/

#include <micro_ros_arduino.h>
#include <ESP32Servo.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>

#include <sensor_msgs/msg/joint_state.h>
#include <std_msgs/msg/float32_multi_array.h>

// ===========================================================================
//  CONFIGURACIÓN DE MODO
// ===========================================================================
// 1 = joint_4 (roll) bloqueado a 90° (modo A, 4 GDL efectivos)  |  0 = activo
#define JOINT4_LOCKED  1

#define STATUS_LED_PIN 13
#define NUM_CH         6          // 5 juntas de brazo + 1 gripper
#define JOINT4_INDEX   3
#define GRIPPER_INDEX  5

const int SERVO_PINS[NUM_CH] = {  2,  4,  5, 18, 19, 21 };
const int POT_PINS[NUM_CH]   = { 32, 33, 34, 35, 27, 26 };

static const char JOINT_LABEL[NUM_CH][12] =
    { "joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "gripper" };

// ===========================================================================
//  CALIBRACIÓN (grados / cuentas ADC). Ajustar en pruebas (ver README).
// ===========================================================================
// RAW_ZERO[i]: lectura ADC (0..4095) de cada pot en HOME (juntas a 0 rad).
int RAW_ZERO[NUM_CH] = {
  1194,   // joint_1  (pot 32)
  1165,   // joint_2  (pot 33)
  1191,   // joint_3  (pot 34)
  1415,   // joint_4  (pot 35, roll)
  1423,   // joint_5  (pot 27, pitch de pinza)
  1300    // gripper  (pot 26)
};

// FEEDBACK_DIRECTION[i]: sentido del pot hacia la convención DH (-1 invierte).
// joint_5 va en -1 porque su eje DH es +Y (opuesto a los pitch 2 y 3). CALIBRAR.
const float FEEDBACK_DIRECTION[NUM_CH] = { 1, 1, 1, 1, -1, 1 };

// SERVO_DIRECTION[i]: sentido del servo respecto al comando (+q en el sentido
// positivo de la convención DH). Cambia el signo si gira al revés. CALIBRAR.
const float SERVO_DIRECTION[NUM_CH] = { -1, -1, 1, 1, -1, -1 };

// Rango articular por canal (grados). Brazo ±90; gripper 0..70 (cierra/abre).
const float JOINT_MIN_DEG[NUM_CH] = { -90, -90, -90, -90, -90,  0 };
const float JOINT_MAX_DEG[NUM_CH] = {  90,  90,  90,  90,  90, 70 };
const float SERVO_CENTER_DEG = 90.0f;

// Suavizado: velocidad máxima de cada servo (grados/s) y tasa de la tarea.
const float MAX_SPEED_DPS[NUM_CH] = { 120, 120, 120, 150, 150, 240 };
const int   SMOOTH_RATE_HZ = 50;

const float ADC_TO_DEG = 180.0f / 4095.0f;     // pot de 180° sobre 0..4095
const float DEG2RAD = 0.017453292519943295f;
const float RAD2DEG = 57.29577951308232f;
const float ALPHA = 0.15f;                      // filtro exponencial del ADC

// ===========================================================================
Servo servos[NUM_CH];
float feedback_filtered[NUM_CH];
bool  filter_initialized = false;

// Objetivo (grados de servo 0..180) escrito por el callback; la tarea de
// suavizado lo persigue con velocidad limitada. volatile: compartido entre
// el executor de micro-ROS (núcleo 1) y la tarea de suavizado (núcleo 0).
volatile float servo_target_deg[NUM_CH];
float servo_current_deg[NUM_CH];

rcl_node_t node;
rclc_support_t support;
rcl_allocator_t allocator;
rclc_executor_t executor;
rcl_publisher_t joint_state_pub;
rcl_subscription_t joint_command_sub;
rcl_timer_t timer;

sensor_msgs__msg__JointState joint_state_msg;
std_msgs__msg__Float32MultiArray command_msg;

static rosidl_runtime_c__String joint_names[NUM_CH];
static double position_data[NUM_CH];
static double velocity_data[NUM_CH];
static double effort_data[NUM_CH];
static float  command_data[NUM_CH];

#define RCCHECK(fn) { rcl_ret_t rc = fn; if (rc != RCL_RET_OK) { error_loop(); } }
#define RCSOFTCHECK(fn) { rcl_ret_t rc = fn; (void)rc; }

void error_loop() {
  while (1) { digitalWrite(STATUS_LED_PIN, !digitalRead(STATUS_LED_PIN)); delay(100); }
}

float clampf(float x, float lo, float hi) { return x < lo ? lo : (x > hi ? hi : x); }

// ADC -> radianes. 0 rad corresponde a RAW_ZERO[i] (HOME).
float adc_to_rad(int raw, int i) {
  float deg = FEEDBACK_DIRECTION[i] * (float)(raw - RAW_ZERO[i]) * ADC_TO_DEG;
  deg = clampf(deg, JOINT_MIN_DEG[i], JOINT_MAX_DEG[i]);
  return deg * DEG2RAD;
}

// radianes -> grados de servo (0..180). 0 rad => 90° (centro = HOME).
float rad_to_servo_deg(float q_rad, int i) {
  float q_deg = clampf(q_rad * RAD2DEG, JOINT_MIN_DEG[i], JOINT_MAX_DEG[i]);
  return clampf(SERVO_CENTER_DEG + SERVO_DIRECTION[i] * q_deg, 0.0f, 180.0f);
}

// Fija los objetivos de servo (la tarea de suavizado hace el movimiento).
void set_servo_targets(const float *q_cmd_rad) {
  for (int i = 0; i < NUM_CH; i++) {
#if (JOINT4_LOCKED)
    if (i == JOINT4_INDEX) { servo_target_deg[i] = SERVO_CENTER_DEG; continue; }
#endif
    servo_target_deg[i] = rad_to_servo_deg(q_cmd_rad[i], i);
  }
}

// Callback de /joint_command (RADIANES [q1..q5, gripper]; acepta >=5).
void command_callback(const void *msgin) {
  const std_msgs__msg__Float32MultiArray *msg =
      (const std_msgs__msg__Float32MultiArray *)msgin;
  if (msg->data.size < NUM_CH - 1) return;   // mínimo las 5 juntas del brazo
  for (int i = 0; i < NUM_CH; i++) {
    if ((size_t)i < msg->data.size) {
      float lo = JOINT_MIN_DEG[i] * DEG2RAD, hi = JOINT_MAX_DEG[i] * DEG2RAD;
      command_data[i] = clampf(msg->data.data[i], lo, hi);
    }   // si no llega el gripper (size==5) conserva su último valor
  }
  set_servo_targets(command_data);
}

// Tarea FreeRTOS de SUAVIZADO: persigue el objetivo con velocidad limitada.
void smoothing_task(void *arg) {
  (void)arg;
  const TickType_t period = pdMS_TO_TICKS(1000 / SMOOTH_RATE_HZ);
  TickType_t last_wake = xTaskGetTickCount();
  for (;;) {
    for (int i = 0; i < NUM_CH; i++) {
      float step = MAX_SPEED_DPS[i] / (float)SMOOTH_RATE_HZ;
      float diff = servo_target_deg[i] - servo_current_deg[i];
      if (diff > step)       servo_current_deg[i] += step;
      else if (diff < -step) servo_current_deg[i] -= step;
      else                   servo_current_deg[i] = servo_target_deg[i];
      servos[i].write((int)(servo_current_deg[i] + 0.5f));
    }
    vTaskDelayUntil(&last_wake, period);
  }
}

void read_feedback() {
  float measured[NUM_CH];
  for (int i = 0; i < NUM_CH; i++) measured[i] = adc_to_rad(analogRead(POT_PINS[i]), i);
  if (!filter_initialized) {
    for (int i = 0; i < NUM_CH; i++) feedback_filtered[i] = measured[i];
    filter_initialized = true;
  } else {
    for (int i = 0; i < NUM_CH; i++)
      feedback_filtered[i] = ALPHA * measured[i] + (1.0f - ALPHA) * feedback_filtered[i];
  }
  for (int i = 0; i < NUM_CH; i++) position_data[i] = feedback_filtered[i];
}

void timer_callback(rcl_timer_t *t, int64_t last) {
  (void)last;
  if (t == NULL) return;
  read_feedback();
  int64_t ns = rmw_uros_epoch_nanos();
  joint_state_msg.header.stamp.sec     = (int32_t)(ns / 1000000000LL);
  joint_state_msg.header.stamp.nanosec = (uint32_t)(ns % 1000000000LL);
  RCSOFTCHECK(rcl_publish(&joint_state_pub, &joint_state_msg, NULL));
}

void setup_joint_state_msg() {
  for (int i = 0; i < NUM_CH; i++) {
    joint_names[i].data = (char *)JOINT_LABEL[i];
    joint_names[i].size = strlen(JOINT_LABEL[i]);
    joint_names[i].capacity = strlen(JOINT_LABEL[i]) + 1;
  }
  joint_state_msg.name.data = joint_names;
  joint_state_msg.name.size = NUM_CH; joint_state_msg.name.capacity = NUM_CH;
  joint_state_msg.position.data = position_data;
  joint_state_msg.position.size = NUM_CH; joint_state_msg.position.capacity = NUM_CH;
  joint_state_msg.velocity.data = velocity_data;
  joint_state_msg.velocity.size = NUM_CH; joint_state_msg.velocity.capacity = NUM_CH;
  joint_state_msg.effort.data = effort_data;
  joint_state_msg.effort.size = NUM_CH; joint_state_msg.effort.capacity = NUM_CH;
  joint_state_msg.header.frame_id.data = (char *)"";
  joint_state_msg.header.frame_id.size = 0; joint_state_msg.header.frame_id.capacity = 1;
}

void setup_command_msg() {
  command_msg.data.data = command_data;
  command_msg.data.size = NUM_CH; command_msg.data.capacity = NUM_CH;
}

void setup() {
  pinMode(STATUS_LED_PIN, OUTPUT);
  digitalWrite(STATUS_LED_PIN, HIGH);

  for (int i = 0; i < NUM_CH; i++) {
    position_data[i] = velocity_data[i] = effort_data[i] = 0.0;
    command_data[i] = 0.0f; feedback_filtered[i] = 0.0f;
    servo_target_deg[i] = servo_current_deg[i] = SERVO_CENTER_DEG;
  }
  servo_target_deg[GRIPPER_INDEX] = servo_current_deg[GRIPPER_INDEX] =
      rad_to_servo_deg(0.0f, GRIPPER_INDEX);

  Serial.begin(115200);
  set_microros_transports();
  while (rmw_uros_ping_agent(1000, 1) != RMW_RET_OK) {
    digitalWrite(STATUS_LED_PIN, !digitalRead(STATUS_LED_PIN)); delay(500);
  }
  digitalWrite(STATUS_LED_PIN, HIGH);

  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);

  ESP32PWM::allocateTimer(0); ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2); ESP32PWM::allocateTimer(3);
  for (int i = 0; i < NUM_CH; i++) {
    servos[i].setPeriodHertz(50);
    servos[i].attach(SERVO_PINS[i], 500, 2400);
    servos[i].write((int)(servo_current_deg[i] + 0.5f));   // HOME
  }
  delay(1000);

  // Tarea de suavizado (límite de velocidad) en el núcleo 0; micro-ROS gira
  // en loop() (núcleo 1). Solo comparten servo_target_deg (volatile).
  xTaskCreatePinnedToCore(smoothing_task, "smooth", 2048, NULL, 2, NULL, 0);

  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "robotfun_esp32_node", "", &support));
  RCCHECK(rclc_publisher_init_default(
      &joint_state_pub, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState), "/joint_states"));
  RCCHECK(rclc_subscription_init_default(
      &joint_command_sub, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray), "/joint_command"));

  setup_joint_state_msg();
  setup_command_msg();
  rmw_uros_sync_session(1000);

  RCCHECK(rclc_timer_init_default(&timer, &support, RCL_MS_TO_NS(40), timer_callback)); // 25 Hz
  RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));
  RCCHECK(rclc_executor_add_subscription(
      &executor, &joint_command_sub, &command_msg, &command_callback, ON_NEW_DATA));

  digitalWrite(STATUS_LED_PIN, LOW);
}

void loop() {
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(20)));
  delay(1);
}
