/* ============================================================================
 *  robotfun_esp32_microros.ino   —   VERSION DEFINITIVA (movimiento FLUIDO)
 *  ---------------------------------------------------------------------------
 *  BAJO NIVEL (micro-ROS) del brazo de 4 GDL (yaw + 3 pitch) + GRIPPER.
 *
 *  >>> UNICO CAMBIO respecto al codigo del usuario (integracion al workspace):
 *  >>> el canal 5 se publica en /joint_states con el nombre "gripper" (antes
 *  >>> "joint_5") para que coincida con la junta del URDF y RViz anime la
 *  >>> pinza. El orden/tamano de los mensajes NO cambia: /joint_command sigue
 *  >>> siendo [q1, q2, q3, q4, gripper] posicional.
 *
 *  POR QUE ANTES SE VEIA "A PASITOS":
 *    El suavizado corria dentro de loop(), junto a micro-ROS. spin_some() bloquea
 *    de forma IRREGULAR al hablar con el agente => los pasos del servo salian
 *    desiguales => se veia entrecortado.
 *
 *  SOLUCION (reestructuracion):
 *    El movimiento se ejecuta en una TAREA DE TIEMPO REAL dedicada (FreeRTOS) en
 *    el nucleo 0, con temporizacion EXACTA (vTaskDelayUntil @50 Hz). Inmune al
 *    jitter de micro-ROS => movimiento FLUIDO. Ademas:
 *      - Perfil TRAPEZOIDAL (arranca y frena gradual, sin tirones).
 *      - Resolucion FINA en microsegundos (float), no en pasos de 1 grado.
 *    El nucleo 1 (loop) solo hace micro-ROS: recibe /joint_command y publica el
 *    feedback de pots en /joint_states. Ambos nucleos comparten solo el objetivo.
 *
 *  Anti-bucle: es lazo abierto (llega y se queda), loop() cede CPU (delay(1)) y no
 *  hay memoria RTC. Si aun rebotara a HOME, es RESET por hardware (brownout de la
 *  fuente de servos): usa fuente externa 5-6V, GND comun y condensador 1000uF.
 *
 *  Juntas: 4 de brazo (joint_1..joint_4; el antiguo roll fue RETIRADO del
 *  hardware y su canal se reutilizo para el pitch de muñeca) + gripper.
 *
 *  Pipeline:
 *    /joint_command (Float32MultiArray, RADIANES [q1,q2,q3,q4, gripper]) -> objetivo
 *    tarea servo @50 Hz (perfil trapezoidal, writeMicroseconds) -> 5 servos FLUIDO
 *    5 potenciometros -> /joint_states (JointState, RADIANES) @25 Hz (para RViz)
 *
 *  PINES: SERVO {2,4,5,18,19}  POT {32,33,34,35,27}.  LED de estado: GPIO 13.
 *  (Si tu cableado de servos sigue en {2,4,5,19,21}, ajusta SERVO_PINS[].)
 * ==========================================================================*/

#include <math.h>

#include <micro_ros_arduino.h>
#include <ESP32Servo.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>

#include <sensor_msgs/msg/joint_state.h>
#include <std_msgs/msg/float32_multi_array.h>

#define STATUS_LED_PIN 13
#define NUM_CH         5          // joint_1..joint_4 (brazo) + gripper
#define GRIPPER_INDEX  4

const int SERVO_PINS[NUM_CH] = {  2,  4,  5, 18, 19 };
const int POT_PINS[NUM_CH]   = { 32, 33, 34, 35, 27 };

// Nombres publicados en /joint_states (DEBEN coincidir con el URDF).
const char *JOINT_LABEL[NUM_CH] = { "joint_1", "joint_2", "joint_3", "joint_4", "gripper" };

// ===========================================================================
//  MAPEO DE SERVO (grados). servo_deg = CENTER + DIR*q_deg, saturado a [0,180].
//  SERVO_DIRECTION confirmado {1,-1,1,-1,1}. Si el gripper gira al reves, pon [4]=-1.
// ===========================================================================
const int   SERVO_DIRECTION[NUM_CH]  = {  1, -1,  1, -1,  1 };
const float SERVO_CENTER_DEG[NUM_CH] = { 90, 90, 90, 90, 90 };
const float JOINT_MIN_DEG[NUM_CH]    = { -90, -90, -90, -90,  0 };
const float JOINT_MAX_DEG[NUM_CH]    = {  90,  90,  90,  90, 70 };
const int   SERVO_US_MIN = 500;      // us a 0 grados   (coincide con attach)
const int   SERVO_US_MAX = 2400;     // us a 180 grados

// ===========================================================================
//  FLUIDEZ: perfil trapezoidal por junta (grados/s, grados/s^2).
//  MAX_VEL = velocidad de crucero (mas bajo = mas lento). MAX_ACC = suavidad del
//  arranque/frenado (mas bajo = mas gradual). El gripper va un poco mas agil.
// ===========================================================================
// idx:                            j1     j2     j3     j4    gripper
const float MAX_VEL[NUM_CH] = {  70.0f, 70.0f, 70.0f, 70.0f, 120.0f };  // grados/s
const float MAX_ACC[NUM_CH] = { 150.0f,150.0f,150.0f,150.0f, 300.0f };  // grados/s^2
#define SERVO_TASK_HZ 50                        // 50 Hz = trama del servo
const float SERVO_DT = 1.0f / (float)SERVO_TASK_HZ;

// ===========================================================================
//  FEEDBACK de potenciometros (para /joint_states). Calibracion del usuario.
// ===========================================================================
int RAW_ZERO[NUM_CH] = {
  1267,   // joint_1
  1232,   // joint_2
  1265,   // joint_3
  1405,   // joint_4  (pitch de muñeca)
  1302    // gripper
};
const float FEEDBACK_DIRECTION[NUM_CH] = { 1, 1, -1, -1, 1 };
const float ADC_TO_DEG = 180.0f / 4095.0f;
const float DEG2RAD = 0.017453292519943295f;
const float RAD2DEG = 57.29577951308232f;
const float ALPHA = 0.15f;                      // filtro exponencial del ADC

// ===========================================================================
Servo servos[NUM_CH];

// Objetivo COMPARTIDO entre nucleos (grados de junta). Escrito por micro-ROS
// (command_callback, nucleo 1) y leido por la tarea de servo (nucleo 0).
// float de 32 bits => escritura/lectura atomica en el ESP32.
volatile float target_deg[NUM_CH];

// Estado del perfil (solo lo usa la tarea de servo).
float cur_deg[NUM_CH];
float cur_vel[NUM_CH];

// Feedback
float feedback_filtered[NUM_CH];
bool  filter_initialized = false;

rcl_node_t node;
rclc_support_t support;
rcl_allocator_t allocator;
rclc_executor_t executor;
rcl_publisher_t joint_state_pub;
rcl_subscription_t joint_command_sub;
rcl_timer_t timer;

sensor_msgs__msg__JointState joint_state_msg;
std_msgs__msg__Float32MultiArray command_msg;

static char joint_name_buf[NUM_CH][12] = { "joint_1", "joint_2", "joint_3", "joint_4", "gripper" };
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

// grados de junta q -> microsegundos PWM (float -> resolucion fina => fluido).
int deg_to_us(float q_deg, int i) {
  q_deg = clampf(q_deg, JOINT_MIN_DEG[i], JOINT_MAX_DEG[i]);
  float servo_deg = SERVO_CENTER_DEG[i] + SERVO_DIRECTION[i] * q_deg;
  servo_deg = clampf(servo_deg, 0.0f, 180.0f);
  return (int)(SERVO_US_MIN + servo_deg * (float)(SERVO_US_MAX - SERVO_US_MIN) / 180.0f);
}

// ADC -> radianes (feedback). 0 rad corresponde a RAW_ZERO[i] (HOME).
float adc_to_rad(int raw, int i) {
  float deg = FEEDBACK_DIRECTION[i] * (float)(raw - RAW_ZERO[i]) * ADC_TO_DEG;
  deg = clampf(deg, JOINT_MIN_DEG[i], JOINT_MAX_DEG[i]);
  return deg * DEG2RAD;
}

// ---------------------------------------------------------------------------
//  TAREA DE SERVO (nucleo 0): perfil trapezoidal a 50 Hz EXACTOS => FLUIDO.
// ---------------------------------------------------------------------------
void servo_task(void *pv) {
  (void)pv;
  const TickType_t period = pdMS_TO_TICKS(1000 / SERVO_TASK_HZ);   // 20 ms
  TickType_t last_wake = xTaskGetTickCount();
  for (;;) {
    for (int i = 0; i < NUM_CH; i++) {
      float tgt = clampf(target_deg[i], JOINT_MIN_DEG[i], JOINT_MAX_DEG[i]);
      float err = tgt - cur_deg[i];
      if (fabsf(err) < 1e-4f) { cur_vel[i] = 0.0f; continue; }   // ya en el objetivo
      // Velocidad con la que aun puedo frenar a 0 justo en el objetivo:
      float v_stop = sqrtf(2.0f * MAX_ACC[i] * fabsf(err));
      float v_des  = (err >= 0.0f ? 1.0f : -1.0f) * fminf(MAX_VEL[i], v_stop);
      // Rampa de aceleracion (arranque/frenado gradual => fluido):
      float dv = v_des - cur_vel[i];
      float dvm = MAX_ACC[i] * SERVO_DT;
      if (dv >  dvm) dv =  dvm;
      if (dv < -dvm) dv = -dvm;
      cur_vel[i] += dv;
      float next = cur_deg[i] + cur_vel[i] * SERVO_DT;
      // Deteccion de cruce: si este paso ALCANZA o PASA el objetivo, fija exacto y
      // para (elimina overshoot y micro-oscilacion residual).
      if ((tgt - cur_deg[i]) * (tgt - next) <= 0.0f) {
        cur_deg[i] = tgt; cur_vel[i] = 0.0f;
      } else {
        cur_deg[i] = next;
      }
      servos[i].writeMicroseconds(deg_to_us(cur_deg[i], i));
    }
    vTaskDelayUntil(&last_wake, period);        // temporizacion EXACTA (sin jitter)
  }
}

// Callback de /joint_command (RADIANES [q1..q4, gripper]; acepta >=4).
// Solo fija el OBJETIVO (grados); la tarea de servo lo alcanza fluido.
void command_callback(const void *msgin) {
  const std_msgs__msg__Float32MultiArray *msg =
      (const std_msgs__msg__Float32MultiArray *)msgin;
  if (msg->data.size < 4) return;
  for (int i = 0; i < NUM_CH; i++) {
    if ((size_t)i < msg->data.size) {
      float q_deg = msg->data.data[i] * RAD2DEG;
      target_deg[i] = clampf(q_deg, JOINT_MIN_DEG[i], JOINT_MAX_DEG[i]);
    }   // si no llega el gripper (size==4) conserva su objetivo anterior
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
    joint_names[i].data = joint_name_buf[i];
    joint_names[i].size = strlen(joint_name_buf[i]);
    joint_names[i].capacity = strlen(joint_name_buf[i]) + 1;
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
    target_deg[i] = cur_deg[i] = cur_vel[i] = 0.0f;   // arranca en HOME, quieto
  }

  // Servos + posicion HOME
  ESP32PWM::allocateTimer(0); ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2); ESP32PWM::allocateTimer(3);
  for (int i = 0; i < NUM_CH; i++) {
    servos[i].setPeriodHertz(50);
    servos[i].attach(SERVO_PINS[i], SERVO_US_MIN, SERVO_US_MAX);
    servos[i].writeMicroseconds(deg_to_us(0.0f, i));
  }
  delay(800);

  // Lanza la TAREA DE SERVO en el nucleo 0 (aislada del micro-ROS del nucleo 1).
  xTaskCreatePinnedToCore(servo_task, "servo_task", 4096, NULL, 2, NULL, 0);

  // micro-ROS (nucleo 1, en loop)
  Serial.begin(115200);
  set_microros_transports();
  while (rmw_uros_ping_agent(1000, 1) != RMW_RET_OK) {
    digitalWrite(STATUS_LED_PIN, !digitalRead(STATUS_LED_PIN)); delay(500);
  }
  digitalWrite(STATUS_LED_PIN, HIGH);

  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);

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
  // Nucleo 1: SOLO micro-ROS (comandos + feedback). El movimiento va en su tarea.
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10)));
  delay(1);   // cede CPU (evita reset por watchdog). NO quitar.
}
