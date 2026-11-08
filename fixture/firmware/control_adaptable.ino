/*
 * ============================================================
 *  CONTROL ADAPTABLE DE POSICIÓN - MOTOR NUEVO (Codigos_Nuevo_Motor/03)
 *  Motor DC 12 V | ESP32 DevKit V1 + Encoder + INA226 + L298N
 * ============================================================
 *
 *  Basado en control_posicion_adaptable_registro.ino (misma ley, mismos
 *  comandos y mismo registro D,...). Cambios para el motor nuevo:
 *   - Lazo interno = PI SIMC del paso 02 (Kp 1.00, Ti 0.985 ms) con
 *     linealización por el mapa estático (mapa_corriente.h).
 *   - Zona muerta (ZM) ENCENDIDA por defecto y con dos modos:
 *       ZMMODO 1: congela toda la adaptación (como antes).
 *       ZMMODO 2: congela solo la parte INTEGRAL (gamma*q~/(1+q~^2));
 *                 el término -k*qdot sigue activo, así que dentro de la
 *                 banda el control conserva la rigidez p + k (defecto).
 *   - BREAK: prueba de despegue (rampa lenta de corriente en lazo
 *     abierto hasta que el eje se mueve) para medir la fricción estática.
 *
 *  LAZO EXTERNO (posición, 1 kHz) - PD adaptable de Tomei, Y = [sin(q) 1]
 *
 *      q~      = q - qd
 *      sigma   = gamma*q~/(1+q~^2)
 *      s       = qdot + sigma
 *      tau     = -p*q~ - d*qdot + sin(q)*phi1^ + phi2^          [mA]
 *      phi1^'  = -k1*sin(q)*s                                     (gravedad)
 *      phi2^'  = -k2*s                                            (par constante)
 *
 *  Corriente enviada al lazo interno:
 *      tau_m = sat(tau) + dist      (dist = perturbación artificial DIST/DSTEP,
 *                                    el controlador NO la conoce)
 *      iref  = tau_m + Idz*sgn(tau_m)                             (consumo del L298N)
 *
 *  OPCIONES PARA EXPERIMENTAR (por defecto = control final):
 *   - ADAPT1 / ADAPT2 [ON|OFF]: congela phi1^ o phi2^ (conserva su valor).
 *     ADAPT OFF + PHIRESET = PD puro (referencia de comparación).
 *   - PHI1 / PHI2 <mA>: fija el valor inicial (p. ej. PHI1 140 = gravedad
 *     conocida). PHIMAX1 / PHIMAX2: límites (anti-windup).
 *   - ZM: zona muerta con histéresis en la adaptación (contra el stick-slip),
 *     ENCENDIDA por defecto. Congela phi^ si |q~| < E_IN, |qdot| < V_REP y la
 *     rampa terminó; vuelve a adaptar si |q~| > E_OUT o cambia el objetivo.
 *   - DIST / DSTEP: perturbación artificial; MARCA: anota un evento.
 *
 *  LAZO INTERNO (corriente, 4 kHz) - PI SIMC + linealización (paso 02)
 *      w   = Kp*e + Ki*integral(e)      [mA]  (e = |iref| - i medida)
 *      PWM = mapa^-1(w)                        (mapa_corriente.h, por sentido)
 *      sgn(iref) -> sentido del puente (IN1 adelante / IN2 atrás)
 *
 *  PRUEBA DE DESPEGUE (BREAK <+1|-1> [mA/s]), con el control DETENIDO:
 *      iref sube en rampa desde 0 hasta que |q - q0| >= BRKDQ (defecto
 *      0.55 grados = 3 cuentas) o hasta BRKMAX. Imprime
 *        BREAK,<sentido>,<iref mA>,<tau equivalente = iref - Idz>,<q0 grados>
 *      Hazlo con la carga colgando en q = 0 (ahí la gravedad no actúa):
 *      tau equivalente = fricción estática Fs de ese sentido.
 *
 *  REGISTRO DE DATOS (LOG ON / LOG OFF), 921600 baud:
 *    Cada LOGDT ms (defecto 2 ms = 500 Hz) se envía una línea de ENTEROS.
 *    La envía una tarea propia en el núcleo 0 (no estorba al control).
 *      D,t_ms,qd,q,w,u,iref,i,phi1,phi2,d,flags
 *        qd, q [mili-grados] | w = qdot [centi-grados/s]
 *        u = tau del control (saturado), iref = corriente pedida con signo,
 *        i = corriente medida (magnitud), phi1^, phi2^, d  [décimas de mA]
 *        flags: 1 control activo, 2 saturado, 4 adapta phi1^, 8 adapta phi2^,
 *               16 ZM congelada, 32 rampa moviéndose, 64 phi^ en su límite
 *    Al iniciar el registro: líneas H (columnas) y P (parámetros). Cada
 *    comando recibido durante el registro se repite como  E,t_ms,<comando>.
 *    El script matlab/ensayo_adaptable.m guarda todo en CSV.
 *
 *  SALIDA DEL MONITOR (si no se registra), cada MONDT ms:
 *      q:<grados>,qd:<grados>,e:<grados>,i:<mA>,u:<mA>,p1:<mA>,p2:<mA>
 *    OJO: el Monitor Serie debe estar a 921600 baud.
 *
 *  CONVENCIONES:
 *   - q = 0 es la carga COLGANDO (equilibrio estable). Al encender el
 *     ESP32 el encoder vale 0: enciéndelo con la carga colgando y quieta,
 *     o usa ZERO con el motor detenido.
 *   - tau > 0 (PWM en IN1) hace crecer a q.
 *  Requiere Arduino-ESP32 core 3.x (ledcAttach / timerBegin(freq)).
 * ============================================================
 */

#include <Arduino.h>
#include <Wire.h>
#include "mapa_corriente.h"   // mapa PWM -> corriente (de 02_control_PI_corriente)

const uint32_t SERIAL_BAUD = 921600;          // alto para el registro de datos


// ============================================================
// PINES
// ============================================================

const int IN1 = 25;          // PWM adelante
const int IN2 = 26;          // PWM atrás
const int ENA = 27;          // Habilitación (HIGH = puente activo)

const int ENCODER_A = 32;
const int ENCODER_B = 33;

const int I2C_SDA = 21;
const int I2C_SCL = 22;


// ============================================================
// ENCODER
// ============================================================

const float COUNTS_PER_REV = 1975.0f;               // motor nuevo (antes 1134)
const float RAD_PER_COUNT  = TWO_PI / COUNTS_PER_REV;
const float ENCODER_SIGN   = -1.0f;                  // motor nuevo: PWM + bajaba las cuentas (antes +1)


// ============================================================
// INA226
// ============================================================

const uint8_t  INA226_ADDR  = 0x40;
const uint32_t I2C_CLOCK_HZ = 400000;

const float R_SHUNT      = 0.1f;
const float SHUNT_LSB_V  = 2.5e-6f;
const float CURRENT_SIGN = 1.0f;

const uint8_t REG_CONFIG   = 0x00;
const uint8_t REG_SHUNT    = 0x01;
const uint8_t REG_MANUF_ID = 0xFE;
const uint8_t REG_DIE_ID   = 0xFF;

// Continuo, solo shunt, 140 us, sin promedio
const uint16_t INA226_CONFIG_FAST = 0x4000 | (0 << 9) | (0 << 6) | (0 << 3) | 0x5;

const int16_t RAW_SAT = 32760;

const float GLITCH_JUMP_A    = 0.25f;   // salto que obliga a confirmar la lectura
const float GLITCH_CONFIRM_A = 0.10f;   // tolerancia de la lectura de confirmación


// ============================================================
// PWM Y TIEMPOS
// ============================================================

const uint32_t PWM_FREQUENCY  = 21429;
const uint8_t  PWM_RESOLUTION = 10;
const float    PWM_MAX        = 1023.0f;

const uint32_t TS_US     = 250;                     // lazo interno: 4 kHz
const float    Ts        = TS_US * 1e-6f;
const int      OUTER_DIV = 4;                       // lazo externo cada 4 ciclos
const float    Ts_o      = Ts * OUTER_DIV;          // 1 ms -> 1 kHz


// ============================================================
// LÍMITES Y PROTECCIONES
// ============================================================

const float I_TRIP_A   = 0.78f;         // sobrecorriente [A]
const float TAU_MAX_MA = 700.0f;        // |iref| máxima [mA]


// ============================================================
// GANANCIAS (valores finales de la sintonización)
// ============================================================

// Lazo interno: PI SIMC del paso 02 (con LIN: Kp en mA/mA)
const float KP_SIMC  = 1.003f;          // [mA/mA]
const float TI_SIMC  = 0.985e-3f;       // [s]
volatile float KpI = KP_SIMC;
volatile float KiI = KP_SIMC / TI_SIMC; // [1/s] = 1018

// Lazo externo: PD adaptable
volatile float Pg      = 1902.0f;       // p      [mA/rad]
volatile float Dg      = 100.0f;        // d      [mA*s/rad]
volatile float Kg1     = 476.0f;        // k1     [mA/rad]   (gravedad)
volatile float Kg2     = 476.0f;        // k2     [mA/rad]   (par constante)
volatile float Gam     = 32.61f;        // gamma  [1/s]

volatile float phiHat1 = 0.0f;          // phi1^ [mA]
volatile float phiHat2 = 0.0f;          // phi2^ [mA]
volatile float phiMax1 = TAU_MAX_MA;    // límite de |phi1^| [mA]
volatile float phiMax2 = 200.0f;        // límite de |phi2^| [mA]

volatile float velCutHz = 8.0f;         // filtro de la derivada de q [Hz]
volatile float iDz      = 35.0f;        // compensación del consumo del L298N [mA]
volatile float errMax   = PI;           // paro por error de posición [rad] (0 = sin límite)

// Adaptación
volatile bool  adapt1On = true;         // ADAPT1: integra phi1^
volatile bool  adapt2On = true;         // ADAPT2: integra phi2^

// Zona muerta en la adaptación con histéresis (contra el stick-slip)
volatile bool     zmOn     = true;                  // ZM ON/OFF (encendida por defecto)
volatile uint8_t  zmMode   = 2;                     // 1 = congela todo, 2 = solo la parte integral
volatile float    zmEin    = 0.25f * DEG_TO_RAD;    // congela si |q~| < E_IN         [rad]
volatile float    zmEout   = 1.00f * DEG_TO_RAD;    // adapta de nuevo si |q~| > E_OUT [rad]
volatile float    zmVrep   = 5.0f  * DEG_TO_RAD;    // "quieto" si |qdot| < V_REP     [rad/s]
volatile bool     zmFrozen = false;
volatile uint32_t zmEntries = 0;

// Perturbación artificial (simula un par de carga; el control no la conoce)
volatile float    distBase   = 0.0f;                // DIST <mA>   (constante)
volatile float    distPulse  = 0.0f;                // DSTEP <mA> <ms> (pulso)
volatile uint32_t distPulseN = 0;                   // ticks de 1 ms restantes del pulso
volatile float    distNow    = 0.0f;                // aplicada ahora [mA]

// Generador de rampa
volatile bool  rampOn   = true;                     // RAMPA ON/OFF
volatile float rampRate = 90.0f * DEG_TO_RAD;       // velocidad máxima de qd [rad/s]
volatile float qTarget  = 0.0f;                     // objetivo [rad]


// ============================================================
// ESTADO COMPARTIDO
// ============================================================

enum Mode : uint8_t { MODE_OFF = 0, MODE_POS = 1, MODE_BREAK = 2 };

// Prueba de despegue (BREAK)
volatile int8_t   brkDir    = 1;
volatile float    brkRate   = 100.0f;               // [mA/s]
volatile float    brkMax    = 450.0f;               // [mA] límite de la rampa
volatile float    brkDq     = 3.0f * TWO_PI / 1975.0f;   // [rad] movimiento que cuenta como despegue
volatile float    brkI      = 0.0f;                 // [mA] corriente actual de la rampa
volatile float    brkQ0     = 0.0f;
volatile float    brkResult = 0.0f;                 // [mA] (-1 = no se movió)
volatile bool     brkDone   = false;
volatile uint8_t mode = MODE_OFF;

enum Fault : uint8_t {
  FAULT_NONE = 0, FAULT_OVERCURRENT, FAULT_I2C, FAULT_ADC_SAT, FAULT_REGEN, FAULT_POS_ERROR, FAULT_NO_POWER
};
volatile uint8_t faultCode  = FAULT_NONE;
volatile float   faultValue = 0.0f;

volatile float qNow   = 0.0f;           // [rad]
volatile float qdotNow = 0.0f;          // velocidad usada por el control [rad/s]
volatile float qdNow  = 0.0f;           // [rad]
volatile float tauNow = 0.0f;           // [mA]
volatile float iNowA  = 0.0f;           // corriente medida [A]
volatile bool  tauSaturated = false;

volatile uint32_t execMaxUs = 0, overruns = 0;
volatile uint32_t i2cErrorsTotal = 0, glitchCount = 0, inaResets = 0;

TaskHandle_t controlTaskHandle = nullptr;
hw_timer_t*  controlTimer      = nullptr;

bool     monitorOn = true;
uint32_t monDtMs   = 20;                // periodo de envío [ms] (50 Hz)


// ============================================================
// REGISTRO DE DATOS (anillo: lo llena la tarea de control, lo vacía logTask)
// ============================================================

struct LogSample
{
  uint32_t t;                 // [ms] desde LOG ON
  float qd, q, w;             // [grados], [grados], [grados/s]
  float u, ir, im, p1, p2, d; // [mA]
  uint8_t fl;                 // banderas
};

const uint32_t LOG_N = 1024;                        // ~2 s de margen a 500 Hz
LogSample      logBuf[LOG_N];
volatile uint32_t logHead = 0, logTail = 0;         // escritura (control) / lectura (logTask)
volatile uint32_t logLost = 0;                      // muestras perdidas (anillo lleno)
volatile bool     logOn   = false;
volatile uint32_t logDtMs = 2;                      // periodo de registro [ms]
volatile uint32_t logClock = 0;                     // [ms] reloj del registro (lazo externo)

enum LogFlags : uint8_t {
  LF_ACTIVE = 1, LF_SAT = 2, LF_AD1 = 4, LF_AD2 = 8, LF_ZM = 16, LF_RAMP = 32, LF_PHILIM = 64, LF_BREAK = 128
};


// ============================================================
// ENCODER (decodificación x4 por tabla de transiciones)
// ============================================================

volatile long    encoderPosition = 0;
volatile uint8_t previousState   = 0;

const int8_t transitionTable[16] = {
     0, -1,  1,  0,
     1,  0,  0, -1,
    -1,  0,  0,  1,
     0,  1, -1,  0
};

void IRAM_ATTR encoderISR()
{
  uint8_t A = digitalRead(ENCODER_A);
  uint8_t B = digitalRead(ENCODER_B);
  uint8_t currentState = (A << 1) | B;
  encoderPosition += transitionTable[(previousState << 2) | currentState];
  previousState = currentState;
}

inline float readQ()
{
  return ENCODER_SIGN * (float)encoderPosition * RAD_PER_COUNT;   // [rad]
}


// ============================================================
// INA226 - ACCESO DIRECTO POR I2C
// ============================================================

bool inaWriteReg(uint8_t reg, uint16_t value)
{
  Wire.beginTransmission(INA226_ADDR);
  Wire.write(reg);
  Wire.write((uint8_t)(value >> 8));
  Wire.write((uint8_t)(value & 0xFF));
  return Wire.endTransmission() == 0;
}

bool inaReadReg(uint8_t reg, uint16_t &value)
{
  Wire.beginTransmission(INA226_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission() != 0) return false;
  if (Wire.requestFrom(INA226_ADDR, (uint8_t)2) != 2) return false;
  uint8_t hi = Wire.read();
  uint8_t lo = Wire.read();
  value = ((uint16_t)hi << 8) | lo;
  return true;
}

bool inaSetPointer(uint8_t reg)
{
  Wire.beginTransmission(INA226_ADDR);
  Wire.write(reg);
  return Wire.endTransmission() == 0;
}

inline bool inaReadShuntFast(int16_t &raw)
{
  if (Wire.requestFrom(INA226_ADDR, (uint8_t)2) != 2) return false;
  uint8_t hi = Wire.read();
  uint8_t lo = Wire.read();
  raw = (int16_t)(((uint16_t)hi << 8) | lo);
  return true;
}

bool inaInit()
{
  uint16_t id = 0;
  if (!inaReadReg(REG_MANUF_ID, id) || id != 0x5449) return false;
  if (!inaReadReg(REG_DIE_ID, id)   || id != 0x2260) return false;

  inaWriteReg(REG_CONFIG, 0x8000);              // reinicio
  delay(2);
  if (!inaWriteReg(REG_CONFIG, INA226_CONFIG_FAST)) return false;

  uint16_t cfg = 0;
  if (!inaReadReg(REG_CONFIG, cfg) || cfg != INA226_CONFIG_FAST) return false;
  return inaSetPointer(REG_SHUNT);              // el apuntador queda en el shunt
}


// ============================================================
// PUENTE H
// ============================================================

inline void bridgeWrite(float pwm, int8_t dir)
{
  uint32_t c = (uint32_t)(pwm + 0.5f);
  if (dir >= 0) { ledcWrite(IN2, 0); ledcWrite(IN1, c); }
  else          { ledcWrite(IN1, 0); ledcWrite(IN2, c); }
}

inline void motorCoast()
{
  ledcWrite(IN1, 0);
  ledcWrite(IN2, 0);
  digitalWrite(ENA, LOW);
}

// Inversa del mapa estático: PWM que da la corriente w [mA] (interpolación lineal)
inline const float* mapaDir(int8_t dir) { return (dir >= 0) ? MAP_I_POS : MAP_I_NEG; }
float mapaInv(float w, int8_t dir)
{
  const float* I = mapaDir(dir);
  if (w <= I[0]) return 0.0f;
  if (w >= I[MAP_N - 1]) return PWM_MAX;
  int k = 1;
  while (k < MAP_N - 1 && I[k] < w) k++;
  float f = (w - I[k - 1]) / (I[k] - I[k - 1]);
  return MAP_U[k - 1] + f * (MAP_U[k] - MAP_U[k - 1]);
}


// ============================================================
// TIMER Y FALLAS
// ============================================================

void ARDUINO_ISR_ATTR onControlTimer()
{
  BaseType_t woken = pdFALSE;
  vTaskNotifyGiveFromISR(controlTaskHandle, &woken);
  if (woken) portYIELD_FROM_ISR();
}

void tripFault(uint8_t code, float value)
{
  faultValue = value;
  mode = MODE_OFF;
  motorCoast();
  faultCode = code;
}


// ============================================================
// TAREA DE CONTROL (núcleo 1, prioridad máxima, 4 kHz)
// ============================================================

void controlTask(void* arg)
{
  (void)arg;

  uint8_t i2cErrRow = 0, protRow = 0, negRow = 0;
  uint32_t sinPotencia = 0;             // ciclos pidiendo corriente sin que circule (12 V apagado)
  float   iValid = 0.0f;
  bool    haveValid = false;
  float   integ  = 0.0f;
  int8_t  dirNow = 1;
  float   iRef   = 0.0f;                // referencia de corriente con signo [A]

  int     outerCnt = 0;
  float   qPrev = readQ();
  float   qdotF = 0.0f;
  float   qd    = qPrev;
  float   lastTgt = qPrev;              // para detectar un objetivo nuevo (ZM)
  uint32_t logDiv = 0;

  uint8_t prevMode = MODE_OFF;

  for (;;)
  {
    uint32_t pending = ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    uint32_t t0 = micros();
    if (pending > 1) overruns += pending - 1;

    // ---------------- 1) Medición de corriente ----------------
    int16_t raw;
    if (!inaReadShuntFast(raw))
    {
      i2cErrorsTotal++;
      inaSetPointer(REG_SHUNT);
      if (++i2cErrRow >= 3 && mode != MODE_OFF) { tripFault(FAULT_I2C, 0.0f); prevMode = MODE_OFF; }
      continue;
    }
    i2cErrRow = 0;

    float iMeas = CURRENT_SIGN * (float)raw * (SHUNT_LSB_V / R_SHUNT);   // [A]
    if (!haveValid) { iValid = iMeas; haveValid = true; }

    bool verifiedReal = false;
    bool bigJump = (fabsf(iMeas - iValid) > GLITCH_JUMP_A) || raw >= RAW_SAT || raw <= -RAW_SAT;

    if (!bigJump)
    {
      iValid = iMeas;
    }
    else
    {
      // Validación: un salto grande se confirma con una segunda lectura
      int16_t raw2;
      bool ok2 = inaReadShuntFast(raw2);
      float iMeas2 = CURRENT_SIGN * (float)raw2 * (SHUNT_LSB_V / R_SHUNT);
      if (!ok2 || fabsf(iMeas2 - iMeas) > GLITCH_CONFIRM_A)
      {
        glitchCount++;
      }
      else
      {
        uint16_t cfg = 0;
        if (!inaReadReg(REG_CONFIG, cfg) || cfg != INA226_CONFIG_FAST)
        {
          inaWriteReg(REG_CONFIG, INA226_CONFIG_FAST);
          inaResets++;
        }
        else
        {
          iValid = iMeas;
          verifiedReal = true;
        }
      }
      inaSetPointer(REG_SHUNT);
    }

    float i = iValid;                   // [A]
    iNowA = i;

    // ---------------- 2) Protecciones de corriente ----------------
    if (mode != MODE_OFF)
    {
      uint8_t fPos = FAULT_NONE, fNeg = FAULT_NONE;
      bool sat = verifiedReal && (raw >= RAW_SAT || raw <= -RAW_SAT);
      if (i > 0.0f)
      {
        if (sat)               fPos = FAULT_ADC_SAT;
        else if (i > I_TRIP_A) fPos = FAULT_OVERCURRENT;
      }
      else if (sat || i < -I_TRIP_A)
      {
        fNeg = FAULT_REGEN;
      }
      protRow = (fPos != FAULT_NONE) ? protRow + 1 : 0;
      negRow  = (fNeg != FAULT_NONE) ? negRow + 1 : 0;

      uint8_t f = FAULT_NONE;
      if (protRow >= 2) f = fPos;
      if (negRow  >= 8) f = fNeg;
      if (f != FAULT_NONE)
      {
        tripFault(f, i * 1000.0f);
        prevMode = MODE_OFF;
        protRow = negRow = 0;
        continue;
      }
    }

    // Sin potencia: se pide > 150 mA y la corriente medida es < 5 mA durante 0.5 s
    // (fuente de 12 V apagada o L298N desconectado). Si no, el control satura sin que nada se mueva.
    if (mode != MODE_OFF && fabsf(iRef) > 0.150f && fabsf(i) < 0.005f)
    {
      if (++sinPotencia >= 2000) { tripFault(FAULT_NO_POWER, fabsf(iRef) * 1000.0f); prevMode = MODE_OFF; sinPotencia = 0; continue; }
    }
    else sinPotencia = 0;

    // ---------------- 3) Cambio de modo ----------------
    uint8_t m = mode;
    if (m != prevMode)
    {
      if (m == MODE_OFF)
      {
        iRef = 0.0f;
        motorCoast();
      }
      else
      {
        // Arranque sin salto: la referencia empieza en la posición actual
        integ = 0.0f;
        digitalWrite(ENA, HIGH);
        float q0 = readQ();
        qd = q0;  qTarget = q0;  qPrev = q0;  qdotF = 0.0f;
        lastTgt = q0;  zmFrozen = false;
        if (m == MODE_BREAK) { brkI = 0.0f; brkQ0 = q0; iRef = 0.0f; }
      }
      prevMode = m;
    }

    // ---------------- 4) Lazo externo (1 kHz) ----------------
    if (++outerCnt >= OUTER_DIV)
    {
      outerCnt = 0;
      float q = readQ();

      // Velocidad: derivada filtrada de primer orden
      float wf    = TWO_PI * velCutHz;
      float alpha = (wf * Ts_o) / (1.0f + wf * Ts_o);
      float qdotRaw = (q - qPrev) / Ts_o;
      qPrev = q;
      qdotF += alpha * (qdotRaw - qdotF);

      float tauSat = 0.0f;
      float tauOut = 0.0f;                // corriente pedida (con dist e Idz) [mA]
      uint8_t fl   = 0;

      // Perturbación artificial: constante + pulso programado
      float dist = distBase;
      if (distPulseN > 0) { dist += distPulse; distPulseN--; }
      distNow = dist;

      if (m == MODE_POS)
      {
        // Generador de referencia: rampa de velocidad máxima o escalón
        float tgt = qTarget;
        float rr  = rampRate;
        if (!rampOn || rr <= 0.0f)
        {
          qd = tgt;
        }
        else
        {
          float stepQ = rr * Ts_o;
          if      (qd < tgt - stepQ) qd += stepQ;
          else if (qd > tgt + stepQ) qd -= stepQ;
          else                       qd = tgt;
        }

        // Ley de control adaptable, Y = [sin(q) 1]
        float qt  = q - qd;
        float Y1  = sinf(q);
        float tau = -Pg * qt - Dg * qdotF + Y1 * phiHat1 + phiHat2;      // [mA]

        tauSat = constrain(tau, -TAU_MAX_MA, TAU_MAX_MA);
        bool satNow = (tauSat != tau);
        tauSaturated = satNow;

        // Zona muerta en la adaptación con histéresis (ZM)
        bool refFija = (qd == tgt);                                       // rampa terminada
        bool frozen  = zmFrozen;
        if (tgt != lastTgt) { frozen = false; lastTgt = tgt; }            // objetivo nuevo
        if (!zmOn || !refFija)                frozen = false;
        else if (!frozen)
        {
          if (fabsf(qt) < zmEin && fabsf(qdotF) < zmVrep) { frozen = true; zmEntries++; }
        }
        else if (fabsf(qt) > zmEout)          frozen = false;
        zmFrozen = frozen;

        // Ley de adaptación (Euler)
        //   s = qdot + sigma.  ZM modo 1: congela todo. ZM modo 2: congela solo
        //   sigma (la integral); -k*qdot sigue (rigidez p + k dentro de la banda).
        float s = qdotF + Gam * qt / (1.0f + qt * qt);
        bool  zm2 = frozen && (zmMode == 2);
        float sA  = zm2 ? qdotF : s;
        bool ad1 = adapt1On && (!frozen || zm2);
        bool ad2 = adapt2On && (!frozen || zm2);
        if (ad1)
          phiHat1 = constrain(phiHat1 + Ts_o * (-Kg1 * Y1 * sA), -phiMax1, phiMax1);

        float d2 = Ts_o * (-Kg2 * sA);
        if (ad2 && !(satNow && d2 * tau > 0.0f))                          // anti-windup
          phiHat2 = constrain(phiHat2 + d2, -phiMax2, phiMax2);

        // Perturbación artificial y consumo propio del puente
        float tauCmd = constrain(tauSat + dist, -TAU_MAX_MA, TAU_MAX_MA);
        if (iDz > 0.0f && fabsf(tauCmd) > 0.5f)
          tauCmd = constrain(tauCmd + ((tauCmd > 0.0f) ? iDz : -iDz), -TAU_MAX_MA, TAU_MAX_MA);

        iRef   = tauCmd / 1000.0f;                                        // [A], signo = sentido
        tauOut = tauCmd;

        fl = LF_ACTIVE;
        if (satNow)   fl |= LF_SAT;
        if (ad1)      fl |= LF_AD1;
        if (ad2)      fl |= LF_AD2;
        if (frozen)   fl |= LF_ZM;
        if (!refFija) fl |= LF_RAMP;
        if (fabsf(phiHat1) >= phiMax1 || fabsf(phiHat2) >= phiMax2) fl |= LF_PHILIM;

        // Protección por error de posición
        float em = errMax;
        if (em > 0.0f && fabsf(qt) > em)
        {
          tripFault(FAULT_POS_ERROR, qt);
          prevMode = MODE_OFF;
          continue;
        }
      }
      else if (m == MODE_BREAK)
      {
        // Prueba de despegue: rampa lenta de corriente en lazo abierto de posición
        qd = q;
        float bI = brkI + brkRate * Ts_o;
        brkI = bI;
        iRef   = brkDir * bI / 1000.0f;
        tauSat = brkDir * bI;
        tauOut = tauSat;
        fl = LF_ACTIVE | LF_BREAK;
        if (fabsf(q - brkQ0) >= brkDq)      { brkResult = bI;   brkDone = true; iRef = 0.0f; mode = MODE_OFF; }
        else if (bI >= brkMax)              { brkResult = -1.0f; brkDone = true; iRef = 0.0f; mode = MODE_OFF; }
      }
      else
      {
        qd = q;
      }

      qNow = q;  qdNow = qd;  qdotNow = qdotF;  tauNow = tauSat;

      // ---- Registro de datos ----
      if (logOn)
      {
        uint32_t tNow = logClock++;
        if (++logDiv >= logDtMs)
        {
          logDiv = 0;
          uint32_t h = logHead;
          if (h - logTail >= LOG_N)
          {
            logLost++;
          }
          else
          {
            LogSample &sm = logBuf[h % LOG_N];
            sm.t  = tNow;
            sm.qd = qd * RAD_TO_DEG;   sm.q = q * RAD_TO_DEG;   sm.w = qdotF * RAD_TO_DEG;
            sm.u  = tauSat;            sm.ir = tauOut;          sm.im = i * 1000.0f;
            sm.p1 = phiHat1;           sm.p2 = phiHat2;
            sm.d  = (m == MODE_POS) ? dist : 0.0f;
            sm.fl = fl;
            logHead = h + 1;
          }
        }
      }
      else
      {
        logDiv = 0;
      }
    }

    // ---------------- 5) Lazo interno: PI SIMC + linealización ----------------
    if (mode == MODE_POS || mode == MODE_BREAK)
    {
      float r = iRef;
      int8_t dir = (r > 0.0f) ? 1 : ((r < 0.0f) ? -1 : dirNow);
      if (dir != dirNow) { integ = 0.0f; dirNow = dir; }               // cambio de sentido

      const float* Im = mapaDir(dirNow);
      float wMin = Im[0], wMax = Im[MAP_N - 1];
      float rMag = fabsf(r) * 1000.0f, pwm = 0.0f;                     // [mA]
      if (rMag > 0.0f)
      {
        float e = rMag - i * 1000.0f;                                    // [mA]
        float integCand = integ + KiI * Ts * e;
        float wUn = KpI * e + integCand;
        bool windup = (wUn > wMax && e > 0.0f) || (wUn < wMin && e < 0.0f);
        if (!windup) integ = constrain(integCand, -wMax, wMax);
        float w = constrain(KpI * e + integ, wMin, wMax);               // [mA]
        pwm = mapaInv(w, dirNow);
      }
      else
      {
        integ = 0.0f;
      }
      bridgeWrite(pwm, dirNow);
    }

    uint32_t dt = micros() - t0;
    if (dt > execMaxUs) execMaxUs = dt;
  }
}


// ============================================================
// IMPRESIÓN
// ============================================================

void printStatus()
{
  Serial.println();
  Serial.println("------------------- STATUS -------------------");
  Serial.printf("Modo        : %s\n", mode == MODE_POS ? "POSICIÓN" : (mode == MODE_BREAK ? "BREAK" : "DETENIDO"));
  Serial.printf("q / qd / obj: %.2f / %.2f / %.2f grados\n",
                qNow * RAD_TO_DEG, qdNow * RAD_TO_DEG, qTarget * RAD_TO_DEG);
  Serial.printf("tau         : %.1f mA %s | I medida: %.1f mA\n",
                (float)tauNow, tauSaturated ? "(SATURADO)" : "", iNowA * 1000.0f);
  Serial.printf("Rampa       : %s, %.1f grados/s\n", rampOn ? "ON" : "OFF", rampRate * RAD_TO_DEG);
  Serial.printf("p = %.1f | d = %.1f | k1 = %.1f | k2 = %.1f | gamma = %.2f\n",
                (float)Pg, (float)Dg, (float)Kg1, (float)Kg2, (float)Gam);
  Serial.printf("phi1^ = %.2f mA (límite ±%.0f, adapta %s) | phi2^ = %.2f mA (límite ±%.0f, adapta %s)\n",
                (float)phiHat1, (float)phiMax1, adapt1On ? "ON" : "OFF",
                (float)phiHat2, (float)phiMax2, adapt2On ? "ON" : "OFF");
  Serial.printf("Integral equivalente: k2*gamma = %.0f mA/(rad*s)\n", (float)(Kg2 * Gam));
  Serial.printf("Zona muerta : %s (modo %u) | E_IN = %.2f | E_OUT = %.2f grados | V_REP = %.1f grados/s | %s, veces = %lu\n",
                zmOn ? "ON" : "OFF", (unsigned)zmMode, zmEin * RAD_TO_DEG, zmEout * RAD_TO_DEG, zmVrep * RAD_TO_DEG,
                zmFrozen ? "CONGELADA" : "activa", (unsigned long)zmEntries);
  Serial.printf("Perturbación: DIST = %.1f mA | aplicada ahora = %.1f mA\n", (float)distBase, (float)distNow);
  Serial.printf("Registro    : %s cada %lu ms | t = %lu ms | perdidas = %lu\n",
                logOn ? "ON" : "OFF", (unsigned long)logDtMs, (unsigned long)logClock, (unsigned long)logLost);
  Serial.printf("Filtro velocidad = %.1f Hz | Idz = %.1f mA | error máx = %.1f grados\n",
                (float)velCutHz, (float)iDz, errMax * RAD_TO_DEG);
  Serial.printf("PI corriente (SIMC + mapa): Kp = %.4f mA/mA | Ki = %.1f 1/s\n", (float)KpI, (float)KiI);
  Serial.printf("BREAK: %.0f mA/s, máx %.0f mA, despegue = %.2f grados\n", (float)brkRate, (float)brkMax, brkDq * RAD_TO_DEG);
  Serial.printf("Monitor %s cada %lu ms\n", monitorOn ? "ON" : "OFF", (unsigned long)monDtMs);
  Serial.printf("Ejecución máx = %lu us | periodos perdidos = %lu\n",
                (unsigned long)execMaxUs, (unsigned long)overruns);
  Serial.printf("Errores I2C = %lu | lecturas descartadas = %lu | reinicios INA226 = %lu\n",
                (unsigned long)i2cErrorsTotal, (unsigned long)glitchCount, (unsigned long)inaResets);
  Serial.println("----------------------------------------------");
  execMaxUs = 0;
}

void printHelp()
{
  Serial.println();
  Serial.println("COMANDOS:");
  Serial.println("  ZERO             -> q = 0 aquí (motor detenido, carga colgando)");
  Serial.println("  START            -> control de posición (mantiene la posición actual)");
  Serial.println("  STOP             -> detener (motor libre)");
  Serial.println("  POS <grados>     -> nueva posición objetivo");
  Serial.println("  RAMPA [ON|OFF]   -> generador de rampa on/off (sin argumento: alterna)");
  Serial.println("  VEL <grados/s>   -> velocidad máxima de la rampa");
  Serial.println("  P / D / K1 / K2 / G <valor>  -> ganancias del lazo externo");
  Serial.println("  --- adaptación ---");
  Serial.println("  PHIRESET         -> phi1^ = phi2^ = 0");
  Serial.println("  ADAPT [ON|OFF]   -> adaptación de phi1^ y phi2^ (OFF = conservan su valor)");
  Serial.println("  ADAPT1 / ADAPT2 [ON|OFF] -> solo phi1^ / solo phi2^");
  Serial.println("  PHI1 / PHI2 <mA> -> fija el valor de phi1^ / phi2^");
  Serial.println("  PHIMAX1 / PHIMAX2 <mA> -> límites de |phi1^| / |phi2^|");
  Serial.println("  ZM [ON|OFF]      -> zona muerta con histéresis en la adaptación");
  Serial.println("  ZMIN / ZMOUT <grados>, ZMVEL <grados/s> -> umbrales de la ZM");
  Serial.println("  ZMMODO <1|2>     -> 1 congela todo, 2 congela solo la integral (defecto)");
  Serial.println("  --- identificación ---");
  Serial.println("  BREAK <+1|-1> [mA/s] -> prueba de despegue (control detenido, carga en q = 0)");
  Serial.println("  BRKMAX <mA>, BRKDQ <grados> -> límite de la rampa y movimiento de despegue");
  Serial.println("  --- otros ajustes ---");
  Serial.println("  VF <Hz>          -> corte del filtro de velocidad");
  Serial.println("  IDZ <mA>         -> compensación del consumo del L298N");
  Serial.println("  EMAX <grados>    -> paro por error de posición (0 = sin límite)");
  Serial.println("  --- carga / perturbación ---");
  Serial.println("  DIST <mA>        -> perturbación constante (0 = quitar); el control no la ve");
  Serial.println("  DSTEP <mA> [ms]  -> pulso de perturbación (defecto 2000 ms)");
  Serial.println("  MARCA <texto>    -> anota un evento en el registro (p. ej. MARCA CARGA_PUESTA)");
  Serial.println("  --- registro ---");
  Serial.println("  LOG [ON|OFF]     -> registro de datos D,... (el monitor se pausa mientras tanto)");
  Serial.println("  LOGDT <ms>       -> periodo del registro (1-50 ms, defecto 2)");
  Serial.println("  PARAMS           -> imprime la línea P con todos los parámetros");
  Serial.println("  --- monitor ---");
  Serial.println("  MON [ON|OFF]     -> envío de q, qd, e, i, u, phi1^, phi2^");
  Serial.println("  MONDT <ms>       -> periodo de envío (10-1000 ms)");
  Serial.println("  STATUS, HELP");
  Serial.println();
  Serial.println("Salida: q:<grados>,qd:<grados>,e:<grados>,i:<mA>,u:<mA>,p1:<mA>,p2:<mA>   (e = q - qd)");
  Serial.println();
}


// Línea P: todos los parámetros (queda en el registro y en el archivo _meta)
void printParams()
{
  Serial.printf("P,p=%.2f,d=%.2f,k1=%.2f,k2=%.2f,gamma=%.3f,phi1=%.2f,phi2=%.2f,phimax1=%.0f,phimax2=%.0f,"
                "adapt1=%d,adapt2=%d,zm=%d,zmmodo=%u,zmin=%.2f,zmout=%.2f,zmvel=%.1f,"
                "vf=%.1f,idz=%.1f,emax_deg=%.1f,rampa=%d,vel_dps=%.1f,dist=%.1f,"
                "kpi=%.4f,kii=%.2f,logdt=%lu,taumax=%.0f\n",
                (float)Pg, (float)Dg, (float)Kg1, (float)Kg2, (float)Gam,
                (float)phiHat1, (float)phiHat2, (float)phiMax1, (float)phiMax2,
                adapt1On ? 1 : 0, adapt2On ? 1 : 0, zmOn ? 1 : 0, (unsigned)zmMode,
                zmEin * RAD_TO_DEG, zmEout * RAD_TO_DEG, zmVrep * RAD_TO_DEG,
                (float)velCutHz, (float)iDz, errMax * RAD_TO_DEG, rampOn ? 1 : 0, rampRate * RAD_TO_DEG,
                (float)distBase, (float)KpI, (float)KiI, (unsigned long)logDtMs, TAU_MAX_MA);
}

// ---- Tarea de registro (núcleo 0): vacía el anillo con formato entero ----
// (printf con decimales es demasiado lento a 500 Hz: se perdían muestras)
static inline char* putU(char* p, uint32_t v)
{
  char b[10];  int n = 0;
  do { b[n++] = (char)('0' + v % 10);  v /= 10; } while (v);
  while (n) *p++ = b[--n];
  return p;
}
static inline char* putI(char* p, float x)
{
  int32_t v = (int32_t)lroundf(x);
  if (v < 0) { *p++ = '-';  return putU(p, (uint32_t)(-v)); }
  return putU(p, (uint32_t)v);
}

TaskHandle_t logTaskHandle = nullptr;

void logTask(void* arg)
{
  (void)arg;
  char line[128];
  for (;;)
  {
    uint32_t n = 0;
    while (logTail != logHead && n < 32)
    {
      const LogSample &sm = logBuf[logTail % LOG_N];
      char* p = line;
      *p++ = 'D';  *p++ = ',';
      p = putU(p, sm.t);              *p++ = ',';
      p = putI(p, sm.qd * 1000.0f);   *p++ = ',';
      p = putI(p, sm.q  * 1000.0f);   *p++ = ',';
      p = putI(p, sm.w  * 100.0f);    *p++ = ',';
      p = putI(p, sm.u  * 10.0f);     *p++ = ',';
      p = putI(p, sm.ir * 10.0f);     *p++ = ',';
      p = putI(p, sm.im * 10.0f);     *p++ = ',';
      p = putI(p, sm.p1 * 10.0f);     *p++ = ',';
      p = putI(p, sm.p2 * 10.0f);     *p++ = ',';
      p = putI(p, sm.d  * 10.0f);     *p++ = ',';
      p = putU(p, sm.fl);             *p++ = '\n';
      Serial.write((const uint8_t*)line, (size_t)(p - line));
      logTail = logTail + 1;
      n++;
    }
    vTaskDelay(1);
  }
}


// ============================================================
// COMANDOS
// ============================================================

void handleSerial()
{
  if (!Serial.available()) return;

  String cmd = Serial.readStringUntil('\n');
  cmd.trim();
  cmd.toUpperCase();
  if (cmd.length() == 0) return;

  int sp = cmd.indexOf(' ');
  String name = (sp < 0) ? cmd : cmd.substring(0, sp);
  String args = (sp < 0) ? String("") : cmd.substring(sp + 1);
  args.trim();
  float val = args.toFloat();
  bool hasArg = args.length() > 0;

  // Durante el registro, cada comando queda como evento con su tiempo
  if (logOn && name != "LOG")
  {
    String ev = cmd;  ev.replace(",", ";");
    Serial.printf("E,%lu,%s\n", (unsigned long)logClock, ev.c_str());
  }

  if (name == "ZERO")
  {
    if (mode != MODE_OFF) { Serial.println("Primero STOP y espera a que la carga quede quieta."); return; }
    noInterrupts();
    encoderPosition = 0;
    interrupts();
    Serial.println("q = 0 en esta posición.");
  }
  else if (name == "START")
  {
    faultCode = FAULT_NONE;
    mode = MODE_POS;
    Serial.println("Control de posición iniciado (mantiene la posición actual).");
  }
  else if (name == "STOP")
  {
    mode = MODE_OFF;
    Serial.println("Detenido.");
  }
  else if (name == "POS" && hasArg)
  {
    if (mode != MODE_POS) { Serial.println("Primero START."); return; }
    qTarget = val * DEG_TO_RAD;
    Serial.printf("Objetivo = %.2f grados (rampa %s)\n", val, rampOn ? "ON" : "OFF");
  }
  else if (name == "RAMPA")
  {
    if (args == "ON")       rampOn = true;
    else if (args == "OFF") rampOn = false;
    else                    rampOn = !rampOn;
    Serial.printf("Generador de rampa %s (%.1f grados/s)\n", rampOn ? "ON" : "OFF", rampRate * RAD_TO_DEG);
  }
  else if (name == "VEL" && hasArg && val > 0)
  {
    rampRate = constrain(val, 1.0f, 2000.0f) * DEG_TO_RAD;
    Serial.printf("Velocidad de la rampa = %.1f grados/s\n", rampRate * RAD_TO_DEG);
  }
  else if (name == "P"  && hasArg && val > 0)  { Pg  = val; Serial.printf("p = %.2f\n", val); }
  else if (name == "D"  && hasArg && val >= 0) { Dg  = val; Serial.printf("d = %.2f\n", val); }
  else if (name == "K1" && hasArg && val >= 0) { Kg1 = val; Serial.printf("k1 = %.2f\n", val); }
  else if (name == "K2" && hasArg && val >= 0) { Kg2 = val; Serial.printf("k2 = %.2f\n", val); }
  else if (name == "G"  && hasArg && val >= 0) { Gam = val; Serial.printf("gamma = %.2f\n", val); }
  else if (name == "PHIRESET")
  {
    phiHat1 = 0.0f;  phiHat2 = 0.0f;
    Serial.println("phi1^ = phi2^ = 0");
  }
  // ---- adaptación ----
  else if (name == "ADAPT" || name == "ADAPT1" || name == "ADAPT2")
  {
    bool cur = (name == "ADAPT2") ? adapt2On : adapt1On;
    bool on  = (args == "ON") ? true : ((args == "OFF") ? false : !cur);
    if (name != "ADAPT2") adapt1On = on;
    if (name != "ADAPT1") adapt2On = on;
    Serial.printf("Adaptación: phi1^ %s | phi2^ %s\n", adapt1On ? "ON" : "OFF", adapt2On ? "ON" : "OFF");
  }
  else if (name == "PHI1" && hasArg)
  {
    phiHat1 = constrain(val, -(float)phiMax1, (float)phiMax1);
    Serial.printf("phi1^ = %.2f mA\n", (float)phiHat1);
  }
  else if (name == "PHI2" && hasArg)
  {
    phiHat2 = constrain(val, -(float)phiMax2, (float)phiMax2);
    Serial.printf("phi2^ = %.2f mA\n", (float)phiHat2);
  }
  else if (name == "PHIMAX1" && hasArg && val > 0)
  {
    phiMax1 = constrain(val, 1.0f, TAU_MAX_MA);
    Serial.printf("Límite de phi1^ = ±%.1f mA\n", (float)phiMax1);
  }
  else if (name == "PHIMAX2" && hasArg && val > 0)
  {
    phiMax2 = constrain(val, 1.0f, TAU_MAX_MA);
    Serial.printf("Límite de phi2^ = ±%.1f mA\n", (float)phiMax2);
  }
  else if (name == "ZM")
  {
    if (args == "ON")       zmOn = true;
    else if (args == "OFF") zmOn = false;
    else                    zmOn = !zmOn;
    Serial.printf("Zona muerta en la adaptación %s\n", zmOn ? "ON" : "OFF");
  }
  else if (name == "ZMIN" && hasArg && val > 0)
  {
    zmEin = constrain(val, 0.05f, 10.0f) * DEG_TO_RAD;
    if (zmEout < zmEin) zmEout = zmEin;
    Serial.printf("E_IN = %.2f grados | E_OUT = %.2f grados\n", zmEin * RAD_TO_DEG, zmEout * RAD_TO_DEG);
  }
  else if (name == "ZMOUT" && hasArg && val > 0)
  {
    float v = constrain(val, 0.05f, 20.0f) * DEG_TO_RAD;
    if (v < zmEin) { Serial.printf("E_OUT debe ser >= E_IN (%.2f grados)\n", zmEin * RAD_TO_DEG); return; }
    zmEout = v;
    Serial.printf("E_OUT = %.2f grados\n", zmEout * RAD_TO_DEG);
  }
  else if (name == "ZMMODO" && hasArg && (val == 1 || val == 2))
  {
    zmMode = (uint8_t)val;
    Serial.printf("ZM modo %u (%s)\n", (unsigned)zmMode, zmMode == 1 ? "congela todo" : "congela solo la integral");
  }
  else if (name == "BREAK")
  {
    if (mode != MODE_OFF) { Serial.println("BREAK: primero STOP y espera a que la carga quede quieta."); return; }
    int sp2 = args.indexOf(' ');
    float dv = args.substring(0, sp2 < 0 ? args.length() : sp2).toFloat();
    float rt = (sp2 < 0) ? (float)brkRate : args.substring(sp2 + 1).toFloat();
    brkDir  = (dv < 0) ? -1 : 1;
    brkRate = constrain(rt, 5.0f, 2000.0f);
    brkDone = false;
    faultCode = FAULT_NONE;
    mode = MODE_BREAK;
    Serial.printf("BREAK sentido %d, rampa %.0f mA/s ...\n", (int)brkDir, (float)brkRate);
  }
  else if (name == "BRKMAX" && hasArg && val > 0)
  {
    brkMax = constrain(val, 10.0f, TAU_MAX_MA);
    Serial.printf("BRKMAX = %.0f mA\n", (float)brkMax);
  }
  else if (name == "BRKDQ" && hasArg && val > 0)
  {
    brkDq = constrain(val, 0.1f, 10.0f) * DEG_TO_RAD;
    Serial.printf("BRKDQ = %.2f grados\n", brkDq * RAD_TO_DEG);
  }
  else if (name == "ZMVEL" && hasArg && val > 0)
  {
    zmVrep = constrain(val, 0.1f, 100.0f) * DEG_TO_RAD;
    Serial.printf("V_REP = %.1f grados/s\n", zmVrep * RAD_TO_DEG);
  }
  // ---- otros ajustes ----
  else if (name == "VF" && hasArg && val > 0)
  {
    velCutHz = constrain(val, 0.5f, 200.0f);
    Serial.printf("VF = %.1f Hz\n", (float)velCutHz);
  }
  else if (name == "IDZ" && hasArg && val >= 0) { iDz = val; Serial.printf("Idz = %.1f mA\n", val); }
  else if (name == "EMAX" && hasArg && val >= 0)
  {
    errMax = val * DEG_TO_RAD;
    Serial.printf("Error máx = %.1f grados %s\n", val, val == 0 ? "(sin límite)" : "");
  }
  // ---- perturbación ----
  else if (name == "DIST" && hasArg)
  {
    distBase = constrain(val, -400.0f, 400.0f);
    Serial.printf("Perturbación constante = %.1f mA\n", (float)distBase);
  }
  else if (name == "DSTEP" && hasArg)
  {
    int sp2 = args.indexOf(' ');
    float amp = args.substring(0, sp2 < 0 ? args.length() : sp2).toFloat();
    int   ms  = (sp2 < 0) ? 2000 : args.substring(sp2 + 1).toInt();
    ms = constrain(ms, 1, 60000);
    distPulseN = 0;
    distPulse  = constrain(amp, -400.0f, 400.0f);
    distPulseN = (uint32_t)ms;
    Serial.printf("Pulso de perturbación %.1f mA durante %d ms\n", (float)distPulse, ms);
  }
  else if (name == "MARCA")
  {
    Serial.printf("Marca: %s\n", args.c_str());
  }
  // ---- registro ----
  else if (name == "LOG")
  {
    bool on;
    if (args == "ON")       on = true;
    else if (args == "OFF") on = false;
    else                    on = !logOn;
    if (on && !logOn)
    {
      logLost = 0;  logClock = 0;              // el anillo ya está vacío (índices monótonos)
      Serial.println("H,t_ms,qd_mdeg,q_mdeg,w_cdps,u_dmA,iref_dmA,i_dmA,phi1_dmA,phi2_dmA,d_dmA,flags");
      printParams();
      Serial.println("E,0,LOG ON");
      logOn = true;
    }
    else if (!on && logOn)
    {
      logOn = false;
      for (int k = 0; k < 500 && logTail != logHead; k++) vTaskDelay(1);   // termina de enviar
      Serial.printf("E,%lu,LOG OFF perdidas=%lu\n", (unsigned long)logClock, (unsigned long)logLost);
      Serial.println("Registro detenido.");
    }
  }
  else if (name == "LOGDT" && hasArg && val > 0)
  {
    logDtMs = (uint32_t)constrain((int)val, 1, 50);
    Serial.printf("Registro cada %lu ms\n", (unsigned long)logDtMs);
  }
  else if (name == "PARAMS") printParams();
  // ---- monitor ----
  else if (name == "MON")
  {
    if (args == "ON")       monitorOn = true;
    else if (args == "OFF") monitorOn = false;
    else                    monitorOn = !monitorOn;
    Serial.println(monitorOn ? "Monitor ON" : "Monitor OFF");
  }
  else if (name == "MONDT" && hasArg)
  {
    monDtMs = (uint32_t)constrain((int)val, 10, 1000);
    Serial.printf("Monitor cada %lu ms\n", (unsigned long)monDtMs);
  }
  else if (name == "STATUS") printStatus();
  else if (name == "HELP")   printHelp();
  else Serial.println("Comando no reconocido (HELP para ayuda).");
}


// ============================================================
// SETUP
// ============================================================

void halt(const char* msg)
{
  Serial.println(msg);
  while (true) delay(1000);
}

void setup()
{
  Serial.setTxBufferSize(8192);         // margen para el registro
  Serial.setRxBufferSize(512);
  Serial.begin(SERIAL_BAUD);
  Serial.setTimeout(20);
  delay(200);

  // Puente H
  pinMode(ENA, OUTPUT);
  digitalWrite(ENA, LOW);
  if (!ledcAttach(IN1, PWM_FREQUENCY, PWM_RESOLUTION) ||
      !ledcAttach(IN2, PWM_FREQUENCY, PWM_RESOLUTION))
    halt("ERROR: no se pudo configurar PWM");
  ledcWrite(IN1, 0);
  ledcWrite(IN2, 0);

  // Encoder
  pinMode(ENCODER_A, INPUT_PULLUP);
  pinMode(ENCODER_B, INPUT_PULLUP);
  previousState = (digitalRead(ENCODER_A) << 1) | digitalRead(ENCODER_B);
  attachInterrupt(digitalPinToInterrupt(ENCODER_A), encoderISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_B), encoderISR, CHANGE);

  // INA226
  Wire.begin(I2C_SDA, I2C_SCL, I2C_CLOCK_HZ);
  Wire.setTimeOut(5);
  if (!inaInit()) halt("ERROR: no se pudo inicializar INA226");

  // Tarea de control (núcleo 1, prioridad máxima) + timer de 4 kHz
  xTaskCreatePinnedToCore(controlTask, "ctrl", 4096, nullptr,
                          configMAX_PRIORITIES - 1, &controlTaskHandle, 1);
  // Tarea de registro en el núcleo 0 (no compite con el control)
  xTaskCreatePinnedToCore(logTask, "log", 4096, nullptr, 1, &logTaskHandle, 0);

  controlTimer = timerBegin(1000000);
  timerAttachInterrupt(controlTimer, &onControlTimer);
  timerAlarm(controlTimer, TS_US, true, 0);

  Serial.println();
  Serial.println("==============================================");
  Serial.println("   CONTROL ADAPTABLE - MOTOR NUEVO (03) + REGISTRO CSV");
  Serial.println("   PD adaptable Y = [sin(q) 1] (1 kHz) + PI SIMC con mapa (4 kHz)");
  Serial.println("==============================================");
  printHelp();
  printStatus();
}


// ============================================================
// LOOP (comunicación)
// ============================================================

void loop()
{
  static uint32_t lastMon = 0;

  handleSerial();

  // Fallas
  uint8_t f = faultCode;
  if (f != FAULT_NONE)
  {
    faultCode = FAULT_NONE;
    switch (f)
    {
      case FAULT_OVERCURRENT: Serial.printf("*** FALLA: sobrecorriente (%.1f mA). ***\n", (float)faultValue); break;
      case FAULT_I2C:         Serial.println("*** FALLA: comunicación I2C con el INA226. ***"); break;
      case FAULT_ADC_SAT:     Serial.printf("*** FALLA: INA226 saturado (%.1f mA). ***\n", (float)faultValue); break;
      case FAULT_REGEN:       Serial.printf("*** FALLA: corriente negativa sostenida (%.1f mA). ***\n", (float)faultValue); break;
      case FAULT_POS_ERROR:   Serial.printf("*** FALLA: error de posición de %.1f grados. ***\n", faultValue * RAD_TO_DEG); break;
      case FAULT_NO_POWER:    Serial.printf("*** FALLA: se piden %.0f mA y no circula corriente: ¿fuente de 12 V apagada? ***\n", (float)faultValue); break;
    }
    if (logOn) Serial.printf("E,%lu,FALLA %u\n", (unsigned long)logClock, (unsigned)f);
    Serial.println("    Motor detenido (START para reanudar).");
  }

  // Resultado de la prueba de despegue
  if (brkDone)
  {
    brkDone = false;
    float r = brkResult;
    if (r < 0)
      Serial.printf("BREAK,%d,NO_SE_MOVIO,%.1f,%.2f\n", (int)brkDir, (float)brkMax, brkQ0 * RAD_TO_DEG);
    else
      Serial.printf("BREAK,%d,%.1f,%.1f,%.2f\n", (int)brkDir, r, r - iDz, brkQ0 * RAD_TO_DEG);
    if (logOn) Serial.printf("E,%lu,BREAK_RES %d %.1f %.1f\n", (unsigned long)logClock, (int)brkDir, r, r - iDz);
  }

  // Monitor: q, qd, e = q - qd [grados], i medida, u = tau, phi1^, phi2^ [mA]
  if (monitorOn && !logOn && millis() - lastMon >= monDtMs)
  {
    lastMon = millis();
    float q  = qNow  * RAD_TO_DEG;
    float qd = qdNow * RAD_TO_DEG;
    Serial.printf("q:%.2f,qd:%.2f,e:%.2f,i:%.1f,u:%.1f,p1:%.1f,p2:%.1f\n", q, qd, q - qd,
                  iNowA * 1000.0f, (float)tauNow, (float)phiHat1, (float)phiHat2);
  }

  vTaskDelay(1);
}
