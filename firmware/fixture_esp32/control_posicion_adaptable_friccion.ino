/*
 * ============================================================
 *  CONTROL DE POSICIÓN ADAPTABLE EN CASCADA - MOTOR DC 12 V
 *  + COMPENSACIÓN DE FRICCIÓN SECA EN ADELANTO
 *  ESP32 DevKit V1 + Encoder + INA226 + L298N
 *
 *  Igual que control_posicion_adaptable.ino, más:
 *
 *      tau_total = tau_adaptable + tau_f
 *
 *      FMODE 1 (por defecto): tau_f = Fc * sat(tau / FEPS)
 *          signo de la corriente pedida por el controlador
 *      FMODE 2: tau_f = Fc * sat(qdot / VEPS)      signo de la velocidad
 *      FMODE 3: híbrido: en movimiento sign(qdot); detenido -sign(q~)
 *      FMODE 0: sin compensación
 *
 *  Fc puede ser distinto en cada sentido (FCP adelante, FCN atrás).
 *  BREAK mide la corriente de despegue en ambos sentidos.
 * ============================================================
 *
 *  LAZO EXTERNO (posición, 1 kHz) - control adaptable solo para gravedad:
 *
 *      q~      = q - qd
 *      tau     = -p*q~ - d*qdot + Y(q)*phi^          Y(q) = sin(q)
 *      phi^'   = -k*Y(q)*( qdot + gamma*q~/(1+q~^2) )
 *
 *  LAZO INTERNO (corriente, 4 kHz) - PI con salida directa en PWM:
 *
 *      tau [mA] es la referencia de corriente:
 *          |tau|      -> referencia del PI (el INA226 mide magnitud)
 *          signo(tau) -> dirección del puente H (IN1 adelante / IN2 atrás)
 *
 *  Unidades del lazo externo (tau se expresa en mA):
 *      q [rad], qdot [rad/s], p [mA/rad], d [mA*s/rad],
 *      k [mA/rad], gamma [1/s], phi^ [mA]
 *
 *  CONVENCIONES IMPORTANTES:
 *   - q = 0 debe ser la posición de EQUILIBRIO ESTABLE (carga colgando
 *     hacia abajo). Así el par de gravedad es proporcional a sin(q).
 *     Usa ZERO con el motor detenido y la carga colgando.
 *   - tau > 0 (PWM en IN1) debe hacer que q AUMENTE. Si no, el lazo se
 *     vuelve inestable. Verifícalo con ACEL (o cambia ENCODER_SIGN).
 *
 *  Encoder calibrado: 1134 cuentas por vuelta del eje de salida (x4).
 *
 *  Requiere Arduino-ESP32 core 3.x (API ledcAttach / timerBegin(freq)).
 * ============================================================
 */

#include <Arduino.h>
#include <Wire.h>


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

const float COUNTS_PER_REV = 1134.0f;               // calibrado
const float RAD_PER_COUNT  = TWO_PI / COUNTS_PER_REV;

// +1 si tau > 0 (IN1) hace que las cuentas aumenten. Si no, -1.
const float ENCODER_SIGN = 1.0f;


// ============================================================
// INA226 (igual que el código de control de corriente)
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

const uint16_t INA226_CONFIG_FAST = 0x4000 | (0 << 9) | (0 << 6) | (0 << 3) | 0x5;

const int16_t RAW_SAT = 32760;

const float GLITCH_JUMP_A    = 0.25f;
const float GLITCH_CONFIRM_A = 0.10f;


// ============================================================
// PWM
// ============================================================

const uint32_t PWM_FREQUENCY  = 21429;
const uint8_t  PWM_RESOLUTION = 10;
const float    PWM_MAX        = 1023.0f;


// ============================================================
// TIEMPOS DE MUESTREO
// ============================================================

const uint32_t TS_US      = 250;                    // lazo interno: 4 kHz
const float    Ts         = TS_US * 1e-6f;
const int      OUTER_DIV  = 4;                      // lazo externo cada 4 ciclos
const float    Ts_o       = Ts * OUTER_DIV;         // 1 ms -> 1 kHz


// ============================================================
// LÍMITES Y PROTECCIONES
// ============================================================

const float V_SUPPLY    = 12.0f;        // solo para calcular ganancias del PI
const float I_TRIP_A    = 0.78f;        // sobrecorriente [A]
const float TAU_MAX_MA  = 700.0f;       // saturación de tau = |iref| máx [mA]


// ============================================================
// LAZO INTERNO (PI de corriente, salida en PWM)
// ============================================================

// Modelo para calcular Kp/Ki (pon aquí los valores de tu IDENT)
volatile float R_motor = 6.0f;          // [ohm]
volatile float L_motor = 2.0e-3f;       // [H]
volatile float tau_ci  = 1.0e-3f;       // constante de tiempo del lazo de corriente [s]

volatile float KpI = 0.0f;              // [cuentas / mA]
volatile float KiI = 0.0f;              // [cuentas / (mA*s)]

volatile float iRef = 0.0f;             // referencia de corriente [A], signo = dirección


// ============================================================
// LAZO EXTERNO (control adaptable)
// ============================================================

volatile float Pg    = 150.0f;          // p      [mA/rad]
volatile float Dg    = 15.0f;           // d      [mA*s/rad]
volatile float Kg    = 300.0f;          // k      [mA/rad]
volatile float Gam   = 1.0f;            // gamma  [1/s]

volatile float phiHat = 0.0f;           // estimado del parámetro de gravedad [mA]
volatile float phiMax = TAU_MAX_MA;     // límite de |phi^| (proyección práctica)
volatile bool  adaptOn = true;

// Cotas de la planta para verificar gamma (en unidades de tau = mA)
volatile float M2 = 0.0f;               // cota superior de la inercia [mA*s^2/rad] (0 = desconocida)
volatile float KC = 0.0f;               // cota de Coriolis (0 para 1 grado de libertad)

// Referencia
volatile float qTarget  = 0.0f;         // objetivo [rad]
volatile float rampRate = 90.0f * DEG_TO_RAD;   // velocidad máx de qd [rad/s] (0 = escalón)

// Filtro de velocidad (derivada filtrada de primer orden)
volatile float velCutHz = 30.0f;

// Compensación opcional de zona muerta [mA] (0 = desactivada)
volatile float iDz = 0.0f;

// ---------------- Compensación de fricción seca ----------------
volatile uint8_t fMode = 1;             // 0 off, 1 signo(tau), 2 signo(qdot), 3 híbrido
volatile float FcPos  = 60.0f;          // Fc sentido positivo [mA]
volatile float FcNeg  = 60.0f;          // Fc sentido negativo [mA]
volatile float fEps   = 5.0f;           // banda de suavizado sobre tau [mA] (0 = signo puro)
volatile float vEps   = 0.05f;          // banda de suavizado sobre qdot [rad/s]
volatile float qDb    = 0.5f * DEG_TO_RAD;  // zona sin compensar en modo 3 [rad]

// Prueba de despegue (BREAK)
const float BREAK_RATE_MA_S = 40.0f;    // rampa de corriente [mA/s]
const long  BREAK_COUNTS    = 3;        // cuentas para considerar que se movió
volatile float breakResult = 0.0f;      // [mA]
volatile bool  breakDone   = false;
volatile int8_t breakDir   = 1;

// Error de posición máximo antes de parar [rad] (0 = sin límite)
volatile float errMax = PI;


// ============================================================
// ESTADO COMPARTIDO
// ============================================================

enum Mode : uint8_t { MODE_OFF = 0, MODE_POS = 1, MODE_CURR = 2, MODE_BREAK = 3 };
volatile uint8_t mode = MODE_OFF;

enum Fault : uint8_t {
  FAULT_NONE = 0,
  FAULT_OVERCURRENT = 1,
  FAULT_I2C = 2,
  FAULT_ADC_SAT = 3,
  FAULT_REGEN = 4,
  FAULT_POS_ERROR = 5
};
volatile uint8_t faultCode = FAULT_NONE;
volatile float   faultValue = 0.0f;
volatile uint8_t faultMode  = MODE_OFF;

// Variables del lazo externo publicadas para el monitor
volatile float qNow     = 0.0f;         // [rad]
volatile float qdNow    = 0.0f;         // [rad]
volatile float qdotNow  = 0.0f;         // [rad/s]
volatile float tauNow   = 0.0f;         // [mA] (ya saturado)
volatile float ffNow    = 0.0f;         // compensación de fricción aplicada [mA]
volatile bool  tauSaturated = false;

// Lazo interno
volatile float lastI_A = 0.0f;
volatile float lastPwm = 0.0f;

// Estadísticas
volatile uint32_t execMaxUs      = 0;
volatile uint32_t overruns       = 0;
volatile uint32_t i2cErrorsTotal = 0;
volatile uint32_t glitchCount    = 0;
volatile uint32_t inaResets      = 0;
volatile uint32_t satCount       = 0;   // ciclos con tau saturado


// ============================================================
// CAPTURA (muestras del lazo externo, decimadas)
// ============================================================

struct PSample
{
  float qd_deg;
  float q_deg;
  float qdot;       // [rad/s]
  float tau_mA;
  float i_mA;
  float phi_mA;
};

const int CAP_N   = 600;
const int CAP_PRE = 50;

enum CapType : uint8_t { CAP_STEP = 1, CAP_ACEL = 2 };

PSample capBuf[CAP_N];
volatile int     capIdx    = 0;
volatile int     capLen    = CAP_N;
volatile bool    capActive = false;
volatile bool    capReady  = false;
volatile uint8_t capType   = CAP_STEP;
volatile int     capDecim  = 5;        // 1 muestra cada 5 ms -> 3 s
volatile float   stepTarget = 0.0f;    // [rad]
volatile float   acelCurrent = 0.0f;   // [mA]

portMUX_TYPE statMux = portMUX_INITIALIZER_UNLOCKED;

TaskHandle_t controlTaskHandle = nullptr;
hw_timer_t*  controlTimer      = nullptr;

bool monitorOn = true;


// ============================================================
// ENCODER
// ============================================================

volatile long    encoderPosition    = 0;
volatile uint8_t previousState      = 0;

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

  inaWriteReg(REG_CONFIG, 0x8000);
  delay(2);

  if (!inaWriteReg(REG_CONFIG, INA226_CONFIG_FAST)) return false;

  uint16_t cfg = 0;
  if (!inaReadReg(REG_CONFIG, cfg) || cfg != INA226_CONFIG_FAST) return false;

  return inaSetPointer(REG_SHUNT);
}


// ============================================================
// PUENTE H
// ============================================================

inline void bridgeWrite(float pwm, int8_t dir)
{
  uint32_t c = (uint32_t)(pwm + 0.5f);

  if (dir >= 0)
  {
    ledcWrite(IN2, 0);
    ledcWrite(IN1, c);
  }
  else
  {
    ledcWrite(IN1, 0);
    ledcWrite(IN2, c);
  }
}

inline void motorCoast()
{
  ledcWrite(IN1, 0);
  ledcWrite(IN2, 0);
  digitalWrite(ENA, LOW);
}


// ============================================================
// SINTONIZACIÓN DEL PI DE CORRIENTE (igual que el otro código)
// ============================================================

void retuneInner()
{
  const float s = (PWM_MAX / V_SUPPLY) / 1000.0f;

  float kp = (L_motor / tau_ci) * s;
  float ki = (R_motor / tau_ci) * s;

  portENTER_CRITICAL(&statMux);
  KpI = kp;
  KiI = ki;
  portEXIT_CRITICAL(&statMux);
}


// ============================================================
// CONDICIONES DE GANANCIA DEL CONTROLADOR ADAPTABLE
//   c < 2p/d
//   gamma < min{ sqrt(p/m2),  d / (m2 + kc + d/(2c)) }
// Tomando c -> 2p/d  (el c más grande permitido):  d/(2c) -> d^2/(4p)
// ============================================================

float gammaBound()
{
  if (M2 <= 0.0f || Pg <= 0.0f || Dg <= 0.0f) return -1.0f;

  float b1 = sqrtf(Pg / M2);
  float b2 = Dg / (M2 + KC + Dg * Dg / (4.0f * Pg));
  return (b1 < b2) ? b1 : b2;
}


// ============================================================
// TIMER
// ============================================================

void ARDUINO_ISR_ATTR onControlTimer()
{
  BaseType_t woken = pdFALSE;
  vTaskNotifyGiveFromISR(controlTaskHandle, &woken);
  if (woken) portYIELD_FROM_ISR();
}


// ============================================================
// FALLAS
// ============================================================

void tripFault(uint8_t code, float value)
{
  faultMode  = mode;
  faultValue = value;
  mode = MODE_OFF;
  iRef = 0.0f;
  motorCoast();

  if (capActive)
  {
    capActive = false;
    capReady  = true;
  }

  faultCode = code;
}


// ============================================================
// COMPENSACIÓN DE FRICCIÓN SECA
// ============================================================

inline float satUnit(float x)
{
  return (x > 1.0f) ? 1.0f : ((x < -1.0f) ? -1.0f : x);
}

// Devuelve tau_f [mA]
inline float frictionFF(float tau, float qdot, float qt)
{
  float s = 0.0f;     // dirección normalizada [-1, 1]

  switch (fMode)
  {
    case 1:   // signo de la corriente pedida
    {
      float e = fEps;
      if (e > 0.0f) s = satUnit(tau / e);
      else          s = (tau > 0.0f) ? 1.0f : ((tau < 0.0f) ? -1.0f : 0.0f);
      break;
    }
    case 2:   // signo de la velocidad
    {
      float e = vEps;
      s = (e > 0.0f) ? satUnit(qdot / e) : ((qdot > 0.0f) ? 1.0f : ((qdot < 0.0f) ? -1.0f : 0.0f));
      break;
    }
    case 3:   // híbrido
    {
      if (fabsf(qdot) > vEps)  s = (qdot > 0.0f) ? 1.0f : -1.0f;
      else if (fabsf(qt) > qDb) s = (qt < 0.0f) ? 1.0f : -1.0f;   // hacia el objetivo
      else                      s = 0.0f;
      break;
    }
    default:
      return 0.0f;
  }

  return (s >= 0.0f) ? s * FcPos : s * FcNeg;
}


// ============================================================
// TAREA DE CONTROL
// ============================================================

void controlTask(void* arg)
{
  (void)arg;

  // Lazo interno
  uint8_t i2cErrRow = 0;
  uint8_t protRow   = 0;
  uint8_t negRow    = 0;
  float   iValid    = 0.0f;
  bool    haveValid = false;
  float   integ     = 0.0f;
  int8_t  dirNow    = 1;

  // Lazo externo
  int     outerCnt  = 0;
  int     decimCnt  = 0;
  float   qPrev     = readQ();
  float   qdotF     = 0.0f;
  float   qd        = qPrev;
  long    breakStartCounts = 0;
  float   breakI    = 0.0f;

  uint8_t prevMode  = MODE_OFF;

  for (;;)
  {
    uint32_t pending = ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    uint32_t t0 = micros();
    if (pending > 1) overruns += pending - 1;


    // ========================================================
    // 1) MEDIR CORRIENTE (igual que el control de corriente)
    // ========================================================

    int16_t raw;

    if (!inaReadShuntFast(raw))
    {
      i2cErrorsTotal++;
      inaSetPointer(REG_SHUNT);
      if (++i2cErrRow >= 3 && mode != MODE_OFF)
      {
        tripFault(FAULT_I2C, 0.0f);
        prevMode = MODE_OFF;
      }
      continue;
    }
    i2cErrRow = 0;

    float iMeas = CURRENT_SIGN * (float)raw * (SHUNT_LSB_V / R_SHUNT);   // [A]

    if (!haveValid) { iValid = iMeas; haveValid = true; }

    bool verifiedReal = false;
    bool bigJump = (fabsf(iMeas - iValid) > GLITCH_JUMP_A) ||
                   raw >= RAW_SAT || raw <= -RAW_SAT;

    if (!bigJump)
    {
      iValid = iMeas;
    }
    else
    {
      int16_t raw2;
      bool ok2 = inaReadShuntFast(raw2);
      float iMeas2 = CURRENT_SIGN * (float)raw2 * (SHUNT_LSB_V / R_SHUNT);

      if (!ok2 || fabsf(iMeas2 - iMeas) > GLITCH_CONFIRM_A)
      {
        glitchCount++;
        inaSetPointer(REG_SHUNT);
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
        inaSetPointer(REG_SHUNT);
      }
    }

    float i = iValid;    // [A]


    // ========================================================
    // 2) PROTECCIONES DE CORRIENTE
    // ========================================================

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


    // ========================================================
    // 3) CAMBIO DE MODO
    // ========================================================

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
        integ = 0.0f;
        digitalWrite(ENA, HIGH);

        if (m == MODE_BREAK)
        {
          breakStartCounts = encoderPosition;
          breakI = 0.0f;
          iRef = 0.0f;
        }

        if (m == MODE_POS)
        {
          // Arranque sin salto: la referencia empieza en la posición actual
          float q0 = readQ();
          qd = q0;
          qTarget = q0;
          qPrev = q0;
          qdotF = 0.0f;
        }
      }
      prevMode = m;
    }


    // ========================================================
    // 4) LAZO EXTERNO (1 kHz)
    // ========================================================

    if (++outerCnt >= OUTER_DIV)
    {
      outerCnt = 0;

      float q = readQ();

      // ---- Velocidad: derivada filtrada de primer orden ----
      float wf    = TWO_PI * velCutHz;
      float alpha = (wf * Ts_o) / (1.0f + wf * Ts_o);
      float qdotRaw = (q - qPrev) / Ts_o;
      qPrev = q;
      qdotF += alpha * (qdotRaw - qdotF);

      // ---- Paso programado (captura STEP) ----
      if (capActive && capType == CAP_STEP && capIdx == CAP_PRE && decimCnt == 0)
      {
        qTarget = stepTarget;
      }

      float tauSat = 0.0f;

      if (m == MODE_POS)
      {
        // ---- Referencia con rampa (limita velocidad de qd) ----
        float tgt = qTarget;
        float rr  = rampRate;

        if (rr <= 0.0f)
        {
          qd = tgt;
        }
        else
        {
          float stepQ = rr * Ts_o;
          if (qd < tgt - stepQ)      qd += stepQ;
          else if (qd > tgt + stepQ) qd -= stepQ;
          else                       qd = tgt;
        }

        // ---- Ley de control adaptable ----
        float qt  = q - qd;               // q~
        float Y   = sinf(q);              // regresor
        float phi = phiHat;

        float tau = -Pg * qt - Dg * qdotF + Y * phi;      // [mA]

        // ---- Ley de adaptación (Euler) ----
        if (adaptOn)
        {
          float s = qdotF + Gam * qt / (1.0f + qt * qt);
          phi += Ts_o * (-Kg * Y * s);
          float pm = phiMax;
          phiHat = constrain(phi, -pm, pm);
        }

        // ---- Compensación de fricción seca (adelanto) ----
        float ff = frictionFF(tau, qdotF, qt);
        ffNow = ff;
        tau += ff;

        // ---- Saturación del actuador ----
        tauSat = constrain(tau, -TAU_MAX_MA, TAU_MAX_MA);
        bool satNow = (tauSat != tau);
        tauSaturated = satNow;
        if (satNow) satCount++;

        // ---- Compensación opcional de zona muerta ----
        float tauCmd = tauSat;
        float dz = iDz;
        if (dz > 0.0f && fabsf(tauSat) > 0.5f)
        {
          tauCmd += (tauSat > 0.0f) ? dz : -dz;
          tauCmd = constrain(tauCmd, -TAU_MAX_MA, TAU_MAX_MA);
        }

        // ---- tau -> referencia del lazo de corriente ----
        iRef = tauCmd / 1000.0f;   // [A], el signo da la dirección

        // ---- Protección por error de posición ----
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
        qd = q;
        ffNow = 0.0f;

        // Rampa lenta de corriente hasta que el eje se mueva
        breakI += BREAK_RATE_MA_S * Ts_o;
        iRef = breakDir * breakI / 1000.0f;
        tauSat = iRef * 1000.0f;

        long moved = encoderPosition - breakStartCounts;
        if (moved >= BREAK_COUNTS || moved <= -BREAK_COUNTS || breakI >= TAU_MAX_MA)
        {
          breakResult = (breakI >= TAU_MAX_MA) ? -1.0f : breakI;
          mode = MODE_OFF;
          iRef = 0.0f;
          motorCoast();
          prevMode = MODE_OFF;
          breakDone = true;
        }
      }
      else
      {
        qd = q;                 // fuera de POS la referencia sigue a q
        ffNow = 0.0f;
        if (m == MODE_CURR) tauSat = iRef * 1000.0f;
      }

      qNow    = q;
      qdNow   = qd;
      qdotNow = qdotF;
      tauNow  = tauSat;

      // ---- Captura decimada ----
      if (capActive)
      {
        if (decimCnt == 0)
        {
          int k = capIdx;
          capBuf[k].qd_deg = qd * RAD_TO_DEG;
          capBuf[k].q_deg  = q * RAD_TO_DEG;
          capBuf[k].qdot   = qdotF;
          capBuf[k].tau_mA = tauSat;
          capBuf[k].i_mA   = i * 1000.0f;
          capBuf[k].phi_mA = phiHat;
          capIdx = k + 1;

          if (capIdx >= capLen)
          {
            capActive = false;
            if (capType == CAP_ACEL)
            {
              mode = MODE_OFF;
              iRef = 0.0f;
              motorCoast();
              prevMode = MODE_OFF;
            }
            capReady = true;
          }
        }
        if (++decimCnt >= capDecim) decimCnt = 0;
      }
      else
      {
        decimCnt = 0;
      }
    }


    // ========================================================
    // 5) LAZO INTERNO: PI DE CORRIENTE (salida en PWM)
    // ========================================================

    float pwmSigned = 0.0f;

    if (mode == MODE_POS || mode == MODE_CURR || mode == MODE_BREAK)
    {
      float r = iRef;
      int8_t dir = (r > 0.0f) ? 1 : ((r < 0.0f) ? -1 : dirNow);

      if (dir != dirNow)
      {
        integ  = 0.0f;
        dirNow = dir;
      }

      float rMag = fabsf(r);
      float pwm  = 0.0f;

      if (rMag > 0.0f)
      {
        float e = (rMag - i) * 1000.0f;                 // [mA]
        float integCand = integ + KiI * Ts * e;
        float pwmUnsat  = KpI * e + integCand;

        pwm = constrain(pwmUnsat, 0.0f, PWM_MAX);

        bool windup = (pwmUnsat > PWM_MAX && e > 0.0f) ||
                      (pwmUnsat < 0.0f    && e < 0.0f);
        if (!windup) integ = constrain(integCand, 0.0f, PWM_MAX);
      }
      else
      {
        integ = 0.0f;
      }

      bridgeWrite(pwm, dirNow);
      pwmSigned = dirNow * pwm;
    }


    // ========================================================
    // 6) ESTADÍSTICAS
    // ========================================================

    lastI_A = i;
    lastPwm = pwmSigned;

    uint32_t dt = micros() - t0;
    if (dt > execMaxUs) execMaxUs = dt;
  }
}


// ============================================================
// ANÁLISIS DE CAPTURAS
// ============================================================

void printCapture(int zeroIndex, int n)
{
  float dtMs = capDecim * Ts_o * 1000.0f;

  Serial.println();
  Serial.println("BEGIN_CAPTURE");
  Serial.println("t_ms,qd_deg,q_deg,qdot_rad_s,tau_mA,i_mA,phi_mA");

  for (int k = 0; k < n; k++)
  {
    const PSample &s = capBuf[k];
    Serial.printf("%.1f,%.3f,%.3f,%.3f,%.1f,%.1f,%.2f\n",
                  (k - zeroIndex) * dtMs,
                  s.qd_deg, s.q_deg, s.qdot, s.tau_mA, s.i_mA, s.phi_mA);
  }

  Serial.println("END_CAPTURE");
}

void analyzeStep()
{
  float dtMs = capDecim * Ts_o * 1000.0f;

  float y0 = 0.0f;
  for (int k = 0; k < CAP_PRE; k++) y0 += capBuf[k].q_deg;
  y0 /= CAP_PRE;

  float target = stepTarget * RAD_TO_DEG;
  float yss = 0.0f;
  int nTail = 50;
  for (int k = CAP_N - nTail; k < CAP_N; k++) yss += capBuf[k].q_deg;
  yss /= nTail;

  float dy = target - y0;

  Serial.println();
  Serial.println("------- RESPUESTA DE POSICIÓN -------");
  Serial.printf("Inicial  = %.2f grados\n", y0);
  Serial.printf("Objetivo = %.2f grados\n", target);
  Serial.printf("Final    = %.2f grados  (error = %.2f grados)\n", yss, target - yss);

  if (fabsf(dy) < 1.0f)
  {
    Serial.println("Escalón muy pequeño para analizar.");
    Serial.println("-------------------------------------");
    return;
  }

  int k10 = -1, k90 = -1, lastOut = CAP_PRE;
  float nMax = -1e9f;

  for (int k = CAP_PRE; k < CAP_N; k++)
  {
    float n = (capBuf[k].q_deg - y0) / dy;
    if (k10 < 0 && n >= 0.1f) k10 = k;
    if (k90 < 0 && n >= 0.9f) k90 = k;
    if (n > nMax) nMax = n;
    if (fabsf(n - 1.0f) > 0.05f) lastOut = k;
  }

  if (k10 >= 0 && k90 >= 0)
    Serial.printf("t subida (10-90%%)      = %.0f ms\n", (k90 - k10) * dtMs);
  else
    Serial.println("No se alcanzó el 90% en la ventana de captura.");

  Serial.printf("Sobrepaso              = %.1f %%\n", (nMax > 1.0f) ? (nMax - 1.0f) * 100.0f : 0.0f);
  Serial.printf("t establecimiento (5%%) = %.0f ms\n", (lastOut + 1 - CAP_PRE) * dtMs);
  Serial.printf("phi^ final             = %.2f mA\n", capBuf[CAP_N - 1].phi_mA);
  Serial.printf("Ciclos con tau saturado: %lu (acumulado)\n", (unsigned long)satCount);
  Serial.println("-------------------------------------");
}

// Ajuste q(t) = q0 + 0.5*a*t^2 (arranque desde el reposo) -> m = I / a
void analyzeAcel(int n)
{
  float dt = capDecim * Ts_o;
  float q0 = capBuf[0].q_deg * DEG_TO_RAD;

  float num = 0.0f, den = 0.0f;
  for (int k = 1; k < n; k++)
  {
    float t  = k * dt;
    float t2 = t * t;
    float dq = capBuf[k].q_deg * DEG_TO_RAD - q0;
    num += dq * t2;
    den += t2 * t2;
  }

  float a = (den > 0.0f) ? 2.0f * num / den : 0.0f;    // [rad/s^2]
  float I = acelCurrent;                                // [mA]
  float dqTot = capBuf[n - 1].q_deg - capBuf[0].q_deg;

  Serial.println();
  Serial.println("--------- PRUEBA DE ACELERACIÓN ---------");
  Serial.printf("Corriente aplicada = %.0f mA durante %.0f ms\n", I, n * dt * 1000.0f);
  Serial.printf("Desplazamiento     = %.2f grados\n", dqTot);
  Serial.printf("Aceleración (ajuste)= %.2f rad/s^2\n", a);

  if (fabsf(dqTot) < 2.0f)
  {
    Serial.println("Casi no se movió: sube la corriente o el tiempo.");
  }
  else if (I * a < 0.0f)
  {
    Serial.println("*** SIGNO INVERTIDO: corriente positiva produjo q decreciente.");
    Serial.println("    Cambia ENCODER_SIGN a -1.0 (o intercambia A/B del encoder).");
    Serial.println("    NO uses el lazo de posición hasta corregirlo. ***");
  }
  else
  {
    float m = I / a;   // [mA*s^2/rad]
    Serial.printf("Signo correcto. Inercia efectiva m ~ %.3f mA*s^2/rad\n", m);
    Serial.println("(incluye fricción: tiende a SOBREESTIMAR m; repite en ambos sentidos)");
    Serial.printf("Sugerencia: M2 %.3f  (usa el mayor de varias pruebas, +20%%)\n", m * 1.2f);
  }
  Serial.println("-----------------------------------------");
}


// ============================================================
// IMPRESIÓN
// ============================================================

void printGains()
{
  Serial.printf("p = %.2f mA/rad | d = %.2f mA*s/rad | k = %.2f mA/rad | gamma = %.3f 1/s\n",
                (float)Pg, (float)Dg, (float)Kg, (float)Gam);

  float gb = gammaBound();
  if (gb > 0.0f)
  {
    Serial.printf("Condición: c < 2p/d = %.3f ; gamma < %.3f  -> %s\n",
                  2.0f * Pg / Dg, gb, (Gam < gb) ? "CUMPLE" : "NO CUMPLE");
  }
  else
  {
    Serial.println("Condición de gamma: define M2 (cota de inercia) para verificarla.");
  }
}

void printStatus()
{
  const char* modeName[] = { "DETENIDO", "POSICIÓN", "CORRIENTE", "BREAK" };

  Serial.println();
  Serial.println("------------------- STATUS -------------------");
  Serial.printf("Modo         : %s\n", modeName[mode]);
  Serial.printf("q            : %.2f grados (%.4f rad)\n", qNow * RAD_TO_DEG, (float)qNow);
  Serial.printf("qd / objetivo: %.2f / %.2f grados\n", qdNow * RAD_TO_DEG, qTarget * RAD_TO_DEG);
  Serial.printf("qdot         : %.3f rad/s\n", (float)qdotNow);
  Serial.printf("tau (iref)   : %.1f mA %s\n", (float)tauNow, tauSaturated ? "(SATURADO)" : "");
  Serial.printf("I medida     : %.1f mA | PWM: %.0f\n", lastI_A * 1000.0f, (float)lastPwm);
  Serial.printf("phi^         : %.2f mA (límite ±%.0f) | adaptación %s\n",
                (float)phiHat, (float)phiMax, adaptOn ? "ON" : "OFF");
  Serial.println();
  printGains();
  Serial.printf("M2 = %.3f mA*s^2/rad | kc = %.3f\n", (float)M2, (float)KC);
  Serial.printf("Rampa = %.1f grados/s | filtro velocidad = %.1f Hz | zona muerta = %.1f mA\n",
                rampRate * RAD_TO_DEG, (float)velCutHz, (float)iDz);
  Serial.printf("Error máx = %.1f grados (0 = sin límite)\n", errMax * RAD_TO_DEG);
  {
    const char* fName[] = { "OFF", "signo(tau)", "signo(qdot)", "híbrido" };
    Serial.printf("Fricción: FMODE %d (%s) | Fc+ = %.1f mA | Fc- = %.1f mA | tau_f actual = %.1f mA\n",
                  (int)fMode, fName[fMode], (float)FcPos, (float)FcNeg, (float)ffNow);
    Serial.printf("          FEPS = %.1f mA | VEPS = %.3f rad/s | QDB = %.2f grados\n",
                  (float)fEps, (float)vEps, qDb * RAD_TO_DEG);
  }
  Serial.println();
  Serial.printf("PI corriente: Kp = %.4f cuentas/mA | Ki = %.2f cuentas/(mA*s)\n",
                (float)KpI, (float)KiI);
  Serial.printf("Ts interno = %lu us | Ts externo = %.1f ms\n",
                (unsigned long)TS_US, Ts_o * 1000.0f);
  Serial.printf("Ejecución máx = %lu us | periodos perdidos = %lu\n",
                (unsigned long)execMaxUs, (unsigned long)overruns);
  Serial.printf("Errores I2C = %lu | glitches = %lu | reinicios INA226 = %lu\n",
                (unsigned long)i2cErrorsTotal, (unsigned long)glitchCount,
                (unsigned long)inaResets);
  Serial.printf("Encoder: %ld cuentas (%.0f cuentas/vuelta)\n",
                (long)encoderPosition, COUNTS_PER_REV);
  Serial.println("----------------------------------------------");

  execMaxUs = 0;
}

void printHelp()
{
  Serial.println();
  Serial.println("POSICIÓN:");
  Serial.println("  ZERO            -> q = 0 aquí (motor detenido, CARGA COLGANDO)");
  Serial.println("  START           -> control de posición (mantiene la posición actual)");
  Serial.println("  STOP            -> detener (motor libre)");
  Serial.println("  POS <grados>    -> nueva posición objetivo");
  Serial.println("  STEP <grados>   -> escalón con captura y análisis");
  Serial.println("  RAMP <grados/s> -> velocidad máx de la referencia (0 = escalón puro)");
  Serial.println();
  Serial.println("GANANCIAS (tau en mA):");
  Serial.println("  P <mA/rad>  D <mA*s/rad>  K <mA/rad>  G <gamma>");
  Serial.println("  M2 <mA*s^2/rad> -> cota de inercia para verificar gamma");
  Serial.println("  KC <valor>      -> cota de Coriolis (0 para un eslabón)");
  Serial.println("  ADAPT           -> adaptación on/off");
  Serial.println("  PHI <mA>        -> fijar phi^ (ej. PHI 0 para reiniciar)");
  Serial.println("  PHIMAX <mA>     -> límite de |phi^|");
  Serial.println("  VF <Hz>         -> corte del filtro de velocidad");
  Serial.println("  IDZ <mA>        -> compensación de zona muerta (0 = off)");
  Serial.println("  ERRMAX <grados> -> paro por error grande (0 = off)");
  Serial.println();
  Serial.println("FRICCIÓN SECA (adelanto):");
  Serial.println("  FMODE <0-3>     -> 0 off, 1 signo(tau), 2 signo(qdot), 3 híbrido");
  Serial.println("  FC <mA>         -> Fc en ambos sentidos (FCP / FCN por separado)");
  Serial.println("  FEPS <mA>       -> suavizado de signo(tau) (0 = signo puro)");
  Serial.println("  VEPS <rad/s>    -> suavizado de signo(qdot) / umbral de movimiento");
  Serial.println("  QDB <grados>    -> zona sin compensar cerca del objetivo (modo 3)");
  Serial.println("  BREAK [+|-]     -> mide la corriente de despegue (rampa lenta)");
  Serial.println();
  Serial.println("PRUEBAS:");
  Serial.println("  ACEL <mA> <ms>  -> corriente fija desde el reposo: verifica signo y estima m");
  Serial.println("  CORR <mA>       -> solo lazo de corriente (con signo)");
  Serial.println("  CAPDT <ms>      -> periodo de captura (1-20 ms; ventana = 600 muestras)");
  Serial.println("  DUMP            -> reimprimir la última captura");
  Serial.println();
  Serial.println("LAZO DE CORRIENTE:");
  Serial.println("  KPI <cuentas/mA>  KII <cuentas/(mA*s)>  (o R <ohm>, L <mH>, TCI <ms>)");
  Serial.println();
  Serial.println("  MON, STATUS, HELP");
  Serial.println();
  Serial.println("Secuencia sugerida:");
  Serial.println("  1) Carga colgando y quieta -> ZERO");
  Serial.println("  2) ACEL 150 150  y  ACEL -150 150  -> signo OK y M2 sugerido");
  Serial.println("  3) M2 <valor> -> revisa que gamma CUMPLE (STATUS)");
  Serial.println("  4) START -> STEP 45 -> STEP 90 -> STEP 0 ...");
  Serial.println();
}


// ============================================================
// COMANDOS
// ============================================================

bool busy()
{
  if (capActive)
  {
    Serial.println("Captura en curso, espera.");
    return true;
  }
  return false;
}

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

  // ---------------- Posición ----------------
  if (name == "ZERO")
  {
    if (mode != MODE_OFF)
    {
      Serial.println("Primero STOP y espera a que la carga quede quieta.");
      return;
    }
    noInterrupts();
    encoderPosition = 0;
    interrupts();
    Serial.println("q = 0 en esta posición.");
  }
  else if (name == "START")
  {
    if (busy()) return;
    faultCode = FAULT_NONE;
    mode = MODE_POS;
    Serial.println("Control de posición iniciado (mantiene la posición actual).");
  }
  else if (name == "STOP")
  {
    mode = MODE_OFF;
    capActive = false;
    Serial.println("Detenido.");
  }
  else if (name == "POS" && hasArg)
  {
    if (mode != MODE_POS)
    {
      Serial.println("Primero START.");
      return;
    }
    qTarget = val * DEG_TO_RAD;
    Serial.printf("Objetivo = %.2f grados\n", val);
  }
  else if (name == "STEP" && hasArg)
  {
    if (busy()) return;
    if (mode != MODE_POS)
    {
      Serial.println("Primero START.");
      return;
    }
    stepTarget = val * DEG_TO_RAD;
    capType  = CAP_STEP;
    capLen   = CAP_N;
    capIdx   = 0;
    capReady = false;
    capActive = true;
    Serial.printf("Escalón a %.2f grados (captura de %.1f s)...\n",
                  val, CAP_N * capDecim * Ts_o);
  }
  else if (name == "RAMP" && hasArg && val >= 0)
  {
    rampRate = val * DEG_TO_RAD;
    Serial.printf("Rampa = %.1f grados/s %s\n", val, (val == 0) ? "(escalón puro)" : "");
  }

  // ---------------- Ganancias ----------------
  else if (name == "P" && hasArg && val > 0) { Pg = val; printGains(); }
  else if (name == "D" && hasArg && val > 0) { Dg = val; printGains(); }
  else if (name == "K" && hasArg && val >= 0) { Kg = val; printGains(); }
  else if (name == "G" && hasArg && val >= 0) { Gam = val; printGains(); }
  else if (name == "M2" && hasArg && val >= 0) { M2 = val; printGains(); }
  else if (name == "KC" && hasArg && val >= 0) { KC = val; printGains(); }
  else if (name == "ADAPT")
  {
    adaptOn = !adaptOn;
    Serial.println(adaptOn ? "Adaptación ON" : "Adaptación OFF (phi^ congelado)");
  }
  else if (name == "PHI" && hasArg)
  {
    phiHat = val;
    Serial.printf("phi^ = %.2f mA\n", val);
  }
  else if (name == "PHIMAX" && hasArg && val > 0)
  {
    phiMax = val;
    Serial.printf("Límite de phi^ = ±%.1f mA\n", val);
  }
  else if (name == "VF" && hasArg && val > 0)
  {
    velCutHz = constrain(val, 1.0f, 200.0f);
    Serial.printf("Filtro de velocidad = %.1f Hz\n", (float)velCutHz);
  }
  else if (name == "IDZ" && hasArg && val >= 0)
  {
    iDz = constrain(val, 0.0f, 100.0f);
    Serial.printf("Zona muerta = %.1f mA\n", (float)iDz);
  }
  else if (name == "ERRMAX" && hasArg && val >= 0)
  {
    errMax = val * DEG_TO_RAD;
    Serial.printf("Error máx = %.1f grados\n", val);
  }

  // ---------------- Fricción ----------------
  else if (name == "FMODE" && hasArg && val >= 0 && val <= 3)
  {
    fMode = (uint8_t)val;
    const char* fName[] = { "OFF", "signo(tau)", "signo(qdot)", "híbrido" };
    Serial.printf("FMODE %d: %s\n", (int)fMode, fName[fMode]);
  }
  else if (name == "FC" && hasArg && val >= 0)
  {
    FcPos = FcNeg = constrain(val, 0.0f, 300.0f);
    Serial.printf("Fc+ = Fc- = %.1f mA\n", (float)FcPos);
  }
  else if (name == "FCP" && hasArg && val >= 0)
  {
    FcPos = constrain(val, 0.0f, 300.0f);
    Serial.printf("Fc+ = %.1f mA\n", (float)FcPos);
  }
  else if (name == "FCN" && hasArg && val >= 0)
  {
    FcNeg = constrain(val, 0.0f, 300.0f);
    Serial.printf("Fc- = %.1f mA\n", (float)FcNeg);
  }
  else if (name == "FEPS" && hasArg && val >= 0)
  {
    fEps = val;
    Serial.printf("FEPS = %.1f mA\n", val);
  }
  else if (name == "VEPS" && hasArg && val >= 0)
  {
    vEps = val;
    Serial.printf("VEPS = %.3f rad/s\n", val);
  }
  else if (name == "QDB" && hasArg && val >= 0)
  {
    qDb = val * DEG_TO_RAD;
    Serial.printf("QDB = %.2f grados\n", val);
  }
  else if (name == "BREAK")
  {
    if (busy()) return;
    if (mode != MODE_OFF)
    {
      Serial.println("Primero STOP y espera a que la carga quede quieta.");
      return;
    }
    breakDir = (args.length() && args.charAt(0) == '-') ? -1 : 1;
    breakDone = false;
    faultCode = FAULT_NONE;
    mode = MODE_BREAK;
    Serial.printf("BREAK %s: subiendo corriente a %.0f mA/s hasta que se mueva...\n",
                  (breakDir > 0) ? "+" : "-", BREAK_RATE_MA_S);
  }

  // ---------------- Pruebas ----------------
  else if (name == "CORR" && hasArg)
  {
    if (busy()) return;
    iRef = constrain(val, -TAU_MAX_MA, TAU_MAX_MA) / 1000.0f;
    faultCode = FAULT_NONE;
    mode = MODE_CURR;
    Serial.printf("Solo corriente: %.1f mA (STOP para detener)\n", iRef * 1000.0f);
  }
  else if (name == "ACEL" && hasArg)
  {
    if (busy()) return;
    if (mode != MODE_OFF)
    {
      Serial.println("Primero STOP y espera a que la carga quede quieta.");
      return;
    }

    int sp2 = args.indexOf(' ');
    float mA = args.toFloat();
    int ms = (sp2 > 0) ? args.substring(sp2 + 1).toInt() : 150;
    ms = constrain(ms, 20, CAP_N);

    acelCurrent = constrain(mA, -TAU_MAX_MA, TAU_MAX_MA);
    capDecim = 1;                       // 1 ms por muestra
    capType  = CAP_ACEL;
    capLen   = ms;
    capIdx   = 0;
    capReady = false;
    iRef = acelCurrent / 1000.0f;
    faultCode = FAULT_NONE;
    capActive = true;
    mode = MODE_CURR;
    Serial.printf("ACEL: %.0f mA durante %d ms...\n", (float)acelCurrent, ms);
  }
  else if (name == "CAPDT" && hasArg && val >= 1)
  {
    if (busy()) return;
    capDecim = constrain((int)val, 1, 20);
    Serial.printf("Captura cada %d ms (ventana %.1f s)\n",
                  (int)capDecim, CAP_N * capDecim * Ts_o);
  }
  else if (name == "DUMP")
  {
    printCapture(capType == CAP_STEP ? CAP_PRE : 0, capIdx);
  }

  // ---------------- Lazo de corriente ----------------
  else if (name == "KPI" && hasArg && val >= 0) { KpI = val; Serial.printf("KpI = %.4f\n", val); }
  else if (name == "KII" && hasArg && val >= 0) { KiI = val; Serial.printf("KiI = %.2f\n", val); }
  else if (name == "R" && hasArg && val > 0)
  {
    R_motor = val; retuneInner();
    Serial.printf("R = %.3f | KpI = %.4f | KiI = %.2f\n", val, (float)KpI, (float)KiI);
  }
  else if (name == "L" && hasArg && val > 0)
  {
    L_motor = val * 1e-3f; retuneInner();
    Serial.printf("L = %.3f mH | KpI = %.4f | KiI = %.2f\n", val, (float)KpI, (float)KiI);
  }
  else if (name == "TCI" && hasArg && val > 0)
  {
    tau_ci = constrain(val, 0.2f, 20.0f) * 1e-3f; retuneInner();
    Serial.printf("tau_c corriente = %.3f ms | KpI = %.4f | KiI = %.2f\n",
                  tau_ci * 1000.0f, (float)KpI, (float)KiI);
  }

  // ---------------- Otros ----------------
  else if (name == "MON")
  {
    monitorOn = !monitorOn;
    Serial.println(monitorOn ? "Monitor ON" : "Monitor OFF");
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
  Serial.begin(115200);
  Serial.setTimeout(20);
  delay(200);

  // Puente H
  pinMode(ENA, OUTPUT);
  digitalWrite(ENA, LOW);

  if (!ledcAttach(IN1, PWM_FREQUENCY, PWM_RESOLUTION) ||
      !ledcAttach(IN2, PWM_FREQUENCY, PWM_RESOLUTION))
  {
    halt("ERROR: no se pudo configurar PWM");
  }
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

  // Ganancias del PI de corriente
  retuneInner();

  // Tarea de control (núcleo 1, prioridad máxima) + timer 4 kHz
  xTaskCreatePinnedToCore(controlTask, "ctrl", 4096, nullptr,
                          configMAX_PRIORITIES - 1, &controlTaskHandle, 1);

  controlTimer = timerBegin(1000000);
  timerAttachInterrupt(controlTimer, &onControlTimer);
  timerAlarm(controlTimer, TS_US, true, 0);

  Serial.println();
  Serial.println("==============================================");
  Serial.println("   CONTROL DE POSICIÓN ADAPTABLE + FRICCIÓN");
  Serial.println("   externo 1 kHz  |  corriente 4 kHz");
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

  // ---------------- Fallas ----------------
  uint8_t f = faultCode;
  if (f != FAULT_NONE)
  {
    faultCode = FAULT_NONE;
    Serial.println();

    switch (f)
    {
      case FAULT_OVERCURRENT:
        Serial.printf("*** FALLA: sobrecorriente (%.1f mA). ***\n", (float)faultValue);
        break;
      case FAULT_I2C:
        Serial.println("*** FALLA: comunicación I2C con INA226. ***");
        break;
      case FAULT_ADC_SAT:
        Serial.printf("*** FALLA: INA226 saturado (%.1f mA). ***\n", (float)faultValue);
        break;
      case FAULT_REGEN:
        Serial.printf("*** FALLA: corriente negativa sostenida (%.1f mA). ***\n", (float)faultValue);
        break;
      case FAULT_POS_ERROR:
        Serial.printf("*** FALLA: error de posición de %.1f grados (límite %.1f). ***\n",
                      faultValue * RAD_TO_DEG, errMax * RAD_TO_DEG);
        Serial.println("    Si el motor se fue en la dirección contraria: revisa ENCODER_SIGN (ACEL).");
        break;
    }
    Serial.println("    Motor detenido. phi^ se conserva (PHI 0 para reiniciarlo).");
  }

  // ---------------- Resultado de BREAK ----------------
  if (breakDone)
  {
    breakDone = false;
    float r = breakResult;
    if (r < 0.0f)
      Serial.printf("BREAK: no se movió ni con %.0f mA.\n", TAU_MAX_MA);
    else
    {
      Serial.printf("BREAK %s: despegó con %.1f mA\n", (breakDir > 0) ? "+" : "-", r);
      Serial.printf("Sugerencia: %s %.0f  (~80%% del despegue, la fricción en movimiento es menor)\n",
                    (breakDir > 0) ? "FCP" : "FCN", r * 0.8f);
    }
  }

  // ---------------- Capturas ----------------
  if (capReady)
  {
    capReady = false;
    int n = capIdx;

    if (capType == CAP_ACEL)
    {
      printCapture(0, n);
      if (n >= 20) analyzeAcel(n);
      capDecim = 5;                     // vuelve a la decimación normal
    }
    else if (n < CAP_N)
    {
      Serial.println("Captura incompleta (interrumpida):");
      printCapture(CAP_PRE, n);
    }
    else
    {
      printCapture(CAP_PRE, n);
      analyzeStep();
    }
  }

  // ---------------- Monitor 20 Hz (Serial Plotter) ----------------
  if (monitorOn && mode != MODE_OFF && !capActive && millis() - lastMon >= 50)
  {
    lastMon = millis();

    if (mode == MODE_POS)
    {
      Serial.printf("qd:%.2f,q:%.2f,tau:%.1f,I:%.1f,phi:%.2f,tf:%.1f\n",
                    qdNow * RAD_TO_DEG, qNow * RAD_TO_DEG,
                    (float)tauNow, lastI_A * 1000.0f, (float)phiHat, (float)ffNow);
    }
    else
    {
      Serial.printf("Iref:%.1f,I:%.1f,q:%.2f\n",
                    iRef * 1000.0f, lastI_A * 1000.0f, qNow * RAD_TO_DEG);
    }
  }

  vTaskDelay(1);
}
