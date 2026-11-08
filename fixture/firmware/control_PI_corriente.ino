/*
 * ============================================================
 *  CONTROL PI DE CORRIENTE - MOTOR NUEVO (4 kHz)
 *  ESP32 DevKit V1 + INA226 (antes del L298N) + L298N
 * ============================================================
 *
 *  Planta identificada (01_identificacion_corriente, rotor bloqueado):
 *     i = mapa(u) a través de  e^(-theta s)/(tau s + 1)
 *     tau = 0.985 ms, theta = 0.366 ms, mapa(u) ~ cuadrático
 *
 *  Estructura (cada parte se activa/desactiva por comando):
 *
 *   ref -> [modelo de referencia τd] -> rd --> [inversa dinámica] -> w_ff -+
 *                    |                                                    |
 *                    +-> rd retrasada θ ->(+)- e -> PI -> w_pi ->(+)<-----+
 *                                          ^(-)                 | w [mA]
 *                                          i (INA226)           v
 *                                                     LIN: u = mapa^-1(w)
 *                                                               | PWM
 *                                                               v L298N
 *
 *  - LIN ON : el PI calcula w en mA y la inversa del mapa estático da el
 *             PWM. La planta vista por el PI queda con ganancia 1 en todo
 *             el rango (sin LIN la ganancia cambia ~3.6x de PWM 200 a 600).
 *             LIN OFF: PI clásico, salida directa en PWM (Kp en PWM/mA).
 *  - FF ON  : 2 grados de libertad. Se genera una trayectoria rd (primer
 *             orden con τd) y se calcula el w que la produce (inversa del
 *             modelo). El PI solo corrige el error contra rd retrasada θ.
 *             Necesita LIN ON. Con FF OFF el PI ve el escalón directo.
 *  - TI 0   : sin integral (control P, para el ensayo de Ziegler-Nichols
 *             en lazo cerrado).
 *
 *  La dirección la da el signo de la referencia. El INA226 mide magnitud.
 *
 *  COMANDOS (921600 baud, "Nueva línea")
 *   HELP | STATUS | REF <mA> | STOP | PWM <n> (lazo abierto)
 *   KP <v> | TI <ms> (0 = sin integral) | LIN ON|OFF | FF ON|OFF
 *   TAUD <ms> | TAU <ms> | THETA <ms> | IMAX <mA>
 *   SET PRE|POST|SETTLE <ms>
 *   ESCALON <a> <b>   referencia a (SETTLE) -> registro PRE -> b (POST)
 *                     -> referencia 0. a y b en mA, mismo signo.
 *
 *  SALIDA DEL ESCALÓN
 *   BEGIN_PI
 *   # kp= ti_ms= lin= ff= taud_ms= tau_ms= theta_ms= a= b= ts_us= n_pre= n= trip= err_i2c= overruns=
 *   t_ms,ref,rd,i,w,u          (ref, rd, i, w en mA; u = PWM con signo)
 *   END_PI
 *
 *  Requiere Arduino-ESP32 core 3.x.
 * ============================================================
 */

#include <Arduino.h>
#include <Wire.h>
#include "mapa_corriente.h"

// ============================================================
// HARDWARE
// ============================================================
const uint32_t SERIAL_BAUD = 921600;
const int IN1 = 25, IN2 = 26, ENA = 27;
const int I2C_SDA = 21, I2C_SCL = 22;

const uint8_t  INA226_ADDR  = 0x40;
const uint32_t I2C_CLOCK_HZ = 400000;
const float    LSB_MA       = 2.5e-3f / 0.1f;      // 0.025 mA por cuenta
const uint8_t  REG_CONFIG = 0x00, REG_SHUNT = 0x01, REG_MANUF_ID = 0xFE;
const uint16_t INA226_CONFIG_FAST = 0x4005;        // 140 us, sin promedio, solo shunt

const uint32_t PWM_FREQUENCY  = 21429;
const uint8_t  PWM_RESOLUTION = 10;
const float    PWM_MAX        = 1023.0f;

const uint32_t TS_US = 250;
const float    Ts    = TS_US * 1e-6f;

// ============================================================
// PARÁMETROS (por defecto: PI SIMC Kp 1.00, Ti 0.985 ms, LIN ON, FF OFF)
// ============================================================
volatile float Kp      = 1.00f;     // LIN ON: [mA/mA]   LIN OFF: [PWM/mA]
volatile float TiMs    = 0.985f;    // tiempo integral [ms], 0 = sin integral
volatile bool  linOn   = true;
volatile bool  ffOn    = false;    // elegido: SIMC solo (PI + LIN, sin FF)
volatile float tauDMs  = 0.10f;     // constante de la trayectoria deseada
volatile float tauPMs  = 0.985f;    // tau de la planta (para la inversa)
volatile float thetaMs = 0.366f;    // retardo de la planta
volatile float iMaxMa  = 700.0f;    // protección

uint32_t preMs = 2, postMs = 10, settleMs = 200;

// ============================================================
// ESTADO
// ============================================================
enum { MODE_OFF = 0, MODE_PI = 1, MODE_OL = 2 };
volatile int   mode   = MODE_OFF;
volatile float iRef   = 0.0f;       // mA con signo
volatile int   olPwm  = 0;
volatile float lastI  = 0.0f;
volatile float lastW  = 0.0f;
volatile int   lastU  = 0;
volatile bool  trip   = false;
volatile float tripI  = 0.0f;
volatile uint32_t errI2C = 0, overruns = 0;
volatile bool  resetCtl = true;     // pide reiniciar integrador y trayectoria

// Captura
struct Muestra { float r, rd, i, w; int16_t u; };
const int CAP_MAX = 2400;
Muestra cap[CAP_MAX];
enum { CAP_IDLE = 0, CAP_SETTLE, CAP_REC, CAP_DONE };
volatile int capState = CAP_IDLE;
volatile float capA = 0, capB = 0;
volatile int capN = 0, capPre = 0, capSettle = 0, capK = 0;
volatile bool capTrip = false;
uint32_t capErr0 = 0, capOv0 = 0;

TaskHandle_t taskHandle = nullptr;
hw_timer_t* timer = nullptr;

// ============================================================
// INA226
// ============================================================
bool inaWriteReg(uint8_t reg, uint16_t v)
{
  Wire.beginTransmission(INA226_ADDR);
  Wire.write(reg); Wire.write((uint8_t)(v >> 8)); Wire.write((uint8_t)(v & 0xFF));
  return Wire.endTransmission() == 0;
}
bool inaReadReg(uint8_t reg, uint16_t& v)
{
  Wire.beginTransmission(INA226_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission() != 0) return false;
  if (Wire.requestFrom(INA226_ADDR, (uint8_t)2) != 2) return false;
  uint8_t hi = Wire.read(), lo = Wire.read();
  v = ((uint16_t)hi << 8) | lo;
  return true;
}
bool inaSetPointer(uint8_t reg)
{
  Wire.beginTransmission(INA226_ADDR);
  Wire.write(reg);
  return Wire.endTransmission() == 0;
}
int16_t rawPrev = 0;
int16_t inaShunt()
{
  if (Wire.requestFrom(INA226_ADDR, (uint8_t)2) != 2)
  {
    errI2C++;
    inaSetPointer(REG_SHUNT);
    return rawPrev;
  }
  uint8_t hi = Wire.read(), lo = Wire.read();
  rawPrev = (int16_t)(((uint16_t)hi << 8) | lo);
  return rawPrev;
}
bool inaInit()
{
  uint16_t id;
  if (!inaReadReg(REG_MANUF_ID, id) || id != 0x5449) return false;
  inaWriteReg(REG_CONFIG, 0x8000);
  delay(2);
  if (!inaWriteReg(REG_CONFIG, INA226_CONFIG_FAST)) return false;
  uint16_t cfg;
  if (!inaReadReg(REG_CONFIG, cfg) || cfg != INA226_CONFIG_FAST) return false;
  return inaSetPointer(REG_SHUNT);
}

// ============================================================
// MAPA ESTÁTICO
// ============================================================
inline const float* mapaDir(int dir) { return (dir >= 0) ? MAP_I_POS : MAP_I_NEG; }

// u = mapa^-1(w): interpolación lineal en la tabla
float mapaInv(float w, int dir)
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
// MOTOR
// ============================================================
void puente(float u, int dir)
{
  uint32_t c = (uint32_t)constrain(u, 0.0f, PWM_MAX);
  digitalWrite(ENA, HIGH);
  if (dir >= 0) { ledcWrite(IN2, 0); ledcWrite(IN1, c); }
  else          { ledcWrite(IN1, 0); ledcWrite(IN2, c); }
}
void motorStop()
{
  ledcWrite(IN1, 0); ledcWrite(IN2, 0);
  digitalWrite(ENA, LOW);
}

// ============================================================
// LAZO DE CONTROL (tarea a 4 kHz en el núcleo 1)
// ============================================================
void ARDUINO_ISR_ATTR onTimer()
{
  BaseType_t w = pdFALSE;
  vTaskNotifyGiveFromISR(taskHandle, &w);
  if (w) portYIELD_FROM_ISR();
}

void controlTask(void*)
{
  float integ = 0.0f;
  int   dirNow = 1;
  float rk = 0.0f, rnext = 0.0f;
  const int RB = 16;
  float rb[RB] = {0};
  int   rbPos = 0;
  uint8_t sobre = 0;

  for (;;)
  {
    uint32_t pend = ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    if (pend > 1) overruns += pend - 1;

    // ---------- 1) medir ----------
    float y = inaShunt() * LSB_MA;
    lastI = y;

    // ---------- 2) protección ----------
    sobre = (fabsf(y) > iMaxMa) ? sobre + 1 : 0;
    if (sobre >= 2 && mode != MODE_OFF)
    {
      mode = MODE_OFF; trip = true; tripI = y;
      if (capState == CAP_SETTLE || capState == CAP_REC) { capTrip = true; capState = CAP_DONE; }
    }

    // ---------- 3) secuencia de captura ----------
    if (capState == CAP_SETTLE)
    {
      iRef = capA;
      if (--capSettle <= 0) { capState = CAP_REC; capK = 0; }
    }
    else if (capState == CAP_REC && capK == capPre)
    {
      iRef = capB;
    }

    // ---------- 4) control ----------
    float r = iRef, rdel = 0.0f, w = 0.0f, u = 0.0f;
    int m = mode;

    if (m == MODE_PI)
    {
      int dir = (r > 0.0f) ? 1 : ((r < 0.0f) ? -1 : dirNow);
      if (dir != dirNow || resetCtl)
      {
        integ = 0.0f;
        rk = rnext = y;                          // la trayectoria arranca donde está
        for (int j = 0; j < RB; j++) rb[j] = y;
        dirNow = dir;
        resetCtl = false;
      }
      const float* I = mapaDir(dirNow);
      float wMin = linOn ? I[0] : 0.0f;
      float wMax = linOn ? I[MAP_N - 1] : PWM_MAX;
      float rMag = fabsf(r);

      if (rMag <= 0.0f)
      {
        integ = 0.0f; rk = rnext = y; w = 0.0f; u = 0.0f; rdel = 0.0f;
        motorStop();
      }
      else
      {
        float wff = 0.0f;
        bool ff = ffOn && linOn;
        if (ff)
        {
          // trayectoria deseada rd (primer orden, τd) y su inversa exacta
          // (ZOH): w[k] = (rd[k+1] - ad rd[k]) / (1 - ad)
          float a  = expf(-Ts / (tauDMs * 1e-3f));
          float ad = expf(-Ts / (tauPMs * 1e-3f));
          rk    = rnext;
          rnext = a * rk + (1.0f - a) * rMag;
          wff   = (rnext - ad * rk) / (1.0f - ad);
          float wffMax = 0.9f * wMax;            // margen para que el PI corrija
          if (wff > wffMax) wff = wffMax;
          if (wff < wMin)   wff = wMin;
          rnext = ad * rk + (1.0f - ad) * wff;   // trayectoria alcanzable

          // rd retrasada θ (interpolación entre muestras)
          rbPos = (rbPos + 1) % RB;
          rb[rbPos] = rk;
          float d  = thetaMs * 1e-3f / Ts;
          int   d0 = (int)d;
          if (d0 > RB - 2) d0 = RB - 2;
          float fr = d - d0;
          float r0 = rb[(rbPos - d0 + RB) % RB];
          float r1 = rb[(rbPos - d0 - 1 + RB) % RB];
          rdel = (1.0f - fr) * r0 + fr * r1;
        }
        else
        {
          rdel = rMag;
        }

        float e  = rdel - y;
        float ki = (TiMs > 0.0f) ? Kp / (TiMs * 1e-3f) : 0.0f;
        float integCand = integ + ki * Ts * e;
        float wUn = wff + Kp * e + integCand;
        bool windup = (wUn > wMax && e > 0.0f) || (wUn < wMin && e < 0.0f);
        if (!windup) integ = constrain(integCand, -wMax, wMax);
        if (ki == 0.0f) integ = 0.0f;

        w = constrain(wff + Kp * e + integ, wMin, wMax);
        u = linOn ? mapaInv(w, dirNow) : w;
        puente(u, dirNow);
      }
      u *= dirNow;
    }
    else if (m == MODE_OL)
    {
      int n = olPwm;
      puente(abs(n), (n >= 0) ? 1 : -1);
      u = n; r = 0; rdel = 0;
    }
    else
    {
      motorStop();
      resetCtl = true;
    }
    lastW = w; lastU = (int)u;

    // ---------- 5) registro ----------
    if (capState == CAP_REC)
    {
      Muestra& s = cap[capK];
      s.r = fabsf(r); s.rd = rdel; s.i = y; s.w = w; s.u = (int16_t)u;
      if (++capK >= capN)
      {
        iRef = 0.0f;                             // termina en reposo (rotor bloqueado se calienta)
        capState = CAP_DONE;
      }
    }
  }
}

// ============================================================
// COMANDOS
// ============================================================
void status()
{
  Serial.printf("Modo %s | REF %.1f mA | I %.1f mA | w %.1f | u %d | trip %d (%.0f mA)\n",
                mode == MODE_PI ? "PI" : (mode == MODE_OL ? "PWM" : "OFF"),
                iRef, lastI, lastW, lastU, trip, tripI);
  Serial.printf("KP %.4f | TI %.4f ms (Ki %.1f) | LIN %s | FF %s | TAUD %.3f ms | TAU %.3f ms | THETA %.3f ms | IMAX %.0f\n",
                Kp, TiMs, TiMs > 0 ? Kp / (TiMs * 1e-3f) : 0.0f, linOn ? "ON" : "OFF", ffOn ? "ON" : "OFF",
                tauDMs, tauPMs, thetaMs, iMaxMa);
  Serial.printf("PRE %lu ms | POST %lu ms | SETTLE %lu ms | errores I2C %lu | overruns %lu\n",
                (unsigned long)preMs, (unsigned long)postMs, (unsigned long)settleMs,
                (unsigned long)errI2C, (unsigned long)overruns);
}

void ayuda()
{
  Serial.println("HELP | STATUS | REF <mA> | STOP | PWM <n> | KP <v> | TI <ms> | LIN ON|OFF | FF ON|OFF");
  Serial.println("TAUD <ms> | TAU <ms> | THETA <ms> | IMAX <mA> | SET PRE|POST|SETTLE <ms> | ESCALON <a> <b>");
}

void enviarCaptura()
{
  Serial.println("BEGIN_PI");
  Serial.printf("# kp=%.5f ti_ms=%.4f lin=%d ff=%d taud_ms=%.4f tau_ms=%.4f theta_ms=%.4f a=%.1f b=%.1f "
                "ts_us=%lu n_pre=%d n=%d trip=%d err_i2c=%lu overruns=%lu\n",
                Kp, TiMs, linOn, ffOn, tauDMs, tauPMs, thetaMs, capA, capB,
                (unsigned long)TS_US, capPre, capK, capTrip,
                (unsigned long)(errI2C - capErr0), (unsigned long)(overruns - capOv0));
  Serial.println("t_ms,ref,rd,i,w,u");
  for (int k = 0; k < capK; k++)
  {
    const Muestra& s = cap[k];
    Serial.printf("%.3f,%.2f,%.2f,%.2f,%.2f,%d\n", (float)(k - capPre) * (TS_US * 1e-3f),   // float: (k - capPre) es negativo antes del escalón
                  s.r, s.rd, s.i, s.w, s.u);
  }
  Serial.println("END_PI");
  if (capTrip) Serial.printf("*** ABORTADO: sobrecorriente %.1f mA ***\n", tripI);
  capState = CAP_IDLE;
}

void procesar(String s)
{
  s.trim();
  if (!s.length()) return;
  s.toUpperCase();
  char cmd[16] = {0}, arg[16] = {0};
  float v1 = 0, v2 = 0;
  int n = sscanf(s.c_str(), "%15s %f %f", cmd, &v1, &v2);
  sscanf(s.c_str(), "%15s %15s %f", cmd, arg, &v2);

  if (!strcmp(cmd, "HELP"))   { ayuda(); return; }
  if (!strcmp(cmd, "STATUS")) { status(); return; }
  if (!strcmp(cmd, "STOP"))   { mode = MODE_OFF; iRef = 0; capState = CAP_IDLE; Serial.println("OK STOP"); return; }

  if (capState != CAP_IDLE) { Serial.println("ERROR: captura en curso"); return; }

  if (!strcmp(cmd, "REF") && n >= 2)
  {
    trip = false; iRef = v1; mode = MODE_PI;
    Serial.printf("OK REF %.1f mA\n", v1); return;
  }
  if (!strcmp(cmd, "PWM") && n >= 2)
  {
    trip = false; olPwm = constrain((int)v1, -1023, 1023); mode = MODE_OL;
    delay(200); Serial.printf("OK PWM %d -> I %.1f mA\n", olPwm, lastI); return;
  }
  if (!strcmp(cmd, "KP") && n >= 2)    { Kp = max(v1, 0.0f); status(); return; }
  if (!strcmp(cmd, "TI") && n >= 2)    { TiMs = max(v1, 0.0f); resetCtl = true; status(); return; }
  if (!strcmp(cmd, "TAUD") && n >= 2)  { tauDMs = constrain(v1, 0.02f, 50.0f); status(); return; }
  if (!strcmp(cmd, "TAU") && n >= 2)   { tauPMs = constrain(v1, 0.05f, 50.0f); status(); return; }
  if (!strcmp(cmd, "THETA") && n >= 2) { thetaMs = constrain(v1, 0.0f, 3.5f); status(); return; }
  if (!strcmp(cmd, "IMAX") && n >= 2)  { iMaxMa = constrain(v1, 50.0f, 800.0f); status(); return; }
  if (!strcmp(cmd, "LIN"))
  {
    linOn = !strcmp(arg, "ON"); resetCtl = true;
    Serial.println("OJO: KP cambia de unidades (LIN ON: mA/mA, LIN OFF: PWM/mA). Revisa KP/TI.");
    status(); return;
  }
  if (!strcmp(cmd, "FF"))  { ffOn = !strcmp(arg, "ON"); resetCtl = true; status(); return; }
  if (!strcmp(cmd, "SET"))
  {
    if      (!strcmp(arg, "PRE"))    preMs    = (uint32_t)v2;
    else if (!strcmp(arg, "POST"))   postMs   = (uint32_t)v2;
    else if (!strcmp(arg, "SETTLE")) settleMs = (uint32_t)v2;
    else { Serial.println("ERROR: SET PRE|POST|SETTLE"); return; }
    status(); return;
  }
  if (!strcmp(cmd, "ESCALON") && n >= 3)
  {
    if ((v1 < 0 && v2 > 0) || (v1 > 0 && v2 < 0)) { Serial.println("ERROR: a y b del mismo signo"); return; }
    int pre  = preMs * 1000 / TS_US;
    int ntot = pre + postMs * 1000 / TS_US;
    if (ntot > CAP_MAX) { Serial.printf("ERROR: %d muestras > %d\n", ntot, CAP_MAX); return; }
    capA = v1; capB = v2; capPre = pre; capN = ntot;
    capSettle = settleMs * 1000 / TS_US;
    capTrip = false; trip = false;
    capErr0 = errI2C; capOv0 = overruns;
    iRef = v1; resetCtl = true; mode = MODE_PI;
    capState = CAP_SETTLE;
    return;
  }
  Serial.println("ERROR: comando no reconocido (HELP)");
}

// ============================================================
// SETUP / LOOP
// ============================================================
void setup()
{
  Serial.begin(SERIAL_BAUD);
  delay(300);
  pinMode(ENA, OUTPUT);
  digitalWrite(ENA, LOW);
  if (!ledcAttach(IN1, PWM_FREQUENCY, PWM_RESOLUTION) || !ledcAttach(IN2, PWM_FREQUENCY, PWM_RESOLUTION))
  { Serial.println("ERROR: ledcAttach"); while (true) delay(1000); }
  motorStop();

  Wire.begin(I2C_SDA, I2C_SCL, I2C_CLOCK_HZ);
  Wire.setTimeOut(5);
  if (!inaInit()) { Serial.println("ERROR: INA226"); while (true) delay(1000); }

  xTaskCreatePinnedToCore(controlTask, "ctrl", 4096, nullptr, configMAX_PRIORITIES - 1, &taskHandle, 1);
  timer = timerBegin(1000000);
  timerAttachInterrupt(timer, &onTimer);
  timerAlarm(timer, TS_US, true, 0);

  Serial.println();
  Serial.println("=== Control PI de corriente (motor nuevo, 4 kHz) ===");
  ayuda();
  status();
  Serial.println("LISTO");
}

void loop()
{
  static String linea;
  if (capState == CAP_DONE) enviarCaptura();
  while (Serial.available())
  {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') { if (linea.length()) procesar(linea); linea = ""; }
    else if (linea.length() < 80) linea += c;
  }
  delay(1);
}
