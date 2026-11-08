/*
 * ============================================================
 *  IDENTIFICACIÓN SIMPLE PWM -> CORRIENTE (MOTOR NUEVO)
 *  ESP32 DevKit V1 + INA226 + L298N
 * ============================================================
 *
 *  Objetivo: obtener UNA función de transferencia de primer orden
 *
 *            I(s)        K
 *          -------- = --------- e^(-theta s)
 *           U(s)       tau s + 1
 *
 *  u = PWM (cuentas, 0..1023), i = corriente (mA).
 *
 *  Prueba: ROTOR BLOQUEADO (sujeta el eje). Así no hay fuerza
 *  contraelectromotriz y el motor es solo R-L: la respuesta a un
 *  escalón de PWM es una exponencial de primer orden.
 *
 *  El escalón va de un PWM "a" a un PWM "b" (los dos fuera de la zona
 *  muerta). Se usa el CAMBIO de corriente (Δi / Δu), así no importan el
 *  consumo propio del L298N (~30-40 mA) ni la zona muerta.
 *
 *  Cada escalón se repite N veces y se promedia (quita ruido del INA226).
 *
 *  ------------------------------------------------------------
 *  COMANDOS (monitor serie a 921600, fin de línea "Nueva línea")
 *  ------------------------------------------------------------
 *   HELP                 lista de comandos
 *   I                    corriente actual (promedio de 200 lecturas)
 *   PWM <n>              lazo abierto fijo, n en [-1023, 1023]
 *   STOP                 apaga el puente
 *   ESTATICA <a> <b> <p> barrido estático: PWM de a a b en pasos de p,
 *                        corriente promedio en cada punto (para ver zona
 *                        muerta y elegir los niveles del escalón)
 *   ESCALON <a> <b> [N]  escalón a -> b, N repeticiones (por defecto 10)
 *                        a y b del mismo signo (+ = IN1, - = IN2)
 *   SET <param> <valor>  TS (us), PRE (ms), POST (ms), SETTLE (ms),
 *                        REST (ms), IMAX (mA), HOLD (ms, ESTATICA)
 *   CONFIG               muestra los parámetros
 *
 *  ------------------------------------------------------------
 *  SALIDA DEL ESCALÓN (la lee MATLAB: adquirir_escalon.m)
 *  ------------------------------------------------------------
 *   BEGIN_ESCALON
 *   # a=<a> b=<b> N=<N> Ts_us=<Ts> n_pre=<n> n=<n> err_i2c=<e> tarde_max_us=<t>
 *   k,t_ms,pwm,i_mA
 *   ...
 *   END_ESCALON
 *
 *   t_ms = 0 es el instante en que se aplica el escalón.
 *   i_mA = corriente TAL CUAL la da el INA226 (magnitud de la corriente
 *          de la fuente), promedio de las N repeticiones.
 *
 *  Requiere Arduino-ESP32 core 3.x (ledcAttach).
 * ============================================================
 */

#include <Arduino.h>
#include <Wire.h>

// ============================================================
// HARDWARE
// ============================================================
const uint32_t SERIAL_BAUD = 921600;

const int IN1 = 25;          // PWM adelante
const int IN2 = 26;          // PWM atrás
const int ENA = 27;          // HIGH = puente activo

const int I2C_SDA = 21;
const int I2C_SCL = 22;

const uint8_t  INA226_ADDR  = 0x40;
const uint32_t I2C_CLOCK_HZ = 400000;
const float    R_SHUNT      = 0.1f;
const float    LSB_MA       = 2.5e-3f / R_SHUNT;   // 0.025 mA por cuenta

const uint8_t  REG_CONFIG   = 0x00;
const uint8_t  REG_SHUNT    = 0x01;
const uint8_t  REG_MANUF_ID = 0xFE;
const uint8_t  REG_DIE_ID   = 0xFF;
const uint16_t INA226_CONFIG_FAST = 0x4005;        // AVG=1, 140 us, solo shunt continuo

const uint32_t PWM_FREQUENCY  = 21429;
const uint8_t  PWM_RESOLUTION = 10;
const int      PWM_MAX        = 1023;

// ============================================================
// PARÁMETROS DEL EXPERIMENTO (se cambian con SET)
// ============================================================
uint32_t tsUs     = 250;     // periodo de muestreo (4 kHz, igual que el lazo PI)
uint32_t preMs    = 10;      // registro antes del escalón
uint32_t postMs   = 40;      // registro después del escalón (>= 10 tau)
uint32_t settleMs = 300;     // tiempo en PWM "a" antes de registrar
uint32_t restMs   = 400;     // reposo con PWM 0 entre repeticiones (enfría)
uint32_t holdMs   = 300;     // tiempo por punto en ESTATICA
float    iMaxMa   = 700.0f;  // protección (el INA226 R100 satura en 819 mA)

const int MAX_SAMPLES = 4000;
int32_t   acum[MAX_SAMPLES];       // suma de cuentas crudas de las N repeticiones

// ============================================================
// ESTADO
// ============================================================
int      pwmActual  = 0;
uint32_t errI2C     = 0;
int16_t  ultimoRaw  = 0;

// ============================================================
// INA226
// ============================================================
bool inaWriteReg(uint8_t reg, uint16_t v)
{
  Wire.beginTransmission(INA226_ADDR);
  Wire.write(reg);
  Wire.write((uint8_t)(v >> 8));
  Wire.write((uint8_t)(v & 0xFF));
  return Wire.endTransmission() == 0;
}

bool inaReadReg(uint8_t reg, uint16_t &v)
{
  Wire.beginTransmission(INA226_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission() != 0) return false;
  if (Wire.requestFrom(INA226_ADDR, (uint8_t)2) != 2) return false;
  uint8_t hi = Wire.read();
  uint8_t lo = Wire.read();
  v = ((uint16_t)hi << 8) | lo;
  return true;
}

bool inaSetPointer(uint8_t reg)
{
  Wire.beginTransmission(INA226_ADDR);
  Wire.write(reg);
  return Wire.endTransmission() == 0;
}

// Lectura rápida del shunt (el puntero ya apunta a REG_SHUNT).
// Si falla, regresa la última lectura buena y cuenta el error.
int16_t inaShunt()
{
  if (Wire.requestFrom(INA226_ADDR, (uint8_t)2) != 2)
  {
    errI2C++;
    inaSetPointer(REG_SHUNT);
    return ultimoRaw;
  }
  uint8_t hi = Wire.read();
  uint8_t lo = Wire.read();
  ultimoRaw = (int16_t)(((uint16_t)hi << 8) | lo);
  return ultimoRaw;
}

bool inaInit()
{
  uint16_t id;
  if (!inaReadReg(REG_MANUF_ID, id) || id != 0x5449) return false;
  inaWriteReg(REG_CONFIG, 0x8000);                // reset
  delay(2);
  if (!inaWriteReg(REG_CONFIG, INA226_CONFIG_FAST)) return false;
  uint16_t cfg;
  if (!inaReadReg(REG_CONFIG, cfg) || cfg != INA226_CONFIG_FAST) return false;
  return inaSetPointer(REG_SHUNT);
}

float leerCorrientePromedio(int n)
{
  int32_t s = 0;
  for (int k = 0; k < n; k++)
  {
    s += inaShunt();
    delayMicroseconds(150);                       // > tiempo de conversión
  }
  return (s * LSB_MA) / n;
}

// ============================================================
// MOTOR (slow decay: IN activo con PWM, el otro en 0, ENA = 1)
// ============================================================
void motorPWM(int u)
{
  u = constrain(u, -PWM_MAX, PWM_MAX);
  pwmActual = u;
  digitalWrite(ENA, HIGH);
  if (u >= 0) { ledcWrite(IN2, 0);  ledcWrite(IN1, u);  }
  else        { ledcWrite(IN1, 0);  ledcWrite(IN2, -u); }
}

void motorStop()
{
  ledcWrite(IN1, 0);
  ledcWrite(IN2, 0);
  digitalWrite(ENA, LOW);
  pwmActual = 0;
}

// Espera ms milisegundos vigilando la corriente. false = sobrecorriente.
bool esperaVigilando(uint32_t ms)
{
  uint32_t t0 = millis();
  uint8_t sobre = 0;
  while (millis() - t0 < ms)
  {
    float i = inaShunt() * LSB_MA;
    sobre = (fabsf(i) > iMaxMa) ? sobre + 1 : 0;
    if (sobre >= 3) return false;
    delayMicroseconds(500);
  }
  return true;
}

void abortar(float i)
{
  motorStop();
  Serial.printf("*** ABORTADO: corriente %.1f mA > IMAX %.0f mA. Motor apagado. ***\n", i, iMaxMa);
}

// ============================================================
// ESTÁTICA: corriente promedio vs PWM
// ============================================================
void pruebaEstatica(int a, int b, int paso)
{
  if (paso == 0) { Serial.println("ERROR: paso = 0"); return; }
  if ((b - a) * paso < 0) paso = -paso;

  Serial.println("BEGIN_ESTATICA");
  Serial.println("pwm,i_mA");
  for (int u = a; (paso > 0) ? (u <= b) : (u >= b); u += paso)
  {
    motorPWM(u);
    if (!esperaVigilando(holdMs)) { abortar(ultimoRaw * LSB_MA); Serial.println("END_ESTATICA"); return; }
    float i = leerCorrientePromedio(400);
    Serial.printf("%d,%.2f\n", u, i);
  }
  motorStop();
  Serial.println("END_ESTATICA");
}

// ============================================================
// ESCALÓN a -> b, N repeticiones promediadas
// ============================================================
void pruebaEscalon(int a, int b, int N)
{
  if ((a < 0 && b > 0) || (a > 0 && b < 0))
  {
    Serial.println("ERROR: a y b deben tener el mismo signo (el INA226 solo mide magnitud).");
    return;
  }
  if (N < 1) N = 1;

  int nPre = (int)((preMs * 1000UL) / tsUs);
  int n    = nPre + (int)((postMs * 1000UL) / tsUs);
  if (n > MAX_SAMPLES)
  {
    Serial.printf("ERROR: %d muestras > %d. Reduce PRE/POST o sube TS.\n", n, MAX_SAMPLES);
    return;
  }

  memset(acum, 0, sizeof(int32_t) * n);
  uint32_t err0 = errI2C;
  uint32_t tardeMax = 0;

  for (int r = 0; r < N; r++)
  {
    // 1) Llevar a "a" y esperar a que se asiente
    motorPWM(a);
    if (!esperaVigilando(settleMs)) { abortar(ultimoRaw * LSB_MA); return; }

    // 2) Registro con muestreo fijo; el escalón se aplica justo después
    //    de la muestra nPre (t = 0)
    uint8_t sobre = 0;
    uint32_t t0 = micros();
    for (int k = 0; k < n; k++)
    {
      uint32_t tk = t0 + (uint32_t)k * tsUs;
      while ((int32_t)(micros() - tk) < 0) { }
      uint32_t tarde = micros() - tk;
      if (tarde > tardeMax) tardeMax = tarde;

      int16_t raw = inaShunt();
      acum[k] += raw;

      if (k == nPre) motorPWM(b);

      sobre = (fabsf(raw * LSB_MA) > iMaxMa) ? sobre + 1 : 0;
      if (sobre >= 2) { abortar(raw * LSB_MA); return; }
    }

    // 3) Apagar y dejar enfriar
    motorStop();
    delay(restMs);
    Serial.printf("# repeticion %d/%d lista\n", r + 1, N);
  }

  // 4) Enviar el promedio
  Serial.println("BEGIN_ESCALON");
  Serial.printf("# a=%d b=%d N=%d Ts_us=%lu n_pre=%d n=%d err_i2c=%lu tarde_max_us=%lu\n",
                a, b, N, (unsigned long)tsUs, nPre, n,
                (unsigned long)(errI2C - err0), (unsigned long)tardeMax);
  Serial.println("k,t_ms,pwm,i_mA");
  for (int k = 0; k < n; k++)
  {
    float t = (float)(k - nPre) * tsUs * 1e-3f;
    int   u = (k <= nPre) ? a : b;
    float i = (acum[k] * LSB_MA) / N;
    Serial.printf("%d,%.3f,%d,%.3f\n", k, t, u, i);
  }
  Serial.println("END_ESCALON");
}

// ============================================================
// COMANDOS
// ============================================================
void mostrarConfig()
{
  Serial.printf("TS %lu us | PRE %lu ms | POST %lu ms | SETTLE %lu ms | REST %lu ms | HOLD %lu ms | IMAX %.0f mA\n",
                (unsigned long)tsUs, (unsigned long)preMs, (unsigned long)postMs,
                (unsigned long)settleMs, (unsigned long)restMs, (unsigned long)holdMs, iMaxMa);
}

void ayuda()
{
  Serial.println("Comandos: HELP | I | PWM <n> | STOP | ESTATICA <a> <b> <paso> |");
  Serial.println("          ESCALON <a> <b> [N] | SET <TS|PRE|POST|SETTLE|REST|HOLD|IMAX> <v> | CONFIG");
  Serial.println("Recuerda: ROTOR BLOQUEADO para ESTATICA y ESCALON.");
}

void procesar(String s)
{
  s.trim();
  if (s.length() == 0) return;
  String up = s;
  up.toUpperCase();

  char cmd[16] = {0};
  long v1 = 0, v2 = 0, v3 = 0;
  char par[16] = {0};
  float fv = 0;

  int nargs = sscanf(up.c_str(), "%15s %ld %ld %ld", cmd, &v1, &v2, &v3);

  if (!strcmp(cmd, "HELP"))   { ayuda(); return; }
  if (!strcmp(cmd, "CONFIG")) { mostrarConfig(); return; }
  if (!strcmp(cmd, "STOP"))   { motorStop(); Serial.println("OK STOP"); return; }

  if (!strcmp(cmd, "I"))
  {
    float i = leerCorrientePromedio(200);
    Serial.printf("I = %.2f mA (PWM %d, errores I2C %lu)\n", i, pwmActual, (unsigned long)errI2C);
    return;
  }

  if (!strcmp(cmd, "PWM") && nargs >= 2)
  {
    motorPWM((int)v1);
    delay(200);
    float i = leerCorrientePromedio(200);
    Serial.printf("OK PWM %d -> I = %.2f mA\n", pwmActual, i);
    if (fabsf(i) > iMaxMa) abortar(i);
    return;
  }

  if (!strcmp(cmd, "ESTATICA") && nargs >= 4) { pruebaEstatica((int)v1, (int)v2, (int)v3); return; }

  if (!strcmp(cmd, "ESCALON") && nargs >= 3)
  {
    pruebaEscalon((int)v1, (int)v2, (nargs >= 4) ? (int)v3 : 10);
    return;
  }

  if (!strcmp(cmd, "SET") && sscanf(up.c_str(), "%15s %15s %f", cmd, par, &fv) == 3)
  {
    if      (!strcmp(par, "TS"))     tsUs     = constrain((long)fv, 150, 5000);
    else if (!strcmp(par, "PRE"))    preMs    = (uint32_t)fv;
    else if (!strcmp(par, "POST"))   postMs   = (uint32_t)fv;
    else if (!strcmp(par, "SETTLE")) settleMs = (uint32_t)fv;
    else if (!strcmp(par, "REST"))   restMs   = (uint32_t)fv;
    else if (!strcmp(par, "HOLD"))   holdMs   = (uint32_t)fv;
    else if (!strcmp(par, "IMAX"))   iMaxMa   = constrain(fv, 50.0f, 800.0f);
    else { Serial.println("ERROR: parametro desconocido"); return; }
    mostrarConfig();
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
  if (!ledcAttach(IN1, PWM_FREQUENCY, PWM_RESOLUTION) ||
      !ledcAttach(IN2, PWM_FREQUENCY, PWM_RESOLUTION))
  {
    Serial.println("ERROR: ledcAttach");
    while (true) delay(1000);
  }
  motorStop();

  Wire.begin(I2C_SDA, I2C_SCL, I2C_CLOCK_HZ);
  Wire.setTimeOut(5);
  if (!inaInit())
  {
    Serial.println("ERROR: no se pudo inicializar el INA226");
    while (true) delay(1000);
  }

  Serial.println();
  Serial.println("=== Identificacion PWM -> corriente (motor nuevo, rotor bloqueado) ===");
  ayuda();
  mostrarConfig();
  Serial.println("LISTO");
}

void loop()
{
  static String linea;
  while (Serial.available())
  {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r')
    {
      if (linea.length()) procesar(linea);
      linea = "";
    }
    else if (linea.length() < 80) linea += c;
  }
}
