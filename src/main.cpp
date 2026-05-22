// ============================================================================
//  Robot balancin - ESP32-C3 DevKitC-02 + MPU6050 + L298N
// ============================================================================
//  Para ajustar el comportamiento del robot, editar include/Config.h.
//  Este archivo solo contiene la logica (drivers, PID, filtro y lazo).
// ============================================================================

#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Preferences.h>
#include <math.h>

#include "Config.h"

struct RuntimeSettings {
  float kp = Config::KP;
  float ki = Config::KI;
  float kd = Config::KD;
  float balanceAngleDeg = Config::BALANCE_ANGLE_DEG;
  float startAngleWindowDeg = Config::START_ANGLE_WINDOW_DEG;
  float fallAngleDeg = Config::FALL_ANGLE_DEG;
  float complementaryAlpha = Config::COMPLEMENTARY_ALPHA;
  int minAbsSpeed = Config::MIN_ABS_SPEED;
  float leftSpeedFactor = Config::LEFT_SPEED_FACTOR;
  float rightSpeedFactor = Config::RIGHT_SPEED_FACTOR;
  int leftPwmTrim = 0;
  int rightPwmTrim = 0;
  bool invertPitch = Config::INVERT_PITCH;
};

// ============================================================================
//  MPU6050 - driver I2C minimo (sin I2Cdev ni DMP)
// ============================================================================

class Mpu6050 {
public:
  bool begin() {
    if (beginAtAddress(0x68)) {
      return true;
    }

    const uint8_t firstAddress = address;
    const uint8_t firstWhoAmI = lastWhoAmI;
    const bool firstAddressAcked = lastAddressAcked;
    const bool firstSupportedId = lastSupportedId;

    if (beginAtAddress(0x69)) {
      return true;
    }

    if (firstAddressAcked) {
      address = firstAddress;
      lastWhoAmI = firstWhoAmI;
      lastAddressAcked = firstAddressAcked;
      lastSupportedId = firstSupportedId;
    }
    return false;
  }

  bool readRaw() {
    Wire.beginTransmission(address);
    Wire.write(ACCEL_XOUT_H);
    if (Wire.endTransmission(false) != 0) {
      return false;
    }

    const uint8_t expected = 14;
    const uint8_t received = Wire.requestFrom(address, expected);
    if (received != expected) {
      while (Wire.available()) {
        Wire.read();
      }
      return false;
    }

    accelX = readWord();
    accelY = readWord();
    accelZ = readWord();
    temperatureRaw = readWord();
    gyroX = readWord();
    gyroY = readWord();
    gyroZ = readWord();
    return true;
  }

  bool calibrateGyro(uint16_t samples = 1200) {
    int64_t sumX = 0;
    int64_t sumY = 0;
    int64_t sumZ = 0;
    uint16_t validSamples = 0;

    for (uint16_t i = 0; i < samples; ++i) {
      if (readRaw()) {
        sumX += gyroX;
        sumY += gyroY;
        sumZ += gyroZ;
        ++validSamples;
      }
      // delay() bloquea con vTaskDelay: cede CPU a IDLE y alimenta el
      // watchdog mientras dura la calibracion (~6 s).
      delay(Config::CONTROL_PERIOD_US / 1000);
    }

    if (validSamples < samples / 2) {
      return false;
    }

    gyroBiasX = static_cast<float>(sumX) / static_cast<float>(validSamples);
    gyroBiasY = static_cast<float>(sumY) / static_cast<float>(validSamples);
    gyroBiasZ = static_cast<float>(sumZ) / static_cast<float>(validSamples);
    return true;
  }

  uint8_t i2cAddress() const {
    return address;
  }

  uint8_t whoAmI() const {
    return lastWhoAmI;
  }

  bool addressAcked() const {
    return lastAddressAcked;
  }

  bool supportedId() const {
    return lastSupportedId;
  }

  float accelPitchDeg() const {
    const float ax = static_cast<float>(accelX);
    const float ay = static_cast<float>(accelY);
    const float az = static_cast<float>(accelZ);
    return atan2f(ax, sqrtf(ay * ay + az * az)) * 180.0f / PI;
  }

  // Modulo del vector aceleracion en g. En reposo vale ~1.0; se aparta de 1.0
  // cuando hay aceleracion lineal (el robot acelerando para corregirse).
  float accelMagnitudeG() const {
    const float ax = static_cast<float>(accelX);
    const float ay = static_cast<float>(accelY);
    const float az = static_cast<float>(accelZ);
    return sqrtf(ax * ax + ay * ay + az * az) / ACCEL_LSB_PER_G;
  }

  float gyroPitchRateDps() const {
    return (static_cast<float>(gyroY) - gyroBiasY) / GYRO_LSB_PER_DPS;
  }

  float temperatureC() const {
    return static_cast<float>(temperatureRaw) / 340.0f + 36.53f;
  }

  // True si los 3 ejes del gyro y el modulo del accel estan quietos: el robot
  // no se esta moviendo ni lo estan sosteniendo (apoyado o caido y asentado).
  bool looksAtRest() const {
    const float gyroThreshLsb = REST_GYRO_THRESH_DPS * GYRO_LSB_PER_DPS;
    if (fabsf(static_cast<float>(gyroX) - gyroBiasX) > gyroThreshLsb) return false;
    if (fabsf(static_cast<float>(gyroY) - gyroBiasY) > gyroThreshLsb) return false;
    if (fabsf(static_cast<float>(gyroZ) - gyroBiasZ) > gyroThreshLsb) return false;
    return fabsf(accelMagnitudeG() - 1.0f) < REST_ACCEL_THRESH_G;
  }

  // EMA lento de los bias del gyro hacia la lectura cruda actual (usar en reposo).
  void nudgeGyroBias(float k) {
    gyroBiasX += (static_cast<float>(gyroX) - gyroBiasX) * k;
    gyroBiasY += (static_cast<float>(gyroY) - gyroBiasY) * k;
    gyroBiasZ += (static_cast<float>(gyroZ) - gyroBiasZ) * k;
  }

  int16_t accelX = 0;
  int16_t accelY = 0;
  int16_t accelZ = 0;
  int16_t gyroX = 0;
  int16_t gyroY = 0;
  int16_t gyroZ = 0;
  int16_t temperatureRaw = 0;
  float gyroBiasX = 0.0f;
  float gyroBiasY = 0.0f;
  float gyroBiasZ = 0.0f;

private:
  static constexpr float GYRO_LSB_PER_DPS = 65.5f;    // +/-500 dps (headroom).
  static constexpr float ACCEL_LSB_PER_G = 16384.0f;  // +/-2 g.

  // Umbrales de "reposo" para la re-calibracion del bias del gyro.
  static constexpr float REST_GYRO_THRESH_DPS = 2.0f;
  static constexpr float REST_ACCEL_THRESH_G = 0.10f;

  static constexpr uint8_t SMPLRT_DIV = 0x19;
  static constexpr uint8_t CONFIG = 0x1A;
  static constexpr uint8_t GYRO_CONFIG = 0x1B;
  static constexpr uint8_t ACCEL_CONFIG = 0x1C;
  static constexpr uint8_t INT_ENABLE = 0x38;
  static constexpr uint8_t ACCEL_XOUT_H = 0x3B;
  static constexpr uint8_t PWR_MGMT_1 = 0x6B;
  static constexpr uint8_t WHO_AM_I = 0x75;

  bool beginAtAddress(uint8_t candidateAddress) {
    address = candidateAddress;
    lastAddressAcked = false;
    lastSupportedId = false;
    lastWhoAmI = 0xFF;

    Wire.beginTransmission(address);
    if (Wire.endTransmission() != 0) {
      return false;
    }
    lastAddressAcked = true;

    writeRegister(PWR_MGMT_1, 0x80);
    delay(100);
    writeRegister(PWR_MGMT_1, 0x01); // PLL con eje X gyro.
    delay(10);

    lastWhoAmI = readRegister(WHO_AM_I);
    lastSupportedId = isSupportedWhoAmI(lastWhoAmI);
    if (!lastSupportedId) {
      return false;
    }

    writeRegister(CONFIG, 0x03);       // DLPF ~44 Hz gyro / ~42 Hz accel.
    writeRegister(SMPLRT_DIV, 0x04);   // 1 kHz / (1 + 4) = 200 Hz.
    writeRegister(GYRO_CONFIG, 0x08);  // +/-500 dps (headroom anti-saturacion).
    writeRegister(ACCEL_CONFIG, 0x00); // +/-2 g.
    writeRegister(INT_ENABLE, Config::USE_MPU_INTERRUPT ? 0x01 : 0x00);
    readRaw();
    return true;
  }

  bool isSupportedWhoAmI(uint8_t who) const {
    return who == 0x68 || who == 0x69 || who == 0x70 || who == 0x71 || who == 0x73;
  }

  void writeRegister(uint8_t reg, uint8_t value) const {
    Wire.beginTransmission(address);
    Wire.write(reg);
    Wire.write(value);
    Wire.endTransmission();
  }

  uint8_t readRegister(uint8_t reg) const {
    Wire.beginTransmission(address);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0) {
      return 0xFF;
    }
    if (Wire.requestFrom(address, static_cast<uint8_t>(1)) != 1) {
      return 0xFF;
    }
    return Wire.read();
  }

  int16_t readWord() const {
    const uint8_t highByte = Wire.read();
    const uint8_t lowByte = Wire.read();
    return static_cast<int16_t>((highByte << 8) | lowByte);
  }

  uint8_t address = 0x68;
  uint8_t lastWhoAmI = 0xFF;
  bool lastAddressAcked = false;
  bool lastSupportedId = false;
};

// ============================================================================
//  BalancePid - PID con derivada sobre la medicion y anti-windup condicional
// ============================================================================

class BalancePid {
public:
  void reset() {
    integralTerm = 0.0f;
  }

  // measurementRate: derivada de `input` en deg/s (gyro pitch rate).
  // Derivada sobre la medicion: evita derivative kick y el ruido de derivar
  // el angulo filtrado.
  float compute(float setpoint, float input, float measurementRate, float dtSeconds, const RuntimeSettings &settings) {
    const float error = setpoint - input;
    const float derivative = -measurementRate;
    const float maxOut = static_cast<float>(Config::PWM_MAX);

    const float pdOut = settings.kp * error + settings.kd * derivative;
    const float candidateI = integralTerm + settings.ki * error * dtSeconds;
    const float totalUnclamped = pdOut + candidateI;
    const float output = constrain(totalUnclamped, -maxOut, maxOut);

    // Anti-windup condicional: solo acumular integral si la salida no esta
    // saturada, o si el error tira en sentido de des-saturar.
    const bool saturated = totalUnclamped != output;
    const bool unsaturating = (output >= maxOut && error < 0.0f) ||
                              (output <= -maxOut && error > 0.0f);
    if (!saturated || unsaturating) {
      integralTerm = candidateI;
    }
    integralTerm = constrain(integralTerm, -Config::INTEGRAL_LIMIT, Config::INTEGRAL_LIMIT);
    return output;
  }

private:
  float integralTerm = 0.0f;
};

// ============================================================================
//  L298MotorDriver - PWM LEDC + IN1/IN2 por canal
// ============================================================================

class L298MotorDriver {
public:
  void begin() {
    pinMode(Config::LEFT_IN1_PIN, OUTPUT);
    pinMode(Config::LEFT_IN2_PIN, OUTPUT);
    pinMode(Config::RIGHT_IN1_PIN, OUTPUT);
    pinMode(Config::RIGHT_IN2_PIN, OUTPUT);

    ledcSetup(Config::LEFT_PWM_CHANNEL, Config::PWM_FREQUENCY_HZ, Config::LEDC_RESOLUTION_BITS);
    ledcSetup(Config::RIGHT_PWM_CHANNEL, Config::PWM_FREQUENCY_HZ, Config::LEDC_RESOLUTION_BITS);
    ledcAttachPin(Config::LEFT_PWM_PIN, Config::LEFT_PWM_CHANNEL);
    ledcAttachPin(Config::RIGHT_PWM_PIN, Config::RIGHT_PWM_CHANNEL);
    stop();
  }

  void drive(float command, const RuntimeSettings &settings) {
    if (Config::INVERT_MOTOR_OUTPUT) {
      command = -command;
    }

    // El comando se mantiene en punto flotante (unidades de control +/-PWM_MAX)
    // hasta el ledcWrite final, asi no se pierde resolucion antes de tiempo.
    const float maxOut = static_cast<float>(Config::PWM_MAX);
    command = constrain(command, -maxOut, maxOut);
    driveOneMotor(
        Config::LEFT_IN1_PIN,
        Config::LEFT_IN2_PIN,
        Config::LEFT_PWM_CHANNEL,
        command,
        settings.leftSpeedFactor,
        settings.leftPwmTrim,
        settings.minAbsSpeed,
        Config::INVERT_LEFT_MOTOR,
        lastLeftPwm);
    driveOneMotor(
        Config::RIGHT_IN1_PIN,
        Config::RIGHT_IN2_PIN,
        Config::RIGHT_PWM_CHANNEL,
        command,
        settings.rightSpeedFactor,
        settings.rightPwmTrim,
        settings.minAbsSpeed,
        Config::INVERT_RIGHT_MOTOR,
        lastRightPwm);
  }

  void stop() {
    ledcWrite(Config::LEFT_PWM_CHANNEL, 0);
    ledcWrite(Config::RIGHT_PWM_CHANNEL, 0);
    digitalWrite(Config::LEFT_IN1_PIN, LOW);
    digitalWrite(Config::LEFT_IN2_PIN, LOW);
    digitalWrite(Config::RIGHT_IN1_PIN, LOW);
    digitalWrite(Config::RIGHT_IN2_PIN, LOW);
    lastLeftPwm = 0;
    lastRightPwm = 0;
  }

  int leftPwm() const {
    return lastLeftPwm;
  }

  int rightPwm() const {
    return lastRightPwm;
  }

private:
  void driveOneMotor(
      uint8_t in1,
      uint8_t in2,
      uint8_t pwmChannel,
      float command,
      float factor,
      int trim,
      int minAbsSpeed,
      bool invert,
      int &lastSignedPwm) {
    if (invert) {
      command = -command;
    }

    const float maxOut = static_cast<float>(Config::PWM_MAX);
    const float magnitude = fabsf(command);
    float controlDuty = 0.0f; // unidades de control 0..PWM_MAX

    // Compensacion de friccion suave: en vez del escalon 0 -> MIN_ABS_SPEED,
    // cualquier comando no nulo se mapea linealmente a [minAbsSpeed, PWM_MAX].
    // Asi las correcciones finas del PID en el rango bajo si producen cambios
    // de duty proporcionales (menos limit-cycle parado cerca del equilibrio).
    if (magnitude > 0.5f) {
      const float floorOut = static_cast<float>(minAbsSpeed);
      controlDuty = floorOut + (magnitude / maxOut) * (maxOut - floorOut);
      controlDuty = controlDuty * factor + static_cast<float>(trim);
      controlDuty = constrain(controlDuty, 0.0f, maxOut);
    }

    if (command > 0.5f) {
      digitalWrite(in1, HIGH);
      digitalWrite(in2, LOW);
    } else if (command < -0.5f) {
      digitalWrite(in1, LOW);
      digitalWrite(in2, HIGH);
    } else {
      digitalWrite(in1, LOW);
      digitalWrite(in2, LOW);
    }

    // Se redondea una sola vez aca, ya escalado a la resolucion del LEDC.
    const float ledcScale = static_cast<float>(Config::LEDC_MAX) / maxOut;
    ledcWrite(pwmChannel, static_cast<uint32_t>(lroundf(controlDuty * ledcScale)));

    const int reportedDuty = static_cast<int>(lroundf(controlDuty));
    lastSignedPwm = command > 0.5f ? reportedDuty : (command < -0.5f ? -reportedDuty : 0);
  }

  int lastLeftPwm = 0;
  int lastRightPwm = 0;
};

// ============================================================================
//  Estado global
// ============================================================================

Mpu6050 mpu;
BalancePid pid;
L298MotorDriver motors;
RuntimeSettings settings;
WebServer server(80);
Preferences prefs;

volatile bool mpuDataReady = false;

float pitchDeg = 0.0f;
float pitchRateDps = 0.0f;          // velocidad cruda del gyro (para status).
float pitchRateFilteredDps = 0.0f;  // velocidad suavizada que alimenta el termino D.
bool filterInitialized = false;
bool mpuReady = false;
bool gyroCalibrated = false;

bool armed = false;
uint32_t armStartMs = 0;

// Auto-trim: correccion lenta del angulo de equilibrio. El setpoint efectivo
// es balanceAngleDeg + autoTrimDeg.
float autoTrimDeg = 0.0f;
// Momento en que el robot empezo a estar quieto (0 = no esta en reposo).
uint32_t restSinceMs = 0;

uint32_t lastControlUs = 0;
uint32_t lastDebugMs = 0;
uint32_t lastMpuRetryMs = 0;
float lastOutput = 0.0f;
bool lastFallenLed = false;

// Alpha efectivo del filtro complementario adaptativo (para debug/plotter).
float lastFilterAlpha = Config::COMPLEMENTARY_ALPHA;

// Metrica de calidad: RMS del error de pitch mientras esta armado. Es un
// promedio movil exponencial del error al cuadrado; sqrt al mostrarlo.
float pitchErrorMsAccum = 0.0f;
float pitchRmsDeg = 0.0f;

// Buffer circular de pitch para el grafico en vivo de la web (~50 Hz).
constexpr uint16_t PITCH_SAMPLE_CAP = 300;
float pitchSamples[PITCH_SAMPLE_CAP] = {0.0f};
volatile uint16_t pitchSampleHead = 0;
volatile uint16_t pitchSampleCount = 0;
uint8_t pitchSampleDivider = 0;

// Lazo de control en su propia task FreeRTOS, separada del servidor web.
// Las banderas las setea la task del web server y las consume la de control,
// para que el HTTP nunca demore un ciclo de balanceo ni toque motores / I2C.
TaskHandle_t controlTaskHandle = nullptr;
volatile bool calibrateRequested = false;
volatile bool resetRequested = false;
volatile bool saveRequested = false;
volatile bool calibrating = false;
volatile bool nvsHasSavedSettings = false;

// ============================================================================
//  LED integrado de estado
// ============================================================================

uint8_t scaleStatusLed(uint8_t value) {
  return static_cast<uint8_t>((static_cast<uint16_t>(value) * Config::STATUS_LED_BRIGHTNESS) / 255);
}

void setStatusLed(uint8_t red, uint8_t green, uint8_t blue) {
  if (!Config::STATUS_LED_ENABLED) {
    return;
  }

#if defined(RGB_BUILTIN)
  neopixelWrite(RGB_BUILTIN, scaleStatusLed(red), scaleStatusLed(green), scaleStatusLed(blue));
#elif defined(LED_BUILTIN)
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, red || green || blue ? HIGH : LOW);
#else
  (void)red;
  (void)green;
  (void)blue;
#endif
}

// Estados principales pedidos por el robot:
//   blanco -> calibrando sensor
//   verde  -> sensor calibrado y operando
//   rojo   -> robot caido (o falla irrecuperable del MPU)
void setStatusLedCalibrating() {
  setStatusLed(255, 255, 255);
}

void setStatusLedReady() {
  setStatusLed(0, 255, 0);
}

void setStatusLedFallen() {
  setStatusLed(255, 0, 0);
}

void setStatusLedWarning() {
  setStatusLed(255, 80, 0);
}

void setStatusLedError() {
  setStatusLed(255, 0, 0);
}

// ============================================================================
//  Persistencia de ajustes en NVS (flash)
// ============================================================================
//  Guarda los valores tuneados desde la web para que sobrevivan al reinicio.
//  Asi no hay que copiarlos a Config.h ni recompilar para conservar el ajuste.
//  Config.h sigue siendo el default de fabrica si NVS esta vacio.

constexpr char NVS_NAMESPACE[] = "robot";

void loadSettings() {
  // readOnly=true: si el namespace no existe todavia, begin() devuelve false
  // y se mantienen los defaults de Config.h.
  if (!prefs.begin(NVS_NAMESPACE, true)) {
    return;
  }
  const bool hasData = prefs.isKey("kp");
  if (hasData) {
    settings.kp = prefs.getFloat("kp", settings.kp);
    settings.ki = prefs.getFloat("ki", settings.ki);
    settings.kd = prefs.getFloat("kd", settings.kd);
    settings.balanceAngleDeg = prefs.getFloat("bal", settings.balanceAngleDeg);
    settings.startAngleWindowDeg = prefs.getFloat("win", settings.startAngleWindowDeg);
    settings.fallAngleDeg = prefs.getFloat("fall", settings.fallAngleDeg);
    settings.complementaryAlpha = prefs.getFloat("alpha", settings.complementaryAlpha);
    settings.minAbsSpeed = prefs.getInt("minpwm", settings.minAbsSpeed);
    settings.leftSpeedFactor = prefs.getFloat("lfac", settings.leftSpeedFactor);
    settings.rightSpeedFactor = prefs.getFloat("rfac", settings.rightSpeedFactor);
    settings.leftPwmTrim = prefs.getInt("ltrim", settings.leftPwmTrim);
    settings.rightPwmTrim = prefs.getInt("rtrim", settings.rightPwmTrim);
    settings.invertPitch = prefs.getBool("invp", settings.invertPitch);
  }
  prefs.end();
  nvsHasSavedSettings = hasData;
}

// La escritura a flash congela la CPU unos ms: la ejecuta la task de control
// y solo despues de desarmar, asi el robot no esta balanceando durante el commit.
bool saveSettings() {
  if (!prefs.begin(NVS_NAMESPACE, false)) {
    return false;
  }
  prefs.putFloat("kp", settings.kp);
  prefs.putFloat("ki", settings.ki);
  prefs.putFloat("kd", settings.kd);
  prefs.putFloat("bal", settings.balanceAngleDeg);
  prefs.putFloat("win", settings.startAngleWindowDeg);
  prefs.putFloat("fall", settings.fallAngleDeg);
  prefs.putFloat("alpha", settings.complementaryAlpha);
  prefs.putInt("minpwm", settings.minAbsSpeed);
  prefs.putFloat("lfac", settings.leftSpeedFactor);
  prefs.putFloat("rfac", settings.rightSpeedFactor);
  prefs.putInt("ltrim", settings.leftPwmTrim);
  prefs.putInt("rtrim", settings.rightPwmTrim);
  prefs.putBool("invp", settings.invertPitch);
  prefs.end();
  nvsHasSavedSettings = true;
  return true;
}

const char INDEX_HTML[] PROGMEM = R"html(
<!doctype html>
<html lang="es">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>Robot Balancin</title>
  <style>
    :root {
      color: #172033;
      background: #eef2f6;
      font-family: system-ui, -apple-system, BlinkMacSystemFont, "Segoe UI", sans-serif;
    }
    * {
      box-sizing: border-box;
    }
    body {
      margin: 0;
      padding: 18px;
    }
    main {
      max-width: 760px;
      margin: 0 auto;
      display: grid;
      gap: 12px;
    }
    h1,
    h2 {
      margin: 0;
      line-height: 1.15;
    }
    h1 {
      font-size: 1.45rem;
    }
    h2 {
      font-size: 1rem;
    }
    .status {
      display: grid;
      grid-template-columns: repeat(4, minmax(0, 1fr));
      gap: 8px;
    }
    .tile,
    section {
      border: 1px solid #d5dde8;
      border-radius: 8px;
      background: #ffffff;
    }
    .tile {
      min-width: 0;
      padding: 10px;
    }
    section {
      padding: 12px;
      display: grid;
      gap: 10px;
    }
    .label {
      display: block;
      color: #657084;
      font-size: 0.78rem;
    }
    .value {
      display: block;
      margin-top: 3px;
      font-size: 1.15rem;
      font-weight: 750;
      line-height: 1.1;
      overflow-wrap: anywhere;
    }
    .grid {
      display: grid;
      grid-template-columns: repeat(3, minmax(0, 1fr));
      gap: 10px;
    }
    label {
      display: grid;
      gap: 5px;
      color: #657084;
      font-size: 0.82rem;
      font-weight: 650;
    }
    input {
      width: 100%;
      min-height: 42px;
      border: 1px solid #cbd5e1;
      border-radius: 6px;
      padding: 8px;
      color: #172033;
      background: #f8fafc;
      font: inherit;
      font-weight: 650;
    }
    input[type="checkbox"] {
      width: 42px;
      justify-self: start;
      accent-color: #2563eb;
    }
    input[type="range"] {
      min-height: 34px;
      padding: 0;
      border: 0;
      background: transparent;
      accent-color: #2563eb;
    }
    .control-head {
      display: flex;
      align-items: center;
      justify-content: space-between;
      gap: 10px;
    }
    output {
      color: #172033;
      font-variant-numeric: tabular-nums;
      font-weight: 800;
    }
    .chart {
      width: 100%;
      height: 160px;
      display: block;
      border: 1px solid #d5dde8;
      border-radius: 8px;
      background: #ffffff;
    }
    .actions {
      display: grid;
      grid-template-columns: repeat(2, minmax(0, 1fr));
      gap: 10px;
    }
    button {
      min-height: 48px;
      border: 0;
      border-radius: 8px;
      background: #2563eb;
      color: #ffffff;
      font: inherit;
      font-weight: 750;
      cursor: pointer;
    }
    button.secondary {
      background: #334155;
    }
    button:active {
      transform: translateY(1px);
    }
    @media (max-width: 680px) {
      body {
        padding: 12px;
      }
      .status,
      .grid {
        grid-template-columns: repeat(2, minmax(0, 1fr));
      }
    }
  </style>
</head>
<body>
  <main>
    <h1>Robot Balancin</h1>

    <div class="status" aria-live="polite">
      <div class="tile"><span class="label">Estado</span><span class="value" id="state">--</span></div>
      <div class="tile"><span class="label">Pitch</span><span class="value" id="pitch">--</span></div>
      <div class="tile"><span class="label">Salida PID</span><span class="value" id="output">--</span></div>
      <div class="tile"><span class="label">Temperatura</span><span class="value" id="temp">--</span></div>
      <div class="tile"><span class="label">PWM izq.</span><span class="value" id="leftPwm">--</span></div>
      <div class="tile"><span class="label">PWM der.</span><span class="value" id="rightPwm">--</span></div>
      <div class="tile"><span class="label">Gyro Y</span><span class="value" id="rate">--</span></div>
      <div class="tile"><span class="label">MPU</span><span class="value" id="mpuInfo">--</span></div>
      <div class="tile"><span class="label">WiFi</span><span class="value" id="wifi">--</span></div>
      <div class="tile"><span class="label">RMS error</span><span class="value" id="rms">--</span></div>
      <div class="tile"><span class="label">Filtro alpha</span><span class="value" id="alphaEff">--</span></div>
      <div class="tile"><span class="label">Auto-trim</span><span class="value" id="autoTrim">--</span></div>
      <div class="tile"><span class="label">Ajustes NVS</span><span class="value" id="saved">--</span></div>
    </div>

    <section>
      <h2>Pitch en vivo</h2>
      <canvas id="chart" class="chart"></canvas>
      <span class="label" id="chartInfo">--</span>
    </section>

    <section>
      <h2>PID</h2>
      <div class="grid">
        <label><span class="control-head"><span>KP</span><output data-output="kp">--</output></span><input type="range" min="0" max="80" step="0.5" data-key="kp"></label>
        <label><span class="control-head"><span>KI</span><output data-output="ki">--</output></span><input type="range" min="0" max="100" step="0.5" data-key="ki"></label>
        <label><span class="control-head"><span>KD</span><output data-output="kd">--</output></span><input type="range" min="0" max="30" step="0.1" data-key="kd"></label>
      </div>
    </section>

    <section>
      <h2>Motores</h2>
      <div class="grid">
        <label>PWM minimo<input type="number" min="0" max="255" step="1" data-key="minPwm"></label>
        <label>Factor izq.<input type="number" min="0" max="1.5" step="0.01" data-key="leftFactor"></label>
        <label>Factor der.<input type="number" min="0" max="1.5" step="0.01" data-key="rightFactor"></label>
        <label>Trim PWM izq.<input type="number" min="-120" max="120" step="1" data-key="leftTrim"></label>
        <label>Trim PWM der.<input type="number" min="-120" max="120" step="1" data-key="rightTrim"></label>
      </div>
    </section>

    <section>
      <h2>Sensor y equilibrio</h2>
      <div class="grid">
        <label>Angulo equilibrio<input type="number" step="0.1" data-key="balanceAngle"></label>
        <label>Ventana arranque<input type="number" min="0" max="45" step="0.1" data-key="startWindow"></label>
        <label>Angulo caida<input type="number" min="1" max="90" step="0.1" data-key="fallAngle"></label>
        <label>Alpha filtro<input type="number" min="0" max="1" step="0.001" data-key="alpha"></label>
        <label>Bias gyro Y<input type="number" step="0.1" data-key="gyroBiasY"></label>
        <label>Invertir pitch<input type="checkbox" data-key="invertPitch"></label>
      </div>
      <div class="actions">
        <button class="secondary" type="button" data-action="reset">Desarmar</button>
        <button type="button" data-action="calibrate">Calibrar gyro</button>
        <button type="button" data-action="save">Guardar ajustes</button>
      </div>
    </section>
  </main>

  <script>
    const controls = Array.from(document.querySelectorAll('[data-key]'));
    const controlTimers = {};
    let dirty = false;

    function fmt(value, digits = 1) {
      if (value === null || value === undefined) {
        return '--';
      }
      const number = Number(value);
      return Number.isFinite(number) ? number.toFixed(digits) : '--';
    }

    function text(id, value) {
      document.getElementById(id).textContent = value;
    }

    function hex(value) {
      const number = Number(value);
      return Number.isFinite(number) ? '0x' + number.toString(16).toUpperCase().padStart(2, '0') : '--';
    }

    function updateOutput(key, value) {
      const output = document.querySelector('[data-output="' + key + '"]');
      if (output) {
        output.textContent = value;
      }
    }

    function setInput(key, value) {
      const input = controls.find((control) => control.dataset.key === key);
      if (!input) {
        return;
      }

      if (input.type === 'checkbox') {
        input.checked = Boolean(value);
      } else if (document.activeElement !== input) {
        input.value = value;
      }
      updateOutput(key, input.value);
    }

    async function api(path) {
      const response = await fetch(path, { cache: 'no-store' });
      if (!response.ok) {
        throw new Error(await response.text());
      }
      return response.json();
    }

    function render(data) {
      let sensorState;
      if (data.calibrating) {
        sensorState = 'CALIBRANDO';
      } else if (data.mpuReady) {
        sensorState = data.gyroCalibrated ? (data.armed ? 'ARMADO' : 'ESPERA') : 'CALIBRAR';
      } else {
        sensorState = data.mpuAddressAcked ? 'ID NO SOPORTADO' : 'SIN MPU';
      }
      text('state', sensorState);
      text('pitch', fmt(data.pitch, 2) + ' deg');
      text('output', fmt(data.output, 1));
      text('temp', fmt(data.temp, 1) + ' C');
      text('leftPwm', data.leftPwm);
      text('rightPwm', data.rightPwm);
      text('rate', fmt(data.pitchRate, 1) + ' dps');
      text('mpuInfo', data.mpuAddressAcked ? hex(data.mpuAddress) + ' / ' + hex(data.mpuWhoAmI) : '--');
      text('wifi', data.ip || '--');
      text('rms', fmt(data.rms, 2) + ' deg');
      text('alphaEff', fmt(data.alphaEff, 3));
      text('autoTrim', fmt(data.autoTrim, 2) + ' deg');
      text('saved', dirty ? 'cambios sin guardar' : (data.saved ? 'guardados' : 'sin guardar'));

      const settings = data.settings || {};
      Object.keys(settings).forEach((key) => setInput(key, settings[key]));
    }

    function drawChart(data) {
      const canvas = document.getElementById('chart');
      const w = canvas.clientWidth;
      const h = canvas.clientHeight;
      if (!w || !h) {
        return;
      }
      if (canvas.width !== w) {
        canvas.width = w;
      }
      if (canvas.height !== h) {
        canvas.height = h;
      }

      const ctx = canvas.getContext('2d');
      ctx.clearRect(0, 0, w, h);

      const pitch = data.pitch || [];
      const setpoint = Number(data.setpoint) || 0;
      if (pitch.length < 2) {
        text('chartInfo', 'sin datos todavia');
        return;
      }

      let maxDev = 3;
      for (const p of pitch) {
        const d = Math.abs(p - setpoint);
        if (d > maxDev) {
          maxDev = d;
        }
      }
      const span = maxDev * 1.2;
      const yMin = setpoint - span;
      const yMax = setpoint + span;
      const yOf = (v) => h - ((v - yMin) / (yMax - yMin)) * h;
      const xOf = (i) => (i / (pitch.length - 1)) * w;

      ctx.strokeStyle = '#94a3b8';
      ctx.lineWidth = 1;
      ctx.setLineDash([4, 4]);
      ctx.beginPath();
      ctx.moveTo(0, yOf(setpoint));
      ctx.lineTo(w, yOf(setpoint));
      ctx.stroke();
      ctx.setLineDash([]);

      ctx.strokeStyle = '#2563eb';
      ctx.lineWidth = 2;
      ctx.beginPath();
      pitch.forEach((p, i) => {
        const x = xOf(i);
        const y = yOf(p);
        if (i === 0) {
          ctx.moveTo(x, y);
        } else {
          ctx.lineTo(x, y);
        }
      });
      ctx.stroke();

      const windowS = (pitch.length / (Number(data.hz) || 50)).toFixed(1);
      text('chartInfo', 'ventana ' + windowS + ' s | escala ' +
        yMin.toFixed(1) + ' a ' + yMax.toFixed(1) + ' deg (linea = setpoint)');
    }

    async function refreshChart() {
      try {
        drawChart(await api('/samples'));
      } catch (error) {
        text('chartInfo', 'sin conexion');
      }
    }

    async function refresh() {
      try {
        render(await api('/status'));
      } catch (error) {
        text('state', 'SIN CONEXION');
      }
    }

    async function sendControl(input) {
      const value = input.type === 'checkbox' ? (input.checked ? 1 : 0) : input.value;
      dirty = true;
      try {
        render(await api('/set?key=' + encodeURIComponent(input.dataset.key) + '&value=' + encodeURIComponent(value)));
      } catch (error) {
        alert(error.message);
        refresh();
      }
    }

    controls.forEach((input) => {
      input.addEventListener('input', () => {
        if (input.type !== 'range') {
          return;
        }
        updateOutput(input.dataset.key, input.value);
        clearTimeout(controlTimers[input.dataset.key]);
        controlTimers[input.dataset.key] = setTimeout(() => sendControl(input), 140);
      });

      input.addEventListener('change', () => {
        if (input.type !== 'range') {
          sendControl(input);
        }
      });
    });

    document.querySelectorAll('[data-action]').forEach((button) => {
      button.addEventListener('click', async () => {
        button.disabled = true;
        try {
          render(await api('/action?name=' + encodeURIComponent(button.dataset.action)));
          if (button.dataset.action === 'save') {
            dirty = false;
          }
        } catch (error) {
          alert(error.message);
        }
        button.disabled = false;
      });
    });

    refresh();
    setInterval(refresh, 600);
    refreshChart();
    setInterval(refreshChart, 500);
  </script>
</body>
</html>
)html";

// ============================================================================
//  ISR y maquina de armado
// ============================================================================

void IRAM_ATTR onMpuDataReady() {
  mpuDataReady = true;
}

void resetArming() {
  armed = false;
  armStartMs = 0;
  pid.reset();
  motors.stop();
}

void updateArming(float angleDeg) {
  const bool insideWindow = fabsf(angleDeg - settings.balanceAngleDeg) <= settings.startAngleWindowDeg;
  if (!insideWindow) {
    resetArming();
    return;
  }

  if (armStartMs == 0) {
    armStartMs = millis();
  }

  if (!armed && millis() - armStartMs >= Config::ARM_STABLE_TIME_MS) {
    armed = true;
    pid.reset();
    // El RMS arranca de cero en cada sesion armada para medir este ajuste.
    pitchErrorMsAccum = 0.0f;
    pitchRmsDeg = 0.0f;
    Serial.println(F("Control activado."));
  }
}

// ============================================================================
//  Lectura y filtro complementario de pitch
// ============================================================================

bool updatePitch(float dtSeconds) {
  if (!mpu.readRaw()) {
    return false;
  }

  float accelPitch = mpu.accelPitchDeg();
  float gyroRate = mpu.gyroPitchRateDps();

  if (settings.invertPitch) {
    accelPitch = -accelPitch;
    gyroRate = -gyroRate;
  }

  // Velocidad suavizada para el termino D (la integracion del filtro de abajo
  // sigue usando gyroRate crudo para no agregarle lag al angulo).
  if (!filterInitialized) {
    pitchRateFilteredDps = gyroRate;
  } else {
    pitchRateFilteredDps = Config::D_TERM_SMOOTHING * pitchRateFilteredDps +
                           (1.0f - Config::D_TERM_SMOOTHING) * gyroRate;
  }

  if (!filterInitialized) {
    pitchDeg = accelPitch;
    filterInitialized = true;
    lastFilterAlpha = settings.complementaryAlpha;
  } else {
    // Filtro complementario adaptativo: cuando el modulo del vector aceleracion
    // se aparta de 1 g, el accel esta contaminado por aceleracion lineal, asi
    // que subimos alpha para apoyarnos mas en el gyro durante esa muestra.
    const float deviation = fabsf(mpu.accelMagnitudeG() - 1.0f);
    float distrust = (deviation - Config::ACCEL_TRUST_DEV_LOW_G) /
                     (Config::ACCEL_TRUST_DEV_HIGH_G - Config::ACCEL_TRUST_DEV_LOW_G);
    distrust = constrain(distrust, 0.0f, 1.0f);

    const float baseAlpha = settings.complementaryAlpha;
    const float alpha = baseAlpha + (1.0f - baseAlpha) * distrust;
    lastFilterAlpha = alpha;

    pitchDeg = alpha * (pitchDeg + gyroRate * dtSeconds) +
               (1.0f - alpha) * accelPitch;
  }

  pitchRateDps = gyroRate;
  return true;
}

// ============================================================================
//  Servidor web de ajuste
// ============================================================================

void appendFloat(String &json, float value, uint8_t decimals) {
  if (isfinite(value)) {
    json += String(value, static_cast<unsigned int>(decimals));
  } else {
    json += F("null");
  }
}

void appendBool(String &json, bool value) {
  json += value ? F("true") : F("false");
}

bool parseFloatValue(const String &text, float &value) {
  const char *start = text.c_str();
  char *end = nullptr;
  value = strtof(start, &end);
  if (end == start) {
    return false;
  }

  while (*end == ' ') {
    ++end;
  }
  return *end == '\0' && isfinite(value);
}

bool parseBoolValue(String text, bool &value) {
  text.trim();
  text.toLowerCase();
  if (text == F("1") || text == F("true") || text == F("on")) {
    value = true;
    return true;
  }
  if (text == F("0") || text == F("false") || text == F("off")) {
    value = false;
    return true;
  }
  return false;
}

void sendStatus() {
  String json;
  json.reserve(1100);

  json += F("{\"armed\":");
  appendBool(json, armed);
  json += F(",\"mpuReady\":");
  appendBool(json, mpuReady);
  json += F(",\"mpuAddress\":");
  json += mpu.i2cAddress();
  json += F(",\"mpuWhoAmI\":");
  json += mpu.whoAmI();
  json += F(",\"mpuAddressAcked\":");
  appendBool(json, mpu.addressAcked());
  json += F(",\"mpuSupportedId\":");
  appendBool(json, mpu.supportedId());
  json += F(",\"gyroCalibrated\":");
  appendBool(json, gyroCalibrated);
  json += F(",\"calibrating\":");
  appendBool(json, calibrating);
  json += F(",\"ip\":\"");
  json += WiFi.softAPIP().toString();
  json += F("\",\"pitch\":");
  appendFloat(json, mpuReady ? pitchDeg : NAN, 3);
  json += F(",\"pitchRate\":");
  appendFloat(json, mpuReady ? pitchRateDps : NAN, 3);
  json += F(",\"setpoint\":");
  appendFloat(json, settings.balanceAngleDeg, 3);
  json += F(",\"output\":");
  appendFloat(json, lastOutput, 3);
  json += F(",\"leftPwm\":");
  json += motors.leftPwm();
  json += F(",\"rightPwm\":");
  json += motors.rightPwm();
  json += F(",\"temp\":");
  appendFloat(json, mpuReady ? mpu.temperatureC() : NAN, 2);
  json += F(",\"rms\":");
  appendFloat(json, pitchRmsDeg, 3);
  json += F(",\"alphaEff\":");
  appendFloat(json, lastFilterAlpha, 4);
  json += F(",\"autoTrim\":");
  appendFloat(json, autoTrimDeg, 3);
  json += F(",\"saved\":");
  appendBool(json, nvsHasSavedSettings);

  json += F(",\"settings\":{\"kp\":");
  appendFloat(json, settings.kp, 4);
  json += F(",\"ki\":");
  appendFloat(json, settings.ki, 4);
  json += F(",\"kd\":");
  appendFloat(json, settings.kd, 4);
  json += F(",\"balanceAngle\":");
  appendFloat(json, settings.balanceAngleDeg, 4);
  json += F(",\"startWindow\":");
  appendFloat(json, settings.startAngleWindowDeg, 4);
  json += F(",\"fallAngle\":");
  appendFloat(json, settings.fallAngleDeg, 4);
  json += F(",\"alpha\":");
  appendFloat(json, settings.complementaryAlpha, 5);
  json += F(",\"gyroBiasY\":");
  appendFloat(json, mpu.gyroBiasY, 4);
  json += F(",\"minPwm\":");
  json += settings.minAbsSpeed;
  json += F(",\"leftFactor\":");
  appendFloat(json, settings.leftSpeedFactor, 4);
  json += F(",\"rightFactor\":");
  appendFloat(json, settings.rightSpeedFactor, 4);
  json += F(",\"leftTrim\":");
  json += settings.leftPwmTrim;
  json += F(",\"rightTrim\":");
  json += settings.rightPwmTrim;
  json += F(",\"invertPitch\":");
  appendBool(json, settings.invertPitch);
  json += F("}}");

  server.sendHeader(F("Cache-Control"), F("no-store"));
  server.send(200, F("application/json"), json);
}

void handleRoot() {
  server.sendHeader(F("Cache-Control"), F("no-store"));
  server.send_P(200, PSTR("text/html"), INDEX_HTML);
}

// Devuelve el buffer circular de pitch (orden cronologico) para el grafico web.
void handleSamples() {
  const uint16_t count = pitchSampleCount;
  const uint16_t head = pitchSampleHead;
  const uint16_t start = (head + PITCH_SAMPLE_CAP - count) % PITCH_SAMPLE_CAP;

  String json;
  json.reserve(static_cast<unsigned int>(count) * 8 + 96);
  json += F("{\"setpoint\":");
  appendFloat(json, settings.balanceAngleDeg + autoTrimDeg, 3);
  json += F(",\"fallAngle\":");
  appendFloat(json, settings.fallAngleDeg, 2);
  json += F(",\"hz\":50,\"pitch\":[");
  for (uint16_t i = 0; i < count; ++i) {
    if (i != 0) {
      json += ',';
    }
    json += String(pitchSamples[(start + i) % PITCH_SAMPLE_CAP], 2);
  }
  json += F("]}");

  server.sendHeader(F("Cache-Control"), F("no-store"));
  server.send(200, F("application/json"), json);
}

// Calibracion del gyro. La ejecuta SIEMPRE la task de control (nunca la task
// del web server) para que no haya dos tareas tocando el bus I2C a la vez.
bool runGyroCalibration() {
  resetArming();
  calibrating = true;
  setStatusLedCalibrating();
  if (!mpuReady) {
    mpuReady = mpu.begin();
  }
  if (!mpuReady) {
    gyroCalibrated = false;
    calibrating = false;
    setStatusLedError();
    return false;
  }

  Serial.println(F("Calibrando gyro. Mantener el robot quieto..."));
  gyroCalibrated = mpu.calibrateGyro();
  filterInitialized = false;
  lastControlUs = micros();
  lastFallenLed = false;
  autoTrimDeg = 0.0f;
  restSinceMs = 0;
  calibrating = false;
  if (gyroCalibrated) {
    setStatusLedReady();
  } else {
    setStatusLedWarning();
  }

  Serial.print(F("Bias gyro Y: "));
  Serial.println(mpu.gyroBiasY, 2);
  return gyroCalibrated;
}

void handleWebSet() {
  if (!server.hasArg("key") || !server.hasArg("value")) {
    server.send(400, F("text/plain"), F("Faltan parametros key/value"));
    return;
  }

  const String key = server.arg("key");
  const String textValue = server.arg("value");
  float numericValue = 0.0f;
  bool boolValue = false;
  bool resetFilter = false;
  bool stopControl = false;
  const bool isBoolParam = key == F("invertPitch");

  if (isBoolParam) {
    if (!parseBoolValue(textValue, boolValue)) {
      server.send(400, F("text/plain"), F("Valor invalido"));
      return;
    }
  } else if (!parseFloatValue(textValue, numericValue)) {
    server.send(400, F("text/plain"), F("Valor invalido"));
    return;
  }

  if (key == F("kp")) {
    settings.kp = constrain(numericValue, 0.0f, 80.0f);
    pid.reset();
  } else if (key == F("ki")) {
    settings.ki = constrain(numericValue, 0.0f, 200.0f);
    pid.reset();
  } else if (key == F("kd")) {
    settings.kd = constrain(numericValue, 0.0f, 30.0f);
    pid.reset();
  } else if (key == F("balanceAngle")) {
    settings.balanceAngleDeg = constrain(numericValue, -90.0f, 90.0f);
    pid.reset();
    autoTrimDeg = 0.0f;  // el auto-trim es relativo a este angulo base.
  } else if (key == F("startWindow")) {
    settings.startAngleWindowDeg = constrain(numericValue, 0.0f, 45.0f);
  } else if (key == F("fallAngle")) {
    settings.fallAngleDeg = constrain(numericValue, 1.0f, 90.0f);
  } else if (key == F("alpha")) {
    settings.complementaryAlpha = constrain(numericValue, 0.0f, 1.0f);
    resetFilter = true;
  } else if (key == F("gyroBiasY")) {
    mpu.gyroBiasY = numericValue;
    resetFilter = true;
    pid.reset();
  } else if (key == F("minPwm")) {
    settings.minAbsSpeed = constrain(static_cast<int>(lroundf(numericValue)), 0, Config::PWM_MAX);
  } else if (key == F("leftFactor")) {
    settings.leftSpeedFactor = constrain(numericValue, 0.0f, 1.5f);
  } else if (key == F("rightFactor")) {
    settings.rightSpeedFactor = constrain(numericValue, 0.0f, 1.5f);
  } else if (key == F("leftTrim")) {
    settings.leftPwmTrim = constrain(static_cast<int>(lroundf(numericValue)), -120, 120);
  } else if (key == F("rightTrim")) {
    settings.rightPwmTrim = constrain(static_cast<int>(lroundf(numericValue)), -120, 120);
  } else if (key == F("invertPitch")) {
    settings.invertPitch = boolValue;
    resetFilter = true;
    stopControl = true;
    autoTrimDeg = 0.0f;
  } else {
    server.send(400, F("text/plain"), F("Parametro invalido"));
    return;
  }

  if (resetFilter) {
    filterInitialized = false;
  }
  if (stopControl) {
    // El desarmado real (parar motores) lo hace la task de control.
    resetRequested = true;
  }

  sendStatus();
}

void handleWebAction() {
  if (!server.hasArg("name")) {
    server.send(400, F("text/plain"), F("Falta parametro name"));
    return;
  }

  // Las acciones que tocan motores o el bus I2C no se ejecutan aca: solo se
  // dejan pedidas con una bandera y las atiende la task de control.
  const String action = server.arg("name");
  if (action == F("reset")) {
    resetRequested = true;
  } else if (action == F("calibrate")) {
    if (calibrating) {
      server.send(409, F("text/plain"), F("Calibracion ya en curso"));
      return;
    }
    calibrateRequested = true;
  } else if (action == F("save")) {
    // Guardar desarma el robot: la escritura a flash congela la CPU unos ms.
    saveRequested = true;
  } else {
    server.send(400, F("text/plain"), F("Accion invalida"));
    return;
  }

  sendStatus();
}

void setupWebServer() {
  WiFi.mode(WIFI_AP);
  const bool apStarted = WiFi.softAP(
      Config::WIFI_AP_SSID,
      Config::WIFI_AP_PASSWORD,
      Config::WIFI_AP_CHANNEL);

  Serial.print(F("WiFi AP: "));
  Serial.println(apStarted ? Config::WIFI_AP_SSID : "error");
  Serial.print(F("Clave: "));
  Serial.println(Config::WIFI_AP_PASSWORD);
  Serial.print(F("URL: http://"));
  Serial.println(WiFi.softAPIP());

  server.on("/", HTTP_GET, handleRoot);
  server.on("/status", HTTP_GET, sendStatus);
  server.on("/samples", HTTP_GET, handleSamples);
  server.on("/set", HTTP_GET, handleWebSet);
  server.on("/action", HTTP_GET, handleWebAction);
  server.onNotFound([]() {
    server.send(404, F("text/plain"), F("No encontrado"));
  });
  server.begin();

  Serial.println(F("Servidor HTTP iniciado."));
}

// ============================================================================
//  Task de control - lazo de balanceo a 200 Hz
// ============================================================================
//  Corre en su propia task FreeRTOS con prioridad mayor que loop() (que solo
//  atiende el servidor web). Asi una request HTTP lenta no puede retrasar un
//  ciclo de control. El periodo lo marca vTaskDelayUntil (timing deterministico)
//  y el dt real se mide con micros() para el filtro y el PID.

// MPU sin detectar o gyro sin calibrar: motores parados y reintento de deteccion.
void controlWaitStep() {
  motors.stop();
  lastOutput = 0.0f;

  const uint32_t nowMs = millis();
  if (!mpuReady && nowMs - lastMpuRetryMs >= Config::MPU_RETRY_PERIOD_MS) {
    lastMpuRetryMs = nowMs;
    mpuReady = mpu.begin();
    if (mpuReady) {
      filterInitialized = false;
      Serial.print(F("MPU compatible detectado en reintento 0x"));
      Serial.print(mpu.i2cAddress(), HEX);
      Serial.print(F(" WHO_AM_I=0x"));
      Serial.println(mpu.whoAmI(), HEX);
      setStatusLedWarning();
    }
  }

  if (nowMs - lastDebugMs >= Config::DEBUG_PERIOD_MS) {
    lastDebugMs = nowMs;
    if (mpuReady) {
      Serial.println(F("Esperando calibracion del gyro."));
    } else if (mpu.addressAcked()) {
      Serial.print(F("Esperando MPU compatible. Ultimo ACK 0x"));
      Serial.print(mpu.i2cAddress(), HEX);
      Serial.print(F(" WHO_AM_I=0x"));
      Serial.println(mpu.whoAmI(), HEX);
    } else {
      Serial.println(F("Esperando MPU compatible."));
    }
  }
}

// Un ciclo completo de balanceo: lee IMU, filtra, detecta caida y maneja motores.
void controlStep() {
  const uint32_t nowUs = micros();
  float dtSeconds = static_cast<float>(nowUs - lastControlUs) / 1000000.0f;
  if (dtSeconds > 0.02f) {
    dtSeconds = 0.02f;
  }
  lastControlUs = nowUs;
  if (Config::USE_MPU_INTERRUPT) {
    mpuDataReady = false;
  }

  if (!updatePitch(dtSeconds)) {
    resetArming();
    mpuReady = false;
    gyroCalibrated = false;
    setStatusLedError();
    Serial.println(F("Error leyendo MPU6050. Motores detenidos."));
    return;
  }

  const bool fallen = fabsf(pitchDeg - settings.balanceAngleDeg) > settings.fallAngleDeg;
  if (fallen) {
    if (armed) {
      Serial.println(F("Caida detectada. Control desactivado."));
    }
    resetArming();
  }

  if (fallen != lastFallenLed) {
    lastFallenLed = fallen;
    if (fallen) {
      setStatusLedFallen();
    } else {
      setStatusLedReady();
    }
  }

  // Setpoint efectivo = angulo base del usuario + correccion del auto-trim.
  const float setpointEff = settings.balanceAngleDeg + autoTrimDeg;

  if (armed) {
    lastOutput = pid.compute(setpointEff, pitchDeg, pitchRateFilteredDps, dtSeconds, settings);
    motors.drive(lastOutput, settings);

    // RMS del error de pitch: promedio movil exponencial del error al cuadrado.
    const float err = pitchDeg - setpointEff;
    const float k = constrain(dtSeconds / Config::RMS_TAU_S, 0.0f, 1.0f);
    pitchErrorMsAccum += (err * err - pitchErrorMsAccum) * k;
    pitchRmsDeg = sqrtf(pitchErrorMsAccum);

    // Auto-trim: integra MUY lento la salida del PID y corre el setpoint hacia
    // el verdadero punto de equilibrio (donde el esfuerzo promedio tiende a 0).
    // El signo es realimentacion negativa: si la salida es positiva sostenida,
    // baja autoTrimDeg -> baja el setpoint -> baja el error -> baja la salida.
    if (Config::AUTO_TRIM_ENABLED) {
      autoTrimDeg -= Config::AUTO_TRIM_GAIN * lastOutput * dtSeconds;
      autoTrimDeg = constrain(autoTrimDeg,
                              -Config::AUTO_TRIM_LIMIT_DEG, Config::AUTO_TRIM_LIMIT_DEG);
    }
    restSinceMs = 0;
  } else {
    if (!fallen) {
      updateArming(pitchDeg);
    }
    lastOutput = 0.0f;
    motors.stop();

    // Re-calibracion del bias del gyro: si el robot quedo quieto un rato,
    // arrastramos el bias hacia la lectura cruda (compensa drift termico).
    if (Config::REST_RECAL_ENABLED && mpu.looksAtRest()) {
      const uint32_t restNow = millis();
      if (restSinceMs == 0) {
        restSinceMs = restNow;
      } else if (restNow - restSinceMs >= Config::REST_RECAL_DELAY_MS) {
        mpu.nudgeGyroBias(Config::REST_RECAL_GAIN);
      }
    } else {
      restSinceMs = 0;
    }
  }

  // Buffer circular para el grafico en vivo de la web: 1 muestra cada 4 ciclos.
  if (++pitchSampleDivider >= 4) {
    pitchSampleDivider = 0;
    pitchSamples[pitchSampleHead] = pitchDeg;
    pitchSampleHead = (pitchSampleHead + 1) % PITCH_SAMPLE_CAP;
    if (pitchSampleCount < PITCH_SAMPLE_CAP) {
      ++pitchSampleCount;
    }
  }

  const uint32_t nowMs = millis();
  if (nowMs - lastDebugMs >= Config::DEBUG_PERIOD_MS) {
    lastDebugMs = nowMs;
    // Formato del Serial Plotter del IDE de Arduino ("etiqueta:valor,...").
    // Las tres series comparten escala en grados para que el grafico sea util.
    Serial.print(F("pitch:"));
    Serial.print(pitchDeg, 2);
    Serial.print(F(",setpoint:"));
    Serial.print(setpointEff, 2);
    Serial.print(F(",rms:"));
    Serial.println(pitchRmsDeg, 2);
  }
}

void controlTask(void *param) {
  (void)param;
  const TickType_t period = pdMS_TO_TICKS(Config::CONTROL_PERIOD_US / 1000);
  TickType_t lastWake = xTaskGetTickCount();
  lastControlUs = micros();

  for (;;) {
    vTaskDelayUntil(&lastWake, period);

    // Pedidos posteados por la task del servidor web.
    if (resetRequested) {
      resetRequested = false;
      resetArming();
    }
    if (saveRequested) {
      saveRequested = false;
      // Desarmamos antes del commit: la escritura a flash congela la CPU unos
      // ms y no queremos que pase mientras el robot esta balanceando.
      resetArming();
      Serial.println(saveSettings() ? F("Ajustes guardados en NVS.")
                                    : F("Error guardando ajustes en NVS."));
      lastWake = xTaskGetTickCount();
      lastControlUs = micros();
    }
    if (calibrateRequested) {
      calibrateRequested = false;
      runGyroCalibration();
      // La calibracion bloquea ~6 s: resincronizamos el reloj de la task para
      // no disparar una rafaga de ciclos "atrasados" con vTaskDelayUntil.
      lastWake = xTaskGetTickCount();
      lastControlUs = micros();
    }

    if (!mpuReady || !gyroCalibrated) {
      controlWaitStep();
    } else {
      controlStep();
    }
  }
}

// ============================================================================
//  setup() / loop()
// ============================================================================

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println();
  Serial.println(F("Robot balancin ESP32-C3"));
  setStatusLedCalibrating();

  // Cargamos los ajustes guardados en NVS (si hay); si no, quedan los de Config.h.
  loadSettings();
  Serial.println(nvsHasSavedSettings ? F("Ajustes cargados desde NVS.")
                                     : F("Sin ajustes en NVS: se usan los de Config.h."));

  Wire.begin(Config::I2C_SDA_PIN, Config::I2C_SCL_PIN, Config::I2C_CLOCK_HZ);
  Wire.setTimeOut(50);

  motors.begin();
  if (Config::USE_MPU_INTERRUPT) {
    pinMode(Config::MPU_INT_PIN, INPUT);
    attachInterrupt(digitalPinToInterrupt(Config::MPU_INT_PIN), onMpuDataReady, RISING);
  }

  setupWebServer();

  mpuReady = mpu.begin();
  if (mpuReady) {
    Serial.print(F("MPU compatible detectado en 0x"));
    Serial.print(mpu.i2cAddress(), HEX);
    Serial.print(F(" WHO_AM_I=0x"));
    Serial.println(mpu.whoAmI(), HEX);
    Serial.println(F("Mantener el robot quieto para calibrar gyro..."));
    setStatusLedCalibrating();
    gyroCalibrated = mpu.calibrateGyro();
    if (!gyroCalibrated) {
      Serial.println(F("Fallo la calibracion del gyro. Usar la web para reintentar."));
    }
    if (gyroCalibrated) {
      setStatusLedReady();
    } else {
      setStatusLedWarning();
    }
  } else {
    if (mpu.addressAcked()) {
      Serial.print(F("MPU con ACK en 0x"));
      Serial.print(mpu.i2cAddress(), HEX);
      Serial.print(F(" pero WHO_AM_I=0x"));
      Serial.print(mpu.whoAmI(), HEX);
      Serial.println(F(" no fue aceptado."));
    }
    Serial.println(F("MPU compatible no detectado. El servidor web queda activo para diagnostico."));
    setStatusLedError();
  }

  Serial.print(F("Bias gyro Y: "));
  Serial.println(mpu.gyroBiasY, 2);
  Serial.println(F("Colocar el robot cerca del equilibrio para activar el control."));

  // Lanzamos el lazo de balanceo en su propia task FreeRTOS, fijada al core 0
  // (el unico del ESP32-C3) y con prioridad 2: por encima de loop() y de IDLE,
  // por debajo de las tasks de sistema (WiFi/lwIP).
  xTaskCreatePinnedToCore(controlTask, "control", 4096, nullptr, 2, &controlTaskHandle, 0);
  Serial.println(F("Task de control iniciada."));
}

void loop() {
  // loop() corre como task FreeRTOS de prioridad 1 y solo atiende el servidor
  // web. La task de control (prioridad 2) la apropia cada ciclo de balanceo.
  server.handleClient();
  delay(2);
}
