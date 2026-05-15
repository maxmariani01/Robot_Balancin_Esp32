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
      delayMicroseconds(Config::CONTROL_PERIOD_US);
      if ((i & 0x1F) == 0) {
        yield();
      }
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

  float gyroPitchRateDps() const {
    return (static_cast<float>(gyroY) - gyroBiasY) / GYRO_LSB_PER_DPS;
  }

  float temperatureC() const {
    return static_cast<float>(temperatureRaw) / 340.0f + 36.53f;
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
  static constexpr float GYRO_LSB_PER_DPS = 131.0f;

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
    writeRegister(GYRO_CONFIG, 0x00);  // +/-250 dps.
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
    integralTerm = constrain(integralTerm, -maxOut, maxOut);
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

    ledcSetup(Config::LEFT_PWM_CHANNEL, Config::PWM_FREQUENCY_HZ, Config::PWM_RESOLUTION_BITS);
    ledcSetup(Config::RIGHT_PWM_CHANNEL, Config::PWM_FREQUENCY_HZ, Config::PWM_RESOLUTION_BITS);
    ledcAttachPin(Config::LEFT_PWM_PIN, Config::LEFT_PWM_CHANNEL);
    ledcAttachPin(Config::RIGHT_PWM_PIN, Config::RIGHT_PWM_CHANNEL);
    stop();
  }

  void drive(float command, const RuntimeSettings &settings) {
    if (Config::INVERT_MOTOR_OUTPUT) {
      command = -command;
    }

    const int pwm = constrain(static_cast<int>(lroundf(command)), -Config::PWM_MAX, Config::PWM_MAX);
    driveOneMotor(
        Config::LEFT_IN1_PIN,
        Config::LEFT_IN2_PIN,
        Config::LEFT_PWM_CHANNEL,
        pwm,
        settings.leftSpeedFactor,
        settings.leftPwmTrim,
        settings.minAbsSpeed,
        Config::INVERT_LEFT_MOTOR,
        lastLeftPwm);
    driveOneMotor(
        Config::RIGHT_IN1_PIN,
        Config::RIGHT_IN2_PIN,
        Config::RIGHT_PWM_CHANNEL,
        pwm,
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
      int command,
      float factor,
      int trim,
      int minAbsSpeed,
      bool invert,
      int &lastSignedPwm) {
    if (invert) {
      command = -command;
    }

    int duty = abs(command);
    if (duty > 0 && duty < minAbsSpeed) {
      duty = minAbsSpeed;
    }

    if (duty > 0) {
      duty = constrain(static_cast<int>(lroundf(static_cast<float>(duty) * factor)) + trim, 0, Config::PWM_MAX);
    }

    if (command > 0) {
      digitalWrite(in1, HIGH);
      digitalWrite(in2, LOW);
    } else if (command < 0) {
      digitalWrite(in1, LOW);
      digitalWrite(in2, HIGH);
    } else {
      digitalWrite(in1, LOW);
      digitalWrite(in2, LOW);
    }

    ledcWrite(pwmChannel, duty);
    lastSignedPwm = command > 0 ? duty : (command < 0 ? -duty : 0);
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

volatile bool mpuDataReady = false;

float pitchDeg = 0.0f;
float pitchRateDps = 0.0f;
bool filterInitialized = false;
bool mpuReady = false;
bool gyroCalibrated = false;

bool armed = false;
uint32_t armStartMs = 0;

uint32_t lastControlUs = 0;
uint32_t lastDebugMs = 0;
uint32_t lastMpuRetryMs = 0;
float lastOutput = 0.0f;

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

void setStatusLedBooting() {
  setStatusLed(0, 0, 255);
}

void setStatusLedOk() {
  setStatusLed(0, 255, 0);
}

void setStatusLedWarning() {
  setStatusLed(255, 80, 0);
}

void setStatusLedError() {
  setStatusLed(255, 0, 0);
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
    </div>

    <section>
      <h2>PID</h2>
      <div class="grid">
        <label><span class="control-head"><span>KP</span><output data-output="kp">--</output></span><input type="range" min="0" max="100" step="1" data-key="kp"></label>
        <label><span class="control-head"><span>KI</span><output data-output="ki">--</output></span><input type="range" min="0" max="300" step="5" data-key="ki"></label>
        <label><span class="control-head"><span>KD</span><output data-output="kd">--</output></span><input type="range" min="0" max="20" step="0.5" data-key="kd"></label>
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
      </div>
    </section>
  </main>

  <script>
    const controls = Array.from(document.querySelectorAll('[data-key]'));
    const controlTimers = {};

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
      const sensorState = data.mpuReady
        ? (data.gyroCalibrated ? (data.armed ? 'ARMADO' : 'ESPERA') : 'CALIBRAR')
        : (data.mpuAddressAcked ? 'ID NO SOPORTADO' : 'SIN MPU');
      text('state', sensorState);
      text('pitch', fmt(data.pitch, 2) + ' deg');
      text('output', fmt(data.output, 1));
      text('temp', fmt(data.temp, 1) + ' C');
      text('leftPwm', data.leftPwm);
      text('rightPwm', data.rightPwm);
      text('rate', fmt(data.pitchRate, 1) + ' dps');
      text('mpuInfo', data.mpuAddressAcked ? hex(data.mpuAddress) + ' / ' + hex(data.mpuWhoAmI) : '--');
      text('wifi', data.ip || '--');

      const settings = data.settings || {};
      Object.keys(settings).forEach((key) => setInput(key, settings[key]));
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
        } catch (error) {
          alert(error.message);
        }
        button.disabled = false;
      });
    });

    refresh();
    setInterval(refresh, 600);
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

  if (!filterInitialized) {
    pitchDeg = accelPitch;
    filterInitialized = true;
  } else {
    pitchDeg = settings.complementaryAlpha * (pitchDeg + gyroRate * dtSeconds) +
               (1.0f - settings.complementaryAlpha) * accelPitch;
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
  json.reserve(900);

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

bool calibrateGyroNow() {
  resetArming();
  if (!mpuReady) {
    mpuReady = mpu.begin();
  }
  if (!mpuReady) {
    gyroCalibrated = false;
    setStatusLedError();
    return false;
  }

  Serial.println(F("Calibrando gyro desde web. Mantener el robot quieto..."));
  gyroCalibrated = mpu.calibrateGyro();
  filterInitialized = false;
  lastControlUs = micros();
  gyroCalibrated ? setStatusLedOk() : setStatusLedWarning();

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
    settings.kp = constrain(numericValue, 0.0f, 100.0f);
    pid.reset();
  } else if (key == F("ki")) {
    settings.ki = constrain(numericValue, 0.0f, 300.0f);
    pid.reset();
  } else if (key == F("kd")) {
    settings.kd = constrain(numericValue, 0.0f, 20.0f);
    pid.reset();
  } else if (key == F("balanceAngle")) {
    settings.balanceAngleDeg = constrain(numericValue, -90.0f, 90.0f);
    pid.reset();
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
  } else {
    server.send(400, F("text/plain"), F("Parametro invalido"));
    return;
  }

  if (resetFilter) {
    filterInitialized = false;
  }
  if (stopControl) {
    resetArming();
  }

  sendStatus();
}

void handleWebAction() {
  if (!server.hasArg("name")) {
    server.send(400, F("text/plain"), F("Falta parametro name"));
    return;
  }

  const String action = server.arg("name");
  if (action == F("reset")) {
    resetArming();
  } else if (action == F("calibrate")) {
    if (!calibrateGyroNow()) {
      server.send(500, F("text/plain"), F("No se pudo calibrar el gyro"));
      return;
    }
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
  server.on("/set", HTTP_GET, handleWebSet);
  server.on("/action", HTTP_GET, handleWebAction);
  server.onNotFound([]() {
    server.send(404, F("text/plain"), F("No encontrado"));
  });
  server.begin();

  Serial.println(F("Servidor HTTP iniciado."));
}

// ============================================================================
//  setup() / loop()
// ============================================================================

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println();
  Serial.println(F("Robot balancin ESP32-C3"));
  setStatusLedBooting();

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
    gyroCalibrated = mpu.calibrateGyro();
    if (!gyroCalibrated) {
      Serial.println(F("Fallo la calibracion del gyro. Usar la web para reintentar."));
    }
    gyroCalibrated ? setStatusLedOk() : setStatusLedWarning();
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

  lastControlUs = micros();
}

void loop() {
  server.handleClient();

  if (!mpuReady || !gyroCalibrated) {
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
    delay(2);
    return;
  }

  const uint32_t nowUs = micros();
  if (static_cast<uint32_t>(nowUs - lastControlUs) < Config::CONTROL_PERIOD_US) {
    // Cede CPU para que la tarea IDLE alimente el watchdog en C3 (single core).
    delayMicroseconds(100);
    return;
  }

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
    delay(2);
    return;
  }

  const bool fallen = fabsf(pitchDeg - settings.balanceAngleDeg) > settings.fallAngleDeg;
  if (fallen) {
    if (armed) {
      Serial.println(F("Caida detectada. Control desactivado."));
    }
    resetArming();
  }

  if (armed) {
    lastOutput = pid.compute(settings.balanceAngleDeg, pitchDeg, pitchRateDps, dtSeconds, settings);
    motors.drive(lastOutput, settings);
  } else {
    if (!fallen) {
      updateArming(pitchDeg);
    }
    lastOutput = 0.0f;
    motors.stop();
  }

  const uint32_t nowMs = millis();
  if (nowMs - lastDebugMs >= Config::DEBUG_PERIOD_MS) {
    lastDebugMs = nowMs;
    Serial.print(F("Pitch: "));
    Serial.print(pitchDeg, 2);
    Serial.print(F(" | Setpoint: "));
    Serial.print(settings.balanceAngleDeg, 2);
    Serial.print(F(" | Output: "));
    Serial.print(lastOutput, 1);
    Serial.print(F(" | PWM L/R: "));
    Serial.print(motors.leftPwm());
    Serial.print(F("/"));
    Serial.print(motors.rightPwm());
    Serial.print(F(" | Estado: "));
    Serial.print(armed ? F("ARMADO") : F("ESPERA"));
    Serial.print(F(" | Temp: "));
    Serial.println(mpu.temperatureC(), 1);
  }
}
