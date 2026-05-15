#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <WebServer.h>
#include <math.h>

namespace Config {
constexpr uint8_t I2C_SDA_PIN = 8;
constexpr uint8_t I2C_SCL_PIN = 9;
constexpr uint32_t I2C_CLOCK_HZ = 100000;

constexpr uint8_t LEFT_PWM_PIN = 3;   // L298N ENA
constexpr uint8_t LEFT_IN1_PIN = 4;
constexpr uint8_t LEFT_IN2_PIN = 5;

constexpr uint8_t RIGHT_PWM_PIN = 6;  // L298N ENB
constexpr uint8_t RIGHT_IN1_PIN = 7;
constexpr uint8_t RIGHT_IN2_PIN = 10;

constexpr uint8_t LEFT_PWM_CHANNEL = 0;
constexpr uint8_t RIGHT_PWM_CHANNEL = 1;
constexpr uint32_t PWM_FREQUENCY_HZ = 20000;
constexpr uint8_t PWM_RESOLUTION_BITS = 8;
constexpr int PWM_MAX = (1 << PWM_RESOLUTION_BITS) - 1;

constexpr int DEFAULT_PWM = 90;
constexpr int PWM_STEP = 15;
constexpr uint32_t COMMAND_RUN_MS = 2000;

constexpr bool INVERT_LEFT_MOTOR = false;
constexpr bool INVERT_RIGHT_MOTOR = false;

constexpr char WIFI_AP_SSID[] = "RobotTest";
constexpr char WIFI_AP_PASSWORD[] = "12345678";
constexpr uint8_t WIFI_AP_CHANNEL = 1;

constexpr uint32_t MPU_READ_PERIOD_MS = 100;
constexpr uint32_t MPU_DEBUG_PERIOD_MS = 1000;
constexpr uint32_t MPU_RETRY_PERIOD_MS = 2000;
constexpr uint32_t I2C_SCAN_PERIOD_MS = 3000;
} // namespace Config

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

  float accelRollDeg() const {
    const float ay = static_cast<float>(accelY);
    const float az = static_cast<float>(accelZ);
    return atan2f(ay, az) * 180.0f / PI;
  }

  float temperatureC() const {
    return static_cast<float>(temperatureRaw) / 340.0f + 36.53f;
  }

  float gyroXDps() const {
    return static_cast<float>(gyroX) / GYRO_LSB_PER_DPS;
  }

  float gyroYDps() const {
    return static_cast<float>(gyroY) / GYRO_LSB_PER_DPS;
  }

  float gyroZDps() const {
    return static_cast<float>(gyroZ) / GYRO_LSB_PER_DPS;
  }

  int16_t accelX = 0;
  int16_t accelY = 0;
  int16_t accelZ = 0;
  int16_t gyroX = 0;
  int16_t gyroY = 0;
  int16_t gyroZ = 0;
  int16_t temperatureRaw = 0;

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
    writeRegister(PWR_MGMT_1, 0x01);
    delay(10);

    lastWhoAmI = readRegister(WHO_AM_I);
    lastSupportedId = isSupportedWhoAmI(lastWhoAmI);
    if (!lastSupportedId) {
      return false;
    }

    writeRegister(CONFIG, 0x03);
    writeRegister(SMPLRT_DIV, 0x04);
    writeRegister(GYRO_CONFIG, 0x00);
    writeRegister(ACCEL_CONFIG, 0x00);
    writeRegister(INT_ENABLE, 0x00);
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

enum class MotorSide {
  Left,
  Right,
  Both
};

enum class Direction {
  Stop = 0,
  Forward = 1,
  Backward = -1
};

int currentPwm = Config::DEFAULT_PWM;
uint32_t stopAtMs = 0;
MotorSide activeSide = MotorSide::Both;
Direction activeDirection = Direction::Stop;
WebServer server(80);
Mpu6050 mpu;
bool mpuReady = false;
uint32_t lastMpuReadMs = 0;
uint32_t lastMpuDebugMs = 0;
uint32_t lastMpuRetryMs = 0;
uint32_t lastI2cScanMs = 0;
uint8_t lastI2cDeviceCount = 0;
String lastI2cScan = F("sin escanear");

void printHelp();
void setupMpu();
void setupWebServer();
void updateMpu();
void scanI2cBus(bool printResult);
uint32_t remainingRunMs();

const char INDEX_HTML[] PROGMEM = R"html(
<!doctype html>
<html lang="es">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>Robot Test L298N</title>
  <style>
    :root {
      color: #111827;
      background: #eef2f7;
      font-family: system-ui, -apple-system, BlinkMacSystemFont, "Segoe UI", sans-serif;
    }
    * {
      box-sizing: border-box;
    }
    body {
      margin: 0;
      padding: 20px;
    }
    main {
      max-width: 560px;
      margin: 0 auto;
    }
    h1 {
      margin: 0 0 14px;
      font-size: 1.45rem;
      line-height: 1.15;
    }
    .status {
      display: grid;
      grid-template-columns: repeat(2, minmax(0, 1fr));
      gap: 10px;
      margin-bottom: 14px;
    }
    .tile {
      min-width: 0;
      padding: 12px;
      border: 1px solid #d6dde7;
      border-radius: 8px;
      background: #ffffff;
    }
    .label {
      display: block;
      margin-bottom: 4px;
      color: #5b6573;
      font-size: 0.78rem;
    }
    .value {
      font-size: 1.35rem;
      font-weight: 700;
      line-height: 1.1;
    }
    .value.small {
      font-size: 1rem;
      overflow-wrap: anywhere;
    }
    .pad {
      display: grid;
      grid-template-columns: repeat(3, minmax(0, 1fr));
      gap: 10px;
      margin-bottom: 14px;
    }
    button {
      min-height: 58px;
      border: 0;
      border-radius: 8px;
      background: #1f6feb;
      color: #ffffff;
      font: inherit;
      font-weight: 700;
      cursor: pointer;
      touch-action: manipulation;
    }
    button:active {
      transform: translateY(1px);
    }
    .secondary {
      background: #334155;
    }
    .stop {
      grid-column: 1 / -1;
      background: #c2410c;
    }
    .pwm,
    .sensor {
      display: grid;
      gap: 10px;
      padding: 12px;
      border: 1px solid #d6dde7;
      border-radius: 8px;
      background: #ffffff;
    }
    .sensor {
      margin-bottom: 14px;
    }
    .pwm-head {
      display: flex;
      align-items: center;
      justify-content: space-between;
      gap: 12px;
      font-weight: 700;
    }
    .readings {
      display: grid;
      grid-template-columns: repeat(2, minmax(0, 1fr));
      gap: 8px 12px;
      font-size: 0.92rem;
    }
    .reading {
      display: flex;
      justify-content: space-between;
      gap: 10px;
      min-width: 0;
    }
    .reading span:first-child {
      color: #5b6573;
    }
    .reading span:last-child {
      font-variant-numeric: tabular-nums;
      overflow-wrap: anywhere;
      text-align: right;
    }
    input[type="range"] {
      width: 100%;
      accent-color: #1f6feb;
    }
    .pwm-buttons {
      display: grid;
      grid-template-columns: repeat(2, minmax(0, 1fr));
      gap: 10px;
    }
    @media (max-width: 420px) {
      body {
        padding: 14px;
      }
      button {
        min-height: 54px;
      }
    }
  </style>
</head>
<body>
  <main>
    <h1>Robot Test L298N</h1>
    <section class="status" aria-live="polite">
      <div class="tile">
        <span class="label">PWM</span>
        <span class="value" id="pwmValue">--</span>
      </div>
      <div class="tile">
        <span class="label">Motor</span>
        <span class="value" id="motorState">--</span>
      </div>
      <div class="tile">
        <span class="label">MPU6050</span>
        <span class="value small" id="sensorState">--</span>
      </div>
      <div class="tile">
        <span class="label">Pitch</span>
        <span class="value small" id="pitchValue">--</span>
      </div>
    </section>

    <section class="sensor">
      <div class="pwm-head">
        <span>Lecturas sensor</span>
        <span id="mpuAddress">--</span>
      </div>
      <div class="readings">
        <div class="reading"><span>Roll</span><span id="rollValue">--</span></div>
        <div class="reading"><span>Temp</span><span id="tempValue">--</span></div>
        <div class="reading"><span>Acc X</span><span id="accelXValue">--</span></div>
        <div class="reading"><span>Acc Y</span><span id="accelYValue">--</span></div>
        <div class="reading"><span>Acc Z</span><span id="accelZValue">--</span></div>
        <div class="reading"><span>Gyro Y</span><span id="gyroYValue">--</span></div>
        <div class="reading"><span>Bus I2C</span><span id="i2cScanValue">--</span></div>
      </div>
    </section>

    <section class="pad">
      <button class="secondary" type="button" onclick="sendCommand('l')">Izq.</button>
      <button type="button" onclick="sendCommand('f')">Adelante</button>
      <button class="secondary" type="button" onclick="sendCommand('r')">Der.</button>
      <button class="secondary" type="button" onclick="setPwmDelta(-15)">PWM -</button>
      <button type="button" onclick="sendCommand('b')">Atras</button>
      <button class="secondary" type="button" onclick="setPwmDelta(15)">PWM +</button>
      <button class="stop" type="button" onclick="sendCommand('s')">Stop</button>
    </section>

    <section class="pwm">
      <div class="pwm-head">
        <span>Ajuste PWM</span>
        <span id="pwmSliderValue">--</span>
      </div>
      <input id="pwmSlider" type="range" min="0" max="255" step="1" value="90">
      <div class="pwm-buttons">
        <button class="secondary" type="button" onclick="setPwmValue(90)">90</button>
        <button class="secondary" type="button" onclick="setPwmValue(140)">140</button>
      </div>
    </section>
  </main>

  <script>
    const pwmValue = document.getElementById('pwmValue');
    const pwmSliderValue = document.getElementById('pwmSliderValue');
    const motorState = document.getElementById('motorState');
    const pwmSlider = document.getElementById('pwmSlider');
    const sensorState = document.getElementById('sensorState');
    const pitchValue = document.getElementById('pitchValue');
    const rollValue = document.getElementById('rollValue');
    const tempValue = document.getElementById('tempValue');
    const accelXValue = document.getElementById('accelXValue');
    const accelYValue = document.getElementById('accelYValue');
    const accelZValue = document.getElementById('accelZValue');
    const gyroYValue = document.getElementById('gyroYValue');
    const i2cScanValue = document.getElementById('i2cScanValue');
    const mpuAddress = document.getElementById('mpuAddress');
    let pwmTimer = 0;

    function formatHex(value) {
      return Number.isFinite(value) ? '0x' + value.toString(16).toUpperCase().padStart(2, '0') : '--';
    }

    function formatNumber(value, digits = 1) {
      return Number.isFinite(value) ? value.toFixed(digits) : '--';
    }

    function renderStatus(data) {
      pwmValue.textContent = data.pwm;
      pwmSliderValue.textContent = data.pwm;
      pwmSlider.value = data.pwm;
      motorState.textContent = data.running ? 'Activo' : 'Parado';

      const mpu = data.mpu || { ready: false };
      sensorState.textContent = mpu.ready ? 'OK' : (mpu.addressAcked ? 'ID ' + formatHex(mpu.whoAmI) : 'No detectado');
      mpuAddress.textContent = (mpu.ready || mpu.addressAcked) ? formatHex(mpu.address) + ' / WHO ' + formatHex(mpu.whoAmI) : '--';
      pitchValue.textContent = mpu.ready ? formatNumber(mpu.pitch, 1) + ' deg' : '--';
      rollValue.textContent = mpu.ready ? formatNumber(mpu.roll, 1) + ' deg' : '--';
      tempValue.textContent = mpu.ready ? formatNumber(mpu.temp, 1) + ' C' : '--';
      accelXValue.textContent = mpu.ready ? mpu.ax : '--';
      accelYValue.textContent = mpu.ready ? mpu.ay : '--';
      accelZValue.textContent = mpu.ready ? mpu.az : '--';
      gyroYValue.textContent = mpu.ready ? formatNumber(mpu.gyroY, 1) + ' dps' : '--';
      i2cScanValue.textContent = mpu.scan || '--';
    }

    async function request(path) {
      const response = await fetch(path, { cache: 'no-store' });
      if (!response.ok) {
        throw new Error(await response.text());
      }
      renderStatus(await response.json());
    }

    function sendCommand(command) {
      request('/cmd?c=' + encodeURIComponent(command));
    }

    function setPwmDelta(delta) {
      request('/pwm?delta=' + encodeURIComponent(delta));
    }

    function setPwmValue(value) {
      request('/pwm?value=' + encodeURIComponent(value));
    }

    pwmSlider.addEventListener('input', () => {
      pwmSliderValue.textContent = pwmSlider.value;
      clearTimeout(pwmTimer);
      pwmTimer = setTimeout(() => setPwmValue(pwmSlider.value), 120);
    });

    request('/status');
    setInterval(() => request('/status'), 600);
  </script>
</body>
</html>
)html";

void setupPwm() {
  ledcSetup(Config::LEFT_PWM_CHANNEL, Config::PWM_FREQUENCY_HZ, Config::PWM_RESOLUTION_BITS);
  ledcSetup(Config::RIGHT_PWM_CHANNEL, Config::PWM_FREQUENCY_HZ, Config::PWM_RESOLUTION_BITS);
  ledcAttachPin(Config::LEFT_PWM_PIN, Config::LEFT_PWM_CHANNEL);
  ledcAttachPin(Config::RIGHT_PWM_PIN, Config::RIGHT_PWM_CHANNEL);
}

void setupMpu() {
  Wire.begin(Config::I2C_SDA_PIN, Config::I2C_SCL_PIN, Config::I2C_CLOCK_HZ);
  Wire.setTimeOut(50);

  Serial.print(F("I2C MPU6050 SDA=GPIO"));
  Serial.print(Config::I2C_SDA_PIN);
  Serial.print(F(" SCL=GPIO"));
  Serial.println(Config::I2C_SCL_PIN);
  Serial.print(F("I2C clock: "));
  Serial.println(Config::I2C_CLOCK_HZ);

  scanI2cBus(true);
  mpuReady = mpu.begin();
  if (mpuReady) {
    Serial.print(F("MPU6050 detectado en 0x"));
    Serial.print(mpu.i2cAddress(), HEX);
    Serial.print(F(" WHO_AM_I=0x"));
    Serial.println(mpu.whoAmI(), HEX);
  } else {
    if (mpu.addressAcked()) {
      Serial.print(F("MPU con ACK en 0x"));
      Serial.print(mpu.i2cAddress(), HEX);
      Serial.print(F(" pero WHO_AM_I=0x"));
      Serial.print(mpu.whoAmI(), HEX);
      Serial.println(F(" no fue aceptado como MPU compatible."));
    }
    Serial.println(F("MPU6050 no detectado. El test de motores sigue activo."));
  }
}

void scanI2cBus(bool printResult) {
  String found;
  uint8_t count = 0;

  for (uint8_t address = 1; address < 127; ++address) {
    Wire.beginTransmission(address);
    const uint8_t error = Wire.endTransmission();
    if (error == 0) {
      if (count > 0) {
        found += F(", ");
      }
      found += F("0x");
      if (address < 16) {
        found += F("0");
      }
      found += String(address, HEX);
      ++count;
    }
  }

  found.toUpperCase();
  lastI2cDeviceCount = count;
  lastI2cScan = count > 0 ? found : F("ninguno");

  if (printResult) {
    Serial.print(F("I2C scan: "));
    Serial.println(lastI2cScan);
  }
}

void applyMotor(uint8_t in1, uint8_t in2, uint8_t pwmChannel, Direction direction, int pwm, bool invert) {
  int sign = static_cast<int>(direction);
  if (invert) {
    sign = -sign;
  }

  if (sign > 0) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  } else if (sign < 0) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
  } else {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
  }

  ledcWrite(pwmChannel, sign == 0 ? 0 : constrain(pwm, 0, Config::PWM_MAX));
}

void drive(MotorSide side, Direction direction, int pwm) {
  if (side == MotorSide::Left || side == MotorSide::Both) {
    applyMotor(
        Config::LEFT_IN1_PIN,
        Config::LEFT_IN2_PIN,
        Config::LEFT_PWM_CHANNEL,
        direction,
        pwm,
        Config::INVERT_LEFT_MOTOR);
  }

  if (side == MotorSide::Right || side == MotorSide::Both) {
    applyMotor(
        Config::RIGHT_IN1_PIN,
        Config::RIGHT_IN2_PIN,
        Config::RIGHT_PWM_CHANNEL,
        direction,
        pwm,
        Config::INVERT_RIGHT_MOTOR);
  }
}

void stopMotors() {
  drive(MotorSide::Both, Direction::Stop, 0);
  activeSide = MotorSide::Both;
  activeDirection = Direction::Stop;
  stopAtMs = 0;
}

void runTimed(MotorSide side, Direction direction, int pwm, uint32_t durationMs) {
  stopMotors();
  drive(side, direction, pwm);
  activeSide = side;
  activeDirection = direction;
  stopAtMs = millis() + durationMs;

  Serial.print(F("Motor "));
  if (side == MotorSide::Left) {
    Serial.print(F("izquierdo"));
  } else if (side == MotorSide::Right) {
    Serial.print(F("derecho"));
  } else {
    Serial.print(F("ambos"));
  }

  Serial.print(direction == Direction::Forward ? F(" adelante") : F(" atras"));
  Serial.print(F(" | PWM="));
  Serial.println(pwm);
}

void setPwm(int pwm) {
  currentPwm = constrain(pwm, 0, Config::PWM_MAX);
  if (activeDirection != Direction::Stop && remainingRunMs() > 0) {
    drive(activeSide, activeDirection, currentPwm);
  }

  Serial.print(F("PWM actual: "));
  Serial.println(currentPwm);
}

uint32_t remainingRunMs() {
  if (stopAtMs == 0) {
    return 0;
  }

  const int32_t remaining = static_cast<int32_t>(stopAtMs - millis());
  return remaining > 0 ? static_cast<uint32_t>(remaining) : 0;
}

void appendFloat(String &json, float value, uint8_t decimals) {
  if (isfinite(value)) {
    json += String(value, static_cast<unsigned int>(decimals));
  } else {
    json += F("null");
  }
}

void appendJsonString(String &json, const String &value) {
  json += '"';
  for (size_t i = 0; i < value.length(); ++i) {
    const char c = value[i];
    if (c == '"' || c == '\\') {
      json += '\\';
    }
    json += c;
  }
  json += '"';
}

void appendMpuStatus(String &json) {
  json += F(",\"mpu\":{\"ready\":");
  json += (mpuReady ? F("true") : F("false"));
  json += F(",\"scan\":");
  appendJsonString(json, lastI2cScan);
  json += F(",\"deviceCount\":");
  json += lastI2cDeviceCount;
  json += F(",\"address\":");
  json += mpu.i2cAddress();
  json += F(",\"whoAmI\":");
  json += mpu.whoAmI();
  json += F(",\"addressAcked\":");
  json += (mpu.addressAcked() ? F("true") : F("false"));
  json += F(",\"supportedId\":");
  json += (mpu.supportedId() ? F("true") : F("false"));

  if (mpuReady) {
    json += F(",\"ax\":");
    json += mpu.accelX;
    json += F(",\"ay\":");
    json += mpu.accelY;
    json += F(",\"az\":");
    json += mpu.accelZ;
    json += F(",\"gx\":");
    json += mpu.gyroX;
    json += F(",\"gy\":");
    json += mpu.gyroY;
    json += F(",\"gz\":");
    json += mpu.gyroZ;
    json += F(",\"pitch\":");
    appendFloat(json, mpu.accelPitchDeg(), 2);
    json += F(",\"roll\":");
    appendFloat(json, mpu.accelRollDeg(), 2);
    json += F(",\"gyroY\":");
    appendFloat(json, mpu.gyroYDps(), 2);
    json += F(",\"temp\":");
    appendFloat(json, mpu.temperatureC(), 2);
  }

  json += F("}");
}

void sendStatus() {
  const uint32_t remainingMs = remainingRunMs();
  String json = F("{\"pwm\":");
  json += currentPwm;
  json += F(",\"max\":");
  json += Config::PWM_MAX;
  json += F(",\"running\":");
  json += (remainingMs > 0 ? F("true") : F("false"));
  json += F(",\"remainingMs\":");
  json += remainingMs;
  appendMpuStatus(json);
  json += F("}");

  server.send(200, F("application/json"), json);
}

void updateMpu() {
  const uint32_t nowMs = millis();

  if (!mpuReady && nowMs - lastI2cScanMs >= Config::I2C_SCAN_PERIOD_MS) {
    lastI2cScanMs = nowMs;
    scanI2cBus(true);
  }

  if (!mpuReady && nowMs - lastMpuRetryMs >= Config::MPU_RETRY_PERIOD_MS) {
    lastMpuRetryMs = nowMs;
    mpuReady = mpu.begin();
    if (mpuReady) {
      Serial.print(F("MPU6050 detectado en reintento 0x"));
      Serial.println(mpu.i2cAddress(), HEX);
    }
  }

  if (nowMs - lastMpuReadMs < Config::MPU_READ_PERIOD_MS) {
    return;
  }
  lastMpuReadMs = nowMs;

  if (!mpuReady) {
    return;
  }

  if (!mpu.readRaw()) {
    mpuReady = false;
    Serial.println(F("Error leyendo MPU6050. Queda marcado como no detectado."));
    return;
  }

  if (nowMs - lastMpuDebugMs >= Config::MPU_DEBUG_PERIOD_MS) {
    lastMpuDebugMs = nowMs;
    Serial.print(F("MPU pitch="));
    Serial.print(mpu.accelPitchDeg(), 1);
    Serial.print(F(" roll="));
    Serial.print(mpu.accelRollDeg(), 1);
    Serial.print(F(" gyroY="));
    Serial.print(mpu.gyroYDps(), 1);
    Serial.print(F(" temp="));
    Serial.println(mpu.temperatureC(), 1);
  }
}

void handleRoot() {
  server.send_P(200, PSTR("text/html"), INDEX_HTML);
}

void handleWebCommand() {
  if (!server.hasArg("c") || server.arg("c").length() == 0) {
    server.send(400, F("text/plain"), F("Falta parametro c"));
    return;
  }

  const char command = server.arg("c")[0];
  switch (command) {
    case 's':
    case 'S':
      stopMotors();
      Serial.println(F("Motores detenidos."));
      break;

    case 'f':
    case 'F':
      runTimed(MotorSide::Both, Direction::Forward, currentPwm, Config::COMMAND_RUN_MS);
      break;

    case 'b':
    case 'B':
      runTimed(MotorSide::Both, Direction::Backward, currentPwm, Config::COMMAND_RUN_MS);
      break;

    case 'l':
    case 'L':
      runTimed(MotorSide::Left, Direction::Forward, currentPwm, Config::COMMAND_RUN_MS);
      break;

    case 'r':
    case 'R':
      runTimed(MotorSide::Right, Direction::Forward, currentPwm, Config::COMMAND_RUN_MS);
      break;

    default:
      server.send(400, F("text/plain"), F("Comando invalido"));
      return;
  }

  sendStatus();
}

void handleWebPwm() {
  if (server.hasArg("value")) {
    setPwm(server.arg("value").toInt());
  } else if (server.hasArg("delta")) {
    setPwm(currentPwm + server.arg("delta").toInt());
  } else {
    server.send(400, F("text/plain"), F("Falta parametro value o delta"));
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
  server.on("/cmd", HTTP_GET, handleWebCommand);
  server.on("/pwm", HTTP_GET, handleWebPwm);
  server.onNotFound([]() {
    server.send(404, F("text/plain"), F("No encontrado"));
  });
  server.begin();

  Serial.println(F("Servidor HTTP iniciado."));
}

void handleCommand(char command) {
  switch (command) {
    case 'h':
    case 'H':
      printHelp();
      break;

    case 's':
    case 'S':
      stopMotors();
      Serial.println(F("Motores detenidos."));
      break;

    case 'f':
    case 'F':
      runTimed(MotorSide::Both, Direction::Forward, currentPwm, Config::COMMAND_RUN_MS);
      break;

    case 'b':
    case 'B':
      runTimed(MotorSide::Both, Direction::Backward, currentPwm, Config::COMMAND_RUN_MS);
      break;

    case 'l':
    case 'L':
      runTimed(MotorSide::Left, Direction::Forward, currentPwm, Config::COMMAND_RUN_MS);
      break;

    case 'r':
    case 'R':
      runTimed(MotorSide::Right, Direction::Forward, currentPwm, Config::COMMAND_RUN_MS);
      break;

    case '+':
      setPwm(currentPwm + Config::PWM_STEP);
      break;

    case '-':
      setPwm(currentPwm - Config::PWM_STEP);
      break;

    case 'i':
    case 'I':
      scanI2cBus(true);
      break;

    default:
      if (command >= '0' && command <= '9') {
        setPwm(map(command - '0', 0, 9, 0, Config::PWM_MAX));
      }
      break;
  }
}

void printHelp() {
  Serial.println();
  Serial.println(F("=== Test L298N Motores ESP32-C3 ==="));
  Serial.println(F("h: ayuda"));
  Serial.println(F("s: parar"));
  Serial.println(F("f: ambos adelante 2 s"));
  Serial.println(F("b: ambos atras 2 s"));
  Serial.println(F("l: izquierdo adelante 2 s"));
  Serial.println(F("r: derecho adelante 2 s"));
  Serial.println(F("+: subir PWM"));
  Serial.println(F("-: bajar PWM"));
  Serial.println(F("0..9: PWM de 0 a 255"));
  Serial.println(F("i: escanear bus I2C"));
  Serial.print(F("Web: http://"));
  Serial.println(WiFi.softAPIP());
  Serial.print(F("PWM actual: "));
  Serial.println(currentPwm);
  Serial.println();
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  pinMode(Config::LEFT_IN1_PIN, OUTPUT);
  pinMode(Config::LEFT_IN2_PIN, OUTPUT);
  pinMode(Config::RIGHT_IN1_PIN, OUTPUT);
  pinMode(Config::RIGHT_IN2_PIN, OUTPUT);

  setupPwm();
  stopMotors();
  setupMpu();
  setupWebServer();

  printHelp();
}

void loop() {
  server.handleClient();
  updateMpu();

  while (Serial.available()) {
    handleCommand(static_cast<char>(Serial.read()));
  }

  if (stopAtMs != 0 && static_cast<int32_t>(millis() - stopAtMs) >= 0) {
    stopMotors();
    Serial.println(F("Tiempo cumplido. Motores detenidos."));
  }
}
