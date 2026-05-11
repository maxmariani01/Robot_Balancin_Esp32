// ============================================================================
//  Robot balancin - ESP32-C3 DevKitC-02 + MPU6050 + L298N
// ============================================================================
//  Para ajustar el comportamiento del robot, editar include/Config.h.
//  Este archivo solo contiene la logica (drivers, PID, filtro y lazo).
// ============================================================================

#include <Arduino.h>
#include <Wire.h>
#include <math.h>

#include "Config.h"

// ============================================================================
//  MPU6050 - driver I2C minimo (sin I2Cdev ni DMP)
// ============================================================================

class Mpu6050 {
public:
  bool begin() {
    writeRegister(PWR_MGMT_1, 0x80);
    delay(100);
    writeRegister(PWR_MGMT_1, 0x01); // PLL con eje X gyro.
    delay(10);

    const uint8_t who = readRegister(WHO_AM_I);
    if (who != 0x68 && who != 0x69) {
      Serial.print(F("MPU6050 no detectado. WHO_AM_I=0x"));
      Serial.println(who, HEX);
      return false;
    }

    writeRegister(CONFIG, 0x03);       // DLPF ~44 Hz gyro / ~42 Hz accel.
    writeRegister(SMPLRT_DIV, 0x04);   // 1 kHz / (1 + 4) = 200 Hz.
    writeRegister(GYRO_CONFIG, 0x00);  // +/-250 dps.
    writeRegister(ACCEL_CONFIG, 0x00); // +/-2 g.
    writeRegister(INT_ENABLE, 0x01);   // Data ready.
    return true;
  }

  bool readRaw() {
    Wire.beginTransmission(ADDRESS);
    Wire.write(ACCEL_XOUT_H);
    if (Wire.endTransmission(false) != 0) {
      return false;
    }

    const uint8_t expected = 14;
    const uint8_t received = Wire.requestFrom(ADDRESS, expected);
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
    }

    if (validSamples < samples / 2) {
      return false;
    }

    gyroBiasX = static_cast<float>(sumX) / static_cast<float>(validSamples);
    gyroBiasY = static_cast<float>(sumY) / static_cast<float>(validSamples);
    gyroBiasZ = static_cast<float>(sumZ) / static_cast<float>(validSamples);
    return true;
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
  static constexpr uint8_t ADDRESS = 0x68;
  static constexpr float GYRO_LSB_PER_DPS = 131.0f;

  static constexpr uint8_t SMPLRT_DIV = 0x19;
  static constexpr uint8_t CONFIG = 0x1A;
  static constexpr uint8_t GYRO_CONFIG = 0x1B;
  static constexpr uint8_t ACCEL_CONFIG = 0x1C;
  static constexpr uint8_t INT_ENABLE = 0x38;
  static constexpr uint8_t ACCEL_XOUT_H = 0x3B;
  static constexpr uint8_t PWR_MGMT_1 = 0x6B;
  static constexpr uint8_t WHO_AM_I = 0x75;

  void writeRegister(uint8_t reg, uint8_t value) const {
    Wire.beginTransmission(ADDRESS);
    Wire.write(reg);
    Wire.write(value);
    Wire.endTransmission();
  }

  uint8_t readRegister(uint8_t reg) const {
    Wire.beginTransmission(ADDRESS);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0) {
      return 0xFF;
    }
    if (Wire.requestFrom(ADDRESS, static_cast<uint8_t>(1)) != 1) {
      return 0xFF;
    }
    return Wire.read();
  }

  int16_t readWord() const {
    const uint8_t highByte = Wire.read();
    const uint8_t lowByte = Wire.read();
    return static_cast<int16_t>((highByte << 8) | lowByte);
  }
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
  float compute(float setpoint, float input, float measurementRate, float dtSeconds) {
    const float error = setpoint - input;
    const float derivative = -measurementRate;
    const float maxOut = static_cast<float>(Config::PWM_MAX);

    const float pdOut = Config::KP * error + Config::KD * derivative;
    const float candidateI = integralTerm + Config::KI * error * dtSeconds;
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

  void drive(float command) {
    if (Config::INVERT_MOTOR_OUTPUT) {
      command = -command;
    }

    const int pwm = constrain(static_cast<int>(lroundf(command)), -Config::PWM_MAX, Config::PWM_MAX);
    driveOneMotor(
        Config::LEFT_IN1_PIN,
        Config::LEFT_IN2_PIN,
        Config::LEFT_PWM_CHANNEL,
        pwm,
        Config::LEFT_SPEED_FACTOR,
        Config::INVERT_LEFT_MOTOR);
    driveOneMotor(
        Config::RIGHT_IN1_PIN,
        Config::RIGHT_IN2_PIN,
        Config::RIGHT_PWM_CHANNEL,
        pwm,
        Config::RIGHT_SPEED_FACTOR,
        Config::INVERT_RIGHT_MOTOR);
  }

  void stop() {
    ledcWrite(Config::LEFT_PWM_CHANNEL, 0);
    ledcWrite(Config::RIGHT_PWM_CHANNEL, 0);
    digitalWrite(Config::LEFT_IN1_PIN, LOW);
    digitalWrite(Config::LEFT_IN2_PIN, LOW);
    digitalWrite(Config::RIGHT_IN1_PIN, LOW);
    digitalWrite(Config::RIGHT_IN2_PIN, LOW);
  }

private:
  void driveOneMotor(uint8_t in1, uint8_t in2, uint8_t pwmChannel, int command, float factor, bool invert) const {
    if (invert) {
      command = -command;
    }

    int duty = abs(command);
    if (duty > 0 && duty < Config::MIN_ABS_SPEED) {
      duty = Config::MIN_ABS_SPEED;
    }

    duty = constrain(static_cast<int>(lroundf(static_cast<float>(duty) * factor)), 0, Config::PWM_MAX);

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
  }
};

// ============================================================================
//  Estado global
// ============================================================================

Mpu6050 mpu;
BalancePid pid;
L298MotorDriver motors;

volatile bool mpuDataReady = false;

float pitchDeg = 0.0f;
float pitchRateDps = 0.0f;
bool filterInitialized = false;

bool armed = false;
uint32_t armStartMs = 0;

uint32_t lastControlUs = 0;
uint32_t lastDebugMs = 0;
float lastOutput = 0.0f;

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
  const bool insideWindow = fabsf(angleDeg - Config::BALANCE_ANGLE_DEG) <= Config::START_ANGLE_WINDOW_DEG;
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

  if (Config::INVERT_PITCH) {
    accelPitch = -accelPitch;
    gyroRate = -gyroRate;
  }

  if (!filterInitialized) {
    pitchDeg = accelPitch;
    filterInitialized = true;
  } else {
    pitchDeg = Config::COMPLEMENTARY_ALPHA * (pitchDeg + gyroRate * dtSeconds) +
               (1.0f - Config::COMPLEMENTARY_ALPHA) * accelPitch;
  }

  pitchRateDps = gyroRate;
  return true;
}

// ============================================================================
//  setup() / loop()
// ============================================================================

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println();
  Serial.println(F("Robot balancin ESP32-C3"));

  Wire.begin(Config::I2C_SDA_PIN, Config::I2C_SCL_PIN, Config::I2C_CLOCK_HZ);

  motors.begin();
  if (Config::USE_MPU_INTERRUPT) {
    pinMode(Config::MPU_INT_PIN, INPUT);
    attachInterrupt(digitalPinToInterrupt(Config::MPU_INT_PIN), onMpuDataReady, RISING);
  }

  if (!mpu.begin()) {
    Serial.println(F("No se puede iniciar el control sin MPU6050."));
    while (true) {
      motors.stop();
      delay(1000);
    }
  }

  Serial.println(F("MPU6050 detectado. Mantener el robot quieto para calibrar gyro..."));
  if (!mpu.calibrateGyro()) {
    Serial.println(F("Fallo la calibracion del gyro."));
    while (true) {
      motors.stop();
      delay(1000);
    }
  }

  Serial.print(F("Bias gyro Y: "));
  Serial.println(mpu.gyroBiasY, 2);
  Serial.println(F("Colocar el robot cerca del equilibrio para activar el control."));

  lastControlUs = micros();
}

void loop() {
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
    Serial.println(F("Error leyendo MPU6050. Motores detenidos."));
    delay(2);
    return;
  }

  const bool fallen = fabsf(pitchDeg - Config::BALANCE_ANGLE_DEG) > Config::FALL_ANGLE_DEG;
  if (fallen) {
    if (armed) {
      Serial.println(F("Caida detectada. Control desactivado."));
    }
    resetArming();
  }

  if (armed) {
    lastOutput = pid.compute(Config::BALANCE_ANGLE_DEG, pitchDeg, pitchRateDps, dtSeconds);
    motors.drive(lastOutput);
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
    Serial.print(Config::BALANCE_ANGLE_DEG, 2);
    Serial.print(F(" | Output: "));
    Serial.print(lastOutput, 1);
    Serial.print(F(" | Estado: "));
    Serial.print(armed ? F("ARMADO") : F("ESPERA"));
    Serial.print(F(" | Temp: "));
    Serial.println(mpu.temperatureC(), 1);
  }
}
