#include <Arduino.h>
#include <math.h>

namespace Config {
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
} // namespace Config

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

void printHelp();

void setupPwm() {
  ledcSetup(Config::LEFT_PWM_CHANNEL, Config::PWM_FREQUENCY_HZ, Config::PWM_RESOLUTION_BITS);
  ledcSetup(Config::RIGHT_PWM_CHANNEL, Config::PWM_FREQUENCY_HZ, Config::PWM_RESOLUTION_BITS);
  ledcAttachPin(Config::LEFT_PWM_PIN, Config::LEFT_PWM_CHANNEL);
  ledcAttachPin(Config::RIGHT_PWM_PIN, Config::RIGHT_PWM_CHANNEL);
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
  stopAtMs = 0;
}

void runTimed(MotorSide side, Direction direction, int pwm, uint32_t durationMs) {
  stopMotors();
  drive(side, direction, pwm);
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

void waitWithStop(uint32_t ms) {
  const uint32_t start = millis();
  while (millis() - start < ms) {
    if (Serial.available()) {
      const char c = static_cast<char>(Serial.read());
      if (c == 's' || c == 'S') {
        stopMotors();
        Serial.println(F("Secuencia interrumpida."));
        return;
      }
    }
    delay(10);
  }
}

void autoSequence() {
  Serial.println(F("Secuencia automatica. Enviar 's' para cortar."));

  runTimed(MotorSide::Left, Direction::Forward, currentPwm, 1200);
  waitWithStop(1500);
  stopMotors();
  waitWithStop(500);

  runTimed(MotorSide::Right, Direction::Forward, currentPwm, 1200);
  waitWithStop(1500);
  stopMotors();
  waitWithStop(500);

  runTimed(MotorSide::Both, Direction::Forward, currentPwm, 1200);
  waitWithStop(1500);
  stopMotors();
  waitWithStop(500);

  runTimed(MotorSide::Both, Direction::Backward, currentPwm, 1200);
  waitWithStop(1500);
  stopMotors();

  Serial.println(F("Secuencia finalizada."));
}

void setPwm(int pwm) {
  currentPwm = constrain(pwm, 0, Config::PWM_MAX);
  Serial.print(F("PWM actual: "));
  Serial.println(currentPwm);
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

    case 'a':
    case 'A':
      autoSequence();
      break;

    case '+':
      setPwm(currentPwm + Config::PWM_STEP);
      break;

    case '-':
      setPwm(currentPwm - Config::PWM_STEP);
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
  Serial.println(F("a: secuencia automatica corta"));
  Serial.println(F("+: subir PWM"));
  Serial.println(F("-: bajar PWM"));
  Serial.println(F("0..9: PWM de 0 a 255"));
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

  printHelp();
}

void loop() {
  while (Serial.available()) {
    handleCommand(static_cast<char>(Serial.read()));
  }

  if (stopAtMs != 0 && static_cast<int32_t>(millis() - stopAtMs) >= 0) {
    stopMotors();
    Serial.println(F("Tiempo cumplido. Motores detenidos."));
  }
}
