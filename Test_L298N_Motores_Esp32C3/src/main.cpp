#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>

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

constexpr char WIFI_AP_SSID[] = "RobotTest";
constexpr char WIFI_AP_PASSWORD[] = "12345678";
constexpr uint8_t WIFI_AP_CHANNEL = 1;
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
MotorSide activeSide = MotorSide::Both;
Direction activeDirection = Direction::Stop;
WebServer server(80);

void printHelp();
void setupWebServer();
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
    .pwm {
      display: grid;
      gap: 10px;
      padding: 12px;
      border: 1px solid #d6dde7;
      border-radius: 8px;
      background: #ffffff;
    }
    .pwm-head {
      display: flex;
      align-items: center;
      justify-content: space-between;
      gap: 12px;
      font-weight: 700;
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
    let pwmTimer = 0;

    function renderStatus(data) {
      pwmValue.textContent = data.pwm;
      pwmSliderValue.textContent = data.pwm;
      pwmSlider.value = data.pwm;
      motorState.textContent = data.running ? 'Activo' : 'Parado';
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
  json += F("}");

  server.send(200, F("application/json"), json);
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
  setupWebServer();

  printHelp();
}

void loop() {
  server.handleClient();

  while (Serial.available()) {
    handleCommand(static_cast<char>(Serial.read()));
  }

  if (stopAtMs != 0 && static_cast<int32_t>(millis() - stopAtMs) >= 0) {
    stopMotors();
    Serial.println(F("Tiempo cumplido. Motores detenidos."));
  }
}
