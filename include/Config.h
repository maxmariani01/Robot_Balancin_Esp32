#pragma once

#include <Arduino.h>

// ============================================================================
//  Config.h - Parametros del robot balancin ESP32-C3
// ============================================================================
//  Editar este archivo es lo unico necesario para ajustar el robot.
//
//  Secciones:
//    1. [AJUSTAR] PID
//    2. [AJUSTAR] Punto de equilibrio
//    3. [AJUSTAR] Ventanas de armado / caida
//    4. [AJUSTAR] Motores (factores y velocidad minima)
//    5. [AJUSTAR] Inversiones (signos)
//    6. [AJUSTAR] Pines (GPIO segun cableado)
//    7. [AVANZADO] Lazo de control, filtro, PWM e I2C
// ============================================================================

namespace Config {

// ----------------------------------------------------------------------------
// 1. [AJUSTAR] PID
// ----------------------------------------------------------------------------
//   Si oscila rapido:           bajar KP   o subir KD.
//   Si reacciona tarde:         subir KP.
//   Si queda inclinado parado:  subir KI.
//   Si "zumba" / vibra:         bajar KI   o bajar KD.
//
//   Nota: con derivada sobre la medicion, KD pesa distinto que en PID clasico.
//   Estos valores son conservadores para ajustar con movimientos cortos.

constexpr float KP = 25.0f;
constexpr float KI = 0.0f;
constexpr float KD = 5.5f;

// ----------------------------------------------------------------------------
// 2. [AJUSTAR] Punto de equilibrio
// ----------------------------------------------------------------------------
//   Angulo (grados) donde el robot queda parado sin que los motores empujen.
//   Para calibrarlo: cargar el firmware, mantener el robot en pie sin armar,
//   y leer el "Pitch:" que sale por serial. Ese valor va aca.

constexpr float BALANCE_ANGLE_DEG = 4.5f;

// ----------------------------------------------------------------------------
// 3. [AJUSTAR] Ventanas de armado / caida
// ----------------------------------------------------------------------------
//   START_ANGLE_WINDOW_DEG:  cuanto cerca del equilibrio hay que ponerlo
//                            (a mano) para que se active el control.
//   ARM_STABLE_TIME_MS:      cuanto tiempo tiene que estar dentro de esa
//                            ventana antes de armar.
//   FALL_ANGLE_DEG:          si se inclina mas que esto se considera caido,
//                            se cortan los motores y hay que volver a armar.

constexpr float START_ANGLE_WINDOW_DEG = 12.0f;
constexpr uint32_t ARM_STABLE_TIME_MS = 500;
constexpr float FALL_ANGLE_DEG = 45.0f;

// ----------------------------------------------------------------------------
// 4. [AJUSTAR] Motores
// ----------------------------------------------------------------------------
//   MIN_ABS_SPEED:        PWM minimo (0-255) para vencer friccion estatica.
//                         Si las ruedas no arrancan, subir.
//   LEFT/RIGHT_FACTOR:    ganancia individual por rueda (0..1). Si una rueda
//                         gira mas rapido que la otra, bajar su factor para
//                         compensar. 0.60 = se entrega como mucho 60% del PWM
//                         calculado por el PID.

constexpr int MIN_ABS_SPEED = 30;
constexpr float LEFT_SPEED_FACTOR = 0.60f;
constexpr float RIGHT_SPEED_FACTOR = 0.60f;

// ----------------------------------------------------------------------------
// 5. [AJUSTAR] Inversiones
// ----------------------------------------------------------------------------
//   La salida global queda invertida por defecto para este cableado.
//   Si el pitch queda con signo contrario, ajustar INVERT_PITCH.
//
//   INVERT_PITCH:          la IMU esta montada al reves -> invierte el angulo.
//   INVERT_MOTOR_OUTPUT:   invierte el sentido global del comando PWM.
//   INVERT_LEFT_MOTOR:     invierte el sentido de la rueda izquierda
//                          (util si cableaste IN1/IN2 al reves).
//   INVERT_RIGHT_MOTOR:    idem para la derecha.

constexpr bool INVERT_PITCH = false;
constexpr bool INVERT_MOTOR_OUTPUT = true;
constexpr bool INVERT_LEFT_MOTOR = false;
constexpr bool INVERT_RIGHT_MOTOR = false;

// ----------------------------------------------------------------------------
// 6. [AJUSTAR] Pines (GPIO ESP32-C3 DevKitC-02)
// ----------------------------------------------------------------------------

// MPU6050 (I2C):
constexpr uint8_t I2C_SDA_PIN = 8;
constexpr uint8_t I2C_SCL_PIN = 9;

// L298N - motor izquierdo:
constexpr uint8_t LEFT_PWM_PIN = 3;   // ENA
constexpr uint8_t LEFT_IN1_PIN = 4;
constexpr uint8_t LEFT_IN2_PIN = 5;

// L298N - motor derecho:
constexpr uint8_t RIGHT_PWM_PIN = 6;  // ENB
constexpr uint8_t RIGHT_IN1_PIN = 7;
constexpr uint8_t RIGHT_IN2_PIN = 10;

// Pin INT del MPU6050 (solo se usa si USE_MPU_INTERRUPT = true):
constexpr uint8_t MPU_INT_PIN = 2;

// ============================================================================
// 7. [AVANZADO] No tocar salvo que sepas que estas haciendo
// ============================================================================

// Lazo de control:
constexpr uint32_t CONTROL_PERIOD_US = 5000;   // 200 Hz.
constexpr uint32_t DEBUG_PERIOD_MS = 100;
constexpr uint32_t MPU_RETRY_PERIOD_MS = 2000;
constexpr bool USE_MPU_INTERRUPT = false;      // Hoy el loop usa polling.

// Filtro complementario (0..1; mas alto = mas peso al gyro, menos al accel):
constexpr float COMPLEMENTARY_ALPHA = 0.98f;

// I2C:
constexpr uint32_t I2C_CLOCK_HZ = 400000;

// PWM (LEDC):
constexpr uint8_t LEFT_PWM_CHANNEL = 0;
constexpr uint8_t RIGHT_PWM_CHANNEL = 1;
constexpr uint32_t PWM_FREQUENCY_HZ = 20000;
constexpr uint8_t PWM_RESOLUTION_BITS = 8;
constexpr int PWM_MAX = (1 << PWM_RESOLUTION_BITS) - 1;

// WiFi AP para ajustar el robot desde el navegador:
constexpr char WIFI_AP_SSID[] = "RobotBalancin";
constexpr char WIFI_AP_PASSWORD[] = "12345678";
constexpr uint8_t WIFI_AP_CHANNEL = 1;

// LED integrado de estado:
constexpr bool STATUS_LED_ENABLED = true;
constexpr uint8_t STATUS_LED_BRIGHTNESS = 24; // 0..255

} // namespace Config
