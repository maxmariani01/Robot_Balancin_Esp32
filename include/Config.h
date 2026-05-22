#pragma once

#include <Arduino.h>

// ============================================================================
//  Config.h - Parametros del robot balancin ESP32-C3
// ============================================================================
//  Robot:  chasis aluminio, 11 cm de alto, 13 cm entre ruedas, motores TT.
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
//
//   Rango util observado para este robot (11 cm, motores TT):
//     KP 25..70   |   KI 10..60   |   KD 6..22
//   Los sliders de la web cubren un rango algo mas amplio con paso fino
//   para que se pueda explorar arriba y abajo de estos defaults.

constexpr float KP = 50.0f;
constexpr float KI = 20.0f;
constexpr float KD = 14.0f;

// ----------------------------------------------------------------------------
// 2. [AJUSTAR] Punto de equilibrio
// ----------------------------------------------------------------------------
//   Angulo (grados) donde el robot queda parado sin que los motores empujen.
//   Para calibrarlo: cargar el firmware, mantener el robot en pie sin armar,
//   y leer el "Pitch:" que sale por serial. Ese valor va aca.

constexpr float BALANCE_ANGLE_DEG = -3.5f;

// ----------------------------------------------------------------------------
// 3. [AJUSTAR] Ventanas de armado / caida
// ----------------------------------------------------------------------------
//   START_ANGLE_WINDOW_DEG:  cuanto cerca del equilibrio hay que ponerlo
//                            (a mano) para que se active el control.
//   ARM_STABLE_TIME_MS:      cuanto tiempo tiene que estar dentro de esa
//                            ventana antes de armar.
//   FALL_ANGLE_DEG:          si se inclina mas que esto se considera caido,
//                            se cortan los motores y hay que volver a armar.

constexpr float START_ANGLE_WINDOW_DEG = 5.0f;
constexpr uint32_t ARM_STABLE_TIME_MS = 500;
constexpr float FALL_ANGLE_DEG = 45.0f;

// ----------------------------------------------------------------------------
// 4. [AJUSTAR] Motores
// ----------------------------------------------------------------------------
//   MIN_ABS_SPEED:        PWM minimo (0-255) para vencer friccion estatica.
//                         Con la compensacion de friccion suave cualquier
//                         comando no nulo del PID se mapea linealmente al
//                         rango [MIN_ABS_SPEED, 255], sin el escalon abrupto
//                         de antes. Si las ruedas no arrancan, subir.
//   LEFT/RIGHT_FACTOR:    ganancia individual por rueda (0..1). Si una rueda
//                         gira mas rapido que la otra, bajar su factor para
//                         compensar. 0.60 = se entrega como mucho 60% del PWM
//                         calculado por el PID.

constexpr int MIN_ABS_SPEED = 20;
constexpr float LEFT_SPEED_FACTOR = 0.65f;
constexpr float RIGHT_SPEED_FACTOR = 0.65f;

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

// Anti-windup: limite absoluto del termino integral (en unidades de PWM).
// Aprieta cuanto puede aportar la integral al output total. Con KI alto evita
// que se acumule de a poco y termine empujando al robot fuera del equilibrio.
// Si subis KI y notas que la respuesta integral tarda en compensar un offset
// real, subir este limite (hasta PWM_MAX como mucho).
constexpr float INTEGRAL_LIMIT = 128.0f;

// Lazo de control:
constexpr uint32_t CONTROL_PERIOD_US = 5000;   // 200 Hz.
constexpr uint32_t DEBUG_PERIOD_MS = 100;
constexpr uint32_t MPU_RETRY_PERIOD_MS = 2000;
constexpr bool USE_MPU_INTERRUPT = false;      // Hoy el loop usa polling.

// Filtro complementario (0..1; mas alto = mas peso al gyro, menos al accel):
constexpr float COMPLEMENTARY_ALPHA = 0.98f;

// Filtro complementario adaptativo:
//   El acelerometro mide gravedad + aceleracion lineal, asi que se "ensucia"
//   justo cuando el robot acelera para corregirse. Cuando el modulo del vector
//   aceleracion se aparta de 1 g, subimos alpha (confiamos mas en el gyro).
//     desviacion < LOW_G            -> alpha = COMPLEMENTARY_ALPHA (accel limpio)
//     desviacion > HIGH_G           -> alpha ~ 1.0 (solo gyro por esta muestra)
//   entre ambos umbrales se interpola lineal.
constexpr float ACCEL_TRUST_DEV_LOW_G = 0.10f;
constexpr float ACCEL_TRUST_DEV_HIGH_G = 0.50f;

// Ventana (constante de tiempo, s) del RMS del error de pitch que se muestra
// en la web como metrica objetiva para comparar ajustes de PID.
constexpr float RMS_TAU_S = 2.0f;

// Filtro pasabajos del termino derivativo:
//   El gyro tiene ruido y KD lo amplifica -> chatter de PWM. Suavizamos la
//   velocidad que entra al termino D (la integracion del filtro complementario
//   sigue usando el gyro crudo, asi no se le agrega lag al angulo).
//   Es la fraccion del valor viejo que se conserva (0 = sin filtro, mas alto =
//   mas suave pero mas lag). 0.60 ~ corte en ~16 Hz a 200 Hz de lazo.
constexpr float D_TERM_SMOOTHING = 0.60f;

// Auto-trim del angulo de equilibrio:
//   Un balancer con PID de angulo puro deriva: BALANCE_ANGLE_DEG nunca es
//   exactamente el punto de centro de masa. Un integrador MUY lento sobre la
//   salida del PID corre el setpoint efectivo hasta que el esfuerzo promedio
//   de los motores tiende a cero -> el robot encuentra su propio equilibrio.
//   GAIN en grados por (unidad de salida * s); mantener chico (lazo lento).
constexpr bool AUTO_TRIM_ENABLED = true;
constexpr float AUTO_TRIM_GAIN = 0.003f;
constexpr float AUTO_TRIM_LIMIT_DEG = 8.0f;

// Re-calibracion del bias del gyro en reposo:
//   El bias del gyro drifta con la temperatura. Cuando el robot esta quieto
//   (desarmado y sin moverse) arrastramos lentamente el bias hacia la lectura
//   cruda, asi el pitch no se va de a poco en sesiones largas.
constexpr bool REST_RECAL_ENABLED = true;
constexpr uint32_t REST_RECAL_DELAY_MS = 1500;  // hay que estar quieto este rato.
constexpr float REST_RECAL_GAIN = 0.01f;        // EMA por ciclo una vez en reposo.

// I2C:
constexpr uint32_t I2C_CLOCK_HZ = 400000;

// PWM:
//   El control trabaja en "unidades de control" 0..PWM_MAX (255): KP, trims y
//   MIN_ABS_SPEED conservan su significado. El hardware LEDC corre a mayor
//   resolucion (LEDC_RESOLUTION_BITS) y el driver de motores escala el duty
//   final en punto flotante -> mas pasos reales, menos zumbido de cuantizacion.
constexpr uint8_t LEFT_PWM_CHANNEL = 0;
constexpr uint8_t RIGHT_PWM_CHANNEL = 1;
constexpr uint32_t PWM_FREQUENCY_HZ = 20000;
constexpr int PWM_MAX = 255;
constexpr uint8_t LEDC_RESOLUTION_BITS = 10;          // 20 kHz @ 80 MHz admite hasta ~11 bits.
constexpr int LEDC_MAX = (1 << LEDC_RESOLUTION_BITS) - 1;

// WiFi AP para ajustar el robot desde el navegador:
constexpr char WIFI_AP_SSID[] = "RobotBalancin";
constexpr char WIFI_AP_PASSWORD[] = "12345678";
constexpr uint8_t WIFI_AP_CHANNEL = 1;

// LED integrado de estado:
constexpr bool STATUS_LED_ENABLED = true;
constexpr uint8_t STATUS_LED_BRIGHTNESS = 24; // 0..255

} // namespace Config
