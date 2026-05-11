# Robot Balancin ESP32-C3

Proyecto Arduino/PlatformIO para un robot balancin con ESP32-C3 DevKitC-02, MPU6050 y puente H L298N.

## Cambios principales frente al sketch original

- Se elimino `TWBR`, que solo existe en AVR.
- Se reemplazo `attachInterrupt(0, ...)` por `digitalPinToInterrupt(pin)`.
- Se reemplazo `LMotorController` por control directo con PWM LEDC del ESP32-C3.
- Se reemplazo `PID_v1` por un PID simple con anti-windup.
- Se eliminaron dependencias de `I2Cdev` y DMP para que el sketch compile con el core Arduino ESP32 y `Wire`.
- Se agregaron parada por caida, armado gradual y salida serie menos agresiva.

## Pines por defecto

| Funcion | GPIO |
| --- | ---: |
| MPU6050 SDA | 8 |
| MPU6050 SCL | 9 |
| MPU6050 INT opcional | 2 |
| L298N ENA izquierda | 3 |
| L298N IN1 izquierda | 4 |
| L298N IN2 izquierda | 5 |
| L298N ENB derecha | 6 |
| L298N IN3 derecha | 7 |
| L298N IN4 derecha | 10 |

El ESP32-C3 y el MPU6050 deben trabajar a 3.3 V. El L298N necesita alimentacion de motores separada y masa comun con el ESP32-C3.

`INT` del MPU6050 no es obligatorio con la configuracion inicial. Si queres usarlo, conectar `INT` a GPIO2 y cambiar `USE_MPU_INTERRUPT` a `true`.

## Ajustes importantes

Los parametros estan al inicio de `src/main.cpp`, dentro de `namespace Config`.

- `BALANCE_ANGLE_DEG`: angulo real de equilibrio. El valor inicial `-4.7` equivale aproximadamente al `175.3` del sketch original.
- `KP`, `KI`, `KD`: constantes del PID.
- `LEFT_SPEED_FACTOR` y `RIGHT_SPEED_FACTOR`: compensacion de motores.
- `INVERT_PITCH`, `INVERT_MOTOR_OUTPUT`, `INVERT_LEFT_MOTOR`, `INVERT_RIGHT_MOTOR`: invertir signos si el robot corrige hacia el lado equivocado.
- `I2C_SDA_PIN` y `I2C_SCL_PIN`: por defecto usan GPIO8/GPIO9, que son los pines I2C definidos por el core Arduino para `esp32c3`.

## Uso

1. Conectar el robot por USB.
2. Mantener el robot quieto al encender para calibrar el giroscopio.
3. Levantarlo cerca del punto de equilibrio.
4. El control se activa despues de 500 ms dentro de la ventana de arranque.

Compilacion esperada:

```sh
/home/maximomariani/.platformio/penv/bin/pio run
```

En esta maquina el `pio` correcto es el del entorno de PlatformIO en `/home/maximomariani/.platformio/penv/bin/pio`. El `/usr/bin/pio` del sistema falla por una instalacion vieja incompatible con Python 3.12.
