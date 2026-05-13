# Test L298N Motores ESP32-C3

Firmware independiente para probar el ESP32-C3 DevKitC-02 con el puente H L298N y los motores, sin MPU6050.

## Pines

Usa los mismos pines del proyecto del robot:

| Funcion | GPIO |
| --- | ---: |
| L298N ENA izquierda | 3 |
| L298N IN1 izquierda | 4 |
| L298N IN2 izquierda | 5 |
| L298N ENB derecha | 6 |
| L298N IN3 derecha | 7 |
| L298N IN4 derecha | 10 |

Conectar GND del ESP32-C3 y GND del L298N juntos. La alimentacion de motores debe ser externa al ESP32.

## Comandos por monitor serie

Abrir monitor a `115200`.

| Comando | Accion |
| --- | --- |
| `h` | ayuda |
| `s` | parar |
| `f` | ambos motores adelante durante 2 s |
| `b` | ambos motores atras durante 2 s |
| `l` | motor izquierdo adelante durante 2 s |
| `r` | motor derecho adelante durante 2 s |
| `a` | secuencia automatica corta |
| `+` | subir PWM |
| `-` | bajar PWM |
| `0`..`9` | ajustar PWM en pasos aproximados |

## Compilar

```sh
/home/maximomariani/.platformio/penv/bin/pio run
```

## Cargar

```sh
/home/maximomariani/.platformio/penv/bin/pio run -t upload
```
