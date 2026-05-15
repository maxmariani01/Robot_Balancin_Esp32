# Test L298N Motores ESP32-C3

Firmware independiente para probar el ESP32-C3 DevKitC-02 con el puente H L298N, los motores y la lectura basica del MPU6050.

## Pines

Usa los mismos pines del proyecto del robot:

| Funcion | GPIO |
| --- | ---: |
| MPU6050 SDA | 8 |
| MPU6050 SCL | 9 |
| L298N ENA izquierda | 3 |
| L298N IN1 izquierda | 4 |
| L298N IN2 izquierda | 5 |
| L298N ENB derecha | 6 |
| L298N IN3 derecha | 7 |
| L298N IN4 derecha | 10 |

Conectar GND del ESP32-C3 y GND del L298N juntos. La alimentacion de motores debe ser externa al ESP32.

En ESP32-C3, GPIO9 tambien es el pin BOOT/strapping. Si la PCB o el sensor lo dejan bajo durante el arranque, el firmware no inicia y no aparece la red WiFi. Para una placa definitiva conviene mover el SCL del MPU6050 a otro GPIO que no sea de arranque, o asegurar que GPIO9 quede alto al encender.

## Control web

Al arrancar, la placa crea una red WiFi propia:

| Dato | Valor |
| --- | --- |
| SSID | `RobotTest` |
| Clave | `12345678` |
| URL | `http://192.168.4.1` |

Conectarse a esa red desde el celular o la PC y abrir la URL en el navegador. La pagina permite mover ambos motores adelante/atras, mover solo izquierdo/derecho, parar y ajustar el PWM.

La misma pagina muestra el estado del MPU6050. Si el sensor esta conectado correctamente, debe figurar como `OK` y las lecturas de pitch/roll deben cambiar al mover la placa.

El campo `Bus I2C` muestra las direcciones detectadas. Para un MPU6050 normal deberia aparecer `0x68` o `0x69`. Si aparece `ninguno`, revisar SDA/SCL, GND comun, alimentacion del modulo y pull-ups del bus. Junto a `Lecturas sensor` tambien se muestra el valor `WHO_AM_I`; en MPU6050 suele ser `0x68`.

## Comandos por monitor serie

El monitor serie sigue disponible como respaldo de diagnostico. Abrir monitor a `115200`.

| Comando | Accion |
| --- | --- |
| `h` | ayuda |
| `s` | parar |
| `f` | ambos motores adelante durante 2 s |
| `b` | ambos motores atras durante 2 s |
| `l` | motor izquierdo adelante durante 2 s |
| `r` | motor derecho adelante durante 2 s |
| `+` | subir PWM |
| `-` | bajar PWM |
| `0`..`9` | ajustar PWM en pasos aproximados |
| `i` | escanear bus I2C |

Si el MPU6050 es detectado, el monitor serie imprime lecturas del sensor cada 1 s.

## Compilar

```sh
pio run
```

## Cargar

```sh
pio run -t upload
```
