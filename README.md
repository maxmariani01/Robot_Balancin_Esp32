# Robot Balancin ESP32-C3

Proyecto Arduino/PlatformIO para un robot balancin con ESP32-C3 DevKitC-02, MPU6050 y puente H L298N.

## Cambios principales frente al sketch original

- Se elimino `TWBR`, que solo existe en AVR.
- Se reemplazo `attachInterrupt(0, ...)` por `digitalPinToInterrupt(pin)`.
- Se reemplazo `LMotorController` por control directo con PWM LEDC del ESP32-C3.
- Se reemplazo `PID_v1` por un PID simple con anti-windup.
- Se eliminaron dependencias de `I2Cdev` y DMP para que el sketch compile con el core Arduino ESP32 y `Wire`.
- Se agregaron parada por caida, armado gradual y salida serie menos agresiva.
- El lazo de balanceo corre en una task FreeRTOS dedicada (prioridad 2, 200 Hz por `vTaskDelayUntil`), separada de `loop()`, que solo atiende el servidor web. Asi una request HTTP no puede retrasar un ciclo de control. La task de control es la unica que toca el bus I2C y los motores; la web solo deja pedidos (calibrar / desarmar / guardar) con banderas.
- Filtro complementario adaptativo: baja el peso del acelerometro cuando el modulo del vector aceleracion se aparta de 1 g (aceleracion lineal contamina la lectura).
- PWM por LEDC a 10 bits con el duty calculado en punto flotante y compensacion de friccion lineal, en vez del escalon abrupto `MIN_ABS_SPEED`.
- Ajustes persistentes en NVS y metricas de tuning (grafico en vivo, RMS del error).
- Auto-trim del angulo de equilibrio: un integrador lento corre el setpoint efectivo hacia el verdadero punto de centro de masa, asi el robot deja de derivar (lo aprendido se ve en la web como "Auto-trim").
- Re-calibracion del bias del gyro cuando el robot esta en reposo, para compensar el drift termico.
- Filtro pasabajos sobre el termino derivativo (menos chatter de PWM) y gyro configurado a +/-500 dps para tener headroom y no saturar en correcciones rapidas.

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

Los parametros estan en `include/Config.h`, dentro de `namespace Config`.

- `BALANCE_ANGLE_DEG`: angulo real de equilibrio. El valor inicial actual es `4.5`.
- `KP`, `KI`, `KD`: constantes del PID. Los defaults actuales son `25`, `0`, `5.5`.
- `LEFT_SPEED_FACTOR` y `RIGHT_SPEED_FACTOR`: compensacion de motores.
- `INVERT_PITCH`: invertir signo del sensor si el pitch queda al reves. La salida de motores queda invertida por defecto con `INVERT_MOTOR_OUTPUT = true`.
- `I2C_SDA_PIN` y `I2C_SCL_PIN`: por defecto usan GPIO8/GPIO9, que son los pines I2C definidos por el core Arduino para `esp32c3`.

## Uso

1. Conectar el robot por USB.
2. Mantener el robot quieto al encender para calibrar el giroscopio.
3. Levantarlo cerca del punto de equilibrio.
4. El control se activa despues de 500 ms dentro de la ventana de arranque.

## Control web

Al arrancar, el firmware principal crea una red WiFi propia:

| Dato | Valor |
| --- | --- |
| SSID | `RobotBalancin` |
| Clave | `12345678` |
| URL | `http://192.168.4.1` |

Desde esa pagina se pueden ajustar en caliente los valores de PID con sliders discretos, equilibrio/filtro del sensor, bias del gyro Y, PWM minimo, factor de PWM por rueda y trim PWM independiente para cada rueda. Tambien permite desarmar el control y recalibrar el giroscopio.

Ayudas para tunear:

- **Grafico en vivo del pitch:** muestra los ultimos ~6 s (canvas, datos servidos por `/samples`) para ver oscilacion, overshoot y settling de un vistazo.
- **RMS del error de pitch:** metrica objetiva que se reinicia en cada armado; sirve para comparar ajustes ("el RMS bajo de 2.1 a 1.4").
- **Boton "Guardar ajustes":** persiste KP/KI/KD y todo lo demas en NVS (flash). Se cargan al bootear, asi no hay que copiarlos a `Config.h` ni recompilar. Guardar desarma el robot (la escritura a flash congela la CPU unos ms). `Config.h` sigue siendo el default de fabrica si NVS esta vacio.
- **Salida serie en formato Serial Plotter** del IDE de Arduino (`pitch:..,setpoint:..,rms:..`).

La pagina muestra la direccion I2C y el `WHO_AM_I` del modulo. El firmware prueba `0x68` y `0x69`, y acepta IDs compatibles `0x68`, `0x69`, `0x70`, `0x71` y `0x73`.

El LED integrado funciona como indicador de estado: blanco mientras calibra el giroscopio, verde cuando el sensor esta calibrado y operando, rojo cuando detecta caida (o si no encuentra el sensor / falla la lectura). Si el MPU se recupera despues de una falla, queda en naranja como aviso de que falta recalibrar el gyro desde la web.

Los cambios hechos desde la web son temporales salvo que se use "Guardar ajustes": sin guardar, al reiniciar vuelven los valores de NVS o, si NVS esta vacio, los defaults de `include/Config.h`.

Compilacion esperada:

```sh
/home/maximomariani/.platformio/penv/bin/pio run
```

En esta maquina el `pio` correcto es el del entorno de PlatformIO en `/home/maximomariani/.platformio/penv/bin/pio`. El `/usr/bin/pio` del sistema falla por una instalacion vieja incompatible con Python 3.12.

## Test L298N

El directorio `Test_L298N_Motores_Esp32C3` contiene un firmware independiente para probar el puente H L298N y los motores sin usar el MPU6050 ni el lazo de balance.

Para compilarlo:

```sh
cd Test_L298N_Motores_Esp32C3
/home/maximomariani/.platformio/penv/bin/pio run
```
