# UNIR---Actividad-3-Grupo-27-
# Ascensor Inteligente ACME S.A.

<p align="center">
  <img src="media/Picture01.png" width="85%"><br>
  <small><em>Figura 1. Montaje completo del prototipo en el entorno de simulación Wokwi.</em></small>
</p>

<p align="center">
  <strong>Equipos e Instrumentación Electrónica</strong> · MUIT 2026<br>
  Actividad 3 — Grupo 27<br>
  <em>Universidad Internacional de La Rioja (UNIR)</em>
</p>

<p align="center">
  <img src="https://img.shields.io/badge/Plataforma-Arduino%20UNO-00979D?logo=arduino&logoColor=white" alt="Arduino UNO">
  <img src="https://img.shields.io/badge/Simulador-Wokwi-orange" alt="Wokwi">
  <img src="https://img.shields.io/badge/Lenguaje-C%2B%2B-00599C?logo=cplusplus&logoColor=white" alt="C++">
  <img src="https://img.shields.io/badge/Versi%C3%B3n-v4.4-blue" alt="v4.4">
  <img src="https://img.shields.io/badge/Estado-Funcional-success" alt="Estado">
</p>

---

## Tabla de contenidos

1. [Descripción del proyecto](#1-descripción-del-proyecto)
2. [Características principales](#2-características-principales)
3. [Arquitectura del sistema](#3-arquitectura-del-sistema)
4. [Hardware utilizado](#4-hardware-utilizado)
5. [Estructura del repositorio](#5-estructura-del-repositorio)
6. [Cómo simular el proyecto](#6-cómo-simular-el-proyecto)
7. [Lógica de control](#7-lógica-de-control)
   - [7.1. Máquina de estados (FSM)](#71-máquina-de-estados-fsm)
   - [7.2. Algoritmo SCAN de planificación de paradas](#72-algoritmo-scan-de-planificación-de-paradas)
   - [7.3. Control difuso del servomotor](#73-control-difuso-del-servomotor)
   - [7.4. Control PID de temperatura](#74-control-pid-de-temperatura)
   - [7.5. Control de iluminación con histéresis](#75-control-de-iluminación-con-histéresis)
   - [7.6. Control de acceso RFID a la planta 5](#76-control-de-acceso-rfid-a-la-planta-5)
8. [Documentación](#8-documentación)
9. [Historial de versiones](#9-historial-de-versiones)
10. [Equipo de desarrollo](#10-equipo-de-desarrollo)
11. [Licencia](#11-licencia)

---

## 1. Descripción del proyecto

Este repositorio contiene el diseño y la implementación del firmware de un **ascensor
inteligente para un edificio de cinco plantas**, desarrollado sobre la plataforma
Arduino UNO y verificado mediante simulación en el entorno [Wokwi](https://wokwi.com/projects/464679537824441345).

El sistema integra el conjunto de periféricos contemplados en la asignatura
—sensores, actuadores, expansores de E/S sobre bus I²C, registros de desplazamiento,
bus SPI y display LCD I²C— y articula su funcionamiento en torno a **cuatro técnicas
de control diferenciadas**:

- Una **máquina de estados finita** (FSM) para la lógica discreta del ascensor.
- Un **algoritmo de planificación SCAN** para la gestión de la cola de llamadas.
- Un **controlador PID** para la regulación de temperatura de cabina.
- Un **controlador difuso** (Mamdani) para la generación del perfil de velocidad
  del servomotor.

Adicionalmente, el sistema incorpora un **subsistema de control de acceso restringido
a la quinta planta** basado en un lector RFID MFRC522 sobre bus SPI, que emula la
restricción de acceso a zonas autorizadas habitual en edificios de uso terciario.

---

## 2. Características principales

- ✅ **Firmware no bloqueante** basado en cooperative scheduling con `millis()`.
- ✅ **Máquina de estados de 6 estados** con transiciones seguras y temporizadores.
- ✅ **Cola múltiple de solicitudes** con prioridad por distancia, demanda y antigüedad.
- ✅ **Algoritmo SCAN** con redirección dinámica durante el movimiento.
- ✅ **Controlador difuso Mamdani** de 9 reglas para el perfil de velocidad del servo.
- ✅ **Controlador PID** con anti-windup, banda muerta y zona muerta del error.
- ✅ **Control proporcional inverso con histéresis** para la iluminación.
- ✅ **Filtrado anti-rebote temporal** de los pulsadores (25 ms).
- ✅ **Histéresis de planta** que separa el estado lógico del estado físico de la cabina.
- ✅ **Control de acceso RFID** con lista blanca y autorización temporal (15 s).
- ✅ Display LCD I²C 16×2 con dos pantallas alternadas de información.
- ✅ Tres vías de comando independientes: pulsadores físicos, mando IR y RFID.

---

## 3. Arquitectura del sistema

La arquitectura general del sistema responde al patrón habitual en sistemas embebidos
con múltiples tareas concurrentes: una unidad central (Arduino UNO) que coordina las
entradas procedentes de sensores y dispositivos de usuario, y gobierna los actuadores
y los elementos de interfaz.

<p align="center">
  <img src="media/Picture02.png" width="85%"><br>
  <small><em>Figura 2. Arquitectura por bloques del sistema. En azul, las entradas; en verde, los actuadores y la interfaz de usuario.</em></small>
</p>

El bucle principal del firmware ejecuta cada subtarea conforme a su periodo de
planificación específico, garantizando la operación concurrente sin pérdida de eventos:

<p align="center">
  <img src="media/Picture03.png" width="60%"><br>
  <small><em>Figura 3. Diagrama de flujo del bucle principal <code>loop()</code>.</em></small>
</p>

---

## 4. Hardware utilizado

| Categoría        | Componente                | Conexión                                          |
| ---------------- | ------------------------- | ------------------------------------------------- |
| Unidad de control| Arduino UNO R3            | —                                                 |
| Sensores         | DHT22                     | Digital, pin 7                                    |
|                  | HC-SR501 (PIR)            | Digital, pin 8                                    |
|                  | Receptor IR TSOP38238     | Digital, pin A3                                   |
|                  | Lector RFID MFRC522       | SPI: RST=9, SS=10, SCK=13, MOSI=11, MISO=12       |
|                  | LDR (módulo)              | Analógico, pin A0                                 |
| Actuadores       | Servomotor SG90           | PWM, pin 3                                        |
|                  | 8 LEDs iluminación        | Vía 74HC595 (DATA=6, CLOCK=5, LATCH=4)            |
|                  | LED EV_FRIO (azul)        | Digital, pin A1                                   |
|                  | LED EV_CALOR (rojo)       | Digital, pin A2                                   |
| Entradas usuario | 5 pulsadores (P1–P5)      | I²C vía PCF8574 (0x20), pins P0–P4                |
| Visualización    | LCD 16×2                  | I²C (0x27), SDA=A4, SCL=A5                        |
| Expansión        | PCF8574                   | I²C, dirección 0x20                               |
|                  | 74HC595                   | Transmisión serie por software                    |

Para el listado completo con precios escalados por volumen, consulte
[`docs/BOM.md`](docs/BOM.md).

---

## 5. Estructura del repositorio

```
ascensor-acme/
├── README.md                          ← este documento
├── firmware/
│   └── main.cpp                       ← código fuente del firmware
├── hardware/
│   └── diagram.json                   ← esquema del montaje (Wokwi)
├── docs/
│   ├── Memoria_Actividad3_Grupo27.docx ← memoria técnica completa
│   ├── BOM.md                          ← lista de componentes (Bill of Materials)
│   └── ANALISIS_COSTES.md              ← análisis de costes del desarrollo
└── media/
    └── Picture01.png … Picture09.png   ← figuras del README
```

---

## 6. Cómo simular el proyecto

El proyecto se puede ejecutar directamente en el entorno de simulación **Wokwi**, sin
necesidad de instalar nada localmente.

1. Abra el proyecto en Wokwi: <https://wokwi.com/projects/464679537824441345>
2. Pulse el botón ▶ para iniciar la simulación.
3. Interactúe con el sistema mediante los pulsadores, el mando IR o el lector RFID
   (Wokwi permite presentar tarjetas virtuales al lector MFRC522).

> **Nota.** Para reproducir el montaje de forma local en Wokwi, basta con cargar
> el contenido de [`firmware/main.cpp`](firmware/main.cpp) en el editor y el contenido
> de [`hardware/diagram.json`](hardware/diagram.json) en la pestaña *diagram.json*.

---

## 7. Lógica de control

### 7.1. Máquina de estados (FSM)

El comportamiento del ascensor se modela mediante una máquina de estados finita con
seis estados: `REPOSO`, `PUERTA_CERRADA`, `MOVIMIENTO`, `PUERTA_ABIERTA`,
`EMERGENCIA` y `MANTENIMIENTO`. Las transiciones entre estados se centralizan en una
única función `transicionEstado()`, que se responsabiliza tanto de las acciones de
salida del estado actual como de las acciones de entrada al nuevo estado.

<p align="center">
  <img src="media/Picture04.png" width="80%"><br>
  <small><em>Figura 4. Máquina de estados del ascensor con sus transiciones principales.</em></small>
</p>

La transición a EMERGENCIA requiere la concurrencia simultánea de tres condiciones:
cabina en movimiento, detección de presencia por el sensor PIR y más de un pulsador
activo a la vez. La temporización del bucle principal garantiza que ninguna de estas
señales se pierda:

```cpp
void loop() {
  unsigned long ahora = millis();

  // === MÁQUINA DE ESTADOS DEL ASCENSOR ===
  manejarEstadoAscensor();

  // 1. Entradas de usuario cada 10 ms (anti-rebote propio dentro)
  if (ahora - tUltimaPulsaci >= 10) {
    tUltimaPulsaci = ahora;
    if (estadoAscensor != ASCENSOR_EMERGENCIA) {
      leerPulsadores();
      leerIR();
    }
  }

  // 2. Control de acceso RFID cada 200 ms
  if (ahora - tUltimoRFID >= 200) {
    tUltimoRFID = ahora;
    gestionarAccesoP5();
  }

  // 3. Lectura ambiental cada 2 s
  if (ahora - tUltimaLectura >= 2000) {
    tUltimaLectura = ahora;
    leerSensores();
    presencia = digitalRead(PIN_PIR);
  }

  // 4. Control PID de temperatura cada 500 ms
  if (ahora - tUltimoProceso >= PID_INTERVALO) {
    tUltimoProceso = ahora;
    controlTemperatura();
  }

  // 5. Iluminación: cada ciclo, respuesta rápida ante cambios bruscos
  controlIluminacion();
}
```

### 7.2. Algoritmo SCAN de planificación de paradas

El algoritmo SCAN —también denominado *elevator algorithm* en la literatura— determina
en cada instante la planta de destino cuando existen varias solicitudes pendientes.
Su principio consiste en mantener un sentido de marcha atendiendo todas las llamadas
que se encuentren en la trayectoria y, una vez agotadas las solicitudes en ese
sentido, invertir la dirección.

<p align="center">
  <img src="media/Picture05.png" width="70%"><br>
  <small><em>Figura 5. Flujo del algoritmo SCAN implementado en <code>calcularPrioridad()</code>.</em></small>
</p>

La jerarquía de criterios de prioridad es **distancia → contador de demanda →
antigüedad de la solicitud**:

```cpp
uint8_t calcularPrioridad() {
  if (numSolicitudes == 0) return 0;

  // FASE 0: si hay solicitud en la planta actual, atenderla inmediatamente
  if (solicitudes[plantaActual - 1]) return plantaActual;

  uint8_t mejorPlanta    = 0;
  int8_t  mejorDistancia = 127;
  uint8_t mejorContador  = 0;
  unsigned long mejorTiempo = 0xFFFFFFFF;

  // FASE 1: buscar en la dirección actual de marcha
  for (uint8_t i = 0; i < MAX_PLANTAS; i++) {
    if (!solicitudes[i]) continue;
    uint8_t p = i + 1;
    int8_t  dist = 0;
    bool    enDir = false;

    if (direccionActual == DIR_SUBIENDO || direccionActual == DIR_NINGUNA) {
      if (p > plantaActual) { dist = p - plantaActual; enDir = true; }
    }
    if (!enDir && (direccionActual == DIR_BAJANDO ||
                   direccionActual == DIR_NINGUNA)) {
      if (p < plantaActual) { dist = plantaActual - p; enDir = true; }
    }
    if (!enDir) continue;

    // Criterio: distancia → contador → antigüedad
    if (dist < mejorDistancia ||
       (dist == mejorDistancia && contadorSolicitudes[i] > mejorContador) ||
       (dist == mejorDistancia &&
        contadorSolicitudes[i] == mejorContador &&
        tiempoSolicitud[i] < mejorTiempo)) {
      mejorPlanta    = p;
      mejorDistancia = dist;
      mejorContador  = contadorSolicitudes[i];
      mejorTiempo    = tiempoSolicitud[i];
    }
  }
  // FASE 2: si no hay candidatos, mirar en la dirección opuesta...
  return mejorPlanta;
}
```

### 7.3. Control difuso del servomotor

El accionamiento del servomotor se gobierna mediante un controlador difuso de tipo
**Mamdani**, con dos variables de entrada (`distanciaRestante` y `velocidadActual`) y
una variable de salida (`deltaAngulo`). El objetivo es obtener un **perfil de
velocidad trapezoidal** análogo al de un ascensor real: arranque progresivo, tramo de
velocidad de crucero y frenado gradual en la aproximación a la planta.

<p align="center">
  <img src="media/Picture06.png" width="80%"><br>
  <small><em>Figura 6. Conjuntos difusos de entrada: <code>distanciaRestante</code>.</em></small>
</p>

<p align="center">
  <img src="media/Picture07.png" width="80%"><br>
  <small><em>Figura 7. Conjuntos difusos de entrada: <code>velocidadActual</code>.</em></small>
</p>

<p align="center">
  <img src="media/Picture08.png" width="80%"><br>
  <small><em>Figura 8. Singletones de salida (<code>deltaAngulo</code>) del controlador difuso.</em></small>
</p>

La base de reglas (9 reglas) se evalúa con la conjunción AND implementada como
operador mínimo, y la defuzzificación se realiza por **media ponderada de
singletones** —método de bajo coste computacional especialmente adecuado para
microcontroladores—:

```cpp
float controlFuzzy(float distancia, float velocidad) {
  // --- FASE 1: Fuzzificación de las entradas ---
  float muDistCerca = fuzzificarTrapezoidal(distancia, 0, 0, 10, 15);
  float muDistMedia = fuzzificarTrapezoidal(distancia, 10, 15, 35, 45);
  float muDistLejos = fuzzificarTrapezoidal(distancia, 35, 45, 180, 180);

  float muVelLenta  = fuzzificarTrapezoidal(velocidad, 0, 0, 1.5f, 2.5f);
  float muVelMedia  = fuzzificarTriangular (velocidad, 1.5f, 3.0f, 4.5f);
  float muVelRapida = fuzzificarTrapezoidal(velocidad, 3.5f, 4.5f, 8, 8);

  // --- FASE 2: Evaluación de las 9 reglas (AND = min) ---
  float w[9], s[9];
  w[0]=min(muDistCerca,muVelLenta);   s[0]=FUZZY_OUT_MUY_LENTO;
  w[1]=min(muDistCerca,muVelMedia);   s[1]=FUZZY_OUT_MUY_LENTO;
  w[2]=min(muDistCerca,muVelRapida);  s[2]=FUZZY_OUT_LENTO;
  w[3]=min(muDistMedia,muVelLenta);   s[3]=FUZZY_OUT_MEDIO;
  w[4]=min(muDistMedia,muVelMedia);   s[4]=FUZZY_OUT_RAPIDO;
  w[5]=min(muDistMedia,muVelRapida);  s[5]=FUZZY_OUT_LENTO;
  w[6]=min(muDistLejos,muVelLenta);   s[6]=FUZZY_OUT_RAPIDO;
  w[7]=min(muDistLejos,muVelMedia);   s[7]=FUZZY_OUT_RAPIDO;
  w[8]=min(muDistLejos,muVelRapida);  s[8]=FUZZY_OUT_MUY_RAPIDO;

  // --- FASE 3: Defuzzificación por media ponderada de singletones ---
  float num = 0.0f, den = 0.0f;
  for (uint8_t i = 0; i < 9; i++) { num += w[i] * s[i]; den += w[i]; }

  float delta = (den > 0.001f) ? (num / den) : FUZZY_OUT_MUY_LENTO;
  if (delta > FUZZY_VEL_MAX) delta = FUZZY_VEL_MAX;
  return delta;
}
```

### 7.4. Control PID de temperatura

La climatización de la cabina se implementa mediante un controlador PID discreto con
una consigna (*setpoint*) de **25 °C**. Los actuadores son las electroválvulas de
frío (LED azul) y calor (LED rojo). La señal continua del PID se discretiza mediante
una banda muerta para el accionamiento ON/OFF de las válvulas:

<p align="center">
  <img src="media/Picture09.png" width="85%"><br>
  <small><em>Figura 9. Diagrama de bloques del lazo de control PID de temperatura.</em></small>
</p>

**Parámetros de sintonía:**

| Parámetro    | Valor    | Descripción                                  |
| ------------ | -------- | -------------------------------------------- |
| `PID_KP`     |  2.0     | Ganancia proporcional                        |
| `PID_KI`     |  0.05    | Ganancia integral                            |
| `PID_KD`     |  1.0     | Ganancia derivativa                          |
| `PID_INT_MAX`|  50      | Saturación del integrador (anti-windup)      |
| `PID_DEADBAND`| 1.0     | Banda muerta de salida                       |
| `TEMP_ZONA_M`|  3.0 °C  | Zona muerta del error                        |
| `PID_INTERVALO`| 500 ms | Periodo de ejecución del lazo                |

```cpp
void controlTemperatura() {
  unsigned long ahora = millis();
  float dt = (ahora - tUltimoPID) / 1000.0f;     // pasamos a segundos
  if (dt <= 0.0f) dt = 0.001f;
  tUltimoPID = ahora;

  float error = TEMP_SETPOINT - tempMedida;

  // 1) Zona muerta del error: si estamos dentro, no controlamos
  if (fabs(error) < TEMP_ZONA_M) {
    accionTemp = REPOSO_TEMP;
    digitalWrite(PIN_EV_FRIO,  LOW);
    digitalWrite(PIN_EV_CALOR, LOW);
    return;                                       // no se actualiza el integrador
  }

  // 2) PID activo
  pidIntegral += error * dt;
  pidIntegral = constrain(pidIntegral, -PID_INT_MAX, PID_INT_MAX);  // anti-windup

  float derivada = (error - pidErrorPrev) / dt;
  pidErrorPrev   = error;

  float salida = PID_KP * error + PID_KI * pidIntegral + PID_KD * derivada;

  // 3) Decisión discreta sobre las válvulas
  if (salida >  PID_DEADBAND)         { /* CALENTAR */ }
  else if (salida < -PID_DEADBAND)    { /* ENFRIAR  */ }
  else                                { /* REPOSO   */ }
}
```

### 7.5. Control de iluminación con histéresis

La iluminación de la cabina se gobierna mediante 8 LEDs conectados a un registro de
desplazamiento **74HC595**. El objetivo es mantener la iluminancia próxima a un valor
de confort de **500 lux** mediante un **control proporcional inverso con histéresis**
(±10 lux) que evita la conmutación oscilatoria en torno al umbral:

```cpp
void controlIluminacion() {
  if (luzLux < (LUZ_UMBRAL - LUZ_HISTERESIS)) {
    // < 390 lux: aumentar iluminación artificial
    float intervaloLux = (float)LUZ_UMBRAL / 8.0f;        // 50 lux por LED
    int ledsCalculados = 8 - (int)(luzLux / intervaloLux);
    ledsCalculados = constrain(ledsCalculados, 1, 8);

    if (ledsCalculados != ledsEncendidos) {
      ledsEncendidos = ledsCalculados;
      actualizarLeds(ledsEncendidos);
    }
  } else if (luzLux > (LUZ_UMBRAL + LUZ_HISTERESIS)) {
    // > 410 lux: apagar iluminación artificial
    if (ledsEncendidos > 0) {
      ledsEncendidos = 0;
      actualizarLeds(ledsEncendidos);
    }
  }
  // Zona muerta (390-410 lux): sin cambios
}
```

### 7.6. Control de acceso RFID a la planta 5

El acceso a la planta 5 está restringido a tarjetas RFID cuyo UID figure en la lista
blanca interna. Al presentar una tarjeta autorizada, el acceso se habilita durante
**15 segundos**, transcurridos los cuales se restablece automáticamente la
restricción:

```cpp
void gestionarAccesoP5() {
  unsigned long ahora = millis();

  // 1. Timeout de desactivación automática
  if (accesoP5Habilitado && (ahora - tActivacionP5 >= TIEMPO_ACCESO_P5)) {
    accesoP5Habilitado = false;
    Serial.println(F("[ACCESO P5] Tiempo expirado. Acceso restringido."));
  }

  // 2. Detectar tarjeta nueva
  if (!mfrc522.PICC_IsNewCardPresent()) return;
  if (!mfrc522.PICC_ReadCardSerial())   return;

  // 3. Validar UID contra la lista blanca
  if (tarjetaAutorizada(mfrc522.uid.uidByte, mfrc522.uid.size)) {
    accesoP5Habilitado = true;
    tActivacionP5 = ahora;
    Serial.println(F("[ACCESO P5] Tarjeta AUTORIZADA. P5 habilitada 15 s."));
  } else {
    Serial.println(F("[ACCESO P5] Tarjeta NO autorizada."));
  }

  mfrc522.PICC_HaltA();
  mfrc522.PCD_StopCrypto1();
}
```

La autorización gobierna **el derecho a encolar la solicitud, no su permanencia en
cola**: si el ascensor ya ha iniciado el desplazamiento hacia la planta 5, el viaje
se completa aunque expire el periodo de autorización durante el trayecto.

---

## 8. Documentación

| Documento                                                                | Contenido                                                |
| ------------------------------------------------------------------------ | -------------------------------------------------------- |
| [`docs/Memoria_Actividad3_Grupo27.docx`](docs/Memoria_Actividad3_Grupo27.docx) | Memoria técnica completa del firmware (11 capítulos)     |
| [`docs/BOM.md`](docs/BOM.md)                                             | Lista de componentes con precios escalados por volumen   |
| [`docs/ANALISIS_COSTES.md`](docs/ANALISIS_COSTES.md)                     | Análisis de costes de desarrollo y cálculo de PVP        |
| [`firmware/main.cpp`](firmware/main.cpp)                                 | Código fuente del firmware, ampliamente comentado        |

---

## 9. Historial de versiones

| Versión | Cambios principales                                                         |
| ------- | --------------------------------------------------------------------------- |
| v3.0    | Estructura inicial con FSM y cola de solicitudes                            |
| v4.0    | Incorporación del control difuso del servomotor (perfil trapezoidal)        |
| v4.2    | Robustez del SCAN durante el movimiento; validación de redirecciones        |
| v4.3    | Histéresis de planta y SCAN sin efectos colaterales                         |
| v4.4    | Control de acceso RFID a la planta 5 (lector MFRC522 sobre SPI) — *actual* |

El desglose detallado de cada versión, con descripción de los defectos corregidos y
las soluciones aplicadas, se encuentra en el capítulo 10 de la memoria técnica.

---

## 10. Equipo de desarrollo

**Grupo 27** — Equipos e Instrumentación Electrónica · MUIT 2026

- Amaya Álvarez Jiménez
- Jesús Macanás Sánchez
- Pablo Marín González
- Sergio Pérez García

*Profesor:* Iván Araquistain

---

## 11. Licencia

Este proyecto se ha desarrollado en el marco académico de la asignatura *Equipos e
Instrumentación Electrónica* del Máster Universitario en Ingeniería de
Telecomunicación (MUIT) de la Universidad Internacional de La Rioja (UNIR), curso
2025–2026. Su distribución se realiza con fines exclusivamente docentes y de
referencia.
