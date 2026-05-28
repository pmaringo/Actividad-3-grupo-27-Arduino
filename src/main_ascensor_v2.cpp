/********** INSTRUMENTACIÓN ELECTRÓNICA **********
 *
 * Programa ARDUINO MUIT 2026
 * Asignatura: Equipos e Instrumentación Electrónica
 * Profesor: Ivan Araquistain
 * 
 * ============================================================
 * Actividad 2 - Ascensor Inteligente ACME S.A.
 * 
 * Autor: Pablo Marín
 * Fecha: 11 Mayo, 2026 
 * ============================================================
 *  Hardware utilizado:
 *   - Arduino Uno
 *   - Receptor IR (pin 11)  +  mando a distancia IR
 *   - Servomotor SG90       (pin 9)
 *   - Sensor DHT22          (pin 7)
 *   - LDR                   (pin A0)
 *   - Registro 74HC595      (DATA=6, CLOCK=5, LATCH=4) → 8 LEDs
 *   - PIR (presencia)       (pin 8)
 *   - LCD I2C 16x2          (SDA=A4, SCL=A5)
 *   - 5 pulsadores de planta (pines 2,3,A1,A2,A3)
 * ============================================================
 *
 * ============================================================
 *  LÓGICA DE CONTROL DEL ASCENSOR — Máquina de Estados Finitos
 * ============================================================
 *  El ascensor se modela como una FSM (Finite State Machine) con
 *  5 estados bien definidos:
 *
 *    REPOSO         → Cabina detenida en una planta, esperando órdenes.
 *                     Se aceptan comandos de pulsadores e IR.
 *    MOVIMIENTO     → Cabina desplazándose hacia la planta destino.
 *                     El servo avanza SERVO_VELOCIDAD=2°/ciclo (20 ms)
 *                     de forma no bloqueante, de 0° (P1) a 180° (P5).
 *    PUERTA_ABIERTA → La cabina llegó al destino y permanece 3 s con
 *                     "puerta abierta" antes de volver a REPOSO.
 *    EMERGENCIA     → Parada de seguridad. Se activa ÚNICAMENTE cuando
 *                     se cumplen simultáneamente TRES condiciones:
 *                       1. Ascensor en estado MOVIMIENTO.
 *                       2. Sensor PIR detecta presencia en el exterior.
 *                       3. Más de un pulsador está pulsado a la vez
 *                          (situación de pánico o manipulación anómala
 *                          del panel de llamada).
 *                     La combinación de presencia exterior + múltiples
 *                     pulsaciones simultáneas durante el movimiento se
 *                     interpreta como una situación de riesgo real.
 *                     Tras 10 s hace reset automático a REPOSO.
 *    MANTENIMIENTO  → Estado especial para diagnóstico/debug.
 *
 *  Diagrama de transiciones simplificado:
 *
 *   [REPOSO] ──(nueva planta)──► [MOVIMIENTO]
 *      ▲                              │
 *      │         (llegó destino)      │
 *      │◄──[PUERTA_ABIERTA]◄──────────┘
 *      │         (3 s)
 *      │
 *      └◄──[EMERGENCIA]◄──(mvto + presencia + >1 botón)──[MOVIMIENTO]
 *               (reset 10 s)
 *
 *  Fuentes de comando: pulsadores (pines 2,3,A1,A2,A3) y mando IR NEC.
 *  Los comandos solo se procesan en estado REPOSO para evitar
 *  interferencias durante el movimiento o la emergencia.
 *
 * ============================================================
 *  LÓGICA DE CONTROL DE ILUMINACIÓN — Control Proporcional Inverso
 * ============================================================
 *  Objetivo: mantener la iluminación interior próxima a 500 lux
 *  (nivel de confort) usando los 8 LEDs del 74HC595 como fuente
 *  de luz artificial complementaria a la luz natural.
 *
 *  Parámetros:
 *    Setpoint   = 500 lux   (nivel de confort objetivo)
 *    Umbral     = 400 lux   (80 % del setpoint, umbral de activación)
 *    Histéresis = ±10 lux   (zona muerta: 390–410 lux)
 *
 *  Algoritmo (control proporcional inverso discreto con histéresis):
 *
 *    ┌─────────────────────────────────────────────────────────┐
 *    │ luz < 390 lux  →  LEDs = 8 − floor(luz / (400/8))       │
 *    │                   (cuanto menos luz, más LEDs activos)  │
 *    │ luz > 410 lux  →  LEDs = 0  (apagar iluminación artif.) │
 *    │ 390–410 lux    →  sin cambios (zona muerta/histéresis)  │
 *    └─────────────────────────────────────────────────────────┘
 *
 *  La relación es inversa y proporcional: dividiendo el rango de luz
 *  (0–400 lux) en 8 intervalos iguales de 50 lux, se encienden tantos
 *  LEDs como intervalos hayan de luz que "faltan" para llegar al umbral.
 *  La histéresis evita el parpadeo continuo cerca del umbral.
 *  La lectura del LDR se realiza en cada ciclo de loop() para dar
 *  respuesta rápida ante cambios bruscos de iluminación exterior.
 *
 * ============================================================
 *  LÓGICA DE CONTROL DE TEMPERATURA — Controlador PID
 * ============================================================
 *  Objetivo: mantener la temperatura interior en TEMP_SETPOINT = 25 °C
 *  actuando sobre la electroválvula de frío (EV_FRIO, LED azul, pin 10)
 *  y la de calor (EV_CALOR, LED rojo, pin 12).
 *
 *  El controlador PID (Proporcional–Integral–Derivativo) es el algoritmo
 *  de control realimentado más utilizado en la industria. Calcula la
 *  señal de control u(t) a partir del error entre el valor deseado
 *  (setpoint) y el valor medido (tempMedida) por el sensor DHT22:
 *
 *    error(t)  =  TEMP_SETPOINT − tempMedida
 *
 *    u(t)  =  Kp · e(t)
 *           + Ki · ∫₀ᵗ e(τ) dτ          ← suma acumulada × Δt
 *           + Kd · Δe(t)/Δt             ← diferencia / Δt
 *
 *  Acción de cada término:
 *    · Proporcional (Kp = 2.0):
 *        Genera una respuesta inmediata y proporcional al error actual.
 *        A mayor diferencia entre setpoint y temperatura, mayor señal.
 *        Por sí solo puede dejar un error estacionario residual.
 *
 *    · Integral (Ki = 0.05):
 *        Acumula el error a lo largo del tiempo. Aunque el error sea
 *        pequeño, si persiste el integrador sigue creciendo hasta
 *        eliminar el error estacionario por completo.
 *        Se limita con anti-windup (±PID_INT_MAX = ±50) para evitar
 *        que se sature cuando las válvulas llevan mucho tiempo activas
 *        y el sistema no puede responder más rápido.
 *
 *    · Derivativo (Kd = 1.0):
 *        Reacciona a la velocidad de cambio del error. Si la temperatura
 *        se acerca rápidamente al setpoint, el término derivativo genera
 *        una acción opuesta que frena la sobreoscilación (efecto amortiguador).
 *
 *  Como las válvulas son ON/OFF (no modulables), la salida continua u(t)
 *  se convierte en acción discreta mediante una banda muerta (±PID_DEADBAND):
 *
 *    u(t) >  +PID_DEADBAND  →  CALENTAR  (EV_CALOR HIGH, EV_FRIO LOW)
 *    u(t) <  −PID_DEADBAND  →  ENFRIAR   (EV_FRIO HIGH, EV_CALOR LOW)
 *    |u(t)| ≤ PID_DEADBAND  →  REPOSO    (ambas válvulas LOW)
 *
 *  La banda muerta evita micro-ciclos de las válvulas cuando la
 *  temperatura está muy próxima al setpoint.
 *
 *  El PID se ejecuta cada PID_INTERVALO = 500 ms. El sensor DHT22 se
 *  lee cada 2 s; entre lecturas el término derivativo vale 0 y el
 *  controlador actúa fundamentalmente sobre los términos P e I.
 *
 * ============================================================
 *   - LCD: muestra planta actual, Tª, humedad y acción de control
 * ============================================================
 *
 * Simulación en Wokwi: https://wokwi.com/projects/461843709827701761
 */

// --- Inclusión de Bibliotecas ---
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Servo.h>
#include <DHT.h>
#include <IRremote.h>

// --- Definición de Pines ---
#define PIN_IR         11
#define PIN_SERVO       9
#define PIN_DHT         7
#define PIN_PIR         8
#define PIN_LDR        A0
#define PIN_EV_FRIO     10  // LED azul  → electroválvula refrigerante
#define PIN_EV_CALOR    12  // LED rojo  → electroválvula calefacción
#define PIN_595_DATA    6   // Pin de datos serie (DS) del 74HC595 – datos que se desplazan.
#define PIN_595_CLOCK   5   // Pin de reloj de desplazamiento (SHCP) del 74HC595
#define PIN_595_LATCH   4   // Pin de latch (STCP) del 74HC595 – actualiza las salidas QA–QH.

// Pulsadores de planta (P1..P5) — P3..P5 en pines analógicos
const uint8_t PIN_BTN[5] = {2, 3, A1, A2, A3};

// --- Temperatura: Constantes de control ---
#define TEMP_SETPOINT   25.0f   // °C deseados
#define TEMP_ZONA_M      3.0f   // ±zona muerta en °C
#define HUM_SETPOINT    60.0f   // % humedad deseada

// Parámetros de sintonía del PID
#define PID_KP          2.0f    // Ganancia proporcional
#define PID_KI          0.05f   // Ganancia integral
#define PID_KD          1.0f    // Ganancia derivativa
#define PID_DEADBAND    1.0f    // Banda muerta ±°C (sin actuación cuando |u| < deadband)
#define PID_INT_MAX     50.0f   // Límite anti-windup del término integral
#define PID_INTERVALO   500     // ms entre ejecuciones del ciclo PID

// Iluminación: valores en lux (del sensor LDR)
#define GAMMA           0.7f    // Exponente de la curva del LDR
#define RL10           50.0f    // Resistencia en kΩ a 10 lux.
#define LUZ_SETPOINT    500     // lux (100% - nivel óptimo)
#define LUZ_UMBRAL      400     // lux (80% - umbral de activación)
#define LUZ_HISTERESIS  10      // ±lux (zona muerta: 390-410 lux)

#define SERVO_VELOCIDAD  1      // 1 grados/ciclo (65 ms)

// Ángulos del servo para cada planta (0→P1, 180→P5)
const int ANGULO_PLANTA[5] = {0, 45, 90, 135, 180};

// Códigos IR NEC — botones 1..5 (mando WOKWI)
// Mando botón 1 → P1, command=48, raw=0xCF30FF00
// Mando botón 2 → P2, command=24, raw=0xE718FF00
// Mando botón 3 → P3, command=122,raw=0x857AFF00
// Mando botón 4 → P4, command=16, raw=0xEF10FF00
// Mando botón 5 → P5, command=56, raw=0xC738FF00
const uint32_t IR_CODIGO[5] = {
  0xCF30FF00, 0xE718FF00, 0x857AFF00, 0xEF10FF00, 0xC738FF00
};

// --- Inicialización de Objetos ---
LiquidCrystal_I2C lcd(0x27, 16, 2);
Servo             servoAscensor;
DHT               dht(PIN_DHT, DHT22);      // Sensor DHT22

// --- Variables Globales ---
uint8_t plantaActual    = 1;   // Planta donde está la cabina
uint8_t plantaDestino   = 1;   // Planta solicitada
bool    cabinaMov       = false;
int     anguloServo     = 0;

float   tempMedida      = 25.0f;  // Temp. ambiente medida por DHT22
float   humMedida       = 80.0f;  // Humedad real medida
float   luzLux          = 500.0f; // Lux leídos del sensor LDR

bool    presencia       = false;

// Acción de control temperatura (baterías)
enum AccionTemp { REPOSO_TEMP, CALENTAR, ENFRIAR };
AccionTemp accionTemp = REPOSO_TEMP;

// Variables de estado del controlador PID de temperatura
float         pidIntegral  = 0.0f;  // Término integral acumulado
float         pidErrorPrev = 0.0f;  // Error del ciclo anterior (para el término derivativo)
unsigned long tUltimoPID   = 0;     // Timestamp del último ciclo PID (cálculo de Δt)

// --- MÁQUINA DE ESTADOS DEL ASCENSOR ---
enum EstadoAscensor {
  ASCENSOR_REPOSO,        // Cabina parada en planta, esperando comandos
  ASCENSOR_MOVIMIENTO,    // Cabina moviéndose hacia destino
  ASCENSOR_EMERGENCIA,    // Parada de emergencia por seguridad
  ASCENSOR_PUERTA_ABIERTA,// Puerta abierta (simulado)
  ASCENSOR_MANTENIMIENTO  // Estado de mantenimiento/debug
};
EstadoAscensor estadoAscensor = ASCENSOR_REPOSO;  // Estado inicial

// Acción de control iluminación
uint8_t ledsEncendidos = 0;  // 0-8 LEDs encendidos

// Anti-rebote pulsadores
bool btnEstadoPrev[5] = {HIGH, HIGH, HIGH, HIGH, HIGH};
unsigned long btnUltimoRebote[5] = {0, 0, 0, 0, 0};

// Temporizadores no bloqueantes
unsigned long tUltimoProceso  = 0;
unsigned long tUltimaLectura  = 0;
unsigned long tUltimaPulsaci  = 0;
unsigned long tUltimoLCD      = 0;
unsigned long tUltimoServo    = 0;
uint8_t       paginaLCD       = 0;  // alterna entre 2 pantallas

// --- Prototipos ---
void actualizarLCD();
void actualizarLeds(uint8_t cantidad);
uint8_t contarBotonesPulsados();
void controlIluminacion();
void controlTemperatura();
void escribir595(uint8_t valor);
void leerIR();
float leerLux();
void leerPulsadores();
void leerSensores();
void manejarEstadoAscensor();       // Función para máquina de estados
void moverAscensor();
const char* nombreEstado(EstadoAscensor e);
String padLCD(String texto);        // Función para centrar texto en LCD
void transicionEstado(EstadoAscensor nuevoEstado);  

// ============================================================
// --- Configuración Inicial (setup) ---
// ============================================================
void setup() {
  Serial.begin(9600);

  Serial.println(F("UNIR Actividad 2 - Ascensor ACME v2.0"));
  Serial.println(F(""));

  // Inicializar LCD
  Serial.println(F("Inicializando LCD..."));
  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("ACME ASCENSOR v2.0");
  lcd.setCursor(0, 1);
  lcd.print("Iniciando...");
  delay(1500);

  Serial.println(F("Pantalla LCD inicializada."));
  Serial.println(F("------------------------------------"));
  
  // Pulsadores con pull-up interno
  for (uint8_t i = 0; i < 5; i++) {
    pinMode(PIN_BTN[i], INPUT_PULLUP);
  }

  // PIR (sensor de presencia)
  pinMode(PIN_PIR, INPUT);

  // LEDs de control de temperatura
  pinMode(PIN_EV_FRIO, OUTPUT);
  pinMode(PIN_EV_CALOR, OUTPUT);
  digitalWrite(PIN_EV_FRIO, LOW);
  digitalWrite(PIN_EV_CALOR, LOW);
  

  // 74HC595 (Registro de desplazamiento para LEDs)
  pinMode(PIN_595_DATA,  OUTPUT);
  pinMode(PIN_595_CLOCK, OUTPUT);
  pinMode(PIN_595_LATCH, OUTPUT);
  escribir595(0x00);  // Apagar todos los LEDs

  // Inicializar Control de Iluminación
  Serial.println(F(" Control de Iluminacion Inteligente v1.0"));
  Serial.println(F("Setpoint   : 500 lux (100%)"));
  Serial.println(F("Umbral ON  : < 390 lux (80% - 10)"));
  Serial.println(F("Umbral OFF : > 410 lux (80% + 10)"));
  Serial.println(F("Zona muerta: 390 - 410 lux"));
  Serial.println(F(""));
  
  ledsEncendidos = 0;     // Estado inicial: todos los LEDs apagados
  actualizarLeds(ledsEncendidos);

  Serial.println(F("Sistema de Iluminación inicializado."));
  Serial.println(F("------------------------------------"));

  // Servo Ascensor
  servoAscensor.attach(PIN_SERVO);
  servoAscensor.write(ANGULO_PLANTA[0]);
  anguloServo = ANGULO_PLANTA[0];

  // DHT (temperatura y humedad)
  dht.begin();

  // IR (control receptorremoto)
  IrReceiver.begin(PIN_IR);   // Iniciar receptor IR

  // Inicializar Control de temperatura con valor real del DHT22
  Serial.println(F(" Control de Temperatura Inteligente v1.0"));
  Serial.println(F("Ganancia proporcional : 2.0"));
  Serial.println(F("Ganancia integral     : 0.05"));
  Serial.println(F("Ganancia derivativa   : 1.0"));
  Serial.println(F("Banda muerta ±°C      : 3.0 °C"));
  Serial.println(F(""));
  
  float t = dht.readTemperature();
  if (!isnan(t)) {
    tempMedida = t;
  }

  Serial.println(F("Sistema de Temperatura inicializado."));
  Serial.println(F("------------------------------------"));

  // LCD (Sistema listo)
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Sistema LISTO!");
  delay(1000);
  lcd.clear();

  Serial.println(F("¡Sistema ascensor LISTO!"));
  Serial.println(F("------------------------------------"));  
}

// ============================================================
// --- Bucle Principal (loop) ---
// ============================================================
void loop() {
  unsigned long ahora = millis();

  // === MÁQUINA DE ESTADOS DEL ASCENSOR ===
  // Centraliza toda la lógica del ascensor en estados bien definidos
  manejarEstadoAscensor();

  // === FUNCIONES INDEPENDIENTES DEL ESTADO ===
  // Estas se ejecutan siempre, independientemente del estado del ascensor
  
  // 1. Entradas de usuario se procesan solo cada 100 ms para evitar rebotes y sobrecarga
  if (ahora - tUltimaPulsaci >= 100) {
    tUltimaPulsaci = ahora;
    // solo en REPOSO para evitar interferencias durante movimiento o emergencia
    if (estadoAscensor == ASCENSOR_REPOSO) {
      leerPulsadores();   // Leer pulsadores de planta
      leerIR();           // Leer mando IR
    }
  }

  // 2. Lectura de sensores (Temperatura, Humedad, Iluminacion) cada 2 s
  if (ahora - tUltimaLectura >= 2000) {
    tUltimaLectura = ahora;
    leerSensores();
    presencia = digitalRead(PIN_PIR);
  }
  
  // 3. Control PID de temperatura cada PID_INTERVALO ms
  if (ahora - tUltimoProceso >= PID_INTERVALO) {
    tUltimoProceso = ahora;
    controlTemperatura();
  }

  // 4. Actualizar LCD cada 2 s, alternando pantallas
  if (ahora - tUltimoLCD >= 2000) {
    tUltimoLCD = ahora;
    actualizarLCD();
    paginaLCD = !paginaLCD;
    // Forzar página 0 (estado de cabina) durante movimiento
    if (cabinaMov) {
      paginaLCD = 0;  
    }
  }

  // 5. Control de iluminación (LEDs). 
  // Se computa cada vez para mantener respuesta rápida a cambios de luz
  controlIluminacion();

}  // end Loop

// ============================================================
// ---------- FUNCIONES AUXILIARES ----------
// ============================================================

// ============================================================
// Actualizar display LCD (2 páginas rotativas 2 s cada una)
// Página 1: planta actual, destino y estado de cabina
// Página 2: temperatura, humedad y acción de control
void actualizarLCD() {
  lcd.clear();

  String linea;    // 16 caracteres por línea

  if (paginaLCD == 0) 
  {
    // Línea 0: estado de movimiento y planta
    lcd.setCursor(0, 0);
    
    linea = cabinaMov ? "Moviendo " : "En planta ";
    linea += String(plantaActual);
    if (cabinaMov) {
      linea += "->";
      linea += String(plantaDestino);
    }
    lcd.print(padLCD(linea));
    
    // Línea 1: presencia en cabina
    lcd.setCursor(0, 1);

    linea = presencia ? "Cabina: OCUPADA" : "Cabina: LIBRE";
    lcd.print(padLCD(linea));
  } 
  else 
  {
    // Línea 0: Temperatura y Humedad
    lcd.setCursor(0, 0);

    linea = "T:";
    linea += String(tempMedida, 1);   // 1 decimal
    linea += (char)223;               // símbolo °
    linea += "C ";
    linea += "H:";
    linea += String((int)humMedida);
    linea += "%";

    lcd.print(padLCD(linea));

    // Línea 1: Luz y control temperatura
    lcd.setCursor(0, 1);
    
    if (luzLux > 999) {
      linea = "L.DIURNA ";
    }
    else {
      linea = "L:";
      linea += String(luzLux,0);
      linea += "lux ";
    }
    
    switch (accionTemp) {
      case ENFRIAR:  linea += "ENFRIAR";  break;
      case CALENTAR: linea += "CALDEAR";  break;
      default: linea += "";  break;
    }    

    lcd.print(padLCD(linea));

  }
}

// ============================================================
// actualizarLeds()
// ----------------
// Calcula la máscara de bits para encender 'cantidad' LEDs de forma
// acumulativa (LED 1 es el bit 0, LED 8 es el bit 7) y la envía al
// registro de desplazamiento 74HC595.
//
// Ejemplo: cantidad=3 → máscara = 0b00000111 → QA, QB, QC encendidos.
//
// @param cantidad  Número de LEDs a encender (0–8).
//
void actualizarLeds(uint8_t cantidad) {
  // Generar máscara de bits para los N leds encendidos
  uint8_t mascara = 0;
  for (uint8_t i = 0; i < ledsEncendidos; i++) {
    mascara |= (1 << i);
  }
  escribir595(mascara); 
}

// ============================================================
// contarBotonesPulsados()
// -----------------------
// Devuelve el número de pulsadores de planta que están actualmente
// activos (nivel LOW por pull-up interno). No modifica ningún estado;
// se usa exclusivamente para la detección de la condición de emergencia
// durante el movimiento del ascensor.
//
// @return  Número de botones presionados simultáneamente (0–5).
//
uint8_t contarBotonesPulsados() {
  uint8_t count = 0;
  for (uint8_t i = 0; i < 5; i++) {
    if (digitalRead(PIN_BTN[i]) == LOW) count++;
  }
  return count;
}

// ============================================================
// Control de iluminación escalonado 8 LEDs con histéresis
// Implementa control proporcional inverso DISCRETO:
// - Setpoint: 500 lux (nivel óptimo)
// - Umbral de activación: 400 lux
// - Histéresis: ±10 lux (zona muerta: 390-410 lux)
// Lógica:
//   luz < 390 lux → encender 1 LED (contador++)
//   luz > 410 lux → apagar 1 LED (contador--)
//   390-410 lux → sin cambios (zona muerta)
void controlIluminacion() {  
  // Lógica con histéresis
  if (luzLux < (LUZ_UMBRAL - LUZ_HISTERESIS)) 
  {
    // luz < 390 lux: aumentar iluminación artificial
    
    // Intervalo de lux por cada LED
    float intervaloLux = (float)LUZ_UMBRAL / 8.0f;
    
    // Número de LEDs = cuántos intervalos caben en la luz actual
    // A menos luz → más LEDs encendidos (relación inversa)
    int ledsCalculados = 8 - (int)(luzLux / intervaloLux);

    // Acotar entre 1 y 8
    ledsCalculados = constrain(ledsCalculados, 1, 8);
    
    if (ledsCalculados != ledsEncendidos) 
    {
      ledsEncendidos = ledsCalculados;  // Aumentar LEDs si es necesario
      
      // Actualizar LEDs según cantidad calculada
      actualizarLeds(ledsEncendidos);

      Serial.print(F("Luz=")); Serial.print(luzLux);
      Serial.print(F(" LEDs=")); Serial.println(ledsEncendidos);      
    }
  } else if (luzLux > (LUZ_UMBRAL + LUZ_HISTERESIS)) 
  {
    // luz > 410 lux: apagar iluminación artificial
    if (ledsEncendidos > 0) 
    {
      // Apagar todos los LEDs si hay suficiente luz natural
      ledsEncendidos = 0;
      actualizarLeds(ledsEncendidos);  
    }
  }
  else 
  {
    // Zona muerta (390-410 lux): sin cambios
  }

}

// ============================================================
// controlTemperatura() — Controlador PID discreto
//
// Calcula la señal de control u(t) = Kp·e + Ki·∑e·Δt + Kd·Δe/Δt
// y la convierte en actuación ON/OFF sobre las válvulas de frío
// (EV_FRIO, LED azul) y calor (EV_CALOR, LED rojo) mediante una
// banda muerta (±PID_DEADBAND).
//
// Signo del error:
//   error > 0  → temperatura por debajo del setpoint → CALENTAR
//   error < 0  → temperatura por encima del setpoint → ENFRIAR
//   |error| pequeño → |u| < deadband                 → REPOSO
//
// Anti-windup: el integrador se satura en ±PID_INT_MAX para evitar
// que un error prolongado lo desborde e impida la respuesta rápida
// cuando las condiciones cambian.
// ============================================================
void controlTemperatura() {

    unsigned long ahora = millis();
    float dt = (ahora - tUltimoPID) / 1000.0f;
    if (dt <= 0.0f) dt = 0.001f;
    tUltimoPID = ahora;

    // Error: positivo → falta calor, negativo → falta frío
    float error = TEMP_SETPOINT - tempMedida;

    // ------------------------------------------------------------
    // 1) ZONA MUERTA: si |error| < TEMP_ZONA_M → NO CONTROL
    // ------------------------------------------------------------
    if (fabs(error) < TEMP_ZONA_M) {

        // Congelar integrador para evitar windup
        pidIntegral = pidIntegral;  // (no cambia)

        // Apagar válvulas
        AccionTemp accionAnterior = accionTemp;
        accionTemp = REPOSO_TEMP;
        digitalWrite(PIN_EV_FRIO, LOW);
        digitalWrite(PIN_EV_CALOR, LOW);

        if (accionTemp != accionAnterior) {
            Serial.print(F("Temp="));
            Serial.print(tempMedida, 1);
            Serial.print(F("°C (zona muerta) Accion=REPOSO\n"));
        }

        return;  // Salir sin ejecutar PID
    }

    // ------------------------------------------------------------
    // 2) PID ACTIVO (solo fuera de la zona muerta)
    // ------------------------------------------------------------

    // Integral con anti-windup
    pidIntegral += error * dt;
    pidIntegral = constrain(pidIntegral, -PID_INT_MAX, PID_INT_MAX);

    // Derivada
    float derivada = (error - pidErrorPrev) / dt;
    pidErrorPrev = error;

    // Salida PID
    float salida = PID_KP * error + PID_KI * pidIntegral + PID_KD * derivada;

    // ------------------------------------------------------------
    // 3) Banda muerta del PID (PID_DEADBAND)
    // ------------------------------------------------------------
    AccionTemp accionAnterior = accionTemp;

    if (salida > PID_DEADBAND) {
        accionTemp = CALENTAR;
        digitalWrite(PIN_EV_FRIO, LOW);
        digitalWrite(PIN_EV_CALOR, HIGH);

    } else if (salida < -PID_DEADBAND) {
        accionTemp = ENFRIAR;
        digitalWrite(PIN_EV_FRIO, HIGH);
        digitalWrite(PIN_EV_CALOR, LOW);

    } else {
        accionTemp = REPOSO_TEMP;
        digitalWrite(PIN_EV_FRIO, LOW);
        digitalWrite(PIN_EV_CALOR, LOW);
    }

    if (accionTemp != accionAnterior) {
        Serial.print(F("Temp="));
        Serial.print(tempMedida, 1);
        Serial.print(F("°C u="));
        Serial.print(salida, 2);
        Serial.print(F(" Accion="));
        Serial.println(
            accionTemp == ENFRIAR ? F("ENFRIAR") :
            accionTemp == CALENTAR ? F("CALENTAR") :
                                     F("REPOSO")
        );
    }
}

// ============================================================
// Escribir byte al 74HC595
//
// Transfiere un byte al registro de desplazamiento 74HC595 usando
//  comunicación SPI en modo software (bit-banging), empezando por el bit
// más significativo (MSB first), y activa el latch para que las salidas
// QA–QH reflejen el nuevo valor.
//
// Secuencia para cada bit:
//   1. Poner SRCLK a LOW.
//   2. Poner DS al valor del bit actual.
//   3. Poner SRCLK a HIGH → el bit entra en el registro de desplazamiento.
//
// Activación del latch (STCP):
//   1. RCLK a LOW → preparar latch.
//   2. RCLK a HIGH → copiar registro de desplazamiento a registro de salida.
//
// @param valor  Byte de 8 bits a enviar (bit 7 → QA, bit 0 → QH si MSB first).
//
// Nota: Se usa shiftOut() de Arduino, que internamente realiza la secuencia
//       descrita y envía en orden MSBFIRST.
//
void escribir595(uint8_t valor) {
  // Bajar latch antes de enviar datos
  digitalWrite(PIN_595_LATCH, LOW);

  // Enviar los 8 bits al registro de desplazamiento (MSB primero)
  shiftOut(PIN_595_DATA, PIN_595_CLOCK, MSBFIRST, valor);

  // Subir latch para actualizar las salidas QA–QH
  digitalWrite(PIN_595_LATCH, HIGH);
}

// ============================================================
// Leer mando IR 
void leerIR() {
  if (!IrReceiver.decode()) {
    return;
  }

  // Ignorar códigos de repetición y datos inválidos
  if (IrReceiver.decodedIRData.flags & IRDATA_FLAGS_IS_REPEAT) {
    IrReceiver.resume();
    return;
  }

  uint32_t codigo = IrReceiver.decodedIRData.decodedRawData;

  if (codigo == 0 || IrReceiver.decodedIRData.protocol != NEC ||
      IrReceiver.decodedIRData.numberOfBits != 32) {
    Serial.print(F("[DEBUG] IR ignorado: protocolo="));
    Serial.print(IrReceiver.decodedIRData.protocol);
    Serial.print(F(" bits="));
    Serial.print(IrReceiver.decodedIRData.numberOfBits);
    Serial.print(F(" raw=0x"));
    Serial.println(codigo, HEX);
    IrReceiver.resume();
    return;
  }

  Serial.print(F("[DEBUG] IR command: "));
  Serial.println(IrReceiver.decodedIRData.command);
  Serial.print(F("[DEBUG] IR raw: 0x"));
  Serial.println(codigo, HEX);

  for (uint8_t i = 0; i < 5; i++) {
    if (codigo == IR_CODIGO[i] && plantaDestino != i + 1) {
      plantaDestino = i + 1;
      Serial.print(F("IR -> P")); Serial.println(plantaDestino);
      break;
    }
  }
  IrReceiver.resume();
}

// ============================================================
// Calcula la iluminancia en lux a partir del LDR (A0)
//
// Nota: No vamos a tener en cuenta el "ruido" de la digitalización.
// La función puede mostrar una diferencia entre el valor calculado
// y el valor mostrado en el slide debido a la resolución del ADC 
// y el redondeo matemático.
//
float leerLux() {
  // Leer el valor del ADC (0 a 1023)
  int analogValue = analogRead(PIN_LDR);
    
  if (analogValue < 0) return 0;
  
  // Convertir a Lux usando la calibración específica de Wokwi  
  // Transformamos el valor analógico en un valor de resistencia
  // En Wokwi, el ADC de 1023 es oscuridad total (10 lux aprox) 
  // y valores bajos son mucha luz.
  
  // Calcula la resistencia del LDR usando el divisor de tensión del módulo.
  float voltage = analogValue / 1024.0 * 5.0;
  float resistance = 2000 * voltage / (1 - voltage / 5.0);
   
  // Convierte la resistencia a Lux mediante la ecuación característica del sensor.
  // Cálculo de Lux: Aplicación del modelo logarítmico (Ley de Power-Law) del LDR.
  float lux = pow(RL10 * 1e3 * pow(10, GAMMA) / resistance, (1 / GAMMA));

  return lux;
}

// ============================================================
// Leer pulsadores de planta 
void leerPulsadores() {
  for (uint8_t i = 0; i < 5; i++) {
    bool estado = digitalRead(PIN_BTN[i]);
    if (estado == LOW && btnEstadoPrev[i] == HIGH) {
      if (millis() - btnUltimoRebote[i] > 50) {
        btnUltimoRebote[i] = millis();
        plantaDestino = i + 1;
        Serial.print(F("Pulsador P")); Serial.println(plantaDestino);
      }
    }
    btnEstadoPrev[i] = estado;
  }
}

// ============================================================
// Leer sensores ambientales (DHT22 + LDR)
//
// Solo imprime por Serial si alguno de los tres valores monitorizados
// (temperatura, humedad o lux) ha cambiado respecto a la lectura
// anterior. Esto evita inundar el monitor serie con datos idénticos
// en cada ciclo de 2 s cuando el entorno es estable.
void leerSensores() {
  // DHT22
  float t = dht.readTemperature();
  float h = dht.readHumidity();
  float l = leerLux();            // Leer lux del sensor LDR

  bool cambio = false;
  
  if (!isnan(t) && t != tempMedida) {
    tempMedida  = t;
    cambio = true;
  }
  if (!isnan(h) && h != humMedida) {
    humMedida = h;
    cambio = true;
  }

   // Detectar cambio en lux (control en controlIluminacion())
  if (l != luzLux) {
    luzLux = l;
    cambio = true;
  }

  // Solo imprimir si algún valor ha variado desde la última lectura
  if (cambio) {
    Serial.print(F("T="));    Serial.print(tempMedida);
    Serial.print(F("°C H=")); Serial.print(humMedida);
    Serial.print(F("% Luz=")); Serial.print(luzLux);
    Serial.println(F("lux"));
  }

}

// ============================================================
// MÁQUINA DE ESTADOS DEL ASCENSOR
// ============================================================

// ============================================================
// Función principal de la máquina de estados
// Se ejecuta en cada iteración del loop() y maneja el comportamiento
// del ascensor según su estado actual
// ============================================================
void manejarEstadoAscensor() {
  unsigned long ahora = millis();
  
  switch (estadoAscensor) {
    
    case ASCENSOR_REPOSO:
      // === ESTADO: ASCENSOR EN REPOSO ===
      // Cabina parada en una planta, esperando comandos del usuario
      
      // Acciones en este estado:
      // - Leer entrada cada 100ms
      // - Mantener cabina detenida
      // - Mostrar estado normal en LCD      

      // Transiciones:
      // - Si se selecciona planta diferente → MOVIMIENTO
      // - Si hay condición de emergencia → EMERGENCIA
      
      if (plantaDestino != plantaActual) {
        // Nueva planta solicitada
        transicionEstado(ASCENSOR_MOVIMIENTO);
      }
      
      // Verificar condiciones de emergencia
      if (presencia && cabinaMov) {  // Si hay presencia durante movimiento (anómalo)
        transicionEstado(ASCENSOR_EMERGENCIA);
      }
      
      break;
      
    case ASCENSOR_MOVIMIENTO:
      // === ESTADO: ASCENSOR EN MOVIMIENTO ===
      // Cabina moviéndose hacia la planta destino
      
      // Acciones en este estado:
      // - Mover servo gradualmente
      // - Actualizar posición
      // - Verificar llegada a destino
      
      // Transiciones:
      // - Si llega a destino → PUERTA_ABIERTA (o directamente REPOSO)
      // - Si hay emergencia → EMERGENCIA
    
      // Mover servo cada 65ms (≈ 45º cada 3000 ms)
      // para simular movimiento gradual
      if (ahora - tUltimoServo >= 65) {
        tUltimoServo = ahora;
        moverAscensor();
      }
      
      // Verificar si llegó a destino
      if (plantaActual == plantaDestino && !cabinaMov) {
        // Llegó a destino, simular apertura de puerta
        transicionEstado(ASCENSOR_PUERTA_ABIERTA);
      }
      
      // Verificar emergencia: las TRES condiciones deben cumplirse juntas:
      //   1. Ascensor en MOVIMIENTO (estamos dentro de este caso)
      //   2. Presencia detectada por PIR (sensor exterior)
      //   3. Más de un pulsador pulsado simultáneamente (situación de pánico)
      if (presencia && contarBotonesPulsados() > 1) {
        transicionEstado(ASCENSOR_EMERGENCIA);
      }
      
      break;
      
    case ASCENSOR_PUERTA_ABIERTA:
      // === ESTADO: PUERTA ABIERTA ===
      // Cabina llegó a destino, puerta abierta para entrada/salida
      
      // Acciones en este estado:
      // - Simular tiempo de puerta abierta
      // - Esperar antes de cerrar
      
      // Transiciones:
      // - Después de tiempo → REPOSO
      // - Si nueva planta solicitada → MOVIMIENTO (puerta se cierra automáticamente)
      
      // Simular tiempo de puerta abierta (3 segundos)
      static unsigned long tiempoPuertaAbierta = 0;
      if (tiempoPuertaAbierta == 0) {
        tiempoPuertaAbierta = ahora;
      }
      
      if (ahora - tiempoPuertaAbierta >= 3000) {  // 3 segundos
        tiempoPuertaAbierta = 0;
        transicionEstado(ASCENSOR_REPOSO);
      }
      
      // Si se solicita nueva planta, cerrar puerta inmediatamente
      if (plantaDestino != plantaActual) {
        tiempoPuertaAbierta = 0;
        transicionEstado(ASCENSOR_MOVIMIENTO);
      }
      
      break;
      
    case ASCENSOR_EMERGENCIA:
      // === ESTADO: EMERGENCIA ===
      // Parada de seguridad por alguna condición anómala
      
      // Acciones en este estado:
      // - Detener todo movimiento
      // - Activar indicadores de emergencia
      // - Esperar reset manual
      
      // Transiciones:
      // - Solo reset manual → REPOSO
      
      // Detener movimiento
      cabinaMov = false;
      
      // Aquí se podría activar LED de emergencia, buzzer, etc.
      // Por simplicidad, solo mostrar en Serial
      if (millis() % 1000 < 100) {  // Cada segundo, mensaje breve
        Serial.println(F("¡EMERGENCIA! Ascensor detenido."));
      }
      
      // Reset manual: presionar todos los botones al mismo tiempo (simulado)
      // O simplemente esperar y reset después de tiempo
      static unsigned long tiempoEmergencia = 0;
      if (tiempoEmergencia == 0) {
        tiempoEmergencia = ahora;
      }
      
      if (ahora - tiempoEmergencia >= 10000) {  // 10 segundos de emergencia
        tiempoEmergencia = 0;
        Serial.println(F("Reset automático de emergencia."));
        transicionEstado(ASCENSOR_REPOSO);
      }
      
      break;
      
    case ASCENSOR_MANTENIMIENTO:
      // === ESTADO: MANTENIMIENTO ===
      // Estado especial para debug, calibración o mantenimiento
      
      // Acciones en este estado:
      // - Funciones de diagnóstico
      // - Movimiento manual
      // - Tests de sensores
      
      // Transiciones:
      // - Reset manual → REPOSO
      
      // Mostrar información de debug
      if (millis() % 2000 < 100) {
        Serial.println(F("MODO MANTENIMIENTO - Debug activo"));
      }
      
      break;
      
    default:
      // Estado desconocido, reset a reposo
      Serial.println(F("Error: Estado desconocido, reseteando a REPOSO"));
      transicionEstado(ASCENSOR_REPOSO);
      break;
  }
}

// ============================================================
// Mover ascensor gradualmente (sin delay)
void moverAscensor() {
  if (plantaDestino == plantaActual) {
    cabinaMov = false;
    return;
  }

  cabinaMov = true;
  int destino = ANGULO_PLANTA[plantaDestino - 1];

  if (anguloServo < destino) {
    // Servo necesita aumentar ángulo para subir
    anguloServo += SERVO_VELOCIDAD;
    if (anguloServo >= destino) anguloServo = destino;
  } else {
    // Servo necesita disminuir ángulo para bajar
    anguloServo -= SERVO_VELOCIDAD;
    if (anguloServo <= destino) anguloServo = destino;
  }

  servoAscensor.write(anguloServo);

  if (anguloServo == destino) {
    plantaActual = plantaDestino;
    cabinaMov = false;
    Serial.print(F("Llegada P")); Serial.println(plantaActual);
  }
  else {
    // Actualizar planta actual según el ángulo del servo para reflejar el movimiento
    switch (anguloServo) {
    case 180:
      // 180→P5
      plantaActual = 5;   // Planta más alta
      break;
    
    case 135:
      // 135→P4
      plantaActual = 4;   // Planta intermedia
      break;
    
    case 90:
      // 90→P3
      plantaActual = 3;   // Planta intermedia
      break;
    
    case 45:
      // 45→P2
      plantaActual = 2;   // Planta intermedia
      break;
    
    case 0:
      // 0→P1
      plantaActual = 1;   // Planta más baja
      break;
    
    default:
      break;
    }
  }
}

// ============================================================
// Función de transición entre estados
// Maneja el cambio de estado y ejecuta acciones de entrada/salida
// ============================================================
const char* nombreEstado(EstadoAscensor e) {
  switch (e) {
    case ASCENSOR_REPOSO:        return "REPOSO";
    case ASCENSOR_MOVIMIENTO:    return "MOVIMIENTO";
    case ASCENSOR_EMERGENCIA:    return "EMERGENCIA";
    case ASCENSOR_PUERTA_ABIERTA: return "PUERTA_ABIERTA";
    case ASCENSOR_MANTENIMIENTO: return "MANTENIMIENTO";
    default:                     return "DESCONOCIDO";
  }
}

// ============================================================
// padLCD (Función auxiliar)
//
// Rellena con espacios hasta 16 caracteres
String padLCD(String texto) {
  if (texto.length() > 16) return texto.substring(0, 16); // truncar
  while (texto.length() < 16) texto += ' ';               // rellenar
  return texto;
}

void transicionEstado(EstadoAscensor nuevoEstado) {
  // Acciones de salida del estado actual
  switch (estadoAscensor) {
    case ASCENSOR_MOVIMIENTO:
      // Al salir de movimiento, asegurar que cabina esté detenida
      cabinaMov = false;
      break;
      
    case ASCENSOR_EMERGENCIA:
      // Al salir de emergencia, reset indicadores
      Serial.println(F("Saliendo de modo emergencia"));
      break;
      
    default:
      break;
  }
  
  // Cambiar estado
  EstadoAscensor estadoAnterior = estadoAscensor;
  estadoAscensor = nuevoEstado;
  
  // Acciones de entrada al nuevo estado
  switch (nuevoEstado) {
    case ASCENSOR_REPOSO:
      Serial.print(F("Estado: REPOSO en planta "));
      Serial.println(plantaActual);
      break;
      
    case ASCENSOR_MOVIMIENTO:
      Serial.print(F("Estado: MOVIMIENTO P"));
      Serial.print(plantaActual);
      Serial.print(F(" → P"));
      Serial.println(plantaDestino);
      cabinaMov = true;
      paginaLCD = 0;
      actualizarLCD();
      break;
      
    case ASCENSOR_PUERTA_ABIERTA:
      Serial.print(F("Estado: PUERTA_ABIERTA en P"));
      Serial.println(plantaActual);
      break;
      
    case ASCENSOR_EMERGENCIA:
      Serial.println(F("¡ESTADO: EMERGENCIA! Ascensor detenido por seguridad"));
      break;
      
    case ASCENSOR_MANTENIMIENTO:
      Serial.println(F("Estado: MANTENIMIENTO activado"));
      break;
  }
  
  // Log de transición
  if (estadoAnterior != nuevoEstado) {
    Serial.print(F("Transición: "));
    Serial.print(nombreEstado(estadoAnterior));
    Serial.print(F(" → "));
    Serial.println(nombreEstado(nuevoEstado));
  }
}