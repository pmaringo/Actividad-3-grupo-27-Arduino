/********** INSTRUMENTACIÓN ELECTRÓNICA **********
 *
 * Programa ARDUINO MUIT 2026
 * Asignatura: Equipos e Instrumentación Electrónica
 * Profesor: Ivan Araquistain
 * 
 * ============================================================
 * Actividad 3 - Ascensor Inteligente ACME S.A.
 * 
 * Autor: Pablo Marín
 * Fecha: 11 Mayo, 2026 
 * ============================================================
 * MODIFICACIÓN v4.0 - Control Difuso (Fuzzy Logic) de Movimiento
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
 *   - 5 pulsadores de planta (PCF8574 I2C 0x20, P0-P4)
 * ============================================================
 *
 * ============================================================
 *  LÓGICA DE CONTROL DEL ASCENSOR — Máquina de Estados Finitos
 * ============================================================
 *  El ascensor se modela como una FSM (Finite State Machine) con
 *  6 estados bien definidos:
 *
 *    REPOSO         → Cabina detenida en una planta, esperando órdenes.
 *                     Se aceptan comandos de pulsadores e IR.
 *    PUERTA_CERRADA → Transición de seguridad antes
 *                     de iniciar el movimiento. Permite 2 s para que
 *                     la puerta se cierre completamente.
 *    MOVIMIENTO     → Cabina desplazándose hacia la planta destino.
 *                     El servo se controla mediante lógica difusa (fuzzy)
 *                     que genera un perfil de velocidad realista:
 *                     arranque progresivo, crucero estable y frenado suave.
 *                     Durante el movimiento se pueden añadir nuevas
 *                     solicitudes; el algoritmo SCAN recalcula el
 *                     destino prioritario dinámicamente.
 *    PUERTA_ABIERTA → La cabina llegó al destino y permanece 3 s con
 *                     "puerta abierta" antes de pasar a PUERTA_CERRADA.
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
 *  Diagrama de transiciones v4.0:
 *
 *   [REPOSO] ──(nueva solicitud)──► [PUERTA_CERRADA] ──(2 s)──► [MOVIMIENTO]
 *      ▲                              │                              │
 *      │         (llegó destino)      │                              │
 *      │◄──[PUERTA_ABIERTA]◄──────────┘                              │
 *      │         (3 s)                                               │
 *      │                                                             │
 *      └◄──[EMERGENCIA]◄──(mvto + presencia + >1 botón)─────────────┘
 *               (reset 10 s)
 *
 *  Fuentes de comando: pulsadores (PCF8574 P0-P4) y mando IR NEC.
 *  Los comandos se encolan en cualquier estado excepto EMERGENCIA.
 *
 * ============================================================
 *  SISTEMA DE COLA MÚLTIPLE Y ALGORITMO DE PRIORIDAD SCAN v3.0
 * ============================================================
 *  El usuario puede pulsar tantas plantas como desee, incluso con
 *  el ascensor en movimiento. Las solicitudes se gestionan mediante:
 *
 *    · Array booleano solicitudes[5]        → indica plantas pendientes.
 *    · Array contadorSolicitudes[5]         → número de pulsaciones
 *      acumuladas por planta (criterio de prioridad secundario).
 *    · Array tiempoSolicitud[5]           → timestamp de la primera
 *      pulsación de cada planta (criterio de prioridad terciario).
 *    · Dirección actual (DIR_SUBIENDO / DIR_BAJANDO / DIR_NINGUNA).
 *
 *  Algoritmo de prioridad (SCAN):
 *    1. Buscar solicitudes en la dirección actual de marcha.
 *       Se prioriza la menor distancia al piso actual.
 *    2. Si hay empate en distancia, gana la planta con mayor número
 *       de solicitudes acumuladas (contadorSolicitudes).
 *    3. Si persiste el empate, gana la planta con mayor tiempo de
 *       espera (tiempoSolicitud más antiguo).
 *    4. Si no hay solicitudes en la dirección actual, invertir
 *       dirección y repetir la búsqueda.
 *    5. Si la única solicitud es la planta actual, se atiende
 *       reabriendo puertas en esa misma planta.
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
 * ============================================================
 *  LÓGICA DE CONTROL DE TEMPERATURA — Controlador PID
 * ============================================================
 *  Objetivo: mantener la temperatura interior en TEMP_SETPOINT = 25 °C
 *  actuando sobre la electroválvula de frío (EV_FRIO, LED azul, pin 10)
 *  y la de calor (EV_CALOR, LED rojo, pin 12).
 *
 * ============================================================
 *  CONTROL DIFUSO (FUZZY LOGIC) DE MOVIMIENTO — v4.0 [NUEVO]
 * ============================================================
 *  Se sustituye el control de velocidad fija (1°/ciclo) por un
 *  sistema de control difuso que genera un perfil de velocidad
 *  trapezoidal suave, imitando el comportamiento de un ascensor real:
 *
 *    · Arranque progresivo: aceleración suave desde reposo hasta
 *      velocidad de crucero.
 *    · Velocidad estable (crucero): mantenimiento de velocidad
 *      constante en trayectos largos.
 *    · Frenado suave: deceleración progresiva al acercarse al destino.
 *    · Precisión de parada: corrección fina en los últimos grados.
 *
 *  Variables de entrada al controlador difuso:
 *    · distanciaRestante → distancia angular al piso destino (0–180°)
 *    · velocidadActual   → velocidad del ciclo anterior (0–8°/ciclo)
 *
 *  Variable de salida:
 *    · deltaAngulo       → incremento de ángulo para este ciclo (0–6°)
 *
 *  Método de inferencia: Mamdani (min-max) con defuzzificación
 *  por centro de gravedad ponderado (método de los singletones).
 *
 *  Conjuntos difusos de entrada:
 *    · distanciaRestante: CERCA (0–15°), MEDIA (10–45°), LEJOS (35–180°)
 *    · velocidadActual:   LENTA (0–2°), MEDIA (1.5–4.5°), RÁPIDA (4–8°)
 *
 *  Singletones de salida (deltaAngulo):
 *    · MUY_LENTO = 1°, LENTO = 2°, MEDIO = 3°, RÁPIDO = 5°, MUY_RÁPIDO = 6°
 *
 *  Reglas principales (9 reglas):
 *    1. Si distancia=CERCA  Y velocidad=LENTA   → MUY_LENTO (precisión)
 *    2. Si distancia=CERCA  Y velocidad=MEDIA  → MUY_LENTO (frenado)
 *    3. Si distancia=CERCA  Y velocidad=RÁPIDA  → LENTO    (frenado suave)
 *    4. Si distancia=MEDIA  Y velocidad=LENTA   → MEDIO    (aceleración)
 *    5. Si distancia=MEDIA  Y velocidad=MEDIA   → RÁPIDO   (crucero)
 *    6. Si distancia=MEDIA  Y velocidad=RÁPIDA  → LENTO    (frenado)
 *    7. Si distancia=LEJOS  Y velocidad=LENTA    → RÁPIDO   (arranque)
 *    8. Si distancia=LEJOS  Y velocidad=MEDIA    → RÁPIDO   (crucero)
 *    9. Si distancia=LEJOS  Y velocidad=RÁPIDA  → MUY_RÁPIDO (máx. velocidad)
 *
 *  El controlador se ejecuta en cada ciclo de servo (65 ms) dentro de
 *  moverAscensor(), calculando dinámicamente el incremento óptimo en
 *  función del estado actual del sistema.
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
#define PIN_595_DATA    6   // Pin de datos serie (DS) del 74HC595
#define PIN_595_CLOCK   5   // Pin de reloj de desplazamiento (SHCP) del 74HC595
#define PIN_595_LATCH   4   // Pin de latch (STCP) del 74HC595

// Direccion I2C del PCF8574 (A0=A1=A2=GND -> 0x20)
const uint8_t PCF8574_ADDR = 0x20;

// Pines del PCF8574 donde estan conectados los pulsadores de planta
const uint8_t BOTON_P0 = 0;
const uint8_t BOTON_P1 = 1;
const uint8_t BOTON_P2 = 2;
const uint8_t BOTON_P3 = 3;
const uint8_t BOTON_P4 = 4;

// Mascara para los 5 pulsadores (bits 0-4)
const uint8_t BOTONES_MASK = 0x1F;

// Variables para anti-rebote y deteccion de flanco
uint8_t estadoAnterior = 0xFF;
uint8_t ultimaLectura = 0xFF;
unsigned long ultimoCambio = 0;
const unsigned long DEBOUNCE_MS = 25;

// --- Temperatura: Constantes de control ---
#define TEMP_SETPOINT   25.0f
#define TEMP_ZONA_M      3.0f
#define HUM_SETPOINT    60.0f

// Parámetros de sintonía del PID
#define PID_KP          2.0f
#define PID_KI          0.05f
#define PID_KD          1.0f
#define PID_DEADBAND    1.0f
#define PID_INT_MAX     50.0f
#define PID_INTERVALO   500

// Iluminación: valores en lux
#define GAMMA           0.7f
#define RL10           50.0f
#define LUZ_SETPOINT    500
#define LUZ_UMBRAL      400
#define LUZ_HISTERESIS  10

// --- [MODIFICACIÓN v4.0] CONSTANTES DEL CONTROL DIFUSO (FUZZY) ---
// ================================================================
// Parámetros del controlador difuso de movimiento del ascensor.
// Estos valores definen los conjuntos difusos y singletones que
// generan el perfil de velocidad trapezoidal suave.
// ================================================================

// Intervalo de tiempo entre actualizaciones del servo (ms)
// A 65 ms se obtienen ~15 actualizaciones/segundo
#define SERVO_INTERVALO   65

// Conjuntos difusos de ENTRADA 1: distanciaRestante (grados)
// CERCA: 0–15° (trapezoidal: 0,0,10,15)
// MEDIA: 10–45° (trapezoidal: 10,15,35,45)
// LEJOS: 35–180° (trapezoidal: 35,45,180,180)
#define FUZZY_DIST_CERCA_A   0.0f
#define FUZZY_DIST_CERCA_B   0.0f
#define FUZZY_DIST_CERCA_C  10.0f
#define FUZZY_DIST_CERCA_D  15.0f

#define FUZZY_DIST_MEDIA_A  10.0f
#define FUZZY_DIST_MEDIA_B  15.0f
#define FUZZY_DIST_MEDIA_C  35.0f
#define FUZZY_DIST_MEDIA_D  45.0f

#define FUZZY_DIST_LEJOS_A  35.0f
#define FUZZY_DIST_LEJOS_B  45.0f
#define FUZZY_DIST_LEJOS_C 180.0f
#define FUZZY_DIST_LEJOS_D 180.0f

// Conjuntos difusos de ENTRADA 2: velocidadActual (grados/ciclo)
// LENTA:  0–2°/ciclo  (trapezoidal: 0,0,1.5,2.5)
// MEDIA:  1.5–4.5°/ciclo (triangular: 1.5,3,4.5)
// RÁPIDA: 4–8°/ciclo (trapezoidal: 3.5,4.5,8,8)
#define FUZZY_VEL_LENTA_A    0.0f
#define FUZZY_VEL_LENTA_B    0.0f
#define FUZZY_VEL_LENTA_C    1.5f
#define FUZZY_VEL_LENTA_D    2.5f

#define FUZZY_VEL_MEDIA_A    1.5f
#define FUZZY_VEL_MEDIA_B    3.0f
#define FUZZY_VEL_MEDIA_C    4.5f

#define FUZZY_VEL_RAPIDA_A   3.5f
#define FUZZY_VEL_RAPIDA_B   4.5f
#define FUZZY_VEL_RAPIDA_C   8.0f
#define FUZZY_VEL_RAPIDA_D   8.0f

// Singletones de SALIDA: deltaAngulo (grados por ciclo)
// Estos valores determinan la velocidad instantánea del servo
#define FUZZY_OUT_MUY_LENTO  1.0f   // Precisión de parada
#define FUZZY_OUT_LENTO      2.0f   // Frenado suave / arranque inicial
#define FUZZY_OUT_MEDIO      3.0f   // Velocidad intermedia
#define FUZZY_OUT_RAPIDO     5.0f   // Velocidad de crucero
#define FUZZY_OUT_MUY_RAPIDO 6.0f   // Máxima velocidad (trayectos largos)

// Velocidad máxima permitida por ciclo (protección)
#define FUZZY_VEL_MAX        6.0f

// Umbral de distancia para forzar velocidad mínima (precisión final)
#define FUZZY_DIST_MINIMA    3.0f
// ================================================================

// Ángulos del servo para cada planta (0→P1, 180→P5)
const int ANGULO_PLANTA[5] = {0, 45, 90, 135, 180};

// Códigos IR NEC — botones 1..5 (mando WOKWI)
const uint32_t IR_CODIGO[5] = {
  0xCF30FF00, 0xE718FF00, 0x857AFF00, 0xEF10FF00, 0xC738FF00
};

// ============================================================
// SISTEMA DE COLA DE SOLICITUDES Y ALGORITMO DE PRIORIDAD SCAN
// ============================================================
#define MAX_PLANTAS           5
#define TIEMPO_CIERRE_PUERTA  2000

bool          solicitudes[MAX_PLANTAS] = {false, false, false, false, false};
uint8_t       numSolicitudes = 0;
unsigned long tiempoSolicitud[MAX_PLANTAS] = {0, 0, 0, 0, 0};
uint8_t       contadorSolicitudes[MAX_PLANTAS] = {0, 0, 0, 0, 0};

enum DireccionAscensor { DIR_NINGUNA, DIR_SUBIENDO, DIR_BAJANDO };
DireccionAscensor direccionActual = DIR_NINGUNA;

unsigned long tPuertaAbierta = 0;
unsigned long tPuertaCerrada = 0;
unsigned long tEmergencia    = 0;

// --- Inicialización de Objetos ---
LiquidCrystal_I2C lcd(0x27, 16, 2);
Servo             servoAscensor;
DHT               dht(PIN_DHT, DHT22);

// --- Variables Globales ---
uint8_t plantaActual    = 1;
uint8_t plantaDestino   = 1;
bool    cabinaMov       = false;
int     anguloServo     = 0;

float   tempMedida      = 25.0f;
float   humMedida       = 80.0f;
float   luzLux          = 500.0f;

bool    presencia       = false;

enum AccionTemp { REPOSO_TEMP, CALENTAR, ENFRIAR };
AccionTemp accionTemp = REPOSO_TEMP;

float         pidIntegral  = 0.0f;
float         pidErrorPrev = 0.0f;
unsigned long tUltimoPID   = 0;

// --- [MODIFICACIÓN v4.0] VARIABLES DEL CONTROL DIFUSO ---
// Velocidad actual del ascensor (grados por ciclo de servo)
// Se utiliza como realimentación para el controlador difuso
float         velocidadActual = 0.0f;

// --- MÁQUINA DE ESTADOS DEL ASCENSOR ---
enum EstadoAscensor {
  ASCENSOR_REPOSO,
  ASCENSOR_PUERTA_CERRADA,
  ASCENSOR_MOVIMIENTO,
  ASCENSOR_EMERGENCIA,
  ASCENSOR_PUERTA_ABIERTA,
  ASCENSOR_MANTENIMIENTO
};
EstadoAscensor estadoAscensor = ASCENSOR_REPOSO;

uint8_t ledsEncendidos = 0;

unsigned long tUltimoProceso  = 0;
unsigned long tUltimaLectura  = 0;
unsigned long tUltimaPulsaci  = 0;
unsigned long tUltimoLCD      = 0;
unsigned long tUltimoServo    = 0;
uint8_t       paginaLCD       = 0;

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
void manejarEstadoAscensor();
void moverAscensor();
const char* nombreEstado(EstadoAscensor e);
String padLCD(String texto);
uint8_t pcf8574_read();
void pcf8574_write(uint8_t valor);
void transicionEstado(EstadoAscensor nuevoEstado);

void agregarSolicitud(uint8_t planta);
void eliminarSolicitud(uint8_t planta);
uint8_t calcularPrioridad();

// [MODIFICACIÓN v4.0] Prototipos del control difuso
float fuzzificarTrapezoidal(float x, float a, float b, float c, float d);
float fuzzificarTriangular(float x, float a, float b, float c);
float controlFuzzy(float distancia, float velocidad);

// ============================================================
// --- Configuración Inicial (setup) ---
// ============================================================
void setup() {
  Serial.begin(9600);

  Serial.println(F("UNIR Actividad 2 - Ascensor ACME v4.0"));
  Serial.println(F("[MOD v4.0] Control Difuso (Fuzzy Logic) de Movimiento"));
  Serial.println(F(""));

  // Inicializar LCD
  Serial.println(F("Inicializando LCD..."));
  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("ACME ASCENSOR v4.0");
  lcd.setCursor(0, 1);
  lcd.print("Fuzzy Control ON");
  delay(1500);

  Serial.println(F("Pantalla LCD inicializada."));
  Serial.println(F("------------------------------------"));

  // Inicializar bus I2C para lectura de pulsadores (PCF8574)
  Serial.println(F("Inicializando bus I2C para pulsadores..."));

  bool encontrado = false;
  for (uint8_t addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    uint8_t error = Wire.endTransmission();
    if (error == 0) {
      Serial.print(F("0x"));
      if (addr < 16) Serial.print(F("0"));
      Serial.print(addr, HEX);
      Serial.print(F(" "));
      if (addr == PCF8574_ADDR) encontrado = true;
    }
  }
  Serial.println();

  if (!encontrado) {
    Serial.println(F("ADVERTENCIA: No se detecto PCF8574 en 0x20!"));
  } else {
    Serial.println(F("PCF8574 detectado correctamente."));
  }

  pcf8574_write(0xFF);

  Serial.println(F("Pulsadores listos. Presiona cualquier boton..."));
  Serial.println(F("Formato: [BOTON_X] PRESIONADO / SOLTADO"));
  Serial.println();

  pinMode(PIN_PIR, INPUT);

  pinMode(PIN_EV_FRIO, OUTPUT);
  pinMode(PIN_EV_CALOR, OUTPUT);
  digitalWrite(PIN_EV_FRIO, LOW);
  digitalWrite(PIN_EV_CALOR, LOW);

  pinMode(PIN_595_DATA,  OUTPUT);
  pinMode(PIN_595_CLOCK, OUTPUT);
  pinMode(PIN_595_LATCH, OUTPUT);
  escribir595(0x00);

  Serial.println(F(" Control de Iluminacion Inteligente v1.0"));
  Serial.println(F("Setpoint   : 500 lux (100%)"));
  Serial.println(F("Umbral ON  : < 390 lux (80% - 10)"));
  Serial.println(F("Umbral OFF : > 410 lux (80% + 10)"));
  Serial.println(F("Zona muerta: 390 - 410 lux"));
  Serial.println(F(""));

  ledsEncendidos = 0;
  actualizarLeds(ledsEncendidos);

  Serial.println(F("Sistema de Iluminacion inicializado."));
  Serial.println(F("------------------------------------"));

  // Servo Ascensor
  servoAscensor.attach(PIN_SERVO);
  servoAscensor.write(ANGULO_PLANTA[0]);
  anguloServo = ANGULO_PLANTA[0];

  // [MODIFICACIÓN v4.0] Inicializar velocidad del control difuso
  velocidadActual = 0.0f;

  dht.begin();
  IrReceiver.begin(PIN_IR);

  Serial.println(F(" Control de Temperatura Inteligente v1.0"));
  Serial.println(F("Ganancia proporcional : 2.0"));
  Serial.println(F("Ganancia integral     : 0.05"));
  Serial.println(F("Ganancia derivativa   : 1.0"));
  Serial.println(F("Banda muerta ±C      : 3.0 C"));
  Serial.println(F(""));

  float t = dht.readTemperature();
  if (!isnan(t)) {
    tempMedida = t;
  }

  Serial.println(F("Sistema de Temperatura inicializado."));
  Serial.println(F("------------------------------------"));

  // [MODIFICACIÓN v4.0] Informar sobre el control difuso
  Serial.println(F(" Control Difuso (Fuzzy) de Movimiento v4.0"));
  Serial.println(F("Perfil de velocidad: Arranque progresivo | Crucero estable | Frenado suave"));
  Serial.println(F("Metodo: Mamdani (min-max) + Defuzzificacion por media ponderada"));
  Serial.println(F("Entradas: distanciaRestante + velocidadActual"));
  Serial.println(F("Salida: deltaAngulo (incremento por ciclo)"));
  Serial.println(F("------------------------------------"));

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Sistema LISTO!");
  delay(1000);
  lcd.clear();

  Serial.println(F("Sistema ascensor v4.0 LISTO!"));
  Serial.println(F("------------------------------------"));  
}

// ============================================================
// --- Bucle Principal (loop) ---
// ============================================================
void loop() {
  unsigned long ahora = millis();

  manejarEstadoAscensor();

  if (ahora - tUltimaPulsaci >= 10) {
    tUltimaPulsaci = ahora;
    if (estadoAscensor != ASCENSOR_EMERGENCIA) {
      leerPulsadores();
      leerIR();
    }
  }

  if (ahora - tUltimaLectura >= 2000) {
    tUltimaLectura = ahora;
    leerSensores();
    presencia = digitalRead(PIN_PIR);
  }

  if (ahora - tUltimoProceso >= PID_INTERVALO) {
    tUltimoProceso = ahora;
    controlTemperatura();
  }

  if (ahora - tUltimoLCD >= 2000) {
    tUltimoLCD = ahora;
    actualizarLCD();
    paginaLCD = !paginaLCD;
    if (cabinaMov) {
      paginaLCD = 0;  
    }
  }

  controlIluminacion();

}

// ============================================================
// ---------- FUNCIONES AUXILIARES ----------
// ============================================================

void actualizarLCD() {
  lcd.clear();

  String linea;

  if (paginaLCD == 0) 
  {
    lcd.setCursor(0, 0);

    linea = cabinaMov ? "Moviendo " : "En planta ";
    linea += String(plantaActual);
    if (cabinaMov) {
      linea += "->";
      linea += String(plantaDestino);
    }
    lcd.print(padLCD(linea));

    lcd.setCursor(0, 1);

    linea = presencia ? "Cabina: OCUPADA" : "Cabina: LIBRE";
    lcd.print(padLCD(linea));
  } 
  else 
  {
    lcd.setCursor(0, 0);

    linea = "T:";
    linea += String(tempMedida, 1);
    linea += (char)223;
    linea += "C ";
    linea += "H:";
    linea += String((int)humMedida);
    linea += "%";

    lcd.print(padLCD(linea));

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

void actualizarLeds(uint8_t cantidad) {
  uint8_t mascara = 0;
  for (uint8_t i = 0; i < ledsEncendidos; i++) {
    mascara |= (1 << i);
  }
  escribir595(mascara); 
}

uint8_t contarBotonesPulsados() {
  uint8_t estadoActual = pcf8574_read();
  uint8_t count = 0;

  for (uint8_t i = 0; i < 5; i++) {
    if (!(estadoActual & (1 << i))) {
      count++;
    }
  }
  return count;
}

void controlIluminacion() {  
  if (luzLux < (LUZ_UMBRAL - LUZ_HISTERESIS)) 
  {
    float intervaloLux = (float)LUZ_UMBRAL / 8.0f;
    int ledsCalculados = 8 - (int)(luzLux / intervaloLux);
    ledsCalculados = constrain(ledsCalculados, 1, 8);

    if (ledsCalculados != ledsEncendidos) 
    {
      ledsEncendidos = ledsCalculados;
      actualizarLeds(ledsEncendidos);

      Serial.print(F("Luz=")); Serial.print(luzLux);
      Serial.print(F(" LEDs=")); Serial.println(ledsEncendidos);      
    }
  } else if (luzLux > (LUZ_UMBRAL + LUZ_HISTERESIS)) 
  {
    if (ledsEncendidos > 0) 
    {
      ledsEncendidos = 0;
      actualizarLeds(ledsEncendidos);  
    }
  }
}

void controlTemperatura() {
    unsigned long ahora = millis();
    float dt = (ahora - tUltimoPID) / 1000.0f;
    if (dt <= 0.0f) dt = 0.001f;
    tUltimoPID = ahora;

    float error = TEMP_SETPOINT - tempMedida;

    if (fabs(error) < TEMP_ZONA_M) {
        pidIntegral = pidIntegral;

        AccionTemp accionAnterior = accionTemp;
        accionTemp = REPOSO_TEMP;
        digitalWrite(PIN_EV_FRIO, LOW);
        digitalWrite(PIN_EV_CALOR, LOW);

        if (accionTemp != accionAnterior) {
            Serial.print(F("Temp="));
            Serial.print(tempMedida, 1);
            Serial.print(F("C (zona muerta) Accion=REPOSO\n"));
        }

        return;
    }

    pidIntegral += error * dt;
    pidIntegral = constrain(pidIntegral, -PID_INT_MAX, PID_INT_MAX);

    float derivada = (error - pidErrorPrev) / dt;
    pidErrorPrev = error;

    float salida = PID_KP * error + PID_KI * pidIntegral + PID_KD * derivada;

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
        Serial.print(F("C u="));
        Serial.print(salida, 2);
        Serial.print(F(" Accion="));
        Serial.println(
            accionTemp == ENFRIAR ? F("ENFRIAR") :
            accionTemp == CALENTAR ? F("CALENTAR") :
                                     F("REPOSO")
        );
    }
}

void escribir595(uint8_t valor) {
  digitalWrite(PIN_595_LATCH, LOW);
  shiftOut(PIN_595_DATA, PIN_595_CLOCK, MSBFIRST, valor);
  digitalWrite(PIN_595_LATCH, HIGH);
}

void leerIR() {
  if (!IrReceiver.decode()) {
    return;
  }

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
    if (codigo == IR_CODIGO[i]) {
      agregarSolicitud(i + 1);
      Serial.print(F("IR -> P")); Serial.println(i + 1);
      break;
    }
  }
  IrReceiver.resume();
}

float leerLux() {
  int analogValue = analogRead(PIN_LDR);

  if (analogValue < 0) return 0;

  float voltage = analogValue / 1024.0 * 5.0;
  float resistance = 2000 * voltage / (1 - voltage / 5.0);
  float lux = pow(RL10 * 1e3 * pow(10, GAMMA) / resistance, (1 / GAMMA));

  return lux;
}

void leerPulsadores() {
  uint8_t estadoActual = pcf8574_read();

  if (estadoActual != ultimaLectura) {
    ultimoCambio = millis();
  }
  ultimaLectura = estadoActual;

  if ((millis() - ultimoCambio) > DEBOUNCE_MS) {
    uint8_t cambios = (estadoActual ^ estadoAnterior) & BOTONES_MASK;

    if (cambios != 0) {
      for (uint8_t i = 0; i < 5; i++) {
        if (cambios & (1 << i)) {
          bool presionado = !(estadoActual & (1 << i));

          if (presionado) {
            agregarSolicitud(i + 1);
            Serial.print(F("[BOTON_P"));
            Serial.print(i+1);
            Serial.println(F("] PRESIONADO"));
          }          
        }
      }
      estadoAnterior = estadoActual;
    }
  }

}

void leerSensores() {
  float t = dht.readTemperature();
  float h = dht.readHumidity();
  float l = leerLux();

  bool cambio = false;

  if (!isnan(t) && t != tempMedida) {
    tempMedida  = t;
    cambio = true;
  }
  if (!isnan(h) && h != humMedida) {
    humMedida = h;
    cambio = true;
  }

  if (l != luzLux) {
    luzLux = l;
    cambio = true;
  }

  if (cambio) {
    Serial.print(F("T="));    Serial.print(tempMedida);
    Serial.print(F("C H=")); Serial.print(humMedida);
    Serial.print(F("% Luz=")); Serial.print(luzLux);
    Serial.println(F("lux"));
  }

}

void agregarSolicitud(uint8_t planta) {
  if (planta < 1 || planta > MAX_PLANTAS) return;
  uint8_t idx = planta - 1;

  if (!solicitudes[idx]) {
    solicitudes[idx] = true;
    tiempoSolicitud[idx] = millis();
    numSolicitudes++;
    Serial.print(F("[COLA] Nueva solicitud P"));
    Serial.print(planta);
    Serial.print(F(". Total pendientes: "));
    Serial.println(numSolicitudes);
  }
  contadorSolicitudes[idx]++;
}

void eliminarSolicitud(uint8_t planta) {
  if (planta < 1 || planta > MAX_PLANTAS) return;
  uint8_t idx = planta - 1;

  if (solicitudes[idx]) {
    solicitudes[idx] = false;
    contadorSolicitudes[idx] = 0;
    tiempoSolicitud[idx] = 0;
    if (numSolicitudes > 0) numSolicitudes--;
    Serial.print(F("[COLA] Solicitud P"));
    Serial.print(planta);
    Serial.print(F(" atendida. Restantes: "));
    Serial.println(numSolicitudes);
  }
}

uint8_t calcularPrioridad() {
  if (numSolicitudes == 0) return 0;

  uint8_t mejorPlanta = 0;
  int8_t  mejorDistancia = 127;
  uint8_t mejorContador = 0;
  unsigned long mejorTiempo = 0xFFFFFFFF;

  for (uint8_t i = 0; i < MAX_PLANTAS; i++) {
    if (!solicitudes[i]) continue;
    uint8_t p = i + 1;
    int8_t dist = 0;
    bool enDir = false;

    if (direccionActual == DIR_SUBIENDO || direccionActual == DIR_NINGUNA) {
      if (p > plantaActual) {
        dist = p - plantaActual;
        enDir = true;
      }
    }
    if (!enDir && (direccionActual == DIR_BAJANDO || direccionActual == DIR_NINGUNA)) {
      if (p < plantaActual) {
        dist = plantaActual - p;
        enDir = true;
      }
    }
    if (!enDir) continue;

    uint8_t cnt = contadorSolicitudes[i];
    unsigned long tSol = tiempoSolicitud[i];
    bool mejor = false;

    if (dist < mejorDistancia) {
      mejor = true;
    } else if (dist == mejorDistancia && cnt > mejorContador) {
      mejor = true;
    } else if (dist == mejorDistancia && cnt == mejorContador && tSol < mejorTiempo) {
      mejor = true;
    }

    if (mejor) {
      mejorPlanta = p;
      mejorDistancia = dist;
      mejorContador = cnt;
      mejorTiempo = tSol;
    }
  }

  if (mejorPlanta == 0) {
    for (uint8_t i = 0; i < MAX_PLANTAS; i++) {
      if (!solicitudes[i]) continue;
      uint8_t p = i + 1;
      int8_t dist = 0;
      bool enDir = false;

      if (direccionActual == DIR_SUBIENDO && p < plantaActual) {
        dist = plantaActual - p;
        enDir = true;
      } else if (direccionActual == DIR_BAJANDO && p > plantaActual) {
        dist = p - plantaActual;
        enDir = true;
      }
      if (!enDir) continue;

      uint8_t cnt = contadorSolicitudes[i];
      unsigned long tSol = tiempoSolicitud[i];
      bool mejor = false;

      if (dist < mejorDistancia) {
        mejor = true;
      } else if (dist == mejorDistancia && cnt > mejorContador) {
        mejor = true;
      } else if (dist == mejorDistancia && cnt == mejorContador && tSol < mejorTiempo) {
        mejor = true;
      }

      if (mejor) {
        mejorPlanta = p;
        mejorDistancia = dist;
        mejorContador = cnt;
        mejorTiempo = tSol;
      }
    }

    if (mejorPlanta != 0) {
      if (direccionActual == DIR_SUBIENDO) direccionActual = DIR_BAJANDO;
      else if (direccionActual == DIR_BAJANDO) direccionActual = DIR_SUBIENDO;
    }
  }

  if (mejorPlanta == 0) {
    for (uint8_t i = 0; i < MAX_PLANTAS; i++) {
      if (solicitudes[i] && (i + 1) == plantaActual) {
        mejorPlanta = plantaActual;
        break;
      }
    }
  }

  return mejorPlanta;
}

// ============================================================
// [MODIFICACIÓN v4.0] FUNCIONES DEL CONTROLADOR DIFUSO (FUZZY)
// ============================================================
// El controlador difuso implementa un sistema Mamdani con 
// defuzzificación por media ponderada de singletones. Genera
// un perfil de velocidad trapezoidal suave para el ascensor.
// ============================================================

// ============================================================
// fuzzificarTrapezoidal()
// -----------------------
// Calcula el grado de pertenencia de un valor 'x' a un conjunto
// difuso trapezoidal definido por los puntos (a,b,c,d).
//
// Forma del trapecio:
//      a     b       c     d
//      |_____|_______|_____|
/*           /         \
*           /           \
*     0 ___/             \___ 0
*
*/
// @param x  Valor a fuzzificar
// @param a  Inicio del soporte (pertenencia = 0)
// @param b  Inicio del núcleo (pertenencia = 1)
// @param c  Fin del núcleo (pertenencia = 1)
// @param d  Fin del soporte (pertenencia = 0)
// @return  Grado de pertenencia [0.0, 1.0]
// ============================================================
float fuzzificarTrapezoidal(float x, float a, float b, float c, float d) {
  if (x <= a || x >= d) return 0.0f;
  if (x >= b && x <= c) return 1.0f;
  if (x > a && x < b) return (x - a) / (b - a);
  return (d - x) / (d - c);
}

// ============================================================
// fuzzificarTriangular()
// ----------------------
// Calcula el grado de pertenencia de un valor 'x' a un conjunto
// difuso triangular definido por los puntos (a,b,c).
//
// Forma del triángulo:
/*           b (pico = 1)
           /\
          /  \
    0 ___/    \___ 0
      a         c

*/
// @param x  Valor a fuzzificar
// @param a  Inicio del soporte (pertenencia = 0)
// @param b  Vértice/pico (pertenencia = 1)
// @param c  Fin del soporte (pertenencia = 0)
// @return  Grado de pertenencia [0.0, 1.0]
// ============================================================
float fuzzificarTriangular(float x, float a, float b, float c) {
  if (x <= a || x >= c) return 0.0f;
  if (x == b) return 1.0f;
  if (x < b) return (x - a) / (b - a);
  return (c - x) / (c - b);
}

// ============================================================
// controlFuzzy()
// --------------
// Controlador difuso principal. Recibe la distancia restante
// al destino y la velocidad actual, y devuelve el incremento
// de ángulo óptimo para este ciclo.
//
// Método de inferencia:
//   1. Fuzzificación: calcula grados de pertenencia de las
//      entradas a los conjuntos difusos definidos.
//   2. Evaluación de reglas: aplica operador AND (min) para
//      obtener el peso de activación de cada regla.
//   3. Defuzzificación: media ponderada de los singletones
//      de salida usando los pesos de activación.
//
// Reglas implementadas (9 reglas):
//   distancia=CERCA  & vel=LENTA   → MUY_LENTO (1°)
//   distancia=CERCA  & vel=MEDIA  → MUY_LENTO (1°)
//   distancia=CERCA  & vel=RÁPIDA → LENTO     (2°)
//   distancia=MEDIA  & vel=LENTA   → MEDIO     (3°)
//   distancia=MEDIA  & vel=MEDIA   → RÁPIDO    (5°)
//   distancia=MEDIA  & vel=RÁPIDA → LENTO     (2°)
//   distancia=LEJOS  & vel=LENTA   → RÁPIDO    (5°)
//   distancia=LEJOS  & vel=MEDIA   → RÁPIDO    (5°)
//   distancia=LEJOS  & vel=RÁPIDA → MUY_RÁPIDO(6°)
//
// @param distancia  Distancia angular restante al destino (grados)
// @param velocidad  Velocidad actual del ascensor (grados/ciclo)
// @return  Incremento de ángulo recomendado (grados)
// ============================================================
float controlFuzzy(float distancia, float velocidad) {
  // --- FASE 1: Fuzzificación de las entradas ---

  // Grados de pertenencia de distanciaRestante
  float muDistCerca  = fuzzificarTrapezoidal(distancia, 
    FUZZY_DIST_CERCA_A, FUZZY_DIST_CERCA_B, 
    FUZZY_DIST_CERCA_C, FUZZY_DIST_CERCA_D);
  float muDistMedia  = fuzzificarTrapezoidal(distancia, 
    FUZZY_DIST_MEDIA_A, FUZZY_DIST_MEDIA_B, 
    FUZZY_DIST_MEDIA_C, FUZZY_DIST_MEDIA_D);
  float muDistLejos  = fuzzificarTrapezoidal(distancia, 
    FUZZY_DIST_LEJOS_A, FUZZY_DIST_LEJOS_B, 
    FUZZY_DIST_LEJOS_C, FUZZY_DIST_LEJOS_D);

  // Grados de pertenencia de velocidadActual
  float muVelLenta   = fuzzificarTrapezoidal(velocidad, 
    FUZZY_VEL_LENTA_A, FUZZY_VEL_LENTA_B, 
    FUZZY_VEL_LENTA_C, FUZZY_VEL_LENTA_D);
  float muVelMedia   = fuzzificarTriangular(velocidad, 
    FUZZY_VEL_MEDIA_A, FUZZY_VEL_MEDIA_B, FUZZY_VEL_MEDIA_C);
  float muVelRapida  = fuzzificarTrapezoidal(velocidad, 
    FUZZY_VEL_RAPIDA_A, FUZZY_VEL_RAPIDA_B, 
    FUZZY_VEL_RAPIDA_C, FUZZY_VEL_RAPIDA_D);

  // --- FASE 2: Evaluación de reglas (Mamdani, AND = min) ---
  // Cada regla produce un peso de activación w_i y un singleton s_i

  float w[9];    // Pesos de activación
  float s[9];    // Singletones de salida
  uint8_t numReglas = 0;

  // Regla 1: CERCA + LENTA → MUY_LENTO (precisión de parada)
  w[numReglas] = min(muDistCerca, muVelLenta);
  s[numReglas] = FUZZY_OUT_MUY_LENTO;
  numReglas++;

  // Regla 2: CERCA + MEDIA → MUY_LENTO (frenado final)
  w[numReglas] = min(muDistCerca, muVelMedia);
  s[numReglas] = FUZZY_OUT_MUY_LENTO;
  numReglas++;

  // Regla 3: CERCA + RÁPIDA → LENTO (frenado suave)
  w[numReglas] = min(muDistCerca, muVelRapida);
  s[numReglas] = FUZZY_OUT_LENTO;
  numReglas++;

  // Regla 4: MEDIA + LENTA → MEDIO (aceleración)
  w[numReglas] = min(muDistMedia, muVelLenta);
  s[numReglas] = FUZZY_OUT_MEDIO;
  numReglas++;

  // Regla 5: MEDIA + MEDIA → RÁPIDO (crucero)
  w[numReglas] = min(muDistMedia, muVelMedia);
  s[numReglas] = FUZZY_OUT_RAPIDO;
  numReglas++;

  // Regla 6: MEDIA + RÁPIDA → LENTO (frenado anticipado)
  w[numReglas] = min(muDistMedia, muVelRapida);
  s[numReglas] = FUZZY_OUT_LENTO;
  numReglas++;

  // Regla 7: LEJOS + LENTA → RÁPIDO (arranque progresivo)
  w[numReglas] = min(muDistLejos, muVelLenta);
  s[numReglas] = FUZZY_OUT_RAPIDO;
  numReglas++;

  // Regla 8: LEJOS + MEDIA → RÁPIDO (mantener crucero)
  w[numReglas] = min(muDistLejos, muVelMedia);
  s[numReglas] = FUZZY_OUT_RAPIDO;
  numReglas++;

  // Regla 9: LEJOS + RÁPIDA → MUY_RÁPIDO (máxima velocidad)
  w[numReglas] = min(muDistLejos, muVelRapida);
  s[numReglas] = FUZZY_OUT_MUY_RAPIDO;
  numReglas++;

  // --- FASE 3: Defuzzificación por media ponderada ---
  // delta = Σ(w_i * s_i) / Σ(w_i)
  // Si no hay activación, devolver velocidad mínima

  float numerador = 0.0f;
  float denominador = 0.0f;

  for (uint8_t i = 0; i < numReglas; i++) {
    numerador   += w[i] * s[i];
    denominador += w[i];
  }

  float delta;
  if (denominador > 0.001f) {
    delta = numerador / denominador;
  } else {
    delta = FUZZY_OUT_MUY_LENTO;
  }

  // Protección: limitar a velocidad máxima permitida
  if (delta > FUZZY_VEL_MAX) {
    delta = FUZZY_VEL_MAX;
  }

  return delta;
}

// ============================================================
// MÁQUINA DE ESTADOS DEL ASCENSOR — v4.0
// ============================================================

void manejarEstadoAscensor() {
  unsigned long ahora = millis();

  switch (estadoAscensor) {

    case ASCENSOR_REPOSO:
      if (numSolicitudes > 0) {
        transicionEstado(ASCENSOR_PUERTA_CERRADA);
      }
      break;

    case ASCENSOR_PUERTA_CERRADA:
      if (tPuertaCerrada == 0) {
        tPuertaCerrada = ahora;
      }

      if (ahora - tPuertaCerrada >= TIEMPO_CIERRE_PUERTA) {
        tPuertaCerrada = 0;

        uint8_t prio = calcularPrioridad();

        if (prio != 0 && prio != plantaActual) {
          plantaDestino = prio;
          transicionEstado(ASCENSOR_MOVIMIENTO);
        } else if (prio == plantaActual) {
          eliminarSolicitud(plantaActual);
          transicionEstado(ASCENSOR_PUERTA_ABIERTA);
        } else {
          transicionEstado(ASCENSOR_REPOSO);
        }
      }
      break;

    case ASCENSOR_MOVIMIENTO:
      if (numSolicitudes > 0) {
        uint8_t nuevaPrioridad = calcularPrioridad();
        if (nuevaPrioridad != 0 && nuevaPrioridad != plantaDestino) {
          plantaDestino = nuevaPrioridad;
          Serial.print(F("[SCAN] Redirigiendo hacia P"));
          Serial.println(plantaDestino);
        }
      }

      // [MODIFICACIÓN v4.0] Usar intervalo SERVO_INTERVALO en lugar de 65 fijo
      if (ahora - tUltimoServo >= SERVO_INTERVALO) {
        tUltimoServo = ahora;
        moverAscensor();
      }

      if (plantaActual == plantaDestino && !cabinaMov) {
        eliminarSolicitud(plantaActual);
        transicionEstado(ASCENSOR_PUERTA_ABIERTA);
      }

      if (presencia && contarBotonesPulsados() > 1) {
        transicionEstado(ASCENSOR_EMERGENCIA);
      }

      break;

    case ASCENSOR_PUERTA_ABIERTA:
      if (tPuertaAbierta == 0) {
        tPuertaAbierta = ahora;
      }

      if (ahora - tPuertaAbierta >= 3000) {
        tPuertaAbierta = 0;
        transicionEstado(ASCENSOR_PUERTA_CERRADA);
      }

      break;

    case ASCENSOR_EMERGENCIA:
      cabinaMov = false;

      if (millis() % 1000 < 100) {
        Serial.println(F("EMERGENCIA! Ascensor detenido."));
      }

      if (tEmergencia == 0) {
        tEmergencia = ahora;
      }

      if (ahora - tEmergencia >= 10000) {
        tEmergencia = 0;
        Serial.println(F("Reset automatico de emergencia."));
        transicionEstado(ASCENSOR_REPOSO);
      }

      break;

    case ASCENSOR_MANTENIMIENTO:
      if (millis() % 2000 < 100) {
        Serial.println(F("MODO MANTENIMIENTO - Debug activo"));
      }

      break;

    default:
      Serial.println(F("Error: Estado desconocido, reseteando a REPOSO"));
      transicionEstado(ASCENSOR_REPOSO);
      break;
  }
}

// ============================================================
// moverAscensor()
// ---------------
// [MODIFICACIÓN v4.0] Control de movimiento mediante lógica
// difusa (fuzzy logic). 
//
// En cada ciclo (SERVO_INTERVALO ms) se calcula:
//   1. La distancia angular restante al destino.
//   2. Se consulta el controlador difuso para obtener el
//      incremento óptimo deltaAngulo.
//   3. Se actualiza el ángulo del servo y se registra la
//      velocidad actual para el siguiente ciclo.
//
// El perfil resultante es:
//   · Arranque progresivo: desde 0°/ciclo hasta ~5-6°/ciclo
//   · Crucero estable: mantenimiento de velocidad máxima
//   · Frenado suave: reducción progresiva al acercarse
//   · Precisión final: corrección fina en últimos 3°
//
// La detección de planta intermedia usa rangos angulares
// en lugar de valores exactos, para mayor robustez con
// velocidad variable.
// ============================================================
void moverAscensor() {
  if (plantaDestino == plantaActual) {
    cabinaMov = false;
    velocidadActual = 0.0f;
    return;
  }

  cabinaMov = true;
  int destino = ANGULO_PLANTA[plantaDestino - 1];

  // Determinar dirección de marcha para el algoritmo SCAN
  if (plantaDestino > plantaActual) {
    direccionActual = DIR_SUBIENDO;
  } else if (plantaDestino < plantaActual) {
    direccionActual = DIR_BAJANDO;
  }

  // Calcular distancia restante (siempre positiva)
  float distanciaRestante = abs(destino - anguloServo);

  // [MODIFICACIÓN v4.0] Control difuso: obtener incremento óptimo
  float deltaAngulo = controlFuzzy(distanciaRestante, velocidadActual);

  // Corrección de precisión: si estamos muy cerca, forzar velocidad mínima
  // para evitar overshoot y garantizar parada exacta
  if (distanciaRestante < FUZZY_DIST_MINIMA) {
    deltaAngulo = FUZZY_OUT_MUY_LENTO;  // 1 grado por ciclo
  }

  // Aplicar incremento en la dirección correcta
  if (anguloServo < destino) {
    anguloServo += (int)deltaAngulo;
    if (anguloServo >= destino) anguloServo = destino;
  } else {
    anguloServo -= (int)deltaAngulo;
    if (anguloServo <= destino) anguloServo = destino;
  }

  // Actualizar velocidad actual para el siguiente ciclo (realimentación)
  velocidadActual = deltaAngulo;

  servoAscensor.write(anguloServo);

  if (anguloServo == destino) {
    plantaActual = plantaDestino;
    cabinaMov = false;
    velocidadActual = 0.0f;
    Serial.print(F("Llegada P")); Serial.println(plantaActual);
  }
  else {
    // [MODIFICACIÓN v4.0] Detección de planta por rangos (más robusto)
    // Al usar velocidad variable, el ángulo puede no coincidir exactamente
    // con los valores discretos, por lo que usamos rangos ±10°
    if (anguloServo >= 170) {
      plantaActual = 5;
    } else if (anguloServo >= 125 && anguloServo < 170) {
      plantaActual = 4;
    } else if (anguloServo >= 80 && anguloServo < 125) {
      plantaActual = 3;
    } else if (anguloServo >= 35 && anguloServo < 80) {
      plantaActual = 2;
    } else if (anguloServo < 35) {
      plantaActual = 1;
    }
  }
}

void transicionEstado(EstadoAscensor nuevoEstado) {
  switch (estadoAscensor) {
    case ASCENSOR_MOVIMIENTO:
      cabinaMov = false;
      break;

    case ASCENSOR_EMERGENCIA:
      tEmergencia = 0;
      Serial.println(F("Saliendo de modo emergencia"));
      break;

    case ASCENSOR_PUERTA_ABIERTA:
      tPuertaAbierta = 0;
      break;

    case ASCENSOR_PUERTA_CERRADA:
      tPuertaCerrada = 0;
      break;

    default:
      break;
  }

  EstadoAscensor estadoPrevio = estadoAscensor;
  estadoAscensor = nuevoEstado;

  switch (nuevoEstado) {
    case ASCENSOR_REPOSO:
      Serial.print(F("Estado: REPOSO en planta "));
      Serial.println(plantaActual);
      direccionActual = DIR_NINGUNA;
      velocidadActual = 0.0f;
      break;

    case ASCENSOR_PUERTA_CERRADA:
      Serial.print(F("Estado: PUERTA_CERRADA en P"));
      Serial.println(plantaActual);
      break;

    case ASCENSOR_MOVIMIENTO:
      Serial.print(F("Estado: MOVIMIENTO P"));
      Serial.print(plantaActual);
      Serial.print(F(" -> P"));
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
      Serial.println(F("ESTADO: EMERGENCIA! Ascensor detenido por seguridad"));
      break;

    case ASCENSOR_MANTENIMIENTO:
      Serial.println(F("Estado: MANTENIMIENTO activado"));
      break;
  }

  if (estadoPrevio != nuevoEstado) {
    Serial.print(F("Transicion: "));
    Serial.print(nombreEstado(estadoPrevio));
    Serial.print(F(" -> "));
    Serial.println(nombreEstado(nuevoEstado));
  }
}

const char* nombreEstado(EstadoAscensor e) {
  switch (e) {
    case ASCENSOR_REPOSO:        return "REPOSO";
    case ASCENSOR_PUERTA_CERRADA:return "PUERTA_CERRADA";
    case ASCENSOR_MOVIMIENTO:    return "MOVIMIENTO";
    case ASCENSOR_EMERGENCIA:    return "EMERGENCIA";
    case ASCENSOR_PUERTA_ABIERTA: return "PUERTA_ABIERTA";
    case ASCENSOR_MANTENIMIENTO: return "MANTENIMIENTO";
    default:                     return "DESCONOCIDO";
  }
}

String padLCD(String texto) {
  if (texto.length() > 16) return texto.substring(0, 16);
  while (texto.length() < 16) texto += ' ';
  return texto;
}

uint8_t pcf8574_read() {
  uint8_t valor = 0xFF;
  Wire.requestFrom(PCF8574_ADDR, (uint8_t)1);
  if (Wire.available()) {
    valor = Wire.read();
  }
  return valor;
}

void pcf8574_write(uint8_t valor) {
  Wire.beginTransmission(PCF8574_ADDR);
  Wire.write(valor);
  Wire.endTransmission();
}
