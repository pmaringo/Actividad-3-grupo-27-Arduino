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
 * MODIFICACIÓN v4.2 - Control Difuso (Fuzzy Logic) de Movimiento
 *                     (CORRECCIÓN de redirección SCAN + clamping servo)
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
 *    PUERTA_CERRADA → Transición de seguridad antes de iniciar movimiento.
 *    MOVIMIENTO     → Cabina desplazándose con control difuso (fuzzy).
 *    PUERTA_ABIERTA → Llegada al destino, 3 s de espera.
 *    EMERGENCIA     → Parada de seguridad (PIR + >1 botón en movimiento).
 *    MANTENIMIENTO  → Estado especial para diagnóstico/debug.
 *
 * ============================================================
 *  SISTEMA DE COLA MÚLTIPLE Y ALGORITMO DE PRIORIDAD SCAN v4.2
 * ============================================================
 *  [CORRECCIÓN v4.2] El algoritmo SCAN ahora opera con coherencia
 *  física: durante el movimiento, solo se aceptan redirecciones que
 *  no requieran invertir la marcha bruscamente. Una vez que la cabina
 *  ha pasado una planta, esa solicitud queda pendiente para la
 *  siguiente ronda (comportamiento de ascensor real).
 *
 *  Reglas de redirección durante MOVIMIENTO:
 *    · Si subiendo: solo se aceptan plantas SUPERIORES a la actual.
 *    · Si bajando: solo se aceptan plantas INFERIORES a la actual.
 *    · Si la nueva planta está en dirección opuesta: se ignora
 *      temporalmente (queda en cola para después).
 *
 * ============================================================
 *  CONTROL DIFUSO (FUZZY LOGIC) DE MOVIMIENTO — v4.2
 * ============================================================
 *  Se sustituye el control de velocidad fija (1°/ciclo) por un
 *  sistema de control difuso que genera un perfil de velocidad
 *  trapezoidal suave, imitando el comportamiento de un ascensor real.
 *
 *  Variables de entrada:
 *    · distanciaRestante → distancia angular al piso destino (0–180°)
 *    · velocidadActual   → velocidad del ciclo anterior (0–8°/ciclo)
 *
 *  Variable de salida:
 *    · deltaAngulo       → incremento de ángulo para este ciclo (0–6°)
 *
 *  Método: Mamdani (min-max) + defuzzificación por media ponderada.
 *
 * ============================================================
 *  CORRECCIONES v4.2 (Frente a v4.1)
 * ============================================================
 *  [BUG 1] Bucle infinito SCAN al redirigir durante movimiento:
 *    Causa: plantaDestino cambiaba arbitrariamente en cada ciclo,
 *    pero el servo seguía su inercia. El fuzzy recalculaba desde
 *    la posición actual generando oscilaciones.
 *    SOLUCIÓN: Implementado sistema de "commit" de destino. Durante
 *    el movimiento, solo se aceptan redirecciones en la misma
 *    dirección de marcha y que no hayan sido superadas. Las
 *    solicitudes en dirección opuesta quedan en cola para la
 *    siguiente ronda.
 *
 *  [BUG 2] Servo gira más allá de 180° o en círculos:
 *    Causa: falta de clamping del ángulo del servo cuando el
 *    destino cambiaba bruscamente mientras el servo avanzaba.
 *    SOLUCIÓN: Añadido clamping estricto de anguloServo a [0, 180].
 *    Además, la dirección de movimiento se recalcula explícitamente
 *    en cada ciclo comparando anguloServo contra destino real.
 *
 *  [MEJORA] Limpieza de velocidad al cambiar destino:
 *    Cuando se produce una redirección válida, la velocidadActual
 *    se resetea parcialmente para permitir un nuevo perfil fuzzy
 *    desde la posición actual hacia el nuevo destino.
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
#define PIN_EV_FRIO     10
#define PIN_EV_CALOR    12
#define PIN_595_DATA    6
#define PIN_595_CLOCK   5
#define PIN_595_LATCH   4

// Direccion I2C del PCF8574
const uint8_t PCF8574_ADDR = 0x20;

// Pines del PCF8574
const uint8_t BOTON_P0 = 0;
const uint8_t BOTON_P1 = 1;
const uint8_t BOTON_P2 = 2;
const uint8_t BOTON_P3 = 3;
const uint8_t BOTON_P4 = 4;

const uint8_t BOTONES_MASK = 0x1F;

// Anti-rebote
uint8_t estadoAnterior = 0xFF;
uint8_t ultimaLectura = 0xFF;
unsigned long ultimoCambio = 0;
const unsigned long DEBOUNCE_MS = 25;

// Temperatura
#define TEMP_SETPOINT   25.0f
#define TEMP_ZONA_M      3.0f
#define HUM_SETPOINT    60.0f

// PID
#define PID_KP          2.0f
#define PID_KI          0.05f
#define PID_KD          1.0f
#define PID_DEADBAND    1.0f
#define PID_INT_MAX     50.0f
#define PID_INTERVALO   500

// Iluminación
#define GAMMA           0.7f
#define RL10           50.0f
#define LUZ_SETPOINT    500
#define LUZ_UMBRAL      400
#define LUZ_HISTERESIS  10

// --- CONSTANTES DEL CONTROL DIFUSO (FUZZY) ---
#define SERVO_INTERVALO   65

// Conjuntos difusos de ENTRADA 1: distanciaRestante (grados)
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

// Singletones de SALIDA
#define FUZZY_OUT_MUY_LENTO  1.0f
#define FUZZY_OUT_LENTO      2.0f
#define FUZZY_OUT_MEDIO      3.0f
#define FUZZY_OUT_RAPIDO     5.0f
#define FUZZY_OUT_MUY_RAPIDO 6.0f

#define FUZZY_VEL_MAX        6.0f
#define FUZZY_DIST_MINIMA    2.0f

// Límites físicos del servo SG90
#define SERVO_MIN            0
#define SERVO_MAX          180

// Ángulos del servo para cada planta
const int ANGULO_PLANTA[5] = {0, 45, 90, 135, 180};

// Códigos IR NEC
const uint32_t IR_CODIGO[5] = {
  0xCF30FF00, 0xE718FF00, 0x857AFF00, 0xEF10FF00, 0xC738FF00
};

// ============================================================
// SISTEMA DE COLA DE SOLICITUDES
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

float pidIntegral  = 0.0f;
float pidErrorPrev = 0.0f;
unsigned long tUltimoPID = 0;

// Variables del control difuso
float velocidadActual = 0.0f;

// [CORRECCIÓN v4.2] Guardar planta de origen del movimiento actual
// para determinar qué plantas ya han sido "superadas" físicamente
uint8_t plantaOrigenMovimiento = 1;

// --- MÁQUINA DE ESTADOS ---
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

float fuzzificarTrapezoidal(float x, float a, float b, float c, float d);
float fuzzificarTriangular(float x, float a, float b, float c);
float controlFuzzy(float distancia, float velocidad);
uint8_t determinarPlantaDesdeAngulo(int angulo);

// [CORRECCIÓN v4.2] Nuevo prototipo para validación de redirección
bool esRedireccionValida(uint8_t nuevaPlanta);

// ============================================================
// determinarPlantaDesdeAngulo()
// ------------------------------------------------------------
// Calcula la planta lógica mediante redondeo al múltiplo de 45°
// más cercano con zona muerta de ±22°.
// ============================================================
uint8_t determinarPlantaDesdeAngulo(int angulo) {
  int idx = (angulo + 22) / 45;
  if (idx < 0) idx = 0;
  if (idx > 4) idx = 4;
  return (uint8_t)(idx + 1);
}

// ============================================================
// [CORRECCIÓN v4.2] esRedireccionValida()
// ------------------------------------------------------------
// Valida si una nueva solicitud de planta puede aceptarse como
// * redirección durante el movimiento actual.
//
// Reglas de coherencia física (ascensor real):
//   · Si subiendo: solo plantas Estrictamente SUPERIORES a la
//     planta de origen del movimiento actual.
//   · Si bajando: solo plantas Estrictamente INFERIORES a la
//     planta de origen del movimiento actual.
//   · La planta actual de paso (determinada por ángulo) no
//     influye: una vez iniciado el movimiento, no se puede
//     "volver atrás" sin completar la ronda.
//
// @param nuevaPlanta  Planta solicitada (1–5)
// @return  true si la redirección es físicamente válida
// ============================================================
bool esRedireccionValida(uint8_t nuevaPlanta) {
  if (nuevaPlanta < 1 || nuevaPlanta > MAX_PLANTAS) return false;
  
  // Si estamos parados, cualquier planta es válida
  if (!cabinaMov) return true;
  
  // Si la nueva planta es la misma que el destino actual, no es redirección
  if (nuevaPlanta == plantaDestino) return false;
  
  // Validar según dirección de marcha del movimiento ACTUAL
  if (direccionActual == DIR_SUBIENDO) {
    // Subiendo: solo aceptar plantas superiores al origen del movimiento
    // (no se puede bajar sin completar la subida actual)
    return (nuevaPlanta > plantaOrigenMovimiento);
  } else if (direccionActual == DIR_BAJANDO) {
    // Bajando: solo aceptar plantas inferiores al origen del movimiento
    return (nuevaPlanta < plantaOrigenMovimiento);
  }
  
  return true; // DIR_NINGUNA (no debería ocurrir en movimiento)
}

// ============================================================
// --- Configuración Inicial (setup) ---
// ============================================================
void setup() {
  Serial.begin(9600);

  Serial.println(F("UNIR Actividad 2 - Ascensor ACME v4.2"));
  Serial.println(F("[MOD v4.2] Fuzzy Logic - SCAN corregido + Clamping servo"));
  Serial.println(F(""));

  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("ACME ASCENSOR v4.2");
  lcd.setCursor(0, 1);
  lcd.print("SCAN Stable");
  delay(1500);

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

  pinMode(PIN_PIR, INPUT);
  pinMode(PIN_EV_FRIO, OUTPUT);
  pinMode(PIN_EV_CALOR, OUTPUT);
  digitalWrite(PIN_EV_FRIO, LOW);
  digitalWrite(PIN_EV_CALOR, LOW);

  pinMode(PIN_595_DATA,  OUTPUT);
  pinMode(PIN_595_CLOCK, OUTPUT);
  pinMode(PIN_595_LATCH, OUTPUT);
  escribir595(0x00);

  ledsEncendidos = 0;
  actualizarLeds(ledsEncendidos);

  servoAscensor.attach(PIN_SERVO);
  servoAscensor.write(ANGULO_PLANTA[0]);
  anguloServo = ANGULO_PLANTA[0];
  velocidadActual = 0.0f;

  dht.begin();
  IrReceiver.begin(PIN_IR);

  float t = dht.readTemperature();
  if (!isnan(t)) tempMedida = t;

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Sistema LISTO!");
  delay(1000);
  lcd.clear();

  Serial.println(F("Sistema ascensor v4.2 LISTO!"));
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
    if (cabinaMov) paginaLCD = 0;
  }

  controlIluminacion();
}

// ============================================================
// ---------- FUNCIONES AUXILIARES ----------
// ============================================================

void actualizarLCD() {
  lcd.clear();
  String linea;

  if (paginaLCD == 0) {
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
  } else {
    lcd.setCursor(0, 0);
    linea = "T:";
    linea += String(tempMedida, 1);
    linea += (char)223;
    linea += "C H:";
    linea += String((int)humMedida);
    linea += "%";
    lcd.print(padLCD(linea));

    lcd.setCursor(0, 1);
    if (luzLux > 999) linea = "L.DIURNA ";
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
    if (!(estadoActual & (1 << i))) count++;
  }
  return count;
}

void controlIluminacion() {  
  if (luzLux < (LUZ_UMBRAL - LUZ_HISTERESIS)) {
    float intervaloLux = (float)LUZ_UMBRAL / 8.0f;
    int ledsCalculados = 8 - (int)(luzLux / intervaloLux);
    ledsCalculados = constrain(ledsCalculados, 1, 8);
    if (ledsCalculados != ledsEncendidos) {
      ledsEncendidos = ledsCalculados;
      actualizarLeds(ledsEncendidos);
      Serial.print(F("Luz=")); Serial.print(luzLux);
      Serial.print(F(" LEDs=")); Serial.println(ledsEncendidos);      
    }
  } else if (luzLux > (LUZ_UMBRAL + LUZ_HISTERESIS)) {
    if (ledsEncendidos > 0) {
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
            accionTemp == CALENTAR ? F("CALENTAR") : F("REPOSO")
        );
    }
}

void escribir595(uint8_t valor) {
  digitalWrite(PIN_595_LATCH, LOW);
  shiftOut(PIN_595_DATA, PIN_595_CLOCK, MSBFIRST, valor);
  digitalWrite(PIN_595_LATCH, HIGH);
}

void leerIR() {
  if (!IrReceiver.decode()) return;
  if (IrReceiver.decodedIRData.flags & IRDATA_FLAGS_IS_REPEAT) {
    IrReceiver.resume();
    return;
  }

  uint32_t codigo = IrReceiver.decodedIRData.decodedRawData;
  if (codigo == 0 || IrReceiver.decodedIRData.protocol != NEC ||
      IrReceiver.decodedIRData.numberOfBits != 32) {
    IrReceiver.resume();
    return;
  }

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
  return pow(RL10 * 1e3 * pow(10, GAMMA) / resistance, (1 / GAMMA));
}

void leerPulsadores() {
  uint8_t estadoActual = pcf8574_read();
  if (estadoActual != ultimaLectura) ultimoCambio = millis();
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

  if (!isnan(t) && t != tempMedida) { tempMedida = t; cambio = true; }
  if (!isnan(h) && h != humMedida) { humMedida = h; cambio = true; }
  if (l != luzLux) { luzLux = l; cambio = true; }

  if (cambio) {
    Serial.print(F("T=")); Serial.print(tempMedida);
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

    if (dist < mejorDistancia) mejor = true;
    else if (dist == mejorDistancia && cnt > mejorContador) mejor = true;
    else if (dist == mejorDistancia && cnt == mejorContador && tSol < mejorTiempo) mejor = true;

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

      if (dist < mejorDistancia) mejor = true;
      else if (dist == mejorDistancia && cnt > mejorContador) mejor = true;
      else if (dist == mejorDistancia && cnt == mejorContador && tSol < mejorTiempo) mejor = true;

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
// FUNCIONES DEL CONTROLADOR DIFUSO (FUZZY)
// ============================================================

float fuzzificarTrapezoidal(float x, float a, float b, float c, float d) {
  if (x <= a || x >= d) return 0.0f;
  if (x >= b && x <= c) return 1.0f;
  if (x > a && x < b) return (x - a) / (b - a);
  return (d - x) / (d - c);
}

float fuzzificarTriangular(float x, float a, float b, float c) {
  if (x <= a || x >= c) return 0.0f;
  if (x == b) return 1.0f;
  if (x < b) return (x - a) / (b - a);
  return (c - x) / (c - b);
}

float controlFuzzy(float distancia, float velocidad) {
  float muDistCerca  = fuzzificarTrapezoidal(distancia, 
    FUZZY_DIST_CERCA_A, FUZZY_DIST_CERCA_B, 
    FUZZY_DIST_CERCA_C, FUZZY_DIST_CERCA_D);
  float muDistMedia  = fuzzificarTrapezoidal(distancia, 
    FUZZY_DIST_MEDIA_A, FUZZY_DIST_MEDIA_B, 
    FUZZY_DIST_MEDIA_C, FUZZY_DIST_MEDIA_D);
  float muDistLejos  = fuzzificarTrapezoidal(distancia, 
    FUZZY_DIST_LEJOS_A, FUZZY_DIST_LEJOS_B, 
    FUZZY_DIST_LEJOS_C, FUZZY_DIST_LEJOS_D);

  float muVelLenta   = fuzzificarTrapezoidal(velocidad, 
    FUZZY_VEL_LENTA_A, FUZZY_VEL_LENTA_B, 
    FUZZY_VEL_LENTA_C, FUZZY_VEL_LENTA_D);
  float muVelMedia   = fuzzificarTriangular(velocidad, 
    FUZZY_VEL_MEDIA_A, FUZZY_VEL_MEDIA_B, FUZZY_VEL_MEDIA_C);
  float muVelRapida  = fuzzificarTrapezoidal(velocidad, 
    FUZZY_VEL_RAPIDA_A, FUZZY_VEL_RAPIDA_B, 
    FUZZY_VEL_RAPIDA_C, FUZZY_VEL_RAPIDA_D);

  float w[9];
  float s[9];
  uint8_t numReglas = 0;

  w[numReglas] = min(muDistCerca, muVelLenta);
  s[numReglas] = FUZZY_OUT_MUY_LENTO;
  numReglas++;

  w[numReglas] = min(muDistCerca, muVelMedia);
  s[numReglas] = FUZZY_OUT_MUY_LENTO;
  numReglas++;

  w[numReglas] = min(muDistCerca, muVelRapida);
  s[numReglas] = FUZZY_OUT_LENTO;
  numReglas++;

  w[numReglas] = min(muDistMedia, muVelLenta);
  s[numReglas] = FUZZY_OUT_MEDIO;
  numReglas++;

  w[numReglas] = min(muDistMedia, muVelMedia);
  s[numReglas] = FUZZY_OUT_RAPIDO;
  numReglas++;

  w[numReglas] = min(muDistMedia, muVelRapida);
  s[numReglas] = FUZZY_OUT_LENTO;
  numReglas++;

  w[numReglas] = min(muDistLejos, muVelLenta);
  s[numReglas] = FUZZY_OUT_RAPIDO;
  numReglas++;

  w[numReglas] = min(muDistLejos, muVelMedia);
  s[numReglas] = FUZZY_OUT_RAPIDO;
  numReglas++;

  w[numReglas] = min(muDistLejos, muVelRapida);
  s[numReglas] = FUZZY_OUT_MUY_RAPIDO;
  numReglas++;

  float numerador = 0.0f;
  float denominador = 0.0f;
  for (uint8_t i = 0; i < numReglas; i++) {
    numerador   += w[i] * s[i];
    denominador += w[i];
  }

  float delta;
  if (denominador > 0.001f) delta = numerador / denominador;
  else delta = FUZZY_OUT_MUY_LENTO;

  if (delta > FUZZY_VEL_MAX) delta = FUZZY_VEL_MAX;
  return delta;
}

// ============================================================
// MÁQUINA DE ESTADOS DEL ASCENSOR — v4.2
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
      if (tPuertaCerrada == 0) tPuertaCerrada = ahora;
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
      // [CORRECCIÓN v4.2] Recalcular prioridad con validación de redirección
      if (numSolicitudes > 0) {
        uint8_t nuevaPrioridad = calcularPrioridad();
        // Solo redirigir si la nueva planta es diferente Y físicamente válida
        if (nuevaPrioridad != 0 && 
            nuevaPrioridad != plantaDestino && 
            esRedireccionValida(nuevaPrioridad)) {
          plantaDestino = nuevaPrioridad;
          // [CORRECCIÓN v4.2] Reset parcial de velocidad para nuevo perfil fuzzy
          velocidadActual = 0.0f;
          Serial.print(F("[SCAN] Redirigiendo hacia P"));
          Serial.println(plantaDestino);
        }
      }

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
      if (tPuertaAbierta == 0) tPuertaAbierta = ahora;
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
      if (tEmergencia == 0) tEmergencia = ahora;
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
// [CORRECCIÓN v4.2] Control de movimiento con lógica difusa.
//
// CAMBIOS CRÍTICOS:
//   1. Clamping estricto de anguloServo a [0, 180] para evitar
//      que el servo gire fuera de sus límites físicos.
//   2. La dirección de movimiento se determina comparando
//      anguloServo contra destino en CADA ciclo, no usando
//      plantaDestino > plantaActual (que podía quedar obsoleto).
//   3. Al llegar exactamente al destino, se actualiza plantaActual
//      y se limpia velocidadActual.
// ============================================================
void moverAscensor() {
  int destino = ANGULO_PLANTA[plantaDestino - 1];

  // Si ya estamos exactamente en destino, confirmar llegada
  if (anguloServo == destino) {
    plantaActual = plantaDestino;
    cabinaMov = false;
    velocidadActual = 0.0f;
    return;
  }

  cabinaMov = true;

  // [CORRECCIÓN v4.2] Determinar dirección comparando posición actual vs destino REAL
  // No usar plantaDestino > plantaActual, que puede quedar desfasado
  bool subiendo = (anguloServo < destino);
  bool bajando = (anguloServo > destino);

  // Actualizar dirección lógica para SCAN
  if (subiendo) direccionActual = DIR_SUBIENDO;
  else if (bajando) direccionActual = DIR_BAJANDO;

  // Calcular distancia restante (siempre positiva)
  float distanciaRestante = abs(destino - anguloServo);

  // Control difuso: obtener incremento óptimo
  float deltaAngulo = controlFuzzy(distanciaRestante, velocidadActual);

  // Precisión final: a <= 2° forzar avance exacto de 1°
  if (distanciaRestante <= FUZZY_DIST_MINIMA) {
    deltaAngulo = FUZZY_OUT_MUY_LENTO;
  }

  // Aplicar incremento en la dirección correcta
  if (subiendo) {
    anguloServo += (int)deltaAngulo;
    // [CORRECCIÓN v4.2] Clamping estricto superior
    if (anguloServo >= destino) anguloServo = destino;
    if (anguloServo > SERVO_MAX) anguloServo = SERVO_MAX;
  } else if (bajando) {
    anguloServo -= (int)deltaAngulo;
    // [CORRECCIÓN v4.2] Clamping estricto inferior
    if (anguloServo <= destino) anguloServo = destino;
    if (anguloServo < SERVO_MIN) anguloServo = SERVO_MIN;
  }

  // Actualizar velocidad para siguiente ciclo
  velocidadActual = deltaAngulo;

  // [CORRECCIÓN v4.2] Segundo clamping de seguridad antes de escribir al servo
  if (anguloServo < SERVO_MIN) anguloServo = SERVO_MIN;
  if (anguloServo > SERVO_MAX) anguloServo = SERVO_MAX;

  servoAscensor.write(anguloServo);

  if (anguloServo == destino) {
    // Llegada exacta confirmada
    plantaActual = plantaDestino;
    cabinaMov = false;
    velocidadActual = 0.0f;
    Serial.print(F("Llegada P")); Serial.println(plantaActual);
  } else {
    // Actualizar planta lógica de forma robusta
    plantaActual = determinarPlantaDesdeAngulo(anguloServo);
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
      // [CORRECCIÓN v4.2] Registrar planta de origen para validar redirecciones
      plantaOrigenMovimiento = plantaActual;
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
  if (Wire.available()) valor = Wire.read();
  return valor;
}

void pcf8574_write(uint8_t valor) {
  Wire.beginTransmission(PCF8574_ADDR);
  Wire.write(valor);
  Wire.endTransmission();
}