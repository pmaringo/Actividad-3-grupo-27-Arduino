/********** INSTRUMENTACIÓN ELECTRÓNICA **********
 *
 * Programa ARDUINO MUIT 2026
 * Asignatura: Equipos e Instrumentación Electrónica
 * Profesor: Ivan Araquistain
 * 
 * ============================================================
 * Actividad 3 - Ascensor Inteligente ACME S.A.
 * 
 * Autor: Grupo 27
 *      - AMAYA ÁLVAREZ JIMÉNEZ
 *      - JESÚS MACANÁS SÁNCHEZ
 *      - PABLO MARÍN GONZÁLEZ
 *      - SERGIO PÉREZ GARCÍA
 * 
 * Fecha: 01 Junio, 2026 
 * ============================================================
 * MODIFICACIÓN v4.3 - Control Difuso (Fuzzy Logic) de Movimiento
 *                     (Histéresis de planta + SCAN robusto sin
 *                      oscilaciones)
 * ============================================================
 *  Hardware utilizado:
 *   - Arduino Uno
 *   - Receptor IR (pin A3)  +  mando a distancia IR
 *   - Servomotor SG90       (pin 9)
 *   - Sensor DHT22          (pin 7)
 *   - LDR                   (pin A0)
 *   - Registro 74HC595      (DATA=6, CLOCK=5, LATCH=4) → 8 LEDs
 *   - PIR (presencia)       (pin 8)
 *   - LCD I2C 16x2          (SDA=A4, SCL=A5)
 *   - 5 pulsadores de planta (PCF8574 I2C 0x20, P0-P4)
 *   - 2 LEDs indicadores     (pin A1 → frío, pin A2 → calor)
 *   - Lector RFID MFRC522 conectado por SPI:
 *         RST → pin 9, SDA(SS) → pin 10, SCK → 13,
 *         MOSI → 11, MISO → 12, 3.3V/GND alimentación.
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
 *    PUERTA_CERRADA → [MODIFICACIÓN v3.0] Transición de seguridad antes
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
 *  Diagrama de transiciones v4.2:
 *
 *   [REPOSO] ──(nueva solicitud)──► [PUERTA_CERRADA] ──(2 s)──► [MOVIMIENTO]
 *      ▲                              │                              │
 *      │         (llegó destino)      │                              │
 *      │◄──[PUERTA_ABIERTA]◄──────────┘                              │
 *      │         (3 s)                                               │
 *      │                                                             │
 *      └◄──[EMERGENCIA]◄──(mvto + presencia + >1 botón) ─────────────┘
 *               (reset 10 s)
 *
 *  Fuentes de comando: pulsadores (PCF8574 P0-P4) y mando IR NEC.
 *  Los comandos se encolan en cualquier estado excepto EMERGENCIA.
 *
 * ============================================================
 *  SISTEMA DE COLA MÚLTIPLE Y ALGORITMO DE PRIORIDAD SCAN v4.2
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
 *  [CORRECCIÓN v4.2] El algoritmo SCAN ahora opera con coherencia
 *  física: durante el movimiento, solo se aceptan redirecciones que
 *  no requieran invertir la marcha bruscamente. Una vez que la cabina
 *  ha pasado una planta, esa solicitud queda pendiente para la
 *  siguiente ronda (comportamiento de ascensor real).
 *
 *  Reglas de redirección durante MOVIMIENTO:
 *    · Si subiendo: solo se aceptan plantas SUPERIORES a la planta actual.
 *    · Si bajando: solo se aceptan plantas INFERIORES a la planta actual.
 *    · Si la nueva planta está en dirección opuesta o ya superada: se ignora
 *      temporalmente (queda en cola para después).
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
 *  CONTROL DIFUSO (FUZZY LOGIC) DE MOVIMIENTO — v4.0 / v4.2
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
 *  CORRECCIONES v4.2 (Frente a v4.1 / v4.0)
 * ============================================================
 *  [BUG 1] Bucle infinito SCAN al redirigir durante movimiento:
 *    Causa: plantaDestino cambiaba arbitrariamente en cada ciclo,
 *    pero el servo seguía su inercia. El fuzzy recalculaba desde
 *    la posición actual generando oscilaciones.
 *    SOLUCIÓN: Implementado validación de redirección. Durante
 *    el movimiento, solo se aceptan redirecciones en la misma
 *    dirección de marcha y que no hayan sido superadas físicamente.
 *    Las solicitudes en dirección opuesta quedan en cola para la
 *    siguiente ronda.
 *
 *  [BUG 2] Servo gira más allá de 180° o en círculos:
 *    Causa: falta de clamping del ángulo del servo cuando el
 *    destino cambiaba bruscamente mientras el servo avanzaba.
 *    SOLUCIÓN: Añadido clamping estricto de anguloServo a [0, 180].
 *    Además, la dirección de movimiento se recalcula explícitamente
 *    en cada ciclo comparando anguloServo contra destino real.
 *
 *  [BUG 3] Redirección usaba plantaOrigenMovimiento en lugar de
 *    la posición actual:
 *    Causa: esRedireccionValida() comparaba contra la planta de
 *    origen, no contra la planta actual física. Esto hacía que,
 *    por ejemplo, al bajar de P5 a P1, P4 se ignorara incorrectamente
 *    o P5 se aceptara tras haberla superado.
 *    SOLUCIÓN: Ahora se compara nuevaPlanta contra plantaActual
 *    (determinada por el ángulo del servo en tiempo real).
 *
 *  [MEJORA] Limpieza de velocidad al cambiar destino:
 *    Cuando se produce una redirección válida, la velocidadActual
 *    se resetea parcialmente para permitir un nuevo perfil fuzzy
 *    desde la posición actual hacia el nuevo destino.
 *
 * ============================================================
 *  CORRECCIONES v4.3 (Frente a v4.2)
 * ============================================================
 *  [BUG A] Oscilación entre destino y planta intermedia:
 *    Escenario: P5 → P1, en camino se pulsa P3. El servo se quedaba
 *    oscilando entre P3 y P1 sin parar nunca.
 *    Causa raíz combinada (tres factores encadenados):
 *      1. determinarPlantaDesdeAngulo() usaba zona muerta ±22°.
 *         plantaActual saltaba al valor del destino mucho antes
 *         de que el servo llegara físicamente al ángulo exacto,
 *         abriendo una ventana donde plantaActual==plantaDestino
 *         pero la solicitud seguía pendiente.
 *      2. calcularPrioridad() en Fase 1 NO incluía plantaActual
 *         como candidata; sólo la consideraba en Fase 3 como
 *         "último recurso" si no había ninguna otra solicitud.
 *      3. calcularPrioridad() modificaba direccionActual como
 *         efecto secundario en Fase 2; combinado con (1) y (2)
 *         provocaba inversiones de marcha en pleno movimiento.
 *
 *  [BUG B] Plantas intermedias ignoradas como redirección:
 *    Escenario: P5 → P1 bajando, se pulsa P4. Debería detenerse
 *    en P4 (es una parada intermedia natural). El comportamiento
 *    correcto del SCAN es atender plantas estrictamente entre
 *    plantaActual y plantaDestino en la dirección de marcha.
 *
 *  [BUG C] Pulsaciones rápidas en reposo no priorizadas correctamente:
 *    Escenario: estando en P1, se pulsa P5 e inmediatamente P3.
 *    Debería atender P3 primero (parada intermedia subiendo) y
 *    luego P5. Este caso depende del momento exacto de la 2ª
 *    pulsación; se cubre tanto si llega antes del arranque como
 *    si llega ya en movimiento.
 *
 *  SOLUCIÓN — Cinco cambios coordinados:
 *
 *  [FIX 1] Histéresis en determinarPlantaDesdeAngulo():
 *    plantaActual sólo se actualiza cuando el ángulo del servo
 *    está dentro de ±UMBRAL_LLEGADA_PLANTA (10°) del ángulo
 *    nominal de la planta. Entre plantas (zonas de tránsito),
 *    se conserva el valor previo. Esto elimina las falsas
 *    transiciones de planta antes de la llegada física.
 *    Zonas resultantes con UMBRAL=10°:
 *      P1: [0°, 10°]   tránsito: [11°, 34°]   conserva
 *      P2: [35°, 55°]  tránsito: [56°, 79°]   conserva
 *      P3: [80°, 100°] tránsito: [101°, 124°] conserva
 *      P4: [125°, 145°] tránsito: [146°, 169°] conserva
 *      P5: [170°, 180°]
 *
 *  [FIX 2] calcularPrioridad() sin efectos secundarios:
 *    a) Si plantaActual tiene solicitud activa, se devuelve
 *       inmediatamente como prioridad (parada inminente). Esto
 *       resuelve el caso de pulsar la planta en la que ya se
 *       está físicamente (gracias a la histéresis del FIX 1,
 *       plantaActual sólo vale eso si el servo está realmente
 *       en esa planta).
 *    b) Ya NO se modifica direccionActual desde la función. La
 *       decisión de invertir marcha se traslada al llamador,
 *       únicamente en la transición PUERTA_CERRADA → MOVIMIENTO.
 *       Durante MOVIMIENTO la dirección NO se invierte.
 *
 *  [FIX 3] esRedireccionValida() con regla SCAN estricta:
 *    Una nueva planta sólo se acepta como redirección si:
 *      · Coincide con plantaActual y tiene solicitud activa
 *        (parada inmediata en el piso donde está la cabina), O
 *      · Está estrictamente entre plantaActual y plantaDestino
 *        en la dirección de marcha (parada intermedia natural).
 *    Las solicitudes fuera de este rango quedan en cola y se
 *    atienden al llegar al destino (en PUERTA_CERRADA). Este es
 *    el comportamiento de un ascensor real: bajando hacia P1,
 *    una llamada por encima del piso actual nunca invierte la
 *    marcha — espera su turno en la ronda siguiente.
 *
 *  [FIX 4] Selección de dirección en PUERTA_CERRADA → MOVIMIENTO:
 *    Al transicionar de PUERTA_CERRADA a MOVIMIENTO se fija
 *    direccionActual explícitamente comparando plantaDestino
 *    con plantaActual. Esto garantiza coherencia entre el estado
 *    lógico (dirección) y el físico (ángulos).
 *
 *  [FIX 5] Limpieza al volver a REPOSO:
 *    direccionActual se resetea a DIR_NINGUNA al entrar a REPOSO
 *    (ya existía en v4.2); se confirma su corrección.
 *
 *  Casos que ahora se comportan como un ascensor real:
 *    · P5 → P1, en camino pulsa P3 → para en P3 y sigue a P1. ✓
 *    · P5 → P1, en camino pulsa P4 → para en P4 y sigue a P1. ✓
 *    · P4 → P1 (bajando), pulsa P5 → ignora P5 durante la
 *      marcha, la atiende después de P1 (ronda de subida). ✓
 *    · P1 → P5 (subiendo), pulsa P3 → para en P3 y sigue a P5. ✓
 *    · En P1 reposando, pulsa P5 y luego P3 (antes del arranque):
 *      arranque hacia P3 (más cerca), luego P5. ✓
 *    · En P1 reposando, pulsa P5 → ya en marcha pulsa P3:
 *      acepta P3 si aún no se ha pasado físicamente; en caso
 *      contrario la atiende al volver. ✓
 *
 ============================================================
 *  CORRECCIONES v4.4 (Frente a v4.3)
 * ============================================================
 *  [NUEVO] Control de acceso restringido a la Planta 5 mediante
 *           lector RFID MFRC522 (bus SPI):
 *
 *    a) Planta 5 declarada como zona de acceso restringido.
 *    b) Lector MFRC522 conectado por SPI:
 *         RST → pin 9, SDA(SS) → pin 10, SCK → 13,
 *         MOSI → 11, MISO → 12, 3.3V/GND alimentación.
 *    c) Solo tarjetas cuyo UID esté en la lista blanca
 *       TARJETAS_AUTORIZADAS pueden habilitar el botón de P5.
 *    d) Al presentar una tarjeta válida, se habilita el acceso
 *       a P5 durante 15 segundos (timeout automático).
 *       Transcurrido ese tiempo, el botón P5 vuelve a estar
 *       bloqueado hasta una nueva tarjeta autorizada.
 *    e) Durante el tiempo de desbloqueo, tanto el pulsador
 *       físico (PCF8574 P4) como el botón 5 del mando IR
 *       funcionan con normalidad. Si el acceso está bloqueado,
 *       ambas vías deniegan la solicitud y muestran mensaje
 *       por monitor Serie sin alterar la cola ni el movimiento.
 *    f) La lógica de movimiento (SCAN + Fuzzy) no se altera;
 *       si P5 ya estaba encolada o en trayecto cuando expira
 *       el timeout, el ascensor completa el viaje normalmente.
 *       El bloqueo solo afecta a NUEVAS solicitudes de P5.
 *
 *  Funciones añadidas:
 *    · gestionarAccesoP5()    → lectura cíclica RFID + timeout
 *    · tarjetaAutorizada()    → comparación UID contra lista blanca
 *    · mostrarUID()           → debug del UID leído por Serial
 *
 * ============================================================
 * 
 * ============================================================
 *   - LCD: muestra planta actual, Tª, humedad y acción de control
 * ============================================================
 *
 * Simulación en Wokwi: https://wokwi.com/projects/464679537824441345
 */

// --- Inclusión de Bibliotecas ---
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Servo.h>
#include <DHT.h>
#include <IRremote.h>
#include <SPI.h>
#include <MFRC522.h>

// Uncomment to enable debug log output
//#define DEBUG

#ifdef DEBUG
#define DBG_PRINT(...) Serial.print(__VA_ARGS__)
#define DBG_PRINTLN(...) Serial.println(__VA_ARGS__)
#else
#define DBG_PRINT(...) do {} while (0)
#define DBG_PRINTLN(...) do {} while (0)
#endif

// --- Definición de Pines ---
#define PIN_LDR        A0
#define PIN_EV_FRIO    A1   // LED azul  → electroválvula refrigerante
#define PIN_EV_CALOR   A2   // LED rojo  → electroválvula calefacción
#define PIN_IR         A3	  // IR Remote control
#define PIN_595_DATA    6   // Pin de datos serie (DS) del 74HC595 – datos que se desplazan.
#define PIN_595_CLOCK   5   // Pin de reloj de desplazamiento (SHCP) del 74HC595
#define PIN_595_LATCH   4   // Pin de latch (STCP) del 74HC595 – actualiza las salidas QA–QH.
#define PIN_SERVO       3   // PWM para el control del servo SG90
#define PIN_DHT         7
#define PIN_PIR         8

// [MODIFICACIÓN v4.4] Pines del lector RFID MFRC522 (bus SPI)
#define PIN_RFID_RST    9   // Reset del MFRC522
#define PIN_RFID_SS    10   // Slave Select (SDA) del MFRC522

// Direccion I2C del PCF8574 (A0=A1=A2=GND -> 0x20)
const uint8_t PCF8574_ADDR = 0x20;

// Pines del PCF8574 donde estan conectados los pulsadores
// de planta (P1..P5) → Lecura digital con pull-up interno, activo LOW, bus I2C
const uint8_t BOTON_P0 = 0;
const uint8_t BOTON_P1 = 1;
const uint8_t BOTON_P2 = 2;
const uint8_t BOTON_P3 = 3;
const uint8_t BOTON_P4 = 4;

// Mascara para los 5 pulsadores (bits 0-4)
const uint8_t BOTONES_MASK = 0x1F;  // Bits 0-4 (0b00011111)

// Variables para anti-rebote y deteccion de flanco
uint8_t estadoAnterior = 0xFF;    // Estado estable confirmado
uint8_t ultimaLectura = 0xFF;     // Última lectura (puede tener rebotes)
unsigned long ultimoCambio = 0;
const unsigned long DEBOUNCE_MS = 25;

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

// --- [MODIFICACIÓN v4.0 / v4.2] CONSTANTES DEL CONTROL DIFUSO (FUZZY) ---
// ================================================================
// Parámetros del controlador difuso de movimiento del ascensor.
// Estos valores definen los conjuntos difusos y singletones que
// generan el perfil de velocidad trapezoidal suave.
// ================================================================

// Intervalo de tiempo entre actualizaciones del servo (ms)
// A 65 ms se obtienen ~15 actualizaciones/segundo
#define SERVO_INTERVALO   65

// [MODIFICACIÓN v4.4] Control de acceso restringido Planta 5
#define TIEMPO_ACCESO_P5    15000   // ms (15 s) de desactivación automática
#define NUM_TARJETAS_REG      9     // Número de tarjetas registradas

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
#define FUZZY_DIST_MINIMA    2.0f

// Límites físicos del servo SG90
#define SERVO_MIN            0
#define SERVO_MAX          180

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

// [MODIFICACIÓN v4.4] UIDs de tarjetas RFID autorizadas para acceso a P5
// Cada tarjeta tiene 4 bytes de UID (tamaño estándar MIFARE Classic 1K)
// IMPORTANTE: sustituir estos valores por los UIDs reales de sus tarjetas
const byte TARJETAS_AUTORIZADAS[NUM_TARJETAS_REG][10] = {
  {0xA1, 0xB2, 0xC3, 0xD4},   // Tarjeta maestra 1
  {0x12, 0x34, 0x56, 0x78},   // Tarjeta usuario 2
  {0xDE, 0xAD, 0xBE, 0xEF},   // Tarjeta mantenimiento 3
  {0x01, 0x02, 0x03, 0x04},   // Tarjeta usuario 4 (Blue Card)
  {0x11, 0x22, 0x33, 0x44},   // Tarjeta usuario 5 (Green Card)
  {0x55, 0x66, 0x77, 0x88},   // Tarjeta usuario 6 (Yellow Card)
  {0xAA, 0xBB, 0xCC, 0xDD},   // Tarjeta usuario 7 (Red Card)
  {0x04, 0x11, 0x22, 0x33},   // Tarjeta usuario 8 (NFC Card)
  {0xC0, 0xFF, 0xEE, 0x99}    // Tarjeta usuario 9 (Key Fob)
};

// ============================================================
// [MODIFICACIÓN v3.0] ==================================================
// SISTEMA DE COLA DE SOLICITUDES Y ALGORITMO DE PRIORIDAD SCAN
// =====================================================================
#define MAX_PLANTAS           5
#define TIEMPO_CIERRE_PUERTA  2000   // ms que permanece en PUERTA_CERRADA

// [MODIFICACIÓN v4.3] Margen de histéresis para determinarPlantaDesdeAngulo()
// ----------------------------------------------------------------------------
// El servo debe estar dentro de ±UMBRAL_LLEGADA_PLANTA grados del ángulo
// nominal de una planta para que plantaActual se actualice a esa planta.
// Fuera de ese margen (zonas de "tránsito" entre plantas), plantaActual
// conserva el valor previo. Esto evita que la planta lógica se "adelante"
// al destino físico, eliminando las oscilaciones de la v4.2.
//
// Con UMBRAL=10° y plantas separadas 45°, queda una zona de tránsito de
// ~23° entre cada par de plantas en la que plantaActual no cambia.
#define UMBRAL_LLEGADA_PLANTA 10  // grados

// Cola de solicitudes: cada índice representa una planta (0=P1 ... 4=P5)
bool          solicitudes[MAX_PLANTAS] = {false, false, false, false, false};
uint8_t       numSolicitudes = 0;        // Contador de plantas pendientes
unsigned long tiempoSolicitud[MAX_PLANTAS] = {0, 0, 0, 0, 0};   // Timestamp 1ª solicitud
uint8_t       contadorSolicitudes[MAX_PLANTAS] = {0, 0, 0, 0, 0}; // Nº de pulsaciones acumuladas

// Dirección de marcha del ascensor para el algoritmo SCAN
enum DireccionAscensor { DIR_NINGUNA, DIR_SUBIENDO, DIR_BAJANDO };
DireccionAscensor direccionActual = DIR_NINGUNA;

// Temporizadores de estados (reemplazan variables static locales para robustez)
unsigned long tPuertaAbierta = 0;
unsigned long tPuertaCerrada = 0;
unsigned long tEmergencia    = 0;
// =====================================================================

// --- Inicialización de Objetos ---
LiquidCrystal_I2C lcd(0x27, 16, 2);
Servo             servoAscensor;
DHT               dht(PIN_DHT, DHT22);      // Sensor DHT22

// --- Variables Globales ---
uint8_t plantaActual    = 1;   // Planta donde está la cabina
uint8_t plantaDestino   = 1;   // Planta solicitada (salida del algoritmo de prioridad)
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

// [MODIFICACIÓN v4.0] Variables del control difuso
// Velocidad actual del ascensor (grados por ciclo de servo)
// Se utiliza como realimentación para el controlador difuso
float         velocidadActual = 0.0f;

// [MODIFICACIÓN v4.4] Variables de control de acceso P5
MFRC522 mfrc522(PIN_RFID_SS, PIN_RFID_RST);   // Objeto lector RFID
bool accesoP5Habilitado = false;              // true = P5 desbloqueada temporalmente
unsigned long tActivacionP5 = 0;              // Timestamp de activación por tarjeta
unsigned long tUltimoRFID = 0;                // Timer para lectura periódica RFID

// --- MÁQUINA DE ESTADOS DEL ASCENSOR ---
enum EstadoAscensor {
  ASCENSOR_REPOSO,        // Cabina parada en planta, esperando comandos
  ASCENSOR_PUERTA_CERRADA,// [MODIFICACIÓN v3.0] Puerta cerrándose antes de moverse
  ASCENSOR_MOVIMIENTO,    // Cabina moviéndose hacia destino
  ASCENSOR_EMERGENCIA,    // Parada de emergencia por seguridad
  ASCENSOR_PUERTA_ABIERTA,// Puerta abierta (simulado)
  ASCENSOR_MANTENIMIENTO  // Estado de mantenimiento/debug
};
EstadoAscensor estadoAscensor = ASCENSOR_REPOSO;  // Estado inicial

// Acción de control iluminación
uint8_t ledsEncendidos = 0;  // 0-8 LEDs encendidos

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
uint8_t pcf8574_read();
void pcf8574_write(uint8_t valor);
void transicionEstado(EstadoAscensor nuevoEstado);

// [MODIFICACIÓN v3.0] Prototipos del sistema de cola y prioridad
void agregarSolicitud(uint8_t planta);
void eliminarSolicitud(uint8_t planta);
uint8_t calcularPrioridad();

// [MODIFICACIÓN v4.0] Prototipos del control difuso
float fuzzificarTrapezoidal(float x, float a, float b, float c, float d);
float fuzzificarTriangular(float x, float a, float b, float c);
float controlFuzzy(float distancia, float velocidad);

// [CORRECCIÓN v4.2] Nuevos prototipos
uint8_t determinarPlantaDesdeAngulo(int angulo);
bool esRedireccionValida(uint8_t nuevaPlanta);

// [MODIFICACIÓN v4.4] Prototipos control de acceso RFID
void gestionarAccesoP5();
bool tarjetaAutorizada(byte *uidBuffer, byte bufferSize);
void mostrarUID(byte *uidBuffer, byte bufferSize);

// ============================================================
// --- Configuración Inicial (setup) ---
// ============================================================
void setup() {
  Serial.begin(9600);

  Serial.println(F("UNIR Actividad 3 - Ascensor ACME v4.4"));
  Serial.println(F("[MOD v4.4] Fuzzy + SCAN + RFID Acceso P5 (histeresis de planta)"));
  Serial.println(F(""));

  // Inicializar LCD
  Serial.println(F("Inicializando LCD..."));
  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("ACME ASCENSOR v4.4");
  lcd.setCursor(0, 1);
  lcd.print("SCAN Estable");
  delay(1500);

  Serial.println(F("Pantalla LCD inicializada."));
  Serial.println(F("------------------------------------"));

  // Inicializar bus I2C para lectura de pulsadores (PCF8574)
  Serial.println(F("Inicializando bus I2C para pulsadores..."));

  // Escanear bus I2C para verificar que el PCF8574 responde
  Serial.print(F("Buscando dispositivos I2C... "));
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
    Serial.println(F("Verifica las conexiones y la direccion I2C."));
  } else {
    Serial.println(F("PCF8574 detectado correctamente."));
  }

  // Configurar PCF8574: escribir 1 en P0-P4 para activar pull-ups (modo entrada)
  // Los bits 5-7 tambien se ponen en 1 (input con pull-up por defecto)
  pcf8574_write(0xFF);

  Serial.println(F("Pulsadores listos. Presiona cualquier boton..."));
  Serial.println(F("Formato: [BOTON_X] PRESIONADO / SOLTADO"));
  Serial.println(F("Bus I2C inicializado."));
  Serial.println(F("------------------------------------"));

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

  // [MODIFICACIÓN v4.0] Inicializar velocidad del control difuso
  velocidadActual = 0.0f;

  // DHT (temperatura y humedad)
  dht.begin();

  // IR (control receptorremoto)
  IrReceiver.begin(PIN_IR);   // Iniciar receptor IR

  // [MODIFICACIÓN v4.4] Inicializar lector RFID MFRC522
  SPI.begin();
  mfrc522.PCD_Init();
  Serial.println(F("Lector RFID MFRC522 inicializado."));
  // Verificar presencia del MFRC522
  byte version = mfrc522.PCD_ReadRegister(mfrc522.VersionReg);
  if (version == 0x00 || version == 0xFF) {
    Serial.println(F("ADVERTENCIA: MFRC522 no detectado. Verificar conexiones SPI."));
  } else {
    Serial.print(F("MFRC522 detectado. Version: 0x"));
    Serial.println(version, HEX);
  }
  Serial.println(F("------------------------------------"));

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

  // [MODIFICACIÓN v4.0] Informar sobre el control difuso
  Serial.println(F(" Control Difuso (Fuzzy) de Movimiento v4.0"));
  Serial.println(F("Perfil de velocidad: Arranque progresivo | Crucero estable | Frenado suave"));
  Serial.println(F("Metodo: Mamdani (min-max) + Defuzzificacion por media ponderada"));
  Serial.println(F("Entradas: distanciaRestante + velocidadActual"));
  Serial.println(F("Salida: deltaAngulo (incremento por ciclo)"));
  Serial.println(F("------------------------------------"));

  // [MODIFICACIÓN v4.4] Informar sobre control de acceso P5
  Serial.println(F(" Control de Acceso RFID v4.4"));
  Serial.println(F("Planta 5: ACCESO RESTRINGIDO"));
  Serial.println(F("Tarjetas registradas: "));
  for (uint8_t i = 0; i < NUM_TARJETAS_REG; i++) {
    Serial.print(F("  ["));
    Serial.print(i+1);
    Serial.print(F("] "));
    for (uint8_t j = 0; j < 4; j++) {
      if (TARJETAS_AUTORIZADAS[i][j] < 0x10) Serial.print(F("0"));
      Serial.print(TARJETAS_AUTORIZADAS[i][j], HEX);
    }
    Serial.println();
  }
  Serial.println(F("Timeout acceso: 15 segundos"));
  Serial.println(F("------------------------------------"));

  // LCD (Sistema listo)
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Sistema LISTO!");
  delay(1000);
  lcd.clear();

  Serial.println(F("Sistema ascensor v4.4 LISTO!"));
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

  // 1. Entradas de usuario se procesan solo cada 10 ms para evitar rebotes y sobrecarga
  if (ahora - tUltimaPulsaci >= 10) {
    tUltimaPulsaci = ahora;

    // Leer pulsadores siempre, EXCEPTO en emergencia
    if (estadoAscensor != ASCENSOR_EMERGENCIA) {
      leerPulsadores();
      leerIR();
    }
  }

  // [MODIFICACIÓN v4.4] Control de acceso RFID cada 200 ms
  if (ahora - tUltimoRFID >= 200) {
    tUltimoRFID = ahora;
    gestionarAccesoP5();
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
// [SIN MODIFICACIÓN v3.0] Se conserva exactamente igual.
// ============================================================
void actualizarLCD() {
  lcd.clear();

  if (paginaLCD == 0) {
    // Línea 0: estado de movimiento y planta
    lcd.setCursor(0, 0);
    if (cabinaMov) {
      lcd.print(F("Moviendo "));
      lcd.print(plantaActual);
      lcd.print(F("->"));
      lcd.print(plantaDestino);
    } else {
      lcd.print(F("En planta "));
      lcd.print(plantaActual);
    }

    // Línea 1: presencia en cabina
    lcd.setCursor(0, 1);
    if (presencia) {
      lcd.print(F("Cabina: OCUPADA"));
    } else {
      lcd.print(F("Cabina: LIBRE"));
    }
  } else {
    // Línea 0: Temperatura y Humedad
    lcd.setCursor(0, 0);
    lcd.print(F("T:"));
    lcd.print(tempMedida, 1);
    lcd.write((char)223);  // símbolo °
    lcd.print(F("C H:"));
    lcd.print((int)humMedida);
    lcd.print(F("%"));

    // Línea 1: Luz y control temperatura
    lcd.setCursor(0, 1);
    if (luzLux > 999) {
      lcd.print(F("L.DIURNA "));
    } else {
      lcd.print(F("L:"));
      lcd.print((int)luzLux);
      lcd.print(F("lux "));
    }

    switch (accionTemp) {
      case ENFRIAR:
        lcd.print(F("ENFRIAR"));
        break;
      case CALENTAR:
        lcd.print(F("CALDEAR"));
        break;
      default:
        break;
    }
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
// [CORRECCIÓN v4.0] Se usa 'cantidad' en lugar de 'ledsEncendidos'
// para respetar el parámetro de entrada.
// ============================================================
void actualizarLeds(uint8_t cantidad) {
  // Generar máscara de bits para los N leds encendidos
  uint8_t mascara = 0;
  for (uint8_t i = 0; i < cantidad; i++) {
    mascara |= (1 << i);
  }
  escribir595(mascara); 
}

// ============================================================
// [MODIFICACIÓN v3.0] ==================================================
// SISTEMA DE COLA DE SOLICITUDES Y ALGORITMO DE PRIORIDAD SCAN
// =====================================================================

// ============================================================
// agregarSolicitud(uint8_t planta)
// --------------------------------
// Añade una planta a la cola de solicitudes. Si la planta ya
// estaba en cola, solo incrementa su contador de demanda.
// Registra el timestamp de la primera pulsación para el criterio
// de tiempo de espera.
//
// @param planta  Número de planta (1–5).
// ============================================================
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
  // Incrementar contador de pulsaciones (criterio de prioridad secundario)
  // [BUG 4.4] contadorSolicitudes es uint8_t (máx 255). Sin límite superior, la
  //   pulsación 256 desbordaba a 0, invirtiendo la prioridad: una planta muy
  //   solicitada pasaba a tener contador 0, quedando por detrás de plantas
  //   nuevas con contador 1. [FIX 3]: saturar en 255 en lugar de dejar el overflow.
  if (contadorSolicitudes[idx] < 255) contadorSolicitudes[idx]++;
}

// ============================================================
// calcularPrioridad()
// -------------------
// Algoritmo SCAN (Elevator Algorithm) adaptado para 5 plantas.
// Determina la planta destino óptima considerando:
//   1. Dirección actual de marcha (subida/bajada).
//   2. Distancia al piso (menor = mayor prioridad).
//   3. Número de solicitudes acumuladas (mayor = mayor prioridad).
//   4. Tiempo de espera (mayor antigüedad = mayor prioridad).
//
// Si no hay solicitudes en la dirección actual, busca también en
// la dirección opuesta.
//
// [MODIFICACIÓN v4.3]
//   a) Si plantaActual tiene solicitud activa se devuelve PRIMERO
//      (distancia=0, parada inminente). Gracias a la histéresis
//      de determinarPlantaDesdeAngulo() esta situación sólo se da
//      cuando el servo está físicamente en esa planta.
//   b) Esta función YA NO modifica direccionActual como efecto
//      secundario. La decisión de invertir marcha la toma el
//      llamador (manejarEstadoAscensor en PUERTA_CERRADA), nunca
//      desde MOVIMIENTO.
//
// @return  Número de planta prioritario (1–5), o 0 si no hay solicitudes.
// ============================================================
uint8_t calcularPrioridad() {
  if (numSolicitudes == 0) return 0;

  // [MODIFICACIÓN v4.3] FASE 0: solicitud en la planta actual.
  // Si plantaActual está en cola, atenderla inmediatamente.
  // Esto sustituye a la antigua Fase 3 (último recurso) y la
  // sube al primer lugar, donde corresponde por distancia=0.
  if (solicitudes[plantaActual - 1]) {
    return plantaActual;
  }

  uint8_t mejorPlanta = 0;
  int8_t  mejorDistancia = 127;
  uint8_t mejorContador = 0;
  unsigned long mejorTiempo = 0xFFFFFFFF; // más antiguo = menor valor = mayor prioridad

  // ----------------------------------------------------------
  // FASE 1: Buscar solicitudes en la DIRECCIÓN ACTUAL
  // ----------------------------------------------------------
  for (uint8_t i = 0; i < MAX_PLANTAS; i++) {
    if (!solicitudes[i]) continue;
    uint8_t p = i + 1;
    int8_t dist = 0;
    bool enDir = false;

    // Si sube o está parado, evaluar plantas superiores
    if (direccionActual == DIR_SUBIENDO || direccionActual == DIR_NINGUNA) {
      if (p > plantaActual) {
        dist = p - plantaActual;
        enDir = true;
      }
    }
    // Si baja o está parado, evaluar plantas inferiores
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

  // ----------------------------------------------------------
  // FASE 2: Si no hay solicitudes en dirección actual,
  //         buscar en sentido opuesto.
  // [MODIFICACIÓN v4.3] Ya NO se modifica direccionActual aquí.
  // El llamador decidirá si invertir marcha (sólo se hace en la
  // transición PUERTA_CERRADA → MOVIMIENTO, no durante MOVIMIENTO).
  // ----------------------------------------------------------
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
      // Si estaba parado, ya se evaluó todo en Fase 1
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
  }

  // [MODIFICACIÓN v4.3] La antigua Fase 3 (solicitud en planta
  // actual) ya no es necesaria: ahora es la Fase 0, evaluada al
  // principio de la función.

  return mejorPlanta;
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
// [SIN MODIFICACIÓN v3.0]
// ============================================================
uint8_t contarBotonesPulsados() {

  uint8_t estadoActual = pcf8574_read();  // Leer el expansor I2C
  uint8_t count = 0;

  for (uint8_t i = 0; i < 5; i++) {
    if (!(estadoActual & (1 << i))) {  // LOW = presionado (pull-up)
      count++;
    }
  }
  return count;
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
// Control de iluminación escalonado 8 LEDs con histéresis
// Implementa control proporcional inverso DISCRETO:
// - Setpoint: 500 lux (nivel óptimo)
// - Umbral de activación: 400 lux
// - Histéresis: ±10 lux (zona muerta: 390-410 lux)
// Lógica:
//   luz < 390 lux → encender 1 LED (contador++)
//   luz > 410 lux → apagar 1 LED (contador--)
//   390-410 lux → sin cambios (zona muerta)
//
// [SIN MODIFICACIÓN v3.0]
// ============================================================
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
//
// [SIN MODIFICACIÓN v3.0]
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
// determinarPlantaDesdeAngulo()
// ------------------------------------------------------------
// [MODIFICACIÓN v4.3] Histéresis de planta:
// Devuelve la planta correspondiente al ángulo del servo SOLO si
// el servo está dentro de ±UMBRAL_LLEGADA_PLANTA del ángulo nominal
// de esa planta. En las zonas de tránsito entre plantas, conserva
// el valor previo de plantaActual.
//
// Antes (v4.2) se usaba redondeo con zona muerta ±22° que hacía
// que plantaActual saltara al valor del piso destino antes de que
// el servo llegara físicamente, lo que abría una ventana en la que
// calcularPrioridad() veía plantaActual==plantaDestino con la
// solicitud aún pendiente y desencadenaba la oscilación.
//
// Ahora, plantaActual sólo se actualiza al cruzar realmente a la
// zona de la nueva planta. La detección de llegada exacta sigue
// haciéndose en moverAscensor() mediante anguloServo == destino.
// ============================================================
uint8_t determinarPlantaDesdeAngulo(int angulo) {
  for (uint8_t i = 0; i < MAX_PLANTAS; i++) {
    if (abs(angulo - ANGULO_PLANTA[i]) <= UMBRAL_LLEGADA_PLANTA) {
      return i + 1;
    }
  }
  // Zona de tránsito entre plantas: conservar plantaActual previa
  return plantaActual;
}

// ============================================================
// eliminarSolicitud(uint8_t planta)
// ---------------------------------
// Elimina una planta de la cola cuando el ascensor llega a ella.
// Resetea contador y timestamp asociados.
//
// @param planta  Número de planta (1–5).
// ============================================================
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

// ============================================================
// Escribir byte al 74HC595
//
// Transfiere un byte al registro de desplazamiento 74HC595 usando
//  comunicación SPI en modo software (bit-banging), empezando por el bit
// más significativo (MSB first), y activa el latch para que las salidas
// QA–QH reflejen el nuevo valor.
//
// [SIN MODIFICACIÓN v3.0]
// ============================================================
void escribir595(uint8_t valor) {
  // Bajar latch antes de enviar datos
  digitalWrite(PIN_595_LATCH, LOW);

  // Enviar los 8 bits al registro de desplazamiento (MSB primero)
  shiftOut(PIN_595_DATA, PIN_595_CLOCK, MSBFIRST, valor);

  // Subir latch para actualizar las salidas QA–QH
  digitalWrite(PIN_595_LATCH, HIGH);
}

// ============================================================
// [CORRECCIÓN v4.2 / v4.3] esRedireccionValida()
// ------------------------------------------------------------
// Valida si una nueva solicitud de planta puede aceptarse como
// redirección de plantaDestino durante el MOVIMIENTO actual.
//
// [MODIFICACIÓN v4.3] Regla SCAN estricta de parada intermedia:
// Sólo se acepta una redirección si la nueva planta es una parada
// "natural" en el trayecto actual:
//
//   · Si nuevaPlanta == plantaActual y tiene solicitud activa →
//     parada inmediata en el piso donde está la cabina. Gracias
//     a la histéresis de v4.3, este caso sólo ocurre cuando el
//     servo está realmente en la zona de esa planta.
//
//   · SUBIENDO: nuevaPlanta debe estar estrictamente entre
//     plantaActual y plantaDestino (plantaActual < nueva < destino).
//
//   · BAJANDO: nuevaPlanta debe estar estrictamente entre
//     plantaDestino y plantaActual (destino < nueva < plantaActual).
//
//   · Cualquier otra planta (en dirección opuesta, ya superada,
//     o más allá del destino) NO redirige: queda en cola para
//     atenderse en la siguiente ronda. Comportamiento ascensor real.
//
// @param nuevaPlanta  Planta solicitada (1–5)
// @return  true si la redirección es físicamente coherente
// ============================================================
bool esRedireccionValida(uint8_t nuevaPlanta) {
  if (nuevaPlanta < 1 || nuevaPlanta > MAX_PLANTAS) return false;

  // Si estamos parados, cualquier planta es válida
  if (!cabinaMov) return true;

  // Si la nueva planta es la misma que el destino actual, no es redirección
  if (nuevaPlanta == plantaDestino) return false;

  // [MODIFICACIÓN v4.3] Parada inmediata en plantaActual si se ha
  // pulsado el piso en el que está la cabina (con solicitud activa).
  if (nuevaPlanta == plantaActual && solicitudes[nuevaPlanta - 1]) {
    return true;
  }

  // [MODIFICACIÓN v4.3] Sólo paradas intermedias estrictas en la
  // dirección de marcha. NO se invierte la marcha durante MOVIMIENTO.
  if (direccionActual == DIR_SUBIENDO) {
    return (nuevaPlanta > plantaActual && nuevaPlanta < plantaDestino);
  } else if (direccionActual == DIR_BAJANDO) {
    return (nuevaPlanta < plantaActual && nuevaPlanta > plantaDestino);
  }

  return false; // DIR_NINGUNA con cabinaMov=true no debería ocurrir
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
/*
 Forma del trapecio:
      a     b       c     d
      |_____|_______|_____|
           /         \
          /           \
    0 ___/             \___ 0
*/
// @param x  Valor a fuzzificar
// @param a  Inicio del soporte (pertenencia = 0)
// @param b  Inicio del núcleo (pertenencia = 1)
// @param c  Fin del núcleo (pertenencia = 1)
// @param d  Fin del soporte (pertenencia = 0)
// @return  Grado de pertenencia [0.0, 1.0]
// ============================================================
float fuzzificarTrapezoidal(float x, float a, float b, float c, float d) {
  // [BUG 4.4] Límites del soporte usaban <= / >= en vez de < / >.
  //   · Límite izquierdo: con a=b=0 (conjuntos CERCA y LENTA), x=0 devolvía
  //     0.0 en lugar de 1.0. Cuando velocidadActual=0 todos los conjuntos de
  //     velocidad daban membresía 0, el denominador de la defuzzificación era
  //     0 y el fallback forzaba siempre 1°/ciclo en el primer paso tras
  //     cualquier arranque o redirección SCAN.
  //   · Límite derecho: con c=d=180 (conjunto LEJOS), x=180 devolvía 0.0 en
  //     lugar de 1.0. En el primer ciclo del viaje P1→P5 (distancia=180°)
  //     ocurría lo mismo: membresía 0, denominador 0, fallback a 1°/ciclo.
  //   Ambos defectos juntos provocaban que el servo arrancara siempre a
  //   1°/ciclo en lugar de los ~5-6° calculados, contribuyendo a movimientos
  //   erráticos con colas complejas y muchas redirecciones.
  //   [FIX]: cambiar "x <= a" por "x < a"  y  "x >= d" por "x > d".
  if (x < a || x > d) return 0.0f;
  if (x >= b && x <= c) return 1.0f;
  if (x > a && x < b) return (x - a) / (b - a);
  return (d - x) / (d - c);
}

// ============================================================
// fuzzificarTriangular()
// ----------------------
// Calcula el grado de pertenencia de un valor 'x' a un conjunto
// difuso triangular definido por los puntos (a,b,c).
/*
 Forma del triángulo:
           b (pico = 1)
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
// [MODIFICACIÓN v4.4] CONTROL DE ACCESO RFID — Planta 5 Restringida
// ============================================================

// ============================================================
// gestionarAccesoP5()
// -------------------
// Gestiona el ciclo de vida del acceso restringido a la Planta 5.
//   1. Verifica si ha expirado el timeout de 15 segundos.
//   2. Detecta tarjetas RFID presentadas ante el lector MFRC522.
//   3. Si la tarjeta está registrada, habilita el acceso a P5
//      durante TIEMPO_ACCESO_P5 milisegundos.
//   4. Si la tarjeta no está registrada, lo indica por Serial.
//
// La función debe llamarse periódicamente (cada 200 ms es suficiente).
// ============================================================
void gestionarAccesoP5() 
{
  unsigned long ahora = millis();

  // 1. Comprobar timeout de desactivación automática
  if (accesoP5Habilitado && (ahora - tActivacionP5 >= TIEMPO_ACCESO_P5)) {
    accesoP5Habilitado = false;
    Serial.println(F("[ACCESO P5] Tiempo expirado (15 s). Acceso restringido."));
    // NOTA: No se elimina la solicitud de P5 al caducar el acceso.
    // La autorización controla el derecho a ENCOLAR la petición, no a
    // mantenerla. Si el pasajero ya pulsó P5 está dentro del ascensor y
    // debe llegar a su destino aunque el timeout expire durante el trayecto
    // (p.ej. el ascensor deteniéndose en plantas intermedias).
  }

  // 2. Detectar tarjeta RFID
  if (!mfrc522.PICC_IsNewCardPresent()) {
    return;
  }
  if (!mfrc522.PICC_ReadCardSerial()) {
    return;
  }

  // Mostrar UID leído por debug
  DBG_PRINT(F("[RFID] UID leido: "));
  mostrarUID(mfrc522.uid.uidByte, mfrc522.uid.size);
  DBG_PRINTLN();

  // 3. Validar tarjeta
  if (tarjetaAutorizada(mfrc522.uid.uidByte, mfrc522.uid.size)) {
    accesoP5Habilitado = true;
    tActivacionP5 = ahora;
    Serial.print(F("[ACCESO P5] UID tarjeta: "));
    for (uint8_t i = 0; i < mfrc522.uid.size; i++) {
      if (mfrc522.uid.uidByte[i] < 0x10) Serial.print(F("0"));
      Serial.print(mfrc522.uid.uidByte[i], HEX);
      if (i < mfrc522.uid.size - 1) Serial.print(F(":"));
    }
    Serial.println();
    Serial.println(F("[ACCESO P5] Tarjeta AUTORIZADA. Boton P5 habilitado por 15 s."));
  } else {
    Serial.println(F("[ACCESO P5] Tarjeta NO autorizada. Acceso denegado."));
  }

  // 4. Finalizar comunicación con tarjeta
  mfrc522.PICC_HaltA();
  mfrc522.PCD_StopCrypto1();
}

// ============================================================
// Leer mando IR 
// [MODIFICACIÓN v3.0] Ahora encola la solicitud en lugar de
// sobreescribir plantaDestino directamente.
// ============================================================
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
    DBG_PRINT(F("[DEBUG] IR ignorado: protocolo="));
    DBG_PRINT(IrReceiver.decodedIRData.protocol);
    DBG_PRINT(F(" bits="));
    DBG_PRINT(IrReceiver.decodedIRData.numberOfBits);
    DBG_PRINTLN(codigo, HEX);
    IrReceiver.resume();
    return;
  }

  DBG_PRINT(F("[DEBUG] IR command: "));
  DBG_PRINTLN(IrReceiver.decodedIRData.command);
  DBG_PRINT(F("[DEBUG] IR raw: 0x"));
  DBG_PRINTLN(codigo, HEX);

  for (uint8_t i = 0; i < 5; i++) {
    if (codigo == IR_CODIGO[i]) {
      uint8_t plantaSolicitada = i + 1;

      // [MODIFICACIÓN v4.4] Validar acceso restringido a Planta 5
      if (plantaSolicitada == 5) {
        if (!accesoP5Habilitado) {
          Serial.println(F("[ACCESO P5] DENEGADO IR: se requiere tarjeta RFID autorizada"));
          break;
        }
      }

      // [MODIFICACIÓN v3.0] Encolar solicitud en lugar de sobreescribir destino
      agregarSolicitud(plantaSolicitada);
      Serial.print(F("IR -> P")); Serial.println(plantaSolicitada);
      break;
    }
  }
  IrReceiver.resume();
}

// ============================================================
// Calcula la iluminancia en lux a partir del LDR (A0)
//
// [SIN MODIFICACIÓN v3.0]
// ============================================================
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
// [MODIFICACIÓN v3.0] Ahora encola la solicitud en la cola
// mediante agregarSolicitud(), permitiendo múltiples pulsaciones.
// ============================================================
void leerPulsadores() {
  // Leer estado actual del PCF8574
  uint8_t estadoActual = pcf8574_read();

  // Aplicar anti-rebote: detectar si hubo CUALQUIER cambio
  if (estadoActual != ultimaLectura) {
    ultimoCambio = millis();  // Resetear timer de debounce
  }
  ultimaLectura = estadoActual;  // Guardar lectura actual

  if ((millis() - ultimoCambio) > DEBOUNCE_MS) {
    // Detectar cambios estables (solo en los pines P0-P4)
    uint8_t cambios = (estadoActual ^ estadoAnterior) & BOTONES_MASK;

    if (cambios != 0) {
      for (uint8_t i = 0; i < 5; i++) {
        if (cambios & (1 << i)) {
          bool presionado = !(estadoActual & (1 << i)); // LOW = presionado (pull-up a GND)

          if (presionado) {  // ← ignorar el evento "SOLTADO"
            uint8_t plantaSolicitada = i + 1;

            // [MODIFICACIÓN v4.4] Validar acceso restringido a Planta 5
            if (plantaSolicitada == 5) {
              if (!accesoP5Habilitado) {
                Serial.println(F("[ACCESO P5] DENEGADO: se requiere tarjeta RFID autorizada"));
                continue;  // No encolar la solicitud
              }
            }

            // [MODIFICACIÓN v3.0] Encolar solicitud en cola múltiple
            agregarSolicitud(plantaSolicitada);
            Serial.print(F("[BOTON_P"));
            Serial.print(plantaSolicitada);
            Serial.println(F("] PRESIONADO"));
          }          
        }
      }
      estadoAnterior = estadoActual;
    }
  }

}

// ============================================================
// Leer sensores ambientales (DHT22 + LDR)
//
// [SIN MODIFICACIÓN v3.0]
// ============================================================
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
// MÁQUINA DE ESTADOS DEL ASCENSOR — v4.2
// ============================================================

// ============================================================
// Función principal de la máquina de estados
// Se ejecuta en cada iteración del loop() y maneja el comportamiento
// del ascensor según su estado actual.
//
// [MODIFICACIÓN v3.0] Reestructurada para incluir PUERTA_CERRADA,
// cola múltiple y algoritmo SCAN de prioridad.
// [MODIFICACIÓN v4.2] Añadida validación de redirección coherente.
// ============================================================
void manejarEstadoAscensor() {
  unsigned long ahora = millis();

  switch (estadoAscensor) {

    case ASCENSOR_REPOSO:
      // === ESTADO: ASCENSOR EN REPOSO ===
      // Cabina parada en una planta, esperando comandos del usuario.
      // No se procesan movimientos hasta que haya al menos una solicitud.

      // Transición: si hay solicitudes pendientes → PUERTA_CERRADA
      if (numSolicitudes > 0) {
        transicionEstado(ASCENSOR_PUERTA_CERRADA);
      }
      break;

    case ASCENSOR_PUERTA_CERRADA:
      // === ESTADO: PUERTA CERRÁNDOSE === [MODIFICACIÓN v3.0]
      // Transición de seguridad antes de iniciar el movimiento.
      // Permite 2 segundos para que la puerta mecánica se cierre
      // completamente, evitando arranque con pasajeros entrando.

      if (tPuertaCerrada == 0) {
        tPuertaCerrada = ahora;
      }

      if (ahora - tPuertaCerrada >= TIEMPO_CIERRE_PUERTA) {
        tPuertaCerrada = 0;

        // Calcular destino prioritario con el algoritmo SCAN
        uint8_t prio = calcularPrioridad();

        if (prio != 0 && prio != plantaActual) {
          plantaDestino = prio;
          // [MODIFICACIÓN v4.3] Fijar dirección de marcha EXPLÍCITAMENTE
          // comparando destino contra planta actual física. Esta es la
          // ÚNICA función que escribe direccionActual al iniciar un viaje.
          // calcularPrioridad() ya no lo hace como efecto secundario.
          if (prio > plantaActual) {
            direccionActual = DIR_SUBIENDO;
          } else {
            direccionActual = DIR_BAJANDO;
          }
          transicionEstado(ASCENSOR_MOVIMIENTO);
        } else if (prio == plantaActual) {
          // La única solicitud es la planta actual: reabrir puertas
          eliminarSolicitud(plantaActual);
          transicionEstado(ASCENSOR_PUERTA_ABIERTA);
        } else {
          // No hay solicitudes válidas: volver a reposo
          transicionEstado(ASCENSOR_REPOSO);
        }
      }
      break;

    case ASCENSOR_MOVIMIENTO:
      // === ESTADO: ASCENSOR EN MOVIMIENTO ===
      // Cabina moviéndose hacia la planta destino.
      // Durante el movimiento se pueden recibir nuevas solicitudes;
      // el algoritmo SCAN recalcula dinámicamente la prioridad.

      // [CORRECCIÓN v4.2] Recalcular prioridad con validación de redirección
      if (numSolicitudes > 0) {
        uint8_t nuevaPrioridad = calcularPrioridad();
        // Solo redirigir si la nueva planta es diferente Y físicamente válida
        if (nuevaPrioridad != 0 &&
            nuevaPrioridad != plantaDestino &&
            esRedireccionValida(nuevaPrioridad)) {
          plantaDestino = nuevaPrioridad;
          // [BUG 4.4] Tras redirigir plantaDestino durante MOVIMIENTO, direccionActual
          //   no se actualizaba. El SCAN del siguiente ciclo usaba la dirección
          //   antigua, excluyendo o incluyendo plantas incorrectamente hasta que
          //   moverAscensor() la corregía en el ciclo de servo (65 ms después).
          //   Con colas con muchas plantas y redirecciones rápidas esto podía
          //   hacer que el SCAN eligiera destinos erróneos de forma transitoria.
          //   [FIX 2]: actualizar direccionActual inmediatamente al redirigir.
          if (nuevaPrioridad > plantaActual) direccionActual = DIR_SUBIENDO;
          else if (nuevaPrioridad < plantaActual) direccionActual = DIR_BAJANDO;
          // [CORRECCIÓN v4.2] Reset parcial de velocidad para nuevo perfil fuzzy
          velocidadActual = 0.0f;
          Serial.print(F("[SCAN] Redirigiendo hacia P"));
          Serial.println(plantaDestino);
        }
      }

      // [MODIFICACIÓN v4.0] Usar intervalo SERVO_INTERVALO en lugar de 65 fijo
      if (ahora - tUltimoServo >= SERVO_INTERVALO) {
        tUltimoServo = ahora;
        moverAscensor();
      }

      // Verificar si llegó a destino
      if (plantaActual == plantaDestino && !cabinaMov) {
        // Llegó a destino: eliminar solicitud atendida y abrir puerta
        eliminarSolicitud(plantaActual);
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
      // Cabina llegó a destino, puerta abierta para entrada/salida.
      // Permanece 3 segundos antes de pasar a PUERTA_CERRADA.

      if (tPuertaAbierta == 0) {
        tPuertaAbierta = ahora;
      }

      if (ahora - tPuertaAbierta >= 3000) {  // 3 segundos
        tPuertaAbierta = 0;
        transicionEstado(ASCENSOR_PUERTA_CERRADA);
      }

      break;

    case ASCENSOR_EMERGENCIA:
      // === ESTADO: EMERGENCIA ===
      // Parada de seguridad por alguna condición anómala.

      // Detener todo movimiento
      cabinaMov = false;

      // Mensaje periódico por Serial
      if (millis() % 1000 < 100) {
        Serial.println(F("¡EMERGENCIA! Ascensor detenido."));
      }

      // Reset automático tras 10 segundos
      if (tEmergencia == 0) {
        tEmergencia = ahora;
      }

      if (ahora - tEmergencia >= 10000) {  // 10 segundos de emergencia
        tEmergencia = 0;
        Serial.println(F("Reset automático de emergencia."));
        transicionEstado(ASCENSOR_REPOSO);
      }

      break;

    case ASCENSOR_MANTENIMIENTO:
      // === ESTADO: MANTENIMIENTO ===
      // Estado especial para debug, calibración o mantenimiento.

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
// mostrarUID()
// ------------
// Imprime por Serial el UID en formato hexadecimal legible.
//
// @param uidBuffer  Puntero al array de bytes del UID.
// @param bufferSize Tamaño del UID en bytes.
// ============================================================
void mostrarUID(byte *uidBuffer, byte bufferSize) {
  for (uint8_t i = 0; i < bufferSize; i++) {
    if (uidBuffer[i] < 0x10) DBG_PRINT(F("0"));
    DBG_PRINT(uidBuffer[i], HEX);
    if (i < bufferSize - 1) DBG_PRINT(F(":"));
  }
}

// ============================================================
// moverAscensor()
// ---------------
// [MODIFICACIÓN v4.0 / v4.2] Control de movimiento con lógica
// difusa (fuzzy logic). 
//
// CAMBIOS CRÍTICOS v4.2:
//   1. Clamping estricto de anguloServo a [0, 180] para evitar
//      que el servo gire fuera de sus límites físicos.
//   2. La dirección de movimiento se determina comparando
//      anguloServo contra destino en CADA ciclo, no usando
//      plantaDestino > plantaActual (que podía quedar obsoleto).
//   3. Al llegar exactamente al destino, se actualiza plantaActual
//      y se limpia velocidadActual.
//   4. Detección de planta por rangos (más robusto con velocidad variable).
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
    // [CORRECCIÓN v4.2] Actualizar planta lógica de forma robusta por rangos
    plantaActual = determinarPlantaDesdeAngulo(anguloServo);
  }
}

// ============================================================
// nombreEstado()
// -------------
// Devuelve el nombre textual de un estado para logs por Serial.
// [MODIFICACIÓN v3.0] Añadido PUERTA_CERRADA.
// ============================================================
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

// ============================================================
// pcf8574_read (PCF 8574 I2C Read Function)
//
// Lee un byte del PCF8574 via I2C
// [SIN MODIFICACIÓN v3.0]
// ============================================================
uint8_t pcf8574_read() {
  uint8_t valor = 0xFF;
  Wire.requestFrom(PCF8574_ADDR, (uint8_t)1);
  if (Wire.available()) {
    valor = Wire.read();
  }
  return valor;
}

// ============================================================
// pcf8574_write (PCF 8574 I2C Write Function)
//
// Escribe un byte al PCF8574 via I2C
// [SIN MODIFICACIÓN v3.0]
// ============================================================
void pcf8574_write(uint8_t valor) {
  Wire.beginTransmission(PCF8574_ADDR);
  Wire.write(valor);
  Wire.endTransmission();
}

// ============================================================
// tarjetaAutorizada()
// -------------------
// Compara el UID de la tarjeta detectada contra la lista blanca
// de tarjetas registradas en TARJETAS_AUTORIZADAS.
//
// @param uidBuffer  Puntero al array de bytes del UID leído.
// @param bufferSize Tamaño del UID en bytes.
// @return  true si el UID coincide con alguna tarjeta registrada.
// ============================================================
bool tarjetaAutorizada(byte *uidBuffer, byte bufferSize) {
  // Solo se admiten UIDs de 4 bytes (MIFARE Classic 1K estándar)
  if (bufferSize != 4) return false;

  for (uint8_t i = 0; i < NUM_TARJETAS_REG; i++) {
    bool coincide = true;
    for (uint8_t j = 0; j < 4; j++) {
      if (uidBuffer[j] != TARJETAS_AUTORIZADAS[i][j]) {
        coincide = false;
        break;
      }
    }
    if (coincide) return true;
  }
  return false;
}

// ============================================================
// Función de transición entre estados
// Maneja el cambio de estado y ejecuta acciones de entrada/salida
// [MODIFICACIÓN v3.0] Añadido manejo de PUERTA_CERRADA y reset
// de temporizadores globales en cada transición.
// [MODIFICACIÓN v4.2] Añadido reset de velocidadActual.
// ============================================================
void transicionEstado(EstadoAscensor nuevoEstado) {
  // Acciones de salida del estado actual
  switch (estadoAscensor) {
    case ASCENSOR_MOVIMIENTO:
      // Al salir de movimiento, asegurar que cabina esté detenida
      cabinaMov = false;
      break;

    case ASCENSOR_EMERGENCIA:
      // Al salir de emergencia, reset indicadores y temporizador
      tEmergencia = 0;
      Serial.println(F("Saliendo de modo emergencia"));
      break;

    case ASCENSOR_PUERTA_ABIERTA:
      // Al salir de puerta abierta, resetear temporizador
      tPuertaAbierta = 0;
      break;

    case ASCENSOR_PUERTA_CERRADA:
      // Al salir de puerta cerrada, resetear temporizador
      tPuertaCerrada = 0;
      break;

    default:
      break;
  }

  // Cambiar estado
  EstadoAscensor estadoPrevio = estadoAscensor;
  estadoAscensor = nuevoEstado;

  // Acciones de entrada al nuevo estado
  switch (nuevoEstado) {
    case ASCENSOR_REPOSO:
      Serial.print(F("Estado: REPOSO en planta "));
      Serial.println(plantaActual);
      // [MODIFICACIÓN v3.0] Resetear dirección al detenerse
      direccionActual = DIR_NINGUNA;
      // [MODIFICACIÓN v4.2] Resetear velocidad fuzzy al detenerse
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

  // Log de transición
  if (estadoPrevio != nuevoEstado) {
    Serial.print(F("Transicion: "));
    Serial.print(nombreEstado(estadoPrevio));
    Serial.print(F(" -> "));
    Serial.println(nombreEstado(nuevoEstado));
  }
}
/* Fin de archivo */