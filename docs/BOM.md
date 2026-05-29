# BOM — Lista de componentes

**Proyecto:** Ascensor Inteligente ACME S.A. — Actividad 3
**Equipo:** Grupo 27
**Documento:** Bill of Materials (BOM) y escalado de precios por volumen
**Versión:** 1.0

> **Nota sobre los precios.** Los precios unitarios y de volumen reflejados en este
> documento son **simulados con fines docentes**, y se proporcionan únicamente para
> dejar constancia del análisis. Las cifras se han establecido a partir de rangos
> realistas observados en el mercado para componentes equivalentes (proveedores tipo
> Digi-Key, Mouser, LCSC y distribuidores generalistas), pero no constituyen una
> cotización vinculante ni una referencia comercial. Las decisiones de aprovisionamiento
> en un eventual escalado a producción exigirían una cotización formal por parte de los
> proveedores correspondientes.

---

## 1. Alcance del BOM

El presente documento relaciona la totalidad de los componentes electrónicos,
electromecánicos y de envolvente necesarios para la materialización del prototipo del
sistema descrito en la memoria técnica. Se distingue entre:

- **BOM electrónico (núcleo del circuito):** componentes activos, pasivos y módulos
  comerciales que conforman el esquema eléctrico del sistema.
- **BOM de producción:** elementos adicionales requeridos para la fabricación de una
  unidad comercial (PCB, envolvente, alimentación, conectores y accesorios incluidos en
  el embalaje).

Los precios se expresan en euros (€) y se presentan en tres escalas representativas:

| Escala  | Volumen      | Canal habitual                          |
| ------- | ------------ | --------------------------------------- |
| **1u**  | 1 unidad     | Compra retail (prototipado individual)  |
| **100u**| 100 unidades | Lote pequeño (preserie / piloto)        |
| **1000u**| 1 000 unidades | Producción en volumen (serie)          |

---

## 2. BOM electrónico (núcleo del circuito)

### 2.1. Unidad de control

| Ref. | Componente                | Descripción                              | Cant. | €/u (1u) | €/u (100u) | €/u (1000u) |
| ---- | ------------------------- | ---------------------------------------- | ----- | -------- | ---------- | ----------- |
| U1   | Arduino UNO R3            | Placa de desarrollo basada en ATmega328P |   1   |   12,50  |    8,50    |    5,50     |

### 2.2. Sensores

| Ref. | Componente             | Descripción                                    | Cant. | €/u (1u) | €/u (100u) | €/u (1000u) |
| ---- | ---------------------- | ---------------------------------------------- | ----- | -------- | ---------- | ----------- |
| S1   | DHT22 (AM2302)         | Sensor digital de temperatura y humedad        |   1   |   6,20   |    4,30    |    2,80     |
| S2   | HC-SR501               | Sensor PIR de presencia                        |   1   |   2,80   |    1,80    |    1,10     |
| S3   | TSOP38238              | Receptor IR (38 kHz, demodulado)               |   1   |   0,95   |    0,60    |    0,35     |
| S4   | MFRC522                | Lector RFID 13,56 MHz (módulo SPI)             |   1   |   4,50   |    3,20    |    2,10     |
| S5   | Módulo LDR             | Fotorresistencia con divisor resistivo         |   1   |   1,80   |    1,20    |    0,75     |

### 2.3. Actuadores

| Ref. | Componente             | Descripción                                    | Cant. | €/u (1u) | €/u (100u) | €/u (1000u) |
| ---- | ---------------------- | ---------------------------------------------- | ----- | -------- | ---------- | ----------- |
| A1   | SG90                   | Micro-servomotor (180°, PWM)                   |   1   |   3,20   |    2,20    |    1,45     |
| D1–D8| LED 5 mm amarillo      | LED indicador (iluminación de cabina)          |   8   |   0,12   |    0,06    |    0,03     |
| D9   | LED 5 mm azul          | LED indicador EV_FRIO                          |   1   |   0,20   |    0,10    |    0,05     |
| D10  | LED 5 mm rojo          | LED indicador EV_CALOR                         |   1   |   0,12   |    0,06    |    0,03     |

### 2.4. Interfaz de usuario — entradas

| Ref. | Componente             | Descripción                                    | Cant. | €/u (1u) | €/u (100u) | €/u (1000u) |
| ---- | ---------------------- | ---------------------------------------------- | ----- | -------- | ---------- | ----------- |
| SW1–SW5 | Pulsador táctil 6×6 mm | Pulsador momentáneo, montaje en panel       |   5   |   0,30   |    0,15    |    0,07     |

### 2.5. Interfaz de usuario — visualización

| Ref. | Componente             | Descripción                                    | Cant. | €/u (1u) | €/u (100u) | €/u (1000u) |
| ---- | ---------------------- | ---------------------------------------------- | ----- | -------- | ---------- | ----------- |
| LCD1 | LCD 1602 + I²C         | Display 16×2 con backpack PCF8574T (0x27)      |   1   |   5,20   |    3,80    |    2,40     |

### 2.6. Expansión de E/S y registros

| Ref. | Componente             | Descripción                                    | Cant. | €/u (1u) | €/u (100u) | €/u (1000u) |
| ---- | ---------------------- | ---------------------------------------------- | ----- | -------- | ---------- | ----------- |
| U2   | PCF8574                | Expansor de E/S de 8 bits sobre bus I²C (0x20) |   1   |   1,80   |    1,10    |    0,65     |
| U3   | 74HC595                | Registro de desplazamiento de 8 bits (serie)   |   1   |   0,55   |    0,32    |    0,18     |

### 2.7. Componentes pasivos

| Ref. | Componente             | Descripción                                    | Cant. | €/u (1u) | €/u (100u) | €/u (1000u) |
| ---- | ---------------------- | ---------------------------------------------- | ----- | -------- | ---------- | ----------- |
| R1–R8| Resistencia 220 Ω ¼ W  | Limitación de corriente para los 8 LEDs amarillos |   8 |   0,04   |    0,02    |    0,01     |
| R9   | Resistencia 220 Ω ¼ W  | Limitación de corriente LED EV_FRIO            |   1   |   0,04   |    0,02    |    0,01     |
| R10  | Resistencia 220 Ω ¼ W  | Limitación de corriente LED EV_CALOR           |   1   |   0,04   |    0,02    |    0,01     |

### 2.8. Subtotal del BOM electrónico

| Volumen | Subtotal por unidad |
| ------- | ------------------- |
| 1u      | **42,68 €**         |
| 100u    | **28,61 €**         |
| 1000u   | **18,05 €**         |

---

## 3. BOM de producción (envolvente, PCB y accesorios)

Los siguientes elementos no forman parte del esquema eléctrico, pero son necesarios para
la materialización de una unidad comercial del sistema.

| Ref. | Componente                | Descripción                                          | Cant. | €/u (1u) | €/u (100u) | €/u (1000u) |
| ---- | ------------------------- | ---------------------------------------------------- | ----- | -------- | ---------- | ----------- |
| PCB1 | PCB principal             | Placa de circuito impreso 100×80 mm, FR-4, 2 capas   |   1   |  12,00   |    4,50    |    1,20     |
| ENC1 | Envolvente ABS            | Carcasa de ABS con apertura para LCD y pulsadores    |   1   |   8,50   |    5,20    |    2,80     |
| PSU1 | Alimentación              | Adaptador 5 V / 2 A con conector USB-B               |   1   |   7,50   |    5,20    |    3,40     |
| CON1 | Conectores y header pins  | Tira de pines macho/hembra 2,54 mm, conectores Molex |   1   |   2,20   |    1,40    |    0,85     |
| TAG1 | Tarjetas MIFARE 1K        | Llavero/tarjeta RFID 13,56 MHz (incluidas en kit)    |   2   |   1,20   |    0,85    |    0,55     |
| IR1  | Mando IR (NEC, 21 teclas) | Mando a distancia para llamadas remotas              |   1   |   2,80   |    1,90    |    1,25     |
| WH1  | Wiring harness            | Cableado de interconexión preensamblado              |   1   |   3,50   |    2,20    |    1,30     |

### 3.1. Subtotal del BOM de producción

| Volumen | Subtotal por unidad |
| ------- | ------------------- |
| 1u      | **38,90 €**         |
| 100u    | **22,10 €**         |
| 1000u   | **11,90 €**         |

---

## 4. BOM total — Resumen

| Concepto                  | 1u       | 100u     | 1000u    |
| ------------------------- | -------- | -------- | -------- |
| BOM electrónico           | 42,68 €  | 28,61 €  | 18,05 €  |
| BOM de producción         | 38,90 €  | 22,10 €  | 11,90 €  |
| **Total componentes/u**   | **81,58 €** | **50,71 €** | **29,95 €** |

### 4.1. Curva de aprendizaje y descuento por volumen

El abaratamiento progresivo del coste unitario al incrementar el volumen de producción
obedece a tres factores principales:

1. **Descuentos escalonados de los proveedores** sobre componentes activos y módulos
   comerciales, particularmente acusados en el rango de 100 a 1 000 unidades.
2. **Amortización de costes fijos asociados al PCB** (NRE de fabricación, fotolitos,
   máscaras de soldadura), cuyo impacto unitario decrece de forma marcada con el
   volumen.
3. **Optimización del proceso de ensamblaje**, con una reducción del coste de mano de
   obra y de los tiempos de configuración (setup) al producir lotes mayores.

La reducción global obtenida entre la compra unitaria y la producción a 1 000 unidades
es del **63,3 %**, valor coherente con los rangos habituales para productos electrónicos
de complejidad media.

---

## 5. Trazabilidad con el esquema eléctrico

La numeración de referencias empleada en el presente BOM se corresponde con las
designaciones del esquema eléctrico del proyecto (fichero `diagram.json` y sección 2 de
la memoria técnica), conforme a la siguiente convención:

- **U**: circuitos integrados y módulos activos.
- **S**: sensores.
- **A**: actuadores electromecánicos.
- **D**: diodos LED.
- **SW**: pulsadores y conmutadores.
- **R**: resistencias.
- **PCB, ENC, PSU, CON, TAG, IR, WH**: elementos de producción y accesorios.

---

## 6. Referencias

- Memoria técnica del proyecto: `Memoria_Actividad3_Grupo27.docx`
- Análisis de costes de desarrollo: [`ANALISIS_COSTES.md`](./ANALISIS_COSTES.md)
- Esquema eléctrico (formato Wokwi): `diagram.json`
- Código fuente del firmware: `main.cpp`
