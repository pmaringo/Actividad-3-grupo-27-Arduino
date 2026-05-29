# Análisis de costes del desarrollo

**Proyecto:** Ascensor Inteligente ACME S.A. — Actividad 3
**Equipo:** Grupo 27
**Documento:** Aproximación al análisis de costes y cálculo de PVP
**Versión:** 1.0

> **Nota sobre las cifras.** Los importes reflejados en el presente análisis son
> **simulados con fines docentes**, ajustados a rangos representativos del sector
> electrónico de consumo en el mercado europeo. La finalidad del documento no es
> sustituir un plan financiero formal —ejercicio que exigiría tarifas reales,
> presupuestos vinculantes de fabricantes y un estudio de mercado específico— sino
> dejar constancia documental del análisis efectuado y de la metodología empleada.

---

## 1. Introducción y metodología

El presente análisis aborda la estimación de los costes asociados a la
industrialización y comercialización del sistema descrito en la memoria técnica, desde
la perspectiva de un eventual escalado del prototipo a producto comercial. La
metodología empleada es la habitual en el sector de la electrónica de consumo: se
distinguen los **costes no recurrentes** (Non-Recurring Engineering, NRE), de carácter
puntual, de los **costes recurrentes** asociados a cada unidad producida. A partir de
esta descomposición se obtiene el coste total por unidad para distintos volúmenes de
producción, sobre el cual se aplica la regla de fijación del Precio de Venta al Público
(PVP) descrita en el apartado 2.

El alcance del análisis comprende únicamente los costes asociados al producto físico
y a su puesta en mercado. Quedan fuera del alcance los costes operativos de la
estructura empresarial (mantenimiento de oficinas, salarios indirectos, fiscalidad
societaria) que dependen del modelo de negocio del operador.

---

## 2. Regla de fijación del PVP

La determinación del PVP se realiza conforme a la regla habitualmente denominada
**«regla del dedo gordo»** (rule of thumb) en el sector:

> **El PVP debe situarse, como mínimo, en 2,5 veces el coste de fabricación
> unitario del producto.**

El factor 2,5 absorbe el conjunto de partidas que median entre el coste de producción y
el precio final percibido por el usuario:

- Margen del fabricante.
- Margen del distribuidor (peaje habitualmente cifrado entre el 30 % y el 45 % del PVP).
- Margen del minorista (canal de venta final).
- Costes logísticos no imputados directamente al coste unitario.
- Provisión para garantía y servicio postventa.
- Provisión para campañas de marketing y promoción.
- Costes financieros (capital circulante, descuento de pagarés, etc.).

Como referencia ilustrativa, si el coste de fabricación por unidad para una tirada de
1 000 unidades se estima en 15 €, el PVP mínimo viable se situaría en
15 × 2,5 = 37,5 €. Esta cifra constituye el umbral por debajo del cual la
comercialización del producto comprometería la sostenibilidad económica del proyecto.

---

## 3. Costes No Recurrentes (NRE)

Los costes no recurrentes se generan una sola vez a lo largo del ciclo de vida del
producto, con independencia del volumen producido. Conceptualmente se asocian a la
**fase de desarrollo** del proyecto.

### 3.1. Horas de ingeniería

Se ha estimado la dedicación necesaria para cada fase del desarrollo, considerando un
equipo de ingeniería con perfil mixto (electrónica y firmware) y un perfil de soporte
documental. La tarifa horaria aplicada se sitúa en valores representativos del mercado
español para profesionales con experiencia media (3 a 5 años):

| Fase                                | Horas | Tarifa (€/h) | Coste (€)    |
| ----------------------------------- | ----- | ------------ | ------------ |
| Análisis de requisitos              |   40  |      45      |    1 800,00  |
| Diseño electrónico                  |   60  |      45      |    2 700,00  |
| Diseño de PCB                       |   30  |      45      |    1 350,00  |
| Desarrollo de firmware              |  120  |      45      |    5 400,00  |
| Pruebas y depuración                |   80  |      45      |    3 600,00  |
| Documentación técnica               |   50  |      35      |    1 750,00  |
| **Subtotal horas de ingeniería**    | **380** |            | **16 600,00** |

### 3.2. Diseño industrial

| Concepto                                    | Coste (€)  |
| ------------------------------------------- | ---------- |
| Diseño mecánico de la envolvente (20 h × 45 €/h) |   900,00 |
| **Subtotal diseño industrial**              | **900,00** |

El diseño no contempla utillaje de inyección, al optarse por una envolvente comercial
estándar disponible en proveedor con mecanizado a medida. Una decisión de fabricar
envolvente propia mediante inyección de plástico introduciría un coste de utillaje
(molde) habitualmente comprendido entre los 8 000 € y los 25 000 €, que debería ser
amortizado por separado.

### 3.3. Certificación del producto

El producto, al incorporar fuente de alimentación conmutada y emisores/receptores
inalámbricos (radiofrecuencia RFID a 13,56 MHz e infrarrojos), debe acreditar el
cumplimiento de la normativa europea aplicable para su comercialización en el Espacio
Económico Europeo (marcado CE).

| Concepto                                          | Coste (€)    |
| ------------------------------------------------- | ------------ |
| Pre-cumplimiento EMC (laboratorio acreditado)     |   2 500,00   |
| Certificación EMC (EN 55032 / EN 55035)           |   4 500,00   |
| Seguridad eléctrica (EN 62368-1, marcado CE)      |   2 800,00   |
| **Subtotal certificación**                        | **9 800,00** |

### 3.4. Herramentaje y puesta en producción (tooling)

| Concepto                                          | Coste (€)    |
| ------------------------------------------------- | ------------ |
| Programador de producción (jig de programación ISP) |     500,00 |
| Plantillas de test funcional                      |   1 200,00   |
| Setup de línea de producción (primera serie)      |     800,00   |
| **Subtotal tooling**                              | **2 500,00** |

### 3.5. Iteraciones de prototipo

| Concepto                                          | Coste (€)    |
| ------------------------------------------------- | ------------ |
| Tres iteraciones de prototipo (3 × 250 €)         |     750,00   |
| **Subtotal prototipos**                           |   **750,00** |

### 3.6. Total NRE

| Categoría                       | Coste (€)     |
| ------------------------------- | ------------- |
| Horas de ingeniería             |    16 600,00  |
| Diseño industrial               |       900,00  |
| Certificación                   |     9 800,00  |
| Tooling                         |     2 500,00  |
| Prototipos                      |       750,00  |
| **TOTAL NRE**                   | **30 550,00** |

---

## 4. Costes Recurrentes por unidad

Los costes recurrentes se generan en cada unidad producida. Se calculan a partir del
BOM total documentado en [`BOM.md`](./BOM.md), incrementado con las operaciones de
fabricación, prueba, embalaje y logística de entrada.

### 4.1. Desglose para una tirada de 1 000 unidades

| Concepto                                          | Coste (€/u) |
| ------------------------------------------------- | ----------- |
| Componentes (BOM total a 1 000 u, ver `BOM.md`)   |    29,95    |
| Ensamblaje (SMT + THT, mano de obra y proceso)    |     4,50    |
| Test funcional (ICT + funcional, jig dedicado)    |     1,50    |
| Embalaje y manual de usuario                      |     2,20    |
| Transporte de entrada (logística inbound)         |     0,80    |
| **Coste recurrente por unidad**                   | **38,95**   |

### 4.2. Otras partidas recurrentes (no imputadas al coste unitario)

Las siguientes partidas se modelan habitualmente como porcentaje del ingreso por venta,
no como coste fijo por unidad. Su impacto se absorbe a través del factor 2,5 de la
regla de fijación del PVP, conforme a lo expuesto en el apartado 2.

| Partida                              | Porcentaje habitual sobre PVP |
| ------------------------------------ | ----------------------------- |
| Distribución (mayorista + minorista) |          35 % – 45 %          |
| Marketing y promoción                |           5 % – 10 %          |
| Costes financieros (working capital) |           3 % –  5 %          |
| Soporte postventa y garantía         |           2 % –  3 %          |

---

## 5. Coste total por unidad a distintos volúmenes

El coste total por unidad se calcula sumando, al coste recurrente, la amortización por
unidad de los costes no recurrentes. Esta amortización depende directamente del
volumen objetivo de producción:

| Volumen | NRE/u (€) | Coste recurrente/u (€) | **Coste total/u (€)** |
| ------- | --------- | ---------------------- | --------------------- |
|   1 000 |   30,55   |         38,95          |       **69,50**       |
|   2 500 |   12,22   |         38,95          |       **51,17**       |
|   5 000 |    6,11   |         38,95          |       **45,06**       |
|  10 000 |    3,06   |         38,95          |       **42,01**       |
|  25 000 |    1,22   |         38,95          |       **40,17**       |

La curva pone de manifiesto un efecto característico de los desarrollos
electrónicos: a partir de un determinado volumen (en este caso, en torno a las
5 000–10 000 unidades) el coste total por unidad tiende asintóticamente al coste
recurrente, ya que la amortización del NRE se hace marginal. La decisión sobre el
volumen objetivo de producción condiciona, por tanto, de forma determinante la
estructura de precios del producto.

---

## 6. Cálculo del PVP

Aplicando la regla del apartado 2 (PVP ≥ 2,5 × coste total/u) se obtienen los
siguientes precios mínimos para cada escenario de volumen:

| Volumen | Coste total/u (€) | PVP mínimo (€) | PVP comercial sugerido (€) |
| ------- | ----------------- | -------------- | -------------------------- |
|   1 000 |      69,50        |     173,75     |          179,99            |
|   2 500 |      51,17        |     127,93     |          129,99            |
|   5 000 |      45,06        |     112,65     |          119,99            |
|  10 000 |      42,01        |     105,01     |          109,99            |
|  25 000 |      40,17        |     100,43     |           99,99            |

El **PVP comercial sugerido** redondea al alza el PVP mínimo conforme a las prácticas
habituales de fijación de precio psicológico (terminaciones en `,99`). En el escenario
de mayor volumen (25 000 u) se opta por situar el PVP en 99,99 €, ligeramente por
debajo del mínimo teórico, asumiendo que el mayor volumen de ventas compensa la menor
unidad de margen unitario y que el factor 2,5 incorpora cierto colchón sobre los costes
no imputados.

---

## 7. Análisis de sensibilidad y posicionamiento

### 7.1. Punto de equilibrio

Suponiendo que el PVP comercial efectivo del fabricante hacia el distribuidor (precio
de salida de fábrica, ex-works) se sitúa en torno al 55 % del PVP final —repartiendo el
45 % restante entre canal y campaña—, el ingreso neto por unidad y los volúmenes de
equilibrio para amortizar los 30 550 € del NRE son los siguientes:

| Volumen objetivo | PVP final sugerido | Ingreso ex-works (55 %) | Margen sobre coste recurrente | Unidades hasta equilibrio NRE |
| ---------------- | ------------------ | ----------------------- | ----------------------------- | ----------------------------- |
|   1 000          |      179,99 €      |          98,99 €        |            60,04 €            |             509               |
|   5 000          |      119,99 €      |          65,99 €        |            27,04 €            |           1 130               |
|  10 000          |      109,99 €      |          60,49 €        |            21,54 €            |           1 418               |

El cálculo evidencia que el escenario de 1 000 unidades amortiza el NRE con apenas un
51 % del volumen objetivo —margen confortable—, mientras que el escenario de 10 000 u
requiere alcanzar aproximadamente el 14 % del volumen para entrar en zona de beneficio.
Ambos resultados son compatibles con un caso de negocio viable, si bien el escenario de
mayor volumen presenta mayor riesgo de inventario y mayor exigencia de capital
circulante.

### 7.2. Posicionamiento competitivo

Los precios calculados sitúan el producto en un rango compatible con el segmento de
**maquetas educativas y kits de demostración para formación técnica** (PVP típico
entre 80 € y 200 €). Una hipotética orientación a producto industrial completo
—controlador de ascensor real— exigiría un PVP sustancialmente superior, debido a la
necesidad de componentes industrializados (servos, sensores y módulos certificados para
uso industrial, comunicaciones bus de campo, certificaciones específicas de seguridad
funcional) y a un volumen objetivo muy inferior.

### 7.3. Valor aportado al cliente

Conforme al planteamiento de la regla del dedo gordo, el ejercicio de fijación del PVP
no se agota en el cumplimiento del margen mínimo. Es necesario contrastar adicionalmente
que el valor percibido por el cliente —en términos de funcionalidad, calidad,
documentación y soporte— justifica el precio establecido frente a la oferta de la
competencia. En el caso de un kit educativo, el conjunto de prestaciones del sistema
documentado en la memoria técnica (FSM, control PID, lógica difusa, algoritmo SCAN,
control de acceso RFID y arquitectura modular bien documentada) constituye un argumento
diferencial frente a soluciones equivalentes de menor recorrido pedagógico.

---

## 8. Conclusiones del análisis

1. El coste recurrente por unidad para una tirada de 1 000 unidades asciende a
   **38,95 €**, importe coherente con el rango habitual para sistemas embebidos de
   complejidad media en el mercado europeo.
2. El coste de desarrollo no recurrente (NRE) asciende a **30 550 €**, con la
   certificación CE (EMC + seguridad eléctrica) y las horas de ingeniería como
   partidas dominantes (32 % y 54 % del total, respectivamente).
3. La aplicación de la regla PVP ≥ 2,5 × coste sitúa el **PVP mínimo viable** en un
   rango comprendido entre **100 € y 175 €** según el volumen objetivo, y permite
   establecer un **PVP comercial sugerido** entre **99,99 € y 179,99 €**.
4. El escenario de volumen óptimo desde la perspectiva del margen unitario se sitúa en
   las **5 000 unidades**, punto a partir del cual la amortización del NRE deja de
   constituir el factor dominante del coste total.
5. La estructura de costes obtenida es coherente con un posicionamiento del producto
   en el segmento educativo / de demostración técnica, conclusión que deberá
   contrastarse mediante un estudio de mercado específico en una eventual fase de
   industrialización.

---

## 9. Referencias

- Lista de componentes (BOM): [`BOM.md`](./BOM.md)
- Memoria técnica del proyecto: `Memoria_Actividad3_Grupo27.docx`
- Material docente de referencia: Tema 10 — *Nuevas tecnologías en redes
  inteligentes*, sección «Aproximación al análisis de costes de un desarrollo».
