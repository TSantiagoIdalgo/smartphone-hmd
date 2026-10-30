# Plan inicial: smartphone como visor HMD para SteamVR

Objetivo: que SteamVR vea el smartphone como un visor (HMD), con efecto 3D estéreo,
vista por ojo ajustable para un headset barato y recentrado de la mirada.

## 1. Arquitectura

```
 ┌──────────────── Android (Kotlin) ────────────────┐         ┌──────────── PC Windows ─────────────┐
 │ Sensores crudos: giroscopio, acelerómetro,       │  UDP    │ Driver de SteamVR (DLL, C++)         │
 │ magnetómetro  ──────────────────────────────────────9944──▶│  · recibe y limpia los datos         │
 │ Botón de volumen = recentrar                     │         │  · fusión → orientación de la cabeza │
 │                                                  │         │  · declara un HMD estéreo a SteamVR  │
 │ Pantalla: video H.264 ◀─────────────────────────────9945───│  · captura lo que renderiza SteamVR, │
 │  · decodifica (MediaCodec)                       │  UDP    │    lo codifica en H.264 y lo envía   │
 │  · dibuja cada ojo con tamaño/posición ajustable │         └──────────────────────────────────────┘
 │  · corrige la distorsión de la lente (shader)    │
 └──────────────────────────────────────────────────┘
```

Dos piezas, nada más:

- **`android/`**: app Kotlin. Envía los sensores tal cual llegan y muestra la imagen que recibe.
- **`driver/`**: driver de SteamVR en C++ (DLL). Es la "app de escritorio": SteamVR lo carga solo
  al arrancar, así que no hace falta un programa aparte ni comunicación entre procesos.

## 2. Decisiones tomadas (la opción más simple en cada caso)

| Tema | Elección | Por qué |
|---|---|---|
| App de escritorio | Todo dentro del driver de SteamVR, sin `.exe` aparte | SteamVR exige un driver en C++ igualmente; un segundo programa solo añade IPC. |
| Seguimiento | Rotación (3DoF) con altura fija. Opcional: posición de la cabeza con una luz en el visor que sigue PSMoveServiceEx (6DoF) | Un móvil no puede medir posición de forma fiable sin cámaras. Con la PS Eye de los PS Move, la posición viene de la cámara y la rotación sigue viniendo del móvil. |
| Fusión de sensores | En la PC, filtro complementario (Mahony) sobre los datos crudos | Cumple el requisito de enviar los datos "puros" y limpiarlos en la PC, con menos código que otras opciones. |
| Imagen hacia el móvil | Video H.264 por UDP, codificado con Media Foundation (incluido en Windows, usa la GPU) y decodificado con MediaCodec | Sin dependencias externas y funciona con GPU NVIDIA, AMD o Intel. |
| Conexión | Wi-Fi. También funciona por cable USB con "Anclaje USB" activado, con el mismo código | El cable da menos latencia cuando el Wi-Fi no alcanza. |
| Configuración de red | En el móvil se escribe la IP de la PC; la PC responde a la dirección de la que le llegan los sensores | Nada que configurar en la PC. |
| Ajuste por ojo y lentes | En el móvil, con deslizadores y vista previa en vivo, guardado en el teléfono | Se ve el efecto al instante, sin reiniciar SteamVR. |
| Recentrar | Botón de volumen del móvil (funciona con el teléfono dentro del visor) y una tecla en la PC | No requiere tocar la pantalla. |

## 3. Cómo se cumple cada requisito

1. **App Android en Kotlin que envía los sensores crudos por UDP.** Lee `TYPE_GYROSCOPE`,
   `TYPE_ACCELEROMETER` y `TYPE_MAGNETIC_FIELD` a la máxima frecuencia y manda un paquete por cada
   muestra del giroscopio con los últimos valores de los tres sensores, su marca de tiempo y un
   número de secuencia. No filtra nada.
2. **La PC recibe, limpia y entrega a SteamVR.** El driver:
   - descarta paquetes duplicados, desordenados o con valores imposibles (NaN, giro > 35 rad/s);
   - estima el sesgo del giroscopio cuando el móvil está quieto;
   - fusiona giroscopio + acelerómetro con un filtro complementario (el magnetómetro se envía, pero
     solo se usará si la deriva horizontal molesta);
   - convierte los ejes del móvil (en horizontal dentro del visor) a los ejes de SteamVR;
   - entrega también la velocidad angular, para que SteamVR prediga la orientación y compense la latencia;
   - informa la pose a SteamVR como un HMD.
3. **Vista por ojo redimensionable.** El móvil recibe la imagen lado a lado (ojo izquierdo | ojo
   derecho) y dibuja cada mitad en un rectángulo configurable: ancho, alto, separación entre centros
   y desplazamiento vertical, en milímetros de pantalla. Además, dos coeficientes de distorsión
   (k1, k2) para las lentes. Todo se guarda en el teléfono.
4. **3D desde el primer momento.** El driver declara dos ojos con su distancia interpupilar (IPD) y
   su campo de visión, así que SteamVR renderiza cada ojo desde su propia posición. El estéreo ya se
   ve en la ventana espejo de SteamVR desde el hito 1, antes de que llegue el video al móvil.
5. **Calibrar hacia dónde se mira.** Al recentrar, el driver guarda la orientación horizontal actual
   (yaw) y la toma como "frente". La inclinación la corrige el acelerómetro solo.

## 4. Protocolo UDP (versión 1)

Todos los números en little-endian. Puerto 9944 en la PC (móvil → PC) y 9945 en el móvil (PC → móvil).

**Móvil → PC**

| Campo | Tipo | Notas |
|---|---|---|
| magic | u32 | `'HMD1'` |
| type | u8 | 1 = sensores, 2 = recentrar, 3 = pedir fotograma clave, 4 = buscar la PC, 6 = latencia |
| reserved | u8 × 3 | 0 |
| seq | u32 | crece en cada paquete |
| timestamp_ns | i64 | marca de tiempo del sensor (`SensorEvent.timestamp`) |
| gyro x,y,z | f32 × 3 | rad/s |
| accel x,y,z | f32 × 3 | m/s² |
| mag x,y,z | f32 × 3 | µT (0 si no hay) |

Tamaño: 56 bytes. Los paquetes de tipo 2, 3 y 4 llevan solo los primeros 20 bytes. El de tipo 6
lleva tras ellos la latencia medida en milisegundos (f32, 24 bytes en total; 0 = no anticipar).

**Búsqueda de la PC:** el móvil envía el tipo 4 por difusión (broadcast) al puerto 9944 en cada red
en la que está (Wi-Fi y anclaje USB), una vez por segundo. El driver responde a quien lo envió con
una cabecera de tipo 5 (`'HMD1'`, tipo 5, 20 bytes), y el móvil toma la IP de esa respuesta.

**PC → móvil (video):** cada fotograma H.264 se parte en trozos de hasta 1400 bytes con una cabecera
de 20 bytes: `'HMV1'`, número de fotograma (u32), índice del trozo (u16), total de trozos (u16),
banderas (u8, bit 0 = fotograma clave), los 24 bits bajos del seq del último paquete de sensores
que tenía la PC cuando SteamVR dibujó ese fotograma (u24, 0 = desconocido), ancho (u16) y alto
(u16). Si el móvil pierde un trozo, descarta ese fotograma y pide uno clave (tipo 3).

**Latencia:** el móvil guarda cuándo envió cada seq y, al mostrar un fotograma, mide el tiempo desde
el envío del seq que lleva. Envía el promedio con el tipo 6 cada segundo, y el driver anticipa la
orientación ese tiempo menos los 11 ms que ya cubre SteamVR (hasta 80 ms).

## 5. Estructura del repositorio

```
android/           app Kotlin (Gradle)
driver/            driver de SteamVR en C++ (CMake + MSVC, cabeceras de OpenVR)
  src/             receptor UDP, filtro, pose, pantalla virtual, codificador
  tests/           pruebas del filtro y del parser, se ejecutan también en Linux
tools/sim/         simulador que envía paquetes como si fuera el móvil
docs/              este plan y el protocolo
.github/workflows/ compila el APK y la DLL en cada push
```

## 6. Hitos

Cada hito termina con algo que se puede probar.

| # | Entrega | Qué se prueba en tu PC |
|---|---|---|
| 0 | Estructura del repo + GitHub Actions que compilan el APK y la DLL | Nada; solo descargar los artefactos |
| 1 | App envía sensores; driver recibe, filtra y aparece como HMD estéreo; recentrar | Instalar el driver, abrir SteamVR, girar el móvil y ver la vista estéreo moverse en la ventana espejo |
| 2 | Video: el driver captura los fotogramas, los codifica y el móvil los muestra a pantalla completa | Ver SteamVR en el móvil |
| 3 | Ajuste por ojo y corrección de lentes en el móvil, con deslizadores y guardado | Ajustarlo con tu visor |
| 4 | Pulido: menor latencia, reconexión automática, encontrar la PC sin escribir la IP | Uso normal |

Todo el desarrollo y la compilación se hacen en la nube. Las pruebas automáticas cubren el parser,
el filtro y la conversión de ejes con datos grabados o simulados. Solo las pruebas con SteamVR y el
móvil necesitan tu PC.

## 7. Riesgos

- **Latencia del video por Wi-Fi.** Es lo que más afecta la comodidad. Mitigación: router de 5 GHz o
  cable USB con anclaje, resolución y bitrate configurables, predicción de la orientación.
- **Deriva horizontal (yaw).** Sin magnetómetro, la vista gira lentamente con el tiempo. Mitigación:
  usar el magnetómetro cuando sea fiable y el botón de recentrar.
- **Calidad de los sensores del móvil.** Varía según el modelo; el filtro se ajusta por parámetros.

## 8. Datos que necesito más adelante (no bloquean el hito 0 ni el 1)

- Modelo del móvil (resolución de pantalla y tamaño físico).
- Modelo de GPU de la PC (para confirmar que codifica H.264 por hardware).

## 9. Siguiente: timewarp en el móvil y cuadros perdidos al codificar

- **Timewarp en el móvil, para que girar la cabeza se sienta fluido.** Hoy la imagen llega al
  móvil 70-115 ms después de calcularse y se muestra tal cual. La PC compensa prediciendo la cabeza
  hasta 80 ms con el giroscopio, y eso se pasa al empezar o frenar un giro: se siente brusco. Lo que
  hacen las Quest con Air Link es girar la imagen en el visor, justo antes de mostrarla, con su
  propio sensor. Aquí sería:
  - el driver manda con cada cuadro para qué momento del giroscopio del móvil (marca de tiempo del
    móvil más lo que predijo) se calculó;
  - el móvil redibuja a su propia frecuencia y, en el shader de los ojos, gira la imagen lo que la
    cabeza giró desde ese momento (integrando su giroscopio);
  - la PC calcula la imagen con algo de margen alrededor, para que los giros rápidos no muestren
    bordes negros;
  - con eso, bajar mucho la predicción de la PC (menos sobrepaso al frenar).
- **Cuadros que se pierden al codificar.** De los 60 cuadros por segundo que entrega SteamVR solo
  se codifican unos 42 (en un log, 228 de 300 en 5 s), así que la imagen avanza a saltos desparejos.
  Ver por qué el codificador no los toma todos a 2400x1080.
