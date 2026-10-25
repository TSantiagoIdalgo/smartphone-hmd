# smartphone-hmd

Convierte un smartphone Android en un visor (HMD) que SteamVR reconoce.

- `android/app`: app Kotlin que envía los sensores del móvil por UDP y muestra la imagen.
- `android/camera`: app "HMD Cámara" para el body tracking: un móvil viejo como cámara que manda
  su video a la PC (en pruebas, ver al final).
- `driver/`: driver de SteamVR en C++ que recibe los sensores y se presenta como un HMD estéreo.
- `tools/sim/send_imu.py`: simulador que se hace pasar por el móvil, para probar el driver sin él.
- `tools/body_tracker.py`: encuentra tu pose en el video de las cámaras de body tracking.
- `docs/PLAN.md`: plan del proyecto y protocolo.

## Descargar las compilaciones

Cada push a `main` y cada pull request compilan el APK y el driver en GitHub Actions.
Ve a la pestaña **Actions**, abre la ejecución más reciente y descarga:

- `smartphone-hmd-apk`: la app para instalar en el móvil.
- `smartphone-hmd-camera-apk`: la app de cámara para el body tracking.
- `smartphone_hmd-driver`: la carpeta del driver para SteamVR.

## Compilar en local

App (requiere JDK 17 y el SDK de Android, o Android Studio):

```
cd android
./gradlew assembleDebug
```

Driver (Windows, Visual Studio 2022 con C++ y CMake):

```
cmake -S driver -B driver/build -A x64
cmake --build driver/build --config Release
```

La carpeta lista para SteamVR queda en `driver/build/smartphone_hmd`.

## Instalar el driver en SteamVR

Con SteamVR cerrado, registra la carpeta del driver (ajusta las rutas a tu equipo):

```
"C:\Program Files (x86)\Steam\steamapps\common\SteamVR\bin\win64\vrpathreg.exe" adddriver C:\ruta\a\smartphone_hmd
```

Para quitarlo, usa `removedriver` con la misma ruta.

Si SteamVR no muestra el visor, abre `Steam\config\steamvr.vrsettings` y añade dentro de la sección
`"steamvr"` la línea `"forcedDriver": "smartphone_hmd",`.

## Probar con el móvil

1. Instala el APK en el móvil y el driver en SteamVR (ver arriba).
2. Pon en el driver la resolución de la pantalla del móvil en horizontal: en
   `smartphone_hmd\resources\settings\default.vrsettings` cambia `window_width` y `window_height`
   (por ejemplo 2400 y 1080). Deben ser números pares.
3. Conecta el móvil a la PC por cable USB con el anclaje USB activado, o pon los dos en la misma
   red Wi-Fi.
4. Abre SteamVR. La primera vez Windows puede preguntar si `vrserver.exe` puede usar la red:
   permítelo en redes privadas. Si no pregunta y no llegan datos, permite el puerto UDP 9944 en el
   firewall de Windows.
5. Abre la app en el móvil: busca la PC sola y, al encontrarla, muestra su IP. Pulsa **Iniciar**.
   El móvil pasa a pantalla completa y, en unos segundos, muestra la imagen estéreo de SteamVR. Al
   girar el móvil, la vista gira igual. **Atrás** detiene el envío.
6. Abre SteamVR Home o un juego. Sin ninguna aplicación abierta, lo que dibuja SteamVR puede verse
   casi negro.
7. Toca la pantalla del móvil para ajustar cada ojo a tu visor (ver más abajo).

Para recentrar la vista, pulsa cualquier botón de volumen del móvil o **Ctrl+Alt+R** en la PC.

Al girar, los ojos se mueven alrededor del cuello, como en la realidad. Para alinear los PS Move
con la "Playspace Calibration" de VDM, pulsa **Ctrl+Alt+H** antes de empezarla y otra vez al
terminar: mientras tanto el visor queda exactamente quieto, que es lo que VDM necesita.

Si la vista se descentra con el tiempo (deriva del giroscopio), pon `"magnetometer_gain": 0.1` en
`default.vrsettings`. La brújula del móvil corrige entonces el giro horizontal muy despacio, y solo
mientras el campo magnético se ve limpio: si algo lo desvía (imanes, parlantes), el driver la ignora.
Con `0` (el valor por defecto) no se usa. Si notas que la vista se mece sola, bájalo o ponlo en `0`.

Si la app no encuentra la PC (algunos routers bloquean la búsqueda), escribe su IP, que da
`ipconfig` en la PC. Puedes pulsar **Iniciar** aunque aún no la haya encontrado: empieza en cuanto
aparezca. Si el video se corta, por ejemplo al desenchufar el cable o porque la PC cambió de IP, la
app vuelve a buscarla y sigue sola por la nueva conexión.

El móvil va en horizontal dentro del visor, con la pantalla hacia los ojos y la parte de arriba del
teléfono a la izquierda.

## Ajustar cada ojo a tu visor

Con el video en marcha, toca la pantalla del móvil para abrir el ajuste. Los cambios se ven al
instante sobre el video y se guardan en el teléfono. Ajusta con el móvil fuera del visor, pulsa
**Listo** y compruébalo dentro.

- **Ancho** y **alto de cada ojo**: tamaño de la imagen de cada ojo, en milímetros de pantalla.
- **Separación entre centros**: distancia entre los centros de las dos imágenes. Empieza por la
  distancia entre los centros de las lentes de tu visor, medida con una regla.
- **Altura de los centros**: sube o baja las dos imágenes para alinearlas con las lentes.
- **Corrección de lente k1 y k2**: curvan la imagen para compensar la deformación de las lentes.
  Marca **Cuadrícula**, mira a través del visor y sube k1 hasta que las líneas verdes se vean
  rectas; k2 corrige sobre todo los bordes. La cuadrícula sigue visible al cerrar el ajuste hasta
  que la desmarques.

**Restablecer** vuelve a la imagen a pantalla completa. **Listo**, **Atrás** o tocar fuera del
panel lo cierran.

La imagen de cada ojo abarca 90° de ancho y de alto (`fov_degrees` en el driver), así que se ve sin
estirar cuando el ancho y el alto son iguales.

## Latencia

El panel de ajuste muestra la **latencia medida**: el tiempo desde que el móvil envía un movimiento
hasta que muestra la imagen dibujada con él. Con **Anticipar el movimiento de la cabeza** marcado
(viene activado), la PC dibuja cada imagen con la posición que tendrá la cabeza cuando llegue a la
pantalla, así que la vista sigue al giro sin arrastrarse. Si notas que la imagen se adelanta o
tiembla en giros bruscos, desmárcalo. En `vrserver.txt` la línea `Phone latency` dice cuánto se
anticipa.

## Calidad del video

La calidad del video se ajusta con `video_bitrate_mbps` (por defecto 20) en `default.vrsettings`, el
mismo archivo de la resolución. Por cable USB puedes subirlo a 40 o 50 para que la imagen se vea
mejor en movimiento; por Wi-Fi de 2.4 GHz mejor deja 20. Si la imagen se corta o llega con retraso,
conecta el móvil por cable USB con el anclaje USB activado, usa Wi-Fi de 5 GHz o baja ese valor.

Para Wi-Fi de 5 GHz sin router de 5 GHz: activa el punto de acceso del móvil en la banda de 5 GHz y
conecta a él un adaptador Wi-Fi de 5 GHz de la PC. La app encuentra la PC también así. El punto de
acceso de Windows no sirve si la misma tarjeta ya está conectada a una red de 2.4 GHz, porque
entonces solo puede emitir en esa banda.

Si no llega la imagen:

- Mientras no hay video, el móvil muestra cuántos paquetes de video recibe, cuántos fotogramas
  completa (y cuántos pierde) y cuántos muestra.
- Busca las líneas que contienen `smartphone_hmd` en `Steam\logs\vrserver.txt`. Dicen qué GPU y qué
  codificador se usan, y cada 5 segundos una línea `Video:` cuenta los fotogramas capturados,
  codificados y enviados. `brightness` cerca de 16 significa que la imagen capturada es negra.
- Si el video se corta o se congela, prueba una resolución menor con la misma proporción que la
  pantalla del móvil, por ejemplo 1920x864 en vez de 2400x1080. El móvil la escala a pantalla
  completa.

Sin el móvil, `python tools/sim/send_imu.py` envía datos que giran la cabeza a izquierda y derecha.

## Posición de la cabeza con PS Move (opcional)

Sin nada más, el visor solo gira (3DoF). Si usas PSMoveServiceEx con una PS Eye para los PS Move,
el driver también puede leer la posición de una luz montada en el visor (una pelota de ping-pong
con un LED, configurada en PSMoveServiceEx como visor virtual). Agacharte, inclinarte o dar un paso
mueve entonces la vista, y la cabeza y las manos quedan en el mismo espacio. La rotación sigue
llegando del móvil, así que girar la cabeza no tiene más latencia que antes.

Viene desactivado. Para activarlo, con SteamVR cerrado, cambia o añade en
`smartphone_hmd\resources\settings\default.vrsettings`, dentro de `"driver_smartphone_hmd"`
(no en `Steam\config\steamvr.vrsettings`, que SteamVR reescribe):

```
"head_position_enabled" : true,
"head_light_above_eyes_cm" : 7.0,
"head_light_in_front_of_eyes_cm" : 6.0,
"head_light_right_of_eyes_cm" : 0.0
```

Los tres números son dónde está el centro de la luz respecto del punto entre tus ojos, medidos con
el visor puesto. El resto de las claves `head_*` y `psms_*` de `default.vrsettings` tienen valores
por defecto que normalmente no hace falta tocar; si faltan en el archivo, el driver usa esos mismos
valores.

- El driver busca `PSMoveClient_CAPI.dll` junto a PSMoveServiceEx o al Virtual Device Manager
  mientras están abiertos, y reintenta cada 2 segundos. Si no lo encuentra, pon su ruta en
  `psms_client_dll`.
- **Ctrl+Alt+P** congela la posición (la rotación sigue) y la vuelve a soltar.
- Al recentrar con el volumen o **Ctrl+Alt+R**, mira hacia la cámara: la vista se orienta hacia
  ella desde donde estés.
- En `vrserver.txt`, las líneas `Head position:` y `Head light:` dicen qué DLL se cargó, si hay
  conexión, dónde está la cámara y si la luz se ve, y cada 10 segundos una línea de estadísticas da
  la altura de los ojos y cuánto tiembla la medida.
- `psms_controller_id` (por defecto -1) hace que siga un PS Move en vez del visor virtual: sirve
  para probar con un Move pegado al visor.

## Body tracking: cámaras y pose en la PC (en pruebas)

Primer paso del body tracking: cada móvil que hace de cámara corre la app **HMD Cámara**, que manda
a la PC el video de su cámara (H.264, unos 2 Mb/s). En la PC, `tools/body_tracker.py` encuentra tu
pose en cada cuadro con el modelo de pose de MediaPipe, repartido en varios núcleos, y le devuelve
los puntos al móvil, que los dibuja sobre la imagen.
Así un móvil viejo solo filma, y el trabajo pesado lo hace la PC. El driver todavía no usa la pose;
por ahora sirve para medir cuántos cuadros por segundo llegan, el retraso y si la pose es estable.

En la PC, con Python 3.10 o más nuevo, una sola vez (y otra vez al actualizar, porque cambian las
librerías):

```
pip install -r tools\requirements-body.txt
```

Y cada vez:

```
python tools\body_tracker.py
python tools\body_tracker.py --model heavy --threads 8
python tools\body_tracker.py --record prueba.jsonl
```

La primera vez descarga el modelo de pose a `tools\models` y Windows pregunta si Python puede usar
la red: permítelo en redes privadas (puerto UDP 9946). `--model lite` usa un modelo más rápido y
menos exacto, y `--model heavy` uno más exacto y unas tres veces más lento que el normal (`full`).
`--threads` dice cuántos núcleos usa cada cámara (4 si no se dice): con 4, el modelo tarda menos de
la mitad que en uno solo, y con `heavy` se acerca a lo que tardaba `full` en un núcleo. `--record`
guarda cada pose en un archivo para reproducirla después. `--engine mediapipe` corre el mismo
modelo con la librería de MediaPipe, que usa un solo núcleo (`pip install mediapipe` aparte), para
comparar.

1. Instala `smartphone-hmd-camera-apk` en el móvil que hará de cámara y dale permiso de cámara. Si
   tenías la versión anterior, desinstálala primero.
2. Conéctalo como el visor: anclaje USB por cable, o la misma red Wi-Fi que la PC. Busca la PC
   solo; si no la encuentra, escribe su IP.
3. Ponlo fijo, a la altura de la cadera o algo más, de modo que te vea entero de pie donde juegas.
   La imagen se endereza sola esté el móvil vertical u horizontal.
4. En la pantalla: los puntos verdes son seguros, los amarillos dudosos y los rojos adivinados. Las
   líneas de arriba dicen los cuadros por segundo de la cámara y del video (con el modo del
   codificador: CBR es un bitrate parejo, lo mejor para el Wi-Fi), la demora de cada tramo dentro
   del móvil (de la cámara a la app, el codificador y el envío de los paquetes), las poses por
   segundo que vuelven de la PC, el tiempo de ida y vuelta (desde que la cámara toma el cuadro
   hasta que su pose vuelve al móvil) y qué tan seguros están la cadera, las rodillas y los
   tobillos. Lo que falta del "llega a los … ms" de la PC después de esos tramos es el Wi-Fi. Un
   toque en la imagen oculta los controles.
5. Con los puntos en verde, pulsa **Fijar exposición** para que la imagen deje de cambiar de brillo
   y de foco. Los botones cambian también los fps de la cámara y la cámara trasera o frontal.

Cada 2 segundos `body_tracker.py` muestra por móvil los cuadros por segundo y los Mb/s que llegan,
las poses por segundo, cuánto tarda el modelo, a los cuántos ms de tomado el cuadro llega a la PC y
está lista su pose, los cuadros perdidos y la seguridad de cada punto. "saltados" son cuadros que
llegaron mientras el modelo seguía con el anterior: la pose se calcula solo en el más nuevo, para
que el retraso no crezca. Si son muchos, prueba más `--threads` o un modelo más rápido.
