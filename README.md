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
- Si la cabeza queda muy alta o lejos de la PS Eye (y los Moves se ven muy adelante tuyo), la línea
  `Head light: the eyes come out … m high` lo avisa: PSMoveServiceEx calcula la distancia por el
  tamaño aparente de la bola, y si el tamaño que tiene configurado no es el de tu bola (o la ve
  tapada a medias) la pone más lejos. Para corregirlo, con PSMoveServiceEx siguiendo un Move
  (body tracking o `controllers_enabled`):
  1. Pulsa **Ctrl+Alt+L** en la PC. No mide todavía: espera hasta un minuto, para que vuelvas a
     donde juegas con el visor puesto.
  2. Ahí, apoya la bola del Move contra la luz del visor, donde la PS Eye vea las dos. Cuando las ve
     juntas, el Move vibra un instante: quédate quieto.
  3. A los 3 segundos vibra otra vez, más largo: listo. Si la bola se apartó de la luz en ese rato,
     no cuenta y vuelve a esperar (sin vibrar al final).

  Como la distancia del Move sí es correcta, el driver mide cuánto se equivoca con la luz y lo
  guarda en `head_light_distance_scale` (la línea `Head light: PSMoveServiceEx had it … times as
  far`; si no pudo, `vrserver.txt` dice por qué).
  También puedes corregir el tamaño de la bola en la configuración del visor virtual de
  PSMoveServiceEx, y entonces dejar `head_light_distance_scale` en 1.
- Con el body tracking, los móviles también ayudan a la cabeza: ver "La cabeza con las cámaras del
  cuerpo".

## Body tracking: cámaras y pose en la PC (en pruebas)

Cada móvil que hace de cámara corre la app **HMD Cámara**, que manda a la PC el video de su cámara
(H.264, unos 2 Mb/s). En la PC, `tools/body_tracker.py` encuentra tu pose en cada cuadro con el
modelo de pose de MediaPipe, repartido en varios núcleos, y le devuelve los puntos al móvil, que
los dibuja sobre la imagen. Así un móvil viejo solo filma, y el trabajo pesado lo hace la PC.
`body_tracker.py` también le pasa los puntos de cada cámara al driver, que con dos cámaras
calibradas arma el cuerpo en 3D y lo publica en SteamVR como tres trackers: cadera y los dos pies
(ver "Cuerpo completo en SteamVR" más abajo).

En la PC, con Python de 64 bits entre el 3.10 y el 3.14 (LiteRT, que corre el modelo, no tiene
paquetes para otros: pip dice "from versions: none"), una sola vez (y otra vez al actualizar,
porque cambian las librerías):

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
`--threads` dice cuántos núcleos usa cada cámara; si no se dice, la cuarta parte de los de la PC,
entre 2 y 8, para que con dos cámaras quede la mitad para el juego. Con 4, el modelo tarda menos de
la mitad que en uno solo, y con `heavy` se acerca a lo que tardaba `full` en un núcleo. `--record`
guarda cada pose en un archivo para reproducirla después. `--engine mediapipe` corre el mismo
modelo con la librería de MediaPipe, que usa un solo núcleo (`pip install mediapipe` aparte), para
comparar.

1. Instala `smartphone-hmd-camera-apk` en el móvil que hará de cámara y dale permiso de cámara. Si
   tenías una versión de antes de octubre de 2026, desinstálala primero; las siguientes se instalan
   encima y guardan sus ajustes (las dos apps se firman ahora siempre con la misma clave).
2. Conéctalo como el visor: anclaje USB por cable, o la misma red Wi-Fi que la PC. Busca la PC
   solo; si no la encuentra, escribe su IP.
3. Ponlo fijo, a la altura de la cadera o algo más, de modo que te vea entero de pie donde juegas.
   La imagen se endereza sola esté el móvil vertical u horizontal.
4. En la pantalla: los puntos verdes son seguros, los amarillos dudosos y los rojos adivinados. Las
   líneas de arriba dicen los cuadros por segundo de la cámara y del video (con el modo del
   codificador: CBR es un bitrate parejo, lo mejor para el Wi-Fi; y «sin retener» si aceptó que se
   le pida no guardarse cada cuadro hasta que llega el siguiente), la demora de cada tramo dentro
   del móvil (de la cámara a la app, el codificador y el envío de los paquetes), las poses por
   segundo que vuelven de la PC, el tiempo de ida y vuelta (desde que la cámara toma el cuadro
   hasta que su pose vuelve al móvil) y qué tan seguros están la cadera, las rodillas y los
   tobillos. Lo que falta del "llega a los … ms" de la PC después de esos tramos es el Wi-Fi. Un
   toque en la imagen oculta los controles.
5. Con los puntos en verde, pulsa **Fijar exposición** para que la imagen deje de cambiar de brillo
   y de foco. Si el móvil deja manejar la cámara a mano, además acorta la exposición a 10 ms como
   mucho y sube la ganancia (ISO) para que se vea igual de clara: cada cuadro sale antes (la cámara
   no lo entrega hasta cerrar la exposición, que con luz de interior llega a 33 ms) y los pies y la
   bola del Move salen menos movidos, a cambio de algo de grano. La línea de arriba dice la
   exposición en uso. Los botones cambian también los fps de la cámara (con 60, si el móvil los
   da, cada cuadro tarda menos en la cámara y en el codificador) y la cámara trasera o frontal.

Cada 2 segundos `body_tracker.py` muestra por móvil los cuadros por segundo y los Mb/s que llegan,
las poses por segundo, cuánto tarda el modelo, a los cuántos ms de tomado el cuadro llega a la PC y
está lista su pose, los cuadros perdidos y la seguridad de cada punto. "saltados" son cuadros que
llegaron mientras el modelo seguía con el anterior: la pose se calcula solo en el más nuevo, para
que el retraso no crezca. Si son muchos, prueba más `--threads` o un modelo más rápido.

`--show` muestra en la PC lo que ve cada cámara, en vivo y con sus puntos y el anillo de la bola
del Move: abre en el navegador una página (`http://localhost:8080`, u otro puerto con
`--show 8081`) con un video de cada móvil, de unos 15 cuadros por segundo. Así se puede filmar con
la cámara trasera, que es mejor, y ver igual cómo encuadra cada móvil sin mirar su pantalla. La
página se arma aparte de la pose y no la atrasa.

`--snapshots carpeta` guarda cada 2 segundos una foto de lo que ve cada cámara, con los puntos en
verde y la bola del Move en un anillo: sirve para ver por qué no se detecta algo. Van numeradas
(`camara_19BB_0001.jpg`, …), y las que tienen una bola terminan en `_bola`.

`--ball-color magenta,cyan` (colores entre `magenta`, `cyan`, `yellow`, `red`, `green`, `blue`,
separados por comas) toma por bola de un Move solo algo de esos colores. **Úsalo siempre**, con los
colores con que PSMoveServiceEx enciende tus Moves (el primero sale magenta, el segundo cian) y no
el de la luz del visor: sin él, la luz del visor (y una lámpara o una prenda que brille) junto a la
mano se toma por un Move. Al calibrar alcanza con el color del Move que usas.

### Una ESP32-CAM como cámara

Una placa ESP32-CAM (AI Thinker, con cámara OV3660 u OV2640) sirve de cámara igual que un móvil, y
se mezcla con ellos: `body_tracker.py` y el driver la tratan como una cámara más, que se calibra con
Ctrl+Alt+C como las otras. Solo necesita 5 V (un cargador o un powerbank de al menos 1 A: con menos
se reinicia sola al conectar el Wi-Fi), así que la puedes poner donde quieras.

1. En el Arduino IDE, instala las placas "esp32" de Espressif (Preferencias → URLs adicionales:
   `https://espressif.github.io/arduino-esp32/package_esp32_index.json`) y elige la placa
   **AI Thinker ESP32-CAM**.
2. Abre `esp32/hmd_camera/hmd_camera.ino`, pon tu red en `WIFI_SSID` y `WIFI_PASSWORD` y súbelo
   (con el adaptador USB-serie, GPIO0 a GND al subir).
3. En el monitor serie (115200) ves su número (`HMD Cámara ESP32 XXXX`), cuando encuentra la PC y
   sus cuadros por segundo. La PC la muestra como cualquier cámara: `Cámara XXXX`.

Ajustes, arriba en el programa:

- `QUARTER_TURNS`: cuartos de vuelta a la derecha para que la imagen quede derecha según cómo
  montaste la placa; míralo con `--show`. En vertical (placa parada) entras más entero.
- `MIRROR`: la imagen no debe salir espejada (un texto se lee al revés), o no se puede calibrar.
- `EXPOSURE` (0, automática): un valor fijo corto, 150-400, para que brazos y pies no salgan
  movidos; necesita buena luz. `GAIN` aclara a cambio de ruido.
- `FRAME_SIZE` (640x480) y `JPEG_QUALITY` (12): más resolución o calidad, menos cuadros por segundo.

Solo tiene Wi-Fi de 2,4 GHz y una antena chica: cerca del router anda mejor.

### Cuerpo completo en SteamVR

Necesita dos cámaras, la posición de la cabeza con la luz del visor (ver "Posición de la cabeza con
PS Move") y un PS Move para calibrar. Con SteamVR cerrado, pon en `default.vrsettings`:

```
"body_tracking_enabled" : true,
```

Al abrir SteamVR aparecen tres trackers (cadera, pie izquierdo y pie derecho) con su rol ya puesto,
así que VRChat los usa sin configurar nada en "Manage Trackers". Hasta que tienen posición por
primera vez (dos cámaras calibradas que te vean entero) figuran desconectados, para que VRChat no
entre en el modo de cuerpo completo y te deje en pose de T sin poder calibrar. Desde entonces
quedan conectados aunque los móviles dejen de verte un rato (se quedan donde estaban), como un
tracker real tapado. **Si VRChat no ofrece "Calibrar"** (solo sentado/parado), es que todavía no vio
los tres: los móviles tienen que verte los pies donde juegas. Mira con `--show` que en los dos
salgas entero, de la cabeza a los pies, y en `vrserver.txt` que `Body trackers` no diga `nowhere`.

**Dónde poner los móviles** (visto desde arriba):

```
    Móvil 1          PS Eye          Móvil 2
      [=]              [o]             [=]
         \              |              /
           \            |            /
     2-3 m   \          |          /  2-3 m
               \        |        /
                 \  45° | 45°  /
                   \    |    /
                     \  |  /
                       (tú)  <- donde juegas
```

- **Separados entre 60° y 90°** vistos desde donde juegas (en el dibujo, 90°): a 2-3 m de ti y a
  2,5-3,5 m uno del otro. También sirve uno adelante y otro a un costado.
- En vertical, a 1,0-1,2 m del piso y apuntando hacia ti, de modo que te vean entero, con los pies.
- La PS Eye queda donde ya la tengas: los dos móviles y ella tienen que ver el Move en la zona de
  juego.
- Juntos (uno al lado del otro junto a la PS Eye) cada uno ve bien hacia los lados, pero la
  profundidad sale varias veces peor: a medio metro uno del otro, el cuerpo tiembla y se va adelante
  y atrás varios centímetros.

**Calibrar las cámaras** (una vez, y otra si mueves un móvil):

1. Pon los dos móviles fijos como en el dibujo. Abre la app en los dos y en la PC
   `python tools\body_tracker.py --ball-color magenta`, con el color de la bola del Move con que
   vas a calibrar.
2. Con PSMoveServiceEx y SteamVR abiertos, enciende **un solo** Move y pulsa **Ctrl+Alt+C** en la
   PC. El otro, apagado del todo: aunque la PS Eye no lo vea, su bola sigue encendida y los móviles
   pueden confundirla con la del que mueves.
3. A los 5 segundos empieza. Durante un minuto, con el Move en la mano, muévelo despacio por toda
   la zona de juego: alto y bajo, cerca y lejos, donde lo vean los dos móviles y la PS Eye. Cada
   móvil marca la bola con un anillo celeste cuando la ve. Termina sola cuando tiene suficiente, o
   pulsa Ctrl+Alt+C otra vez para cancelar. Cada cámara necesita al menos 60 muestras: si una se
   queda atrás en las líneas `Body calibration:`, mueve el Move donde la ve ella (sin taparlo con
   el cuerpo, de frente a ese móvil).
4. En `vrserver.txt`, las líneas `Body calibration:` dicen cuántas muestras juntó cada cámara y, al
   final, si quedó calibrada, cuánto se aparta (en píxeles y centímetros), dónde está y hacia dónde
   mira. Si una cámara no juntó suficientes, conserva su calibración anterior solo si todavía
   encaja con lo que vio; si no (la moviste), deja de usarse hasta calibrarla de nuevo, y lo dice.
   La calibración se guarda en SteamVR y se usa sola la próxima vez, por número de cámara: si
   desinstalas la app o borras sus datos, el móvil sale con otro número y hay que calibrar de nuevo.
5. Para jugar, abre `body_tracker.py` con los colores de los dos Moves, por ejemplo
   `--ball-color magenta,cyan`, para que los móviles vean las bolas de los dos (ver "Mandos") y no
   la luz del visor.

**Comprobarla:** con un solo Move encendido, como al calibrar, déjalo quieto en varios lugares
donde lo vean los dos móviles. Cada 10 segundos, la línea `Body check:` dice a cuántos centímetros
ponen las cámaras la bola de donde la pone PSMoveServiceEx (con menos de 3 cm está bien), cuánto de
eso es en profundidad y cuánto de lado, y con cuántos grados de diferencia la ven los dos móviles.
Si casi todo es profundidad y los grados son menos de 40, los móviles están muy juntos: sepáralos y
calibra de nuevo. Con los dos Moves encendidos no comprueba nada y lo dice.

En `vrserver.txt`, cada 10 segundos, `Body camera …` dice por cámara los cuadros por segundo, a los
cuántos ms de tomado llega cada cuadro, qué parte tiene un cuadro de la otra cámara a menos de
20 ms, en qué parte se ve una persona y la bola, y si está calibrada; `Body:` cuántos puntos se usan
y cuántos se descartan (por no encajar, o por llegar más viejos que lo que ya había), y en cuántos
cuadros el modelo de pose tenía la izquierda y la derecha al revés (unos pocos es normal; casi
todos quiere decir que el cuerpo quedó dado vuelta: el driver lo corrige solo en unos segundos,
cuando la cabeza y las puntas de los pies coinciden en que la cadera mira hacia atrás, y lo dice
con `Body: the hips faced away…`); y
`Body trackers` por tracker (cadera, pie izquierdo, pie derecho) a qué
altura del piso estuvo en promedio, la más baja y la más alta, cuánto se apartó de su lugar
promedio y qué parte del tiempo tuvo posición. Quieto, lo que se aparta es su temblor: debe quedar
en menos de 1 cm, y si no, baja `body_agility`. Al caminar, la altura más baja de cada pie es la del
tobillo apoyado, unos 0,07-0,10 m.

**Antes de calibrar el cuerpo en VRChat:** VRChat da cada tracker al hueso del avatar más cercano,
con el avatar parado debajo del visor y mirando hacia donde mira el visor. Si la cabeza queda lejos
de tu cuerpo (la distancia de la luz mal) o mirando hacia otro lado, te cruza las piernas: la
izquierda mueve la derecha y al revés. Así que párate donde te vean los dos móviles, mira a la PS
Eye, recentra (tecla de volumen) y recién entonces calibra en VRChat. La línea `Body and head:` de
`vrserver.txt` dice cada 10 s a cuántos cm de estar sobre la cadera está el visor y cuántos grados
mira desde donde mira la cadera; con más de 30 cm o 60° avisa que VRChat cruzaría las piernas.

Ajustes de `default.vrsettings`, que normalmente no hace falta tocar:

- `body_prediction` (0.3): cuánto se adelanta el cuerpo según su velocidad, para compensar el
  retraso. Más alto reacciona antes pero tiembla más al moverse.
- `body_agility` (1.0): cuánto se espera que cambie la velocidad de cada punto. Más bajo suaviza más
  y sigue más lento.
- `body_pixel_noise` (3.0): cuánto se cree que tiemblan los puntos de la pose, en píxeles.
- `body_psms_latency_ms` (40): el retraso de PSMoveServiceEx al calibrar.
- `body_calibration_controller_id` (-1): qué Move usar al calibrar y comprobar, si no quieres
  apagar el otro; ese otro no debe estar donde lo vean los móviles.
- `body_calibration_seconds` (60) y `body_port` (9947).

### La cabeza con las cámaras del cuerpo

Con el body tracking y la luz del visor encendidos, los móviles también ven tu cabeza (el punto
entre las orejas) y el driver la usa junto con la luz:

- **Quieta de verdad, y sobre tu cuerpo.** La PS Eye sola ve muy bien hacia los lados pero mal en
  profundidad (la saca del tamaño de la luz): la cabeza se mueve adelante y atrás sin que te muevas,
  y puede quedar hasta un metro más lejos de lo que estás. Mientras los móviles te ven, de la luz se
  toma solo la dirección desde la PS Eye y la profundidad sale de los móviles; si al empezar la luz
  dice otra distancia, la cabeza pasa a donde la ven los móviles, sobre la línea de la luz. Además,
  cuando estás quieto el driver suaviza más (aprende cuánto te mueves), con o sin móviles.
- **De espaldas a la PS Eye.** Si la cámara deja de ver la luz (giras 180°, la tapas), la cabeza
  sigue a donde la ven los móviles en vez de quedarse clavada; al volver a ver la luz pasa a ella
  suavemente. Sin la luz, la posición llega unos 0,15 s más tarde y algo menos fina, pero no salta.

Viene activado cuando están las dos cosas. **Ctrl+Alt+B** lo apaga (la primera vez que lo pulsas)
y lo vuelve a encender, para comparar; `vrserver.txt` dice cuál quedó (`the light alone` es
apagado). No lo dejes apagado: si PSMoveServiceEx pone la luz lejos, la cabeza queda detrás del
cuerpo y de los Moves. En `vrserver.txt`:

- `Head light: out of sight; the body cameras follow the head` cuando la siguen solo los móviles.
- Cada 10 segundos, `Head from the body cameras:` cuántas vistas de los móviles usó, cuántas
  descartó y cuántas llegaron tarde; cuántas veces puso la cabeza donde la ven los móviles (sin la
  luz, o a otra distancia que la de la luz); a cuántos cm de donde la ven pone la luz la cabeza (si
  son decenas, la distancia de PSMoveServiceEx está mal: ver `head_light_distance_scale`); y dónde
  aprendió que están tus orejas respecto del cuello. La línea de estadísticas de `Head light:`
  termina con `agility`: baja (0,01-0,1) quieto, sube al moverte.

Ajustes de `default.vrsettings`:

- `head_position_depth_noise_with_body_cm` (100): cuánto se cree en la profundidad de la PS Eye
  mientras los móviles ven la cabeza. Alto, la profundidad sale de los móviles: más quieta y sobre
  tu cuerpo, pero unos 0,15 s más lenta al acercarte o alejarte de la PS Eye. Más bajo (9, como
  antes), más de la PS Eye: solo si su distancia es buena.
- `head_body_cameras` (true): `false` no usa los móviles para la cabeza.

## Mandos: los PS Move desde este driver (opcional)

El driver puede publicar los dos PS Move como mandos en vez del Virtual Device Manager. Lo que se
gana, con el body tracking:

- **Los móviles también ven las bolas.** Con las cámaras calibradas, cada móvil busca la bola
  encendida junto a cada mano y el driver la suma a la de la PS Eye, como con la cabeza: la
  profundidad sale de los móviles (la de la PS Eye es la que tiembla) y, cuando la PS Eye deja de ver
  la bola (detrás del cuerpo, fuera de su vista, tapada por la otra mano), la siguen los móviles,
  unos 0,2 s más tarde. Cada vista va a la mano cuya bola pasa más cerca de esa línea de visión, si
  la otra mano está claramente más lejos.
- **Y si nadie ve la bola**, la mano sigue a tu muñeca, que ven los móviles, en vez de quedarse
  clavada. El driver aprende dónde queda la bola respecto de tu muñeca mientras ve las dos cosas, y
  también cuál muñeca sostiene cada Move.

Viene desactivado. Para activarlo:

1. Con SteamVR cerrado, pon en `default.vrsettings`: `"controllers_enabled" : true,`
2. Haz que VDM deje de publicar los Moves, o tendrás cada mando dos veces (y SteamVR los confunde):
   en VDM, en "Virtual Motion Tracker", quita o desactiva los dos PS Move. **No** desactives el
   complemento `psmsex` de SteamVR: es el driver de ese Virtual Motion Tracker y el servidor OSC de
   VDM depende de él. Si VDM no te deja quitarlos, deja `controllers_enabled` en `false`.
3. Abre SteamVR: aparecen dos mandos, que los juegos ven como mandos Vive.

Los Moves van a la mano que tengan asignada en PSMoveServiceEx; si no tienen ninguna, el primero es
la derecha y el segundo la izquierda.

| Move | Mando Vive |
|------|------------|
| Gatillo | Gatillo |
| Botón Move (el grande), mantenido | Trackpad tocado donde inclines el Move desde donde apuntaba al pulsarlo (como en VDM: el menú de acciones y los gestos de VRChat) |
| Triángulo / Equis / Cuadrado / Círculo | Trackpad arriba / abajo / izquierda / derecha (tocado y pulsado) |
| Start | Grip |
| Select | Menú |
| PS | Sistema (el menú de SteamVR) |

Las vibraciones de los juegos hacen vibrar el Move. Cada juego se puede reasignar en SteamVR →
Mandos → Administrar asignaciones, como un mando Vive.

- **Recentrar un Move**: apúntalo a la PS Eye y mantén **Select + Start** un segundo. Corrige el
  giro que acumula.
- Si en SteamVR el mando apunta hacia arriba o hacia abajo cuando el Move apunta al frente, pon
  `controller_pitch_degrees` en 90 o -90.
- **Temblor**: la posición de cada bola pasa por el mismo filtro que la luz del visor. La PS Eye ve
  bien de costado y mal en profundidad, peor cuanto más lejos; el filtro suaviza más con la mano
  quieta, sigue de cerca al moverla y deja afuera los saltos de una bola tapada a medias. Mientras
  los móviles la ven, la profundidad es la suya: `controller_depth_noise_with_body_cm` (100) es
  cuánto se cree entonces en la de la PS Eye (más bajo, una bola tapada a medias, que la PS Eye
  pone lejos, arrastra el mando). Si aún tiembla quieto, sube
  `controller_depth_noise_cm` (2, sin los móviles) o baja `controller_agility` (10); si va atrasado
  al moverlo, al revés. `controller_prediction` (1) adelanta la mano según su velocidad para
  compensar el retraso de PSMoveServiceEx.
- En `vrserver.txt`, `Hands:` dice qué Move quedó en cada mano y, cada 10 segundos, qué parte del
  tiempo lo vieron la PS Eye o los móviles, cuánto siguió a la muñeca y cuánto quedó quieto donde
  estaba (sin bola ni muñeca), cuántas lecturas de la bola dejó afuera, **cuánto tiembla el mando
  quieto** (`held still it trembles … mm`: lo que ves en el juego; menos de 5 mm está bien), cuánto
  tiembla la profundidad que da la PS Eye y de costado, cuántas vistas de los móviles usó y cuántas
  descartó, y a cuántos
  centímetros de la muñeca aprendió que está la bola (unos 15-20 cm). En `body_tracker.py`, "bola
  del Move en N%" dice en qué parte de los cuadros encontró alguna.
