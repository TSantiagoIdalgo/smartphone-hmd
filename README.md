# smartphone-hmd

Convierte un móvil Android en un visor que SteamVR reconoce. El móvil da el giro de la cabeza y
muestra la imagen; con una PS3 Eye y PSMoveServiceEx, la cabeza también se mueve en el espacio y los
dos PS Move son los mandos.

Paso a paso para dejarlo todo funcionando desde cero. Los pasos 1 a 5 bastan para jugar sentado o
de pie sin moverte del sitio (solo giro). Los pasos 6 y 7 son para la posición de la cabeza y los
mandos.

## Qué necesitas

- Una PC con Windows, Steam y SteamVR.
- Un móvil con Android 8 o más nuevo, y un visor de móvil (de plástico o cartón) donde quepa.
- Un cable USB para el móvil, o Wi-Fi de 5 GHz. El Wi-Fi de 2,4 GHz también sirve, con peor imagen.
- Para los pasos 6 y 7: una PS3 Eye, uno o dos PS Move, PSMoveServiceEx con su Virtual Device
  Manager (VDM) ya funcionando, y una luz en el visor que PSMoveServiceEx pueda seguir, por ejemplo
  una pelota de ping-pong con un LED dentro.

## 1. Descarga la app y el driver

En GitHub, entra en la pestaña **Actions**, abre la ejecución más reciente de **Build** y, abajo,
en **Artifacts**, descarga:

- `smartphone-hmd-apk`: la app del móvil.
- `smartphone_hmd-driver`: el driver de SteamVR.

Descomprime los dos.

## 2. Instala el driver en SteamVR

1. Copia la carpeta `smartphone_hmd` del driver a un lugar fijo, por ejemplo `C:\smartphone_hmd`.
   Dentro deben quedar `driver.vrdrivermanifest`, `bin` y `resources`.
2. Con SteamVR cerrado, regístrala, abriendo `cmd` y ajustando las rutas a tu equipo:

   ```
   "C:\Program Files (x86)\Steam\steamapps\common\SteamVR\bin\win64\vrpathreg.exe" adddriver C:\smartphone_hmd
   ```

   Para quitarlo más adelante, usa `removedriver` con la misma ruta.
3. Si al abrir SteamVR no aparece el visor, abre `Steam\config\steamvr.vrsettings` y añade, dentro
   de la sección `"steamvr"`, la línea `"forcedDriver": "smartphone_hmd",`.

Para actualizar el driver más adelante, con SteamVR cerrado basta con reemplazar
`C:\smartphone_hmd\bin\win64\driver_smartphone_hmd.dll`.

Todos los ajustes del driver están en `C:\smartphone_hmd\resources\settings\default.vrsettings`.
Edítalo siempre con SteamVR cerrado. Si a tu archivo le falta una línea, el driver usa el valor que
se indica en esta guía.

## 3. Ajusta el driver a tu móvil

En `default.vrsettings`:

- `window_width` y `window_height`: la resolución de la pantalla del móvil en horizontal, por
  ejemplo `2400` y `1080`. Deben ser números pares. Si el video se corta, prueba una menor con la
  misma proporción, por ejemplo `1920` y `864`.
- `video_bitrate_mbps` (20): la calidad del video. Por cable puedes subirlo a 40 o 50. Por Wi-Fi de
  2,4 GHz déjalo en 20 o bájalo.

## 4. Conecta el móvil

1. Instala el APK en el móvil.
2. Conéctalo a la PC por cable USB con el **anclaje USB** activado, o ponlos en la misma red Wi-Fi.
3. Abre SteamVR. Si Windows pregunta si `vrserver.exe` puede usar la red, permítelo en redes
   privadas. Si no pregunta y no llega nada al móvil, permite el puerto UDP 9944 en el firewall.
4. Abre la app en el móvil. Busca la PC sola y, al encontrarla, muestra su IP. Si no la encuentra,
   escribe la IP que da `ipconfig` en la PC.
5. Pulsa **Iniciar**. En unos segundos se ve SteamVR en el móvil, y al girarlo la vista gira igual.
   Abre SteamVR Home o un juego: sin ninguno abierto, la imagen puede verse casi negra.
6. Deja el móvil **plano y quieto sobre la mesa 5 segundos**. Así el driver mide cuánto se desvía su
   giroscopio, y la vista no gira sola mientras juegas. Repítelo cada vez que vuelvas a empezar.
7. Pon el móvil en el visor en horizontal, con la pantalla hacia los ojos y la parte de arriba del
   teléfono a la izquierda.

**Atrás** en el móvil detiene el envío. Si la conexión se corta, la app vuelve a buscar la PC sola.

## 5. Ajusta la imagen a tu visor

Toca la pantalla del móvil para abrir el ajuste. Los cambios se ven al instante y se guardan en el
teléfono. Ajusta con el móvil fuera del visor, pulsa **Listo** y compruébalo dentro.

1. **Separación entre centros**: empieza por la distancia entre los centros de las lentes de tu
   visor, medida con una regla.
2. **Ancho** y **alto de cada ojo**: el tamaño de la imagen de cada ojo, en milímetros de pantalla.
   Con el ancho igual al alto la imagen no se estira.
3. **Altura de los centros**: sube o baja las dos imágenes hasta alinearlas con las lentes.
4. **Corrección de lente k1 y k2**: marca **Cuadrícula**, mira a través del visor y sube k1 hasta
   que las líneas se vean rectas; k2 corrige sobre todo los bordes. Desmarca **Cuadrícula** al
   terminar.
5. **Anticipar el movimiento de la cabeza** viene marcado: la PC dibuja cada imagen donde estará la
   cabeza cuando llegue al móvil. Si la imagen se adelanta en los giros bruscos, desmárcalo.

**Restablecer** vuelve a la imagen a pantalla completa.

Para recentrar la vista, pulsa un botón de volumen del móvil o **Ctrl+Alt+R** en la PC. Si la vista
se desvía sola aunque dejaste el móvil quieto en la mesa al empezar, prueba
`"magnetometer_gain": 0.1` en `default.vrsettings`. Si entonces la vista se mece sola, vuelve a `0`.

## 6. Posición de la cabeza con la PS Eye (opcional)

PSMoveServiceEx sigue la luz del visor con la PS Eye, y el driver toma de ella la posición de la
cabeza: agacharte, inclinarte o dar un paso mueve la vista. El giro sigue llegando del móvil.

1. En VDM, pon **Virtual HMD count** en 1 y configura en PSMoveServiceEx ese visor virtual con el
   color y el tamaño de tu luz.
2. Mide con el visor puesto dónde está el centro de la luz respecto del punto entre tus ojos.
3. En `default.vrsettings` pon:

   ```
   "head_position_enabled" : true,
   "head_light_above_eyes_cm" : 7.0,
   "head_light_in_front_of_eyes_cm" : 6.0,
   "head_light_right_of_eyes_cm" : 0.0,
   ```

   Usa tus medidas: centímetros por encima de los ojos, por delante y hacia la derecha.
4. Abre PSMoveServiceEx (desde VDM) y SteamVR, ponte el visor, mira hacia la PS Eye y recentra con
   el volumen. Así la vista queda orientada hacia la cámara desde donde estés.
5. Corrige la distancia de la luz. PSMoveServiceEx calcula a qué distancia está la luz por su
   tamaño aparente, y casi siempre se equivoca. Con la distancia mal, la cabeza queda detrás o
   delante de tus manos. Necesita los mandos del paso 7 activados y un Move encendido:
   1. Pulsa **Ctrl+Alt+L** en la PC. Tienes un minuto para volver a donde juegas con el visor
      puesto.
   2. Apoya la bola del Move contra la luz del visor, donde la PS Eye vea las dos. Cuando las ve
      juntas, el Move vibra un instante: quédate quieto.
   3. A los 3 segundos vibra más largo: listo. La corrección queda guardada para las próximas veces.

   Repítelo si mueves la PS Eye o cambias la luz.

**Ctrl+Alt+P** congela la posición de la cabeza y la vuelve a soltar.

## 7. Los PS Move como mandos (opcional)

El driver publica los dos PS Move como mandos de SteamVR. PSMoveServiceEx los sigue igual que antes,
con la PS Eye y sus sensores; el driver solo decide qué mostrar:

- Mientras la PS Eye ve la bola, el mando está en la bola.
- Si la pierde más de 0,3 s, por ejemplo detrás de la espalda, el mando va frente al visor y lo
  sigue hacia donde miras. Queda 50 cm bajo los ojos, 30 cm adelante y 20 cm hacia su lado.
- Para volver a la bola, la PS Eye tiene que verla seguido 0,3 s. Así, si estás de espaldas y la
  cámara alcanza a ver un poco de la luz, el mando no salta. Tampoco va nunca a una bola a más de
  1,2 m del visor medidos sobre el piso, porque ningún brazo llega tan lejos.
- Con la mano quieta, el mando queda quieto. La PS Eye y PSMoveServiceEx hacen temblar la bola unos
  mm y el giro una fracción de grado, y el driver no pasa ese temblor mientras el giroscopio del
  Move dice que no te mueves. Al mover la mano, el mando la sigue sin retraso.

Para activarlo:

1. En `default.vrsettings` pon `"controllers_enabled" : true,`.
2. En VDM, en **Virtual Motion Tracker**, quita o desactiva los PS Move, o cada mando aparecerá dos
   veces. **No** desactives el complemento `psmsex` de SteamVR: VDM lo necesita.
3. Abre SteamVR: aparecen dos mandos, que los juegos ven como mandos Vive. Cada Move va a la mano que
   tenga asignada en PSMoveServiceEx; si no tiene ninguna, el primero es la derecha y el segundo la
   izquierda.
4. Si el mando apunta hacia arriba o hacia abajo cuando el Move apunta al frente, pon
   `controller_pitch_degrees` en 90 o -90.
5. Si un Move va girando de a poco, apúntalo a la PS Eye y mantén **Select + Start** un segundo.

Sin el paso 6, la cabeza no sabe dónde estás y los mandos pueden quedar corridos. Para alinearlos,
haz la **Playspace Calibration** de VDM pulsando **Ctrl+Alt+H** antes de empezarla y otra vez al
terminar: mientras tanto el visor queda exactamente quieto, que es lo que VDM necesita.

| Move | Mando Vive |
|------|------------|
| Gatillo | Gatillo |
| Botón Move (el grande), mantenido | Trackpad tocado hacia donde inclinas el Move (el menú de acciones y los gestos de VRChat) |
| Triángulo / Equis / Cuadrado / Círculo | Trackpad arriba / abajo / izquierda / derecha |
| Start | Grip |
| Select | Menú |
| PS | Sistema (el menú de SteamVR) |

Las vibraciones de los juegos hacen vibrar el Move. Cada juego se puede reasignar en SteamVR →
Mandos → Administrar asignaciones.

Ajustes en `default.vrsettings`, si hacen falta:

| Ajuste | Por defecto | Qué hace |
|--------|-------------|----------|
| `controller_rest_below_cm`, `controller_rest_forward_cm`, `controller_rest_side_cm` | 50, 30, 20 | Dónde queda un mando perdido frente al visor |
| `controller_rest_enabled` | true | `false`: el mando perdido se queda donde se perdió |
| `controller_found_ms` | 300 | Cuánto tiene que ver la PS Eye la bola para volver a ella |
| `controller_reach_cm` | 120 | A más distancia del visor, medida sobre el piso, la bola no se cree. `0` lo apaga |
| `controller_still_mm`, `controller_still_degrees` | 15, 0.6 | Cuánto temblor se tapa con la mano quieta. Súbelos si aún tiembla |
| `controller_depth_noise_cm` | 4 | Cuánto se suaviza la distancia de la bola a la PS Eye. Más alto, más suave y más lento |

## Si algo no funciona

Busca las líneas con `smartphone_hmd` en `Steam\logs\vrserver.txt`:

- `Video:`, cada 5 segundos: cuántas imágenes se capturan, codifican y envían. Mientras no llega
  video, el móvil muestra cuántos paquetes recibe.
- `Phone latency`: cuánto tarda la imagen en llegar al móvil y cuánto se anticipa la cabeza.
- `Phone:`, cada 10 segundos: si se perdieron datos del móvil por el camino, cuánto estuvo quieto
  en la mesa y cuánto se desvía su giroscopio. Si dice `its gyroscope's bias was not learned`, deja
  el móvil plano y quieto 5 segundos (paso 4.6).
- `Recentered`: cuántos grados se había desviado la vista desde el último recentrado.
- `Head light:`, cada 10 segundos: si la PS Eye ve la luz y a qué altura quedan los ojos. Si dice
  `the eyes come out … m high`, la distancia de la luz está mal: repite el paso 6.5. Si ya lo
  hiciste (`already measured`), PSMoveServiceEx tiene la PS Eye inclinada respecto de como está:
  vuelve a calibrar su posición en PSMoveServiceEx, o revisa la medida de `head_light_above_eyes_cm`.
- `Hands:`, cada 10 segundos, por cada mano: cuánto tiempo estuvo en la bola, frente al visor o
  quieto donde se perdió. También dice cuántas veces no creyó a la bola, por un salto, por verla
  solo un momento o por estar demasiado lejos, dónde quedó en promedio respecto de los ojos, y
  cuánto tembló con la mano quieta. Si la bola queda casi siempre `out of reach`, la distancia de
  la luz del visor está mal: repite el paso 6.5.

Si la imagen se corta o llega tarde, usa el cable USB o Wi-Fi de 5 GHz, o baja
`video_bitrate_mbps`.
