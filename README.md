# smartphone-hmd

Convierte un smartphone Android en un visor (HMD) que SteamVR reconoce.

- `android/`: app Kotlin que envía los sensores del móvil por UDP y muestra la imagen.
- `driver/`: driver de SteamVR en C++ que recibe los sensores y se presenta como un HMD estéreo.
- `tools/sim/send_imu.py`: simulador que se hace pasar por el móvil, para probar el driver sin él.
- `docs/PLAN.md`: plan del proyecto y protocolo.

## Descargar las compilaciones

Cada push a `main` y cada pull request compilan el APK y el driver en GitHub Actions.
Ve a la pestaña **Actions**, abre la ejecución más reciente y descarga:

- `smartphone-hmd-apk`: la app para instalar en el móvil.
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
3. Conecta el móvil y la PC a la misma red Wi-Fi. Averigua la IP de la PC con `ipconfig`.
4. Abre SteamVR. La primera vez Windows puede preguntar si `vrserver.exe` puede usar la red:
   permítelo en redes privadas. Si no pregunta y no llegan datos, permite el puerto UDP 9944 en el
   firewall de Windows.
5. En el móvil escribe la IP de la PC y pulsa **Iniciar**. El móvil pasa a pantalla completa y, en
   unos segundos, muestra la imagen estéreo de SteamVR. Al girar el móvil, la vista gira igual.
   **Atrás** detiene el envío.

Para recentrar la vista, pulsa cualquier botón de volumen del móvil o **Ctrl+Alt+R** en la PC.

El móvil va en horizontal dentro del visor, con la pantalla hacia los ojos y la parte de arriba del
teléfono a la izquierda.

La calidad del video se ajusta con `video_bitrate_mbps` (por defecto 20) en el mismo archivo de
configuración. Si la imagen se corta o llega con retraso, prueba un router de 5 GHz, bajar ese valor
o conectar el móvil por cable USB con el anclaje USB activado.

Si no llega la imagen, busca las líneas que contienen `smartphone_hmd` en
`Steam\logs\vrserver.txt`: dicen qué GPU y qué codificador se usan y si algo falló.

Sin el móvil, `python tools/sim/send_imu.py` envía datos que giran la cabeza a izquierda y derecha.
