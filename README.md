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

## Probar el seguimiento de la cabeza

1. Instala el APK en el móvil y el driver en SteamVR (ver arriba).
2. Conecta el móvil y la PC a la misma red Wi-Fi. Averigua la IP de la PC con `ipconfig`.
3. Abre SteamVR. La primera vez Windows puede preguntar si `vrserver.exe` puede usar la red:
   permítelo en redes privadas. Si no pregunta y no llegan datos, permite el puerto UDP 9944 en el
   firewall de Windows.
4. En el móvil escribe la IP de la PC y pulsa **Iniciar**.
5. En SteamVR abre **Mostrar vista de VR** (Display VR View). Al girar el móvil, la vista estéreo
   debe girar igual.

Para recentrar la vista, pulsa cualquier botón de volumen del móvil o **Ctrl+Alt+R** en la PC.

El móvil va en horizontal dentro del visor, con la pantalla hacia los ojos y la parte de arriba del
teléfono a la izquierda.

Sin el móvil, `python tools/sim/send_imu.py` envía datos que giran la cabeza a izquierda y derecha.
