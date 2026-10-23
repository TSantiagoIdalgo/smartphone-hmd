#!/usr/bin/env bash
# Installs the body camera app on a running emulator, opens it and checks that it stays alive,
# loads the pose model and opens the camera or says there is none. Prints the crash stack trace
# if it died. Usage: camera_smoke_test.sh camera.apk
set -u
APK="$1"
PKG=com.smartphonehmd.camera
TAG=HmdCamera

alive() {
    adb shell pidof "$PKG" > /dev/null 2>&1
}

fail() {
    echo "::error::$1"
    echo "----- crash buffer -----"
    adb logcat -d -b crash
    echo "----- app log -----"
    adb logcat -d | grep -E "AndroidRuntime|$PKG|$TAG|mediapipe|tflite" | tail -200
    exit 1
}

# True once the app has logged a line matching the regex $1.
logged() {
    adb logcat -d -s "$TAG:*" | grep -qE "$1"
}

adb install -r "$APK" || fail "Install failed"
adb shell pm grant "$PKG" android.permission.CAMERA || fail "Could not grant the camera permission"
adb logcat -c

echo "== Opening the app"
adb shell am start -W -n "$PKG/.MainActivity"
for _ in $(seq 1 30); do
    alive || fail "The app closed after opening"
    logged 'Pose model (ready|failed)' && break
    sleep 2
done
logged 'Pose model ready' || fail "The pose model did not load"
adb logcat -d -s "$TAG:*" | grep -E 'Pose model|Camera|No camera'
logged 'Camera (front|back)|No camera' || fail "The app neither opened a camera nor said there is none"

echo "== Going to the background and back"
adb shell input keyevent 3 # Home
sleep 2
adb shell am start -W -n "$PKG/.MainActivity"
sleep 5
alive || fail "The app closed after coming back from the background"

echo "Camera smoke test passed"
