#!/usr/bin/env bash
# Checks the body camera APK packs the pose model and library, installs it on a running emulator,
# opens it and checks that it stays alive and opens the camera or says there is none. Prints the
# crash stack trace if it died. Usage: camera_smoke_test.sh camera.apk
set -u
APK="$1"
PKG=com.smartphonehmd.camera
TAG=HmdCamera
# What the app logs once it has a camera, has none, or could not start CameraX.
CAMERA_RESULT='Camera (front|back|provider failed)|No camera'

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

echo "== Checking the APK carries the pose model, stored uncompressed, and the phone library"
unzip -l "$APK" | grep -E '\.so$|\.task$'
zipinfo "$APK" assets/pose_landmarker_lite.task | grep -q ' stor ' ||
    fail "The pose model is missing from the APK or compressed"
unzip -l "$APK" | grep -q 'lib/arm64-v8a/libmediapipe_tasks_vision_jni.so' ||
    fail "The APK has no arm64 pose library"

adb install -r "$APK" || fail "Install failed"
adb shell pm grant "$PKG" android.permission.CAMERA || fail "Could not grant the camera permission"
adb logcat -c

echo "== Opening the app"
adb shell am start -W -n "$PKG/.MainActivity"
# The model and the camera start on their own; wait for both to say how it went.
for _ in $(seq 1 30); do
    alive || fail "The app closed after opening"
    logged 'Pose model (ready|failed)' && logged "$CAMERA_RESULT" && break
    sleep 2
done
# MediaPipe ships no x86_64 library, so on the emulator the model cannot load: the app must say
# so and keep running. Phones (arm64) load it.
if logged 'Pose model ready'; then
    echo "Pose model loaded"
elif adb logcat -d -s "$TAG:*" | grep -q 'libmediapipe_tasks_vision_jni.so'; then
    echo "No MediaPipe library for this emulator's x86_64; the app reported it and kept running"
else
    fail "The pose model did not load"
fi
adb logcat -d -s "$TAG:*" | grep -E "Pose model (ready|failed)|$CAMERA_RESULT"
logged "$CAMERA_RESULT" || fail "The app neither opened a camera nor said why not"

echo "== Going to the background and back"
adb shell input keyevent 3 # Home
sleep 2
adb shell am start -W -n "$PKG/.MainActivity"
sleep 5
alive || fail "The app closed after coming back from the background"

echo "Camera smoke test passed"
