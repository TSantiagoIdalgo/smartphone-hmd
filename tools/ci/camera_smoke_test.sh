#!/usr/bin/env bash
# Installs the body camera APK on a running emulator and opens it with its encoder self-test: the
# app must stay alive, open the camera or say there is none, and encode a made-up picture that
# then decodes on this side (as on the PC) into the same picture. Prints the crash stack trace if
# it died. Usage: camera_smoke_test.sh camera.apk  (needs Python with PyAV)
set -u
APK="$1"
PKG=com.smartphonehmd.camera
TAG=HmdCamera
# What the app logs once it has a camera, has none, or could not start CameraX.
CAMERA_RESULT='Camera (front|back|provider failed)|No camera'
SELF_TEST_RESULT='Encoder self-test'

alive() {
    adb shell pidof "$PKG" > /dev/null 2>&1
}

fail() {
    echo "::error::$1"
    echo "----- crash buffer -----"
    adb logcat -d -b crash
    echo "----- app log -----"
    adb logcat -d | grep -E "AndroidRuntime|$PKG|$TAG|MediaCodec|CCodec|OMX" | tail -200
    exit 1
}

# True once the app has logged a line matching the regex $1.
logged() {
    adb logcat -d -s "$TAG:*" | grep -qE "$1"
}

adb install -r "$APK" || fail "Install failed"
adb shell pm grant "$PKG" android.permission.CAMERA || fail "Could not grant the camera permission"
adb logcat -c

echo "== Opening the app with the encoder self-test"
adb shell am start -W -n "$PKG/.MainActivity" --ez selftest true
for _ in $(seq 1 30); do
    alive || fail "The app closed after opening"
    logged "$SELF_TEST_RESULT" && logged "$CAMERA_RESULT" && break
    sleep 2
done
adb logcat -d -s "$TAG:*" | grep -E "$SELF_TEST_RESULT|$CAMERA_RESULT|Video encoder"
logged "$CAMERA_RESULT" || fail "The app neither opened a camera nor said why not"
logged "$SELF_TEST_RESULT: [0-9]+ frames, ([2-9]|[1-9][0-9]+) keyframes" ||
    fail "The encoder self-test did not encode two seconds with a requested keyframe"

echo "== Decoding what the phone's encoder made"
adb exec-out run-as "$PKG" cat files/selftest.h264 > selftest.h264 || fail "Could not copy the self-test video"
python3 tools/ci/check_selftest_video.py selftest.h264 || fail "The self-test video is wrong"

echo "== Going to the background and back"
adb shell input keyevent 3 # Home
sleep 2
adb shell am start -W -n "$PKG/.MainActivity"
sleep 5
alive || fail "The app closed after coming back from the background"

echo "Camera smoke test passed"
