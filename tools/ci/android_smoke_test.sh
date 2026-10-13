#!/usr/bin/env bash
# Installs the APK on a running emulator, opens the app, starts streaming and checks that it
# is still alive. Prints the crash stack trace if it died. Usage: android_smoke_test.sh app.apk
set -u
APK="$1"
PKG=com.smartphonehmd.app

alive() {
    adb shell pidof "$PKG" > /dev/null 2>&1
}

fail() {
    echo "::error::$1"
    echo "----- crash buffer -----"
    adb logcat -d -b crash
    echo "----- app log -----"
    adb logcat -d | grep -E "AndroidRuntime|$PKG|smartphonehmd" | tail -200
    exit 1
}

# Taps the centre of the first view whose uiautomator node matches the regex $1, ignoring case
# (newer Android reports button text in capitals).
tap_node() {
    local attempt
    for attempt in 1 2 3; do
        adb shell rm -f /sdcard/ui.xml
        adb shell uiautomator dump /sdcard/ui.xml > /dev/null 2>&1
        adb shell ls /sdcard/ui.xml > /dev/null 2>&1 && break
        sleep 2
    done
    local bounds
    bounds=$(adb shell cat /sdcard/ui.xml | tr '>' '\n' | grep -iE "$1" | head -1 |
        grep -oE 'bounds="\[[0-9]+,[0-9]+\]\[[0-9]+,[0-9]+\]"' | grep -oE '[0-9]+')
    [ -n "$bounds" ] || return 1
    set -- $bounds
    adb shell input tap $(( ($1 + $3) / 2 )) $(( ($2 + $4) / 2 ))
}

adb install -r "$APK" || fail "Install failed"
adb logcat -c

echo "== Opening the app"
adb shell am start -W -n "$PKG/.MainActivity"
sleep 5
alive || fail "The app closed right after opening"
echo "Still running after opening"

echo "== Typing an IP and pressing Iniciar"
tap_node 'class="android.widget.EditText"' || fail "Could not find the IP field"
adb shell input text "10.0.2.2"
adb shell input keyevent 111 # Escape closes the keyboard
tap_node 'text="Iniciar"' || fail "Could not find the Iniciar button"
sleep 5
alive || fail "The app closed after pressing Iniciar"
echo "Still running while streaming"

echo "== Recentering with volume up, then Back"
adb shell input keyevent 24
sleep 1
adb shell input keyevent 4
sleep 2
alive || fail "The app closed after Back"

echo "== Rotating through pause and resume"
adb shell input keyevent 3 # Home
sleep 2
adb shell am start -W -n "$PKG/.MainActivity"
sleep 3
alive || fail "The app closed after coming back from the background"

echo "Smoke test passed"
adb logcat -d | grep -E "AndroidRuntime" | tail -50 || true
