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

UI_XML=$(mktemp)

# Saves the screen's views, one uiautomator node per line, to $UI_XML.
dump_ui() {
    local attempt
    for attempt in 1 2 3; do
        adb shell rm -f /sdcard/ui.xml
        adb shell uiautomator dump /sdcard/ui.xml > /dev/null 2>&1
        adb shell cat /sdcard/ui.xml 2> /dev/null | tr '>' '\n' > "$UI_XML"
        grep -q "<hierarchy" "$UI_XML" && return 0
        sleep 2
    done
    return 1
}

# True if a view on screen matches the regex $1, ignoring case (newer Android reports button
# text in capitals).
has_node() {
    dump_ui && grep -qiE "$1" "$UI_XML"
}

# Taps the centre of the first view matching the regex $1, ignoring case.
tap_node() {
    dump_ui || return 1
    local bounds
    bounds=$(grep -iE "$1" "$UI_XML" | head -1 |
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
has_node 'Enviando sensores' || fail "The streaming screen did not appear"
if grep -qi 'OpenGL' "$UI_XML"; then
    grep -i 'OpenGL' "$UI_XML"
    fail "The video view could not start OpenGL"
fi

echo "== Tapping the screen opens the eye adjustment, Back closes it"
tap_node 'Enviando sensores' || fail "Could not find the streaming screen"
sleep 2
alive || fail "The app closed after tapping the screen"
has_node 'Ancho de cada ojo' || fail "The eye adjustment did not open"
adb shell input keyevent 4
sleep 2
alive || fail "The app closed after closing the eye adjustment"
has_node 'Ancho de cada ojo' && fail "Back did not close the eye adjustment"

echo "== Going to the background and back while streaming"
adb shell input keyevent 3 # Home
sleep 2
adb shell am start -W -n "$PKG/.MainActivity"
sleep 3
alive || fail "The app closed after coming back to the video"
has_node 'Enviando sensores' || fail "The video screen did not come back"

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
