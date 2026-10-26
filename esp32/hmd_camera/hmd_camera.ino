// HMD Cámara on an ESP32-CAM (AI Thinker board, OV3660 or OV2640 camera): a body tracking camera
// you can put anywhere with 5 V. It does what the phone app does, over the same UDP packets, so
// tools/body_tracker.py and the driver take it as one more camera:
// - finds the PC by broadcasting a discover packet to ports 9946 (body_tracker.py) and 9944 (the
//   driver), and keeps doing it, slower, once found;
// - sends each camera frame to the PC's port 9946 as a JPEG picture in video packets (flags
//   keyframe and JPEG), stamped with when the camera took it;
// - answers the PC's pings at once, so the PC can put the frames on its own clock.
// See android/camera/.../BodyProtocol.kt for the packets.
//
// Arduino IDE: the "esp32" boards by Espressif, board "AI Thinker ESP32-CAM" (PSRAM on). Set your
// Wi-Fi below, upload, and open the serial monitor at 115200 baud to see its number and frame rate.

#include <WiFi.h>
#include <sys/time.h>

#include "esp_camera.h"
#include "esp_wifi.h"
#include "lwip/sockets.h"

// ---- Settings ---------------------------------------------------------------------------------

const char* WIFI_SSID = "tu-red";
const char* WIFI_PASSWORD = "tu-clave";

// 640x480: plenty for the pose model, which looks at 256x256, and the most frames a second.
const framesize_t FRAME_SIZE = FRAMESIZE_VGA;
// 0 to 63, lower is sharper and bigger. Around 12 a frame is 20-40 KB.
const int JPEG_QUALITY = 12;
// Quarter turns clockwise that make the picture upright, as the board is mounted (0 to 3): check
// with `python tools/body_tracker.py --show`. Upright pictures, standing boards: more of you fits.
const uint8_t QUARTER_TURNS = 0;
// The picture must not come out mirrored (text reads backwards): the cameras' calibration cannot
// undo that. Flip this if it does.
const bool MIRROR = false;
// 0: automatic exposure. Otherwise a fixed, short one, so moving arms and feet are not blurred
// (needs a well lit room): try 150-400, lower is shorter and darker; GAIN brightens it back
// (0-30), at the cost of noise.
const int EXPOSURE = 0;
const int GAIN = 4;

// ---- The packets ------------------------------------------------------------------------------

const uint16_t BODY_PORT = 9946;
const uint16_t DRIVER_PORT = 9944;
const uint8_t TYPE_DISCOVER = 4;
const uint8_t TYPE_PC_HERE = 5;
const uint8_t TYPE_TIME_PING = 8;
const uint8_t TYPE_TIME_PONG = 9;
const uint8_t TYPE_VIDEO = 10;
const uint8_t FLAG_EXPOSURE_LOCKED = 2;
const uint8_t FLAG_KEYFRAME = 8;
const uint8_t FLAG_JPEG = 16;
const size_t HEADER_SIZE = 20;
const size_t VIDEO_HEADER_SIZE = 36;
const size_t MAX_PAYLOAD = 1400;
const uint32_t SEARCH_MS = 1000;  // between discover packets while looking for the PC
const uint32_t FOUND_MS = 5000;   // and once found, to follow a new address

int sock = -1;
uint32_t device_id = 0;
volatile uint32_t pc_ip = 0;  // network order; 0 until found

// The camera stamps frames with gettimeofday(), the clock since start-up here: the pongs use it too.
int64_t NowNs()
{
    struct timeval tv;
    gettimeofday(&tv, nullptr);
    return (int64_t)tv.tv_sec * 1000000000LL + (int64_t)tv.tv_usec * 1000LL;
}

// Magic "HMD1", type, 3 bytes padding, seq (u32), timestamp (i64 ns); little-endian, as this chip.
void WriteHeader(uint8_t* out, uint8_t type, uint32_t seq, int64_t timestamp_ns)
{
    memcpy(out, "HMD1", 4);
    out[4] = type;
    out[5] = out[6] = out[7] = 0;
    memcpy(out + 8, &seq, 4);
    memcpy(out + 12, &timestamp_ns, 8);
}

void SendTo(const uint8_t* data, size_t size, uint32_t ip, uint16_t port)
{
    sockaddr_in to = {};
    to.sin_family = AF_INET;
    to.sin_port = htons(port);
    to.sin_addr.s_addr = ip;
    // Wi-Fi's send buffers fill up partway through a frame: wait for room rather than lose pieces.
    for (int tries = 0; tries < 20; ++tries) {
        if (sendto(sock, data, size, 0, (sockaddr*)&to, sizeof(to)) >= 0) {
            return;
        }
        delay(1);
    }
}

// Discovery, and the PC's answers and pings, on their own task so pings are answered at once.
void NetworkTask(void*)
{
    uint8_t packet[64];
    uint32_t next_discover = 0;
    for (;;) {
        const uint32_t now = millis();
        if (now >= next_discover && WiFi.isConnected()) {
            next_discover = now + (pc_ip ? FOUND_MS : SEARCH_MS);
            uint8_t discover[HEADER_SIZE];
            WriteHeader(discover, TYPE_DISCOVER, 0, 0);
            for (uint32_t ip : {(uint32_t)IPAddress(255, 255, 255, 255), (uint32_t)WiFi.broadcastIP()}) {
                SendTo(discover, sizeof(discover), ip, BODY_PORT);
                SendTo(discover, sizeof(discover), ip, DRIVER_PORT);
            }
        }
        sockaddr_in from = {};
        socklen_t from_size = sizeof(from);
        const int size = recvfrom(sock, packet, sizeof(packet), 0, (sockaddr*)&from, &from_size);
        if (size < (int)HEADER_SIZE || memcmp(packet, "HMD1", 4) != 0) {
            continue;
        }
        if (packet[4] == TYPE_PC_HERE) {
            if (pc_ip != from.sin_addr.s_addr) {
                pc_ip = from.sin_addr.s_addr;
                Serial.printf("PC found at %s\n", IPAddress(pc_ip).toString().c_str());
            }
        } else if (packet[4] == TYPE_TIME_PING) {
            // The ping's seq, our clock, then the PC's time echoed back and our number.
            const int64_t now_ns = NowNs();
            uint32_t seq;
            memcpy(&seq, packet + 8, 4);
            uint8_t pong[32];
            WriteHeader(pong, TYPE_TIME_PONG, seq, now_ns);
            memcpy(pong + 20, packet + 12, 8);
            memcpy(pong + 28, &device_id, 4);
            sendto(sock, pong, sizeof(pong), 0, (sockaddr*)&from, from_size);
        }
    }
}

void SendFrame(const camera_fb_t* fb, uint32_t seq)
{
    const int64_t taken_ns = (int64_t)fb->timestamp.tv_sec * 1000000000LL + (int64_t)fb->timestamp.tv_usec * 1000LL;
    const uint16_t pieces = (fb->len + MAX_PAYLOAD - 1) / MAX_PAYLOAD;
    const uint16_t width = fb->width, height = fb->height;
    const uint8_t flags = FLAG_KEYFRAME | FLAG_JPEG | (EXPOSURE > 0 ? FLAG_EXPOSURE_LOCKED : 0);
    static uint8_t packet[VIDEO_HEADER_SIZE + MAX_PAYLOAD];
    const uint32_t ip = pc_ip;
    for (uint16_t piece = 0; piece < pieces; ++piece) {
        const size_t offset = (size_t)piece * MAX_PAYLOAD;
        const size_t size = min(MAX_PAYLOAD, fb->len - offset);
        WriteHeader(packet, TYPE_VIDEO, seq, taken_ns);
        memcpy(packet + 20, &device_id, 4);
        memcpy(packet + 24, &width, 2);
        memcpy(packet + 26, &height, 2);
        memcpy(packet + 28, &piece, 2);
        memcpy(packet + 30, &pieces, 2);
        packet[32] = QUARTER_TURNS;
        packet[33] = flags;
        packet[34] = packet[35] = 0;
        memcpy(packet + VIDEO_HEADER_SIZE, fb->buf + offset, size);
        SendTo(packet, VIDEO_HEADER_SIZE + size, ip, BODY_PORT);
    }
}

bool StartCamera()
{
    camera_config_t config = {};
    // The AI Thinker board's pins.
    config.pin_pwdn = 32;
    config.pin_reset = -1;
    config.pin_xclk = 0;
    config.pin_sccb_sda = 26;
    config.pin_sccb_scl = 27;
    config.pin_d7 = 35;
    config.pin_d6 = 34;
    config.pin_d5 = 39;
    config.pin_d4 = 36;
    config.pin_d3 = 21;
    config.pin_d2 = 19;
    config.pin_d1 = 18;
    config.pin_d0 = 5;
    config.pin_vsync = 25;
    config.pin_href = 23;
    config.pin_pclk = 22;
    config.xclk_freq_hz = 20000000;
    config.ledc_timer = LEDC_TIMER_0;
    config.ledc_channel = LEDC_CHANNEL_0;
    config.pixel_format = PIXFORMAT_JPEG;
    config.frame_size = FRAME_SIZE;
    config.jpeg_quality = JPEG_QUALITY;
    // Two frames in PSRAM, always handing out the newest: an old frame is a late one.
    config.fb_count = 2;
    config.fb_location = CAMERA_FB_IN_PSRAM;
    config.grab_mode = CAMERA_GRAB_LATEST;
    if (esp_camera_init(&config) != ESP_OK) {
        return false;
    }
    sensor_t* s = esp_camera_sensor_get();
    if (s->id.PID == OV3660_PID) {
        // As Espressif's example sets it, so the OV3660 is upright and not too washed out.
        s->set_vflip(s, 1);
        s->set_brightness(s, 1);
        s->set_saturation(s, -2);
    }
    s->set_hmirror(s, MIRROR ? 1 : 0);
    if (EXPOSURE > 0) {
        s->set_exposure_ctrl(s, 0);
        s->set_aec_value(s, EXPOSURE);
        s->set_gain_ctrl(s, 0);
        s->set_agc_gain(s, GAIN);
    }
    Serial.printf("Camera %s, %ux%u\n", s->id.PID == OV3660_PID ? "OV3660" : "OV2640 or other",
                  resolution[FRAME_SIZE].width, resolution[FRAME_SIZE].height);
    return true;
}

void setup()
{
    Serial.begin(115200);
    // Its number, as the PC and the driver's log show it: from the board's MAC address.
    const uint64_t mac = ESP.getEfuseMac();
    device_id = (uint32_t)(mac >> 16) ^ (uint32_t)(mac & 0xFFFF);
    Serial.printf("\nHMD Cámara ESP32 %04X\n", (unsigned)(device_id & 0xFFFF));

    if (!StartCamera()) {
        Serial.println("No camera: check its ribbon cable. Restarting");
        delay(2000);
        ESP.restart();
    }

    WiFi.mode(WIFI_STA);
    WiFi.setSleep(false);  // sleeping between packets adds tens of ms to each frame
    WiFi.setAutoReconnect(true);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    Serial.printf("Connecting to %s", WIFI_SSID);
    while (!WiFi.isConnected()) {
        delay(250);
        Serial.print(".");
    }
    esp_wifi_set_ps(WIFI_PS_NONE);
    Serial.printf("\nConnected: %s\n", WiFi.localIP().toString().c_str());

    sock = socket(AF_INET, SOCK_DGRAM, 0);
    const int yes = 1;
    setsockopt(sock, SOL_SOCKET, SO_BROADCAST, &yes, sizeof(yes));
    timeval timeout = {0, 200000};
    setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
    sockaddr_in local = {};
    local.sin_family = AF_INET;
    local.sin_port = 0;
    local.sin_addr.s_addr = htonl(INADDR_ANY);
    bind(sock, (sockaddr*)&local, sizeof(local));
    // The camera loop runs on core 1 (Arduino's); the network next to Wi-Fi, on core 0.
    xTaskCreatePinnedToCore(NetworkTask, "network", 4096, nullptr, 5, nullptr, 0);
}

void loop()
{
    static uint32_t seq = 0;
    static uint32_t frames = 0;
    static uint32_t next_report = 0;
    camera_fb_t* fb = esp_camera_fb_get();
    if (!fb) {
        return;
    }
    if (pc_ip && WiFi.isConnected()) {
        SendFrame(fb, ++seq);
        ++frames;
    }
    esp_camera_fb_return(fb);
    const uint32_t now = millis();
    if (now >= next_report) {
        Serial.printf("%u frames/s%s\n", frames / 5, pc_ip ? "" : " (looking for the PC)");
        frames = 0;
        next_report = now + 5000;
    }
}
