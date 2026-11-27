#include <WiFi.h>
#include <WebServer.h>
#include <driver/i2s.h>
#include <math.h>

// ===================== 配置区 ===============================

const char* WIFI_SSID     = "你的WiFi名称";
const char* WIFI_PASS     = "你的WiFi密码";

// INMP441 在 32kHz 下通常比 44.1kHz 更容易稳定工作。
// 如果你的硬件或麦克风明确支持 44.1kHz，也可以改回 44100。
const int SAMPLE_RATE = 32000;

// 每帧样本数。
// 帧越短，实时性越好，但 CPU 调用更频繁；
// 帧越长，处理效率更高，但瞬时响应会稍慢。
const int FRAME_SAMPLES = 512;

// 用于计算每帧对应的真实时间。
// 32000Hz、512样本时，约为 0.016 秒。
const float FRAME_SECONDS = (float)FRAME_SAMPLES / SAMPLE_RATE;

// Leq 时间窗口。
const float LEQ_DURATION = 5.0f;
const uint32_t WINDOW_MS = (uint32_t)(LEQ_DURATION * 1000.0f);

// 校准偏移，单位 dB。
// 只有经过实际声级计或标准声源校准后，才建议修改。
const float CALIB_OFFSET = 0.0f;

// INMP441 接线引脚。
// 如果你实际接线不同，请同步修改这里。
const int SCK_PIN = 26;  // BCK / SCK
const int WS_PIN  = 25;  // WS / LRCK
const int SD_PIN  = 22;  // SD / DATA OUT

const i2s_port_t I2S_PORT = I2S_NUM_0;

// =============================================================

WebServer server(80);

float energySum = 0.0f;
uint32_t totalSamples = 0;

float lastLeq = 0.0f;
float lastLp = 0.0f;

uint32_t windowStartMs = 0;

bool wifiConnected = false;

const i2s_config_t i2s_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
    .sample_rate = SAMPLE_RATE,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
    .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
    .communication_format = I2S_COMM_FORMAT_STAND_I2S,
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = 8,
    .dma_buf_len = 1024,
    .use_apll = false,
    .tx_desc_auto_clear = false,
    .fixed_mclk = 0
};

const i2s_pin_config_t pin_config = {
    .mck_io_num = I2S_PIN_NO_CHANGE,
    .bck_io_num = SCK_PIN,
    .ws_io_num = WS_PIN,
    .data_out_num = I2S_PIN_NO_CHANGE,
    .data_in_num = SD_PIN
};

// 读取一帧音频，并计算该帧的瞬时声压级 Lp。
// 成功读满一整帧时返回 true。
bool readInstantLp(float& outLp) {
    int16_t sample[FRAME_SAMPLES];
    uint32_t bytesToRead = sizeof(sample);
    uint32_t bytesRead = 0;

    esp_err_t ret = i2s_read(
        I2S_PORT,
        sample,
        bytesToRead,
        &bytesRead,
        0
    );

    if (ret != ESP_OK) {
        return false;
    }

    if (bytesRead == 0) {
        return false;
    }

    if (bytesRead % sizeof(int16_t) != 0) {
        return false;
    }

    uint32_t samplesRead = bytesRead / sizeof(int16_t);

    if (samplesRead == 0) {
        return false;
    }

    float sumSq = 0.0f;

    for (uint32_t i = 0; i < samplesRead; i++) {
        float s = sample[i] / 32768.0f;
        sumSq += s * s;
    }

    float rms = sqrtf(sumSq / samplesRead);

    if (rms < 1e-6f) {
        rms = 1e-6f;
    }

    float lp = 20.0f * log10f(rms);
    lp += CALIB_OFFSET;

    outLp = lp;
    lastLp = lp;

    return true;
}

// 更新 Leq。
// 这里按每个有效帧的实际采样数进行能量加权，
// 避免短读帧被错误当成完整 512 样本帧处理。
void updateLeq() {
    float lp;

    if (!readInstantLp(lp)) {
        return;
    }

    // lp 是根据 RMS 得到的分贝值。
    // 10^(lp/10) 约等于该帧的平均声学能量。
    float energy = powf(10.0f, lp / 10.0f);

    energySum += energy * FRAME_SECONDS;
    totalSamples += FRAME_SAMPLES;

    uint32_t now = millis();

    if (now - windowStartMs >= WINDOW_MS && totalSamples > 0) {
        float avgEnergy = energySum / totalSamples;

        if (avgEnergy > 0.0f) {
            lastLeq = 10.0f * log10f(avgEnergy);
        }

        energySum = 0.0f;
        totalSamples = 0;
        windowStartMs = now;
    }
}

void handleApiData() {
    char buf[256];

    snprintf(
        buf,
        sizeof(buf),
        "{\"point_id\":\"P01\",\"location\":\"教学楼A走廊\",\"time\":%lu,\"db\":%.1f,\"period\":\"课间\",\"lp\":%.1f}",
        (unsigned long)millis(),
        (double)lastLeq,
        (double)lastLp
    );

    server.send(200, "application/json", String(buf));
}

void handleRoot() {
    server.send(
        200,
        "text/plain",
        "EchoGrid Node | Level1\n/api/data 获取噪声JSON\nFlow: INMP441-I2S->ESP32(Leq)->Web JSON"
    );
}

void setup() {
    Serial.begin(115200);
    delay(300);

    esp_err_t i2sRet = i2s_driver_install(I2S_PORT, &i2s_config, 0, NULL);

    if (i2sRet != ESP_OK) {
        Serial.printf("I2S driver install failed: %s\n", esp_err_to_name(i2sRet));
    }

    i2sRet = i2s_set_pin(I2S_PORT, &pin_config);

    if (i2sRet != ESP_OK) {
        Serial.printf("I2S pin config failed: %s\n", esp_err_to_name(i2sRet));
    }

    i2s_zero DMA_BUFFER:
    i2s_zero_dma_buffer(I2S_PORT);

    windowStartMs = millis();

    WiFi.begin(WIFI_SSID, WIFI_PASS);
    Serial.print("WiFi connecting...");

    uint32_t wifiStart = millis();
    const uint32_t WIFI_TIMEOUT = 10000;

    while (WiFi.status() != WL_CONNECTED && millis() - wifiStart < WIFI_TIMEOUT) {
        delay(200);
        Serial.print(".");
    }

    if (WiFi.status() == WL_CONNECTED) {
        wifiConnected = true;
        Serial.println("\nWiFi connected, IP: " + WiFi.localIP().toString());
    } else {
        wifiConnected = false;
        Serial.println("\nWiFi connect timeout, continue run without WiFi");
    }

    server.on("/api/data", handleApiData);
    server.on("/", handleRoot);
    server.begin();

    Serial.println("Web server started");
}

void loop() {
    server.handleClient();
    updateLeq();

    static uint32_t reconnectTimer = 0;

    if (WiFi.status() != WL_CONNECTED && millis() - reconnectTimer > 5000) {
        reconnectTimer = millis();
        Serial.println("WiFi reconnecting...");
        WiFi.begin(WIFI_SSID, WIFI_PASS);
    }
}
