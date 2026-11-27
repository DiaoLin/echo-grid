#include <WiFi.h>
#include <WebServer.h>
#include <driver/i2s.h>
#include <math.h>
// ===================== 配置区 ===============================
const char* WIFI_SSID     = "你的WiFi名称";
const char* WIFI_PASS     = "你的WiFi密码";
const float CALIB_OFFSET  = 0.0;
const int SCK_PIN         = 26;
const int WS_PIN          = 25;
const int SD_PIN          = 22;
const i2s_port_t I2S_PORT = I2S_NUM_0;
const float LEQ_DURATION = 5.0;
const int SAMPLE_RATE = 44100;
const int FRAME_SAMPLES = 512;
// =============================================================
WebServer server(80);
float energySum = 0.0;
uint32_t totalSamples = 0;
float lastLeq = 0.0;
float lastLp = 0.0;

// 新增：用于时间窗口计时
uint32_t windowStartMs = 0;
const uint32_t WINDOW_MS = LEQ_DURATION * 1000;

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
	.bck_io_num = SCK_PIN,
	.ws_io_num = WS_PIN,
	.data_out_num = I2S_PIN_NO_CHANGE,
	.data_in_num = SD_PIN
};
// 成功返回true,非阻塞读取一帧
bool readInstantLp(float &outLp) {
	int16_t sample[FRAME_SAMPLES];
	uint32_t samplesRead;
	esp_err_t ret = i2s_read(I2S_PORT, sample, sizeof(sample), &samplesRead, 0); 
	// 1. 首先处理真正的错误
	if (ret != ESP_OK) {
	    // 这里是硬件或参数错误，可以记录日志或进行错误恢复
	    return false; 
	}
	// 2. 检查是否读到了完整的一帧数据
	if (samplesRead != sizeof(sample)) {
	    // 数据不完整（短读）。在 timeout=0 下，这通常意味着“暂时没有足够数据”。
	    // 这不应该被视为致命错误，而应视为“本轮无完整数据”并等待下一次读取。
	    // 例如，可以返回一个特殊的状态码，或者让调用者决定是否重试。
	    return false; // 或返回其他表示“数据不足”的标识
	}
	// 3. 成功读满一帧
	float sumSq = 0.0;
	for(int i=0; i<FRAME_SAMPLES; i++) {
		float s = sample[i] / 32768.0f;
		sumSq += s * s;
	}
	float rms = sqrt(sumSq / FRAME_SAMPLES);
	if(rms < 1e-6) rms = 1e-6;
	float lp = 20 * log10(rms);
	lp += CALIB_OFFSET;
	outLp = lp;
	lastLp = lp;
	return true;
}

void updateLeq() {
	float lp;
	if(!readInstantLp(lp)) return;

	float energy = pow(10.0f, lp / 10.0f);
	energySum += energy * FRAME_SAMPLES;
	totalSamples += FRAME_SAMPLES;

	// 基于真实时间判断窗口是否到期
	uint32_t now = millis();
	if (now - windowStartMs >= WINDOW_MS && totalSamples > 0) {
		float avgEnergy = energySum / totalSamples;
		lastLeq = 10 * log10(avgEnergy);
		// 重置窗口
		energySum = 0;
		totalSamples = 0;
		windowStartMs = now;
	}
}

void handleApiData() {
	String json = "{";
	json += "\"point_id\":\"P01\",";
	json += "\"location\":\"教学楼A走廊\",";
	json += "\"time\":\"" + String(millis()) + "\",";
	json += "\"db\":" + String(lastLeq,1) + ",";
	json += "\"period\":\"课间\",";
	json += "\"lp\":" + String(lastLp,1);
	json += "}";
	server.send(200, "application/json", json);
}

void handleRoot() {
	server.send(200, "text/plain",
	            "EchoGrid Node | Level1\n/api/data 获取噪声JSON\nFlow: INMP441-I2S->ESP32(Leq)->Web JSON");
}

void setup() {
	Serial.begin(115200);
	delay(300);

	i2s_driver_install(I2S_PORT, &i2s_config, 0, NULL);
	i2s_set_pin(I2S_PORT, &pin_config);
	i2s_zero_dma_buffer(I2S_PORT);

	// 初始化计时起点
	windowStartMs = millis();

	WiFi.begin(WIFI_SSID, WIFI_PASS);
	Serial.print("WiFi connecting...");
	uint32_t wifiStart = millis();
	const uint32_t WIFI_TIMEOUT = 10000; // 10秒超时
	while (WiFi.status() != WL_CONNECTED && millis() - wifiStart < WIFI_TIMEOUT) {
		delay(200);
		Serial.print(".");
	}
	if (WiFi.status() == WL_CONNECTED) {
		Serial.println("\nWiFi connected, IP: " + WiFi.localIP().toString());
	} else {
		Serial.println("\nWiFi connect timeout, continue run without WiFi");
	}

	server.on("/api/data", handleApiData);
	server.begin();
	Serial.println("Web server started");
}

void loop() {
	server.handleClient();
	updateLeq();

	// 可选：断线自动重连
	static uint32_t reconnectTimer = 0;
	if (WiFi.status() != WL_CONNECTED && millis() - reconnectTimer > 5000) {
		reconnectTimer = millis();
		WiFi.begin(WIFI_SSID, WIFI_PASS);
	}
}
