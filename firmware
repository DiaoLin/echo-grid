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
	esp_err_t ret = i2s_read(I2S_PORT, sample, sizeof(sample), &samplesRead, 0); // 0=不阻塞！
	if(ret != ESP_OK || samplesRead != sizeof(sample)) {
		return false;
	}
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

	float energy = pow(10.0, lp / 10.0);
	energySum += energy;
	totalSamples += FRAME_SAMPLES;

	uint32_t windowSamples = SAMPLE_RATE * LEQ_DURATION;
	if(totalSamples >= windowSamples) {
		float avgEnergy = energySum / totalSamples;
		lastLeq = 10 * log10(avgEnergy);
		energySum = 0;
		totalSamples = 0;
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
/* Level2 预留函数
void uploadToCloud(float leqVal){
    return
}
*/
void setup() {
	Serial.begin(115200);
	delay(300);

	i2s_driver_install(I2S_PORT, &i2s_config, 0, NULL);
	i2s_set_pin(I2S_PORT, &pin_config);
	i2s_zero_dma_buffer(I2S_PORT);

	WiFi.begin(WIFI_SSID, WIFI_PASS);
	Serial.print("WiFi connecting...");
	while (WiFi.status() != WL_CONNECTED) {
		delay(400);
		Serial.print(".");
	}
	Serial.println("\nWiFi OK, IP: " + WiFi.localIP().toString());

	server.on("/", handleRoot);
	server.on("/api/data", handleApiData);
	server.begin();
	Serial.println("Web Server Started");
}
void loop() {
	updateLeq();
	server.handleClient();
	// i2s缓冲区
}
