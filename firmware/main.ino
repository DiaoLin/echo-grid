#include <WiFi.h>
#include <WebServer.h>
#include <driver/i2s.h>
#include <math.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <time.h>
#include <esp_err.h>
#include <WiFiClientSecure.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>

// ===================== 配置区 ===============================
const char* WIFI_SSID     = "你的WiFi名称";
const char* WIFI_PASS     = "你的WiFi密码";
const String API_BASE     = "https://echogrid.cc.cd";

const char* POINT_ID      = "P01";
const char* LOCATION      = "教学楼A走廊";

const int SAMPLE_RATE  = 32000;
const int FRAME_SAMPLES = 512;

const float LEQ_DURATION = 5.0f;
const float CALIB_OFFSET = 0.0f;

const int SCK_PIN = 26;
const int WS_PIN  = 25;
const int SD_PIN  = 22;

const i2s_port_t I2S_PORT = I2S_NUM_0;

const uint32_t WIFI_CONN_TIMEOUT        = 10000;
const uint32_t WIFI_RECONNECT_INTERVAL  = 5000;
const uint32_t UPLOAD_INTERVAL          = 5000;
const int MAX_CACHE_NUM = 50;
const int QUEUE_DEPTH   = 8;
// =============================================================

WebServer server(80);

float energySum = 0.0f;
float sumT      = 0.0f;

float lastLeq = 0.0f;
float lastLp  = 0.0f;
bool  leqValid = false;
bool  leqDirty = false;      // C3: 窗口切换置 true，入队成功清 false

// 上报队列（POD 结构，安全 memcpy）
struct ReadingItem {
  char  point_id[16];
  char  location[64];
  char  time_iso[40];
  float db;
  char  period[16];
};

QueueHandle_t     uploadQueue  = NULL;
SemaphoreHandle_t cacheMutex   = NULL;

// 环形缓冲：head 指向最旧，cnt 为当前条数
ReadingItem cacheBuf[MAX_CACHE_NUM];
int cacheHead = 0;
int cacheCnt  = 0;

uint32_t lastUploadTick = 0;
static bool ntpLogPrinted = false;

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

// ---------- 工具函数 ----------
String getISO8601Time()
{
  struct tm now;
  if (!getLocalTime(&now))
  {
    if (!ntpLogPrinted) { Serial.println("[NTP] time sync not ready"); ntpLogPrinted = true; }
    return "";
  }
  ntpLogPrinted = false;
  char buf[32];
  strftime(buf, sizeof(buf), "%Y-%m-%dT%H:%M:%S+08:00", &now);
  return String(buf);
}

void copyToBuf(char* dst, size_t dstSize, const String& s)
{
  if (dstSize == 0) return;
  strncpy(dst, s.c_str(), dstSize - 1);
  dst[dstSize - 1] = '\0';
}

// ---------- WiFi ----------
void wifiServiceTick()
{
  static uint32_t reconnectTimer = 0;
  wl_status_t st = WiFi.status();
  if (st == WL_CONNECTED) return;
  if (st != WL_DISCONNECTED && st != WL_CONNECT_FAILED && st != WL_NO_SSID_AVAIL)
    return;
  if (millis() - reconnectTimer > WIFI_RECONNECT_INTERVAL)
  {
    reconnectTimer = millis();
    Serial.println("[WiFi] reconnecting...");
    WiFi.begin(WIFI_SSID, WIFI_PASS);
  }
}

// ---------- HTTP ----------
static inline bool httpIsSuccess(int code) { return code >= 200 && code < 300; }

bool uploadSingleReading(const ReadingItem& item)
{
  if (WiFi.status() != WL_CONNECTED) return false;
  if (item.time_iso[0] == '\0')     return false;

  WiFiClientSecure client;
  client.setInsecure();
  HTTPClient http;
  http.setTimeout(6000);                    // C6
  String url = API_BASE + "/api/readings";
  http.begin(client, url);
  http.addHeader("Content-Type", "application/json");

  StaticJsonDocument<256> doc;
  doc["point_id"] = item.point_id;
  doc["location"] = item.location;
  doc["time"]     = item.time_iso;
  doc["db"]       = item.db;
  if (item.period[0] != '\0') doc["period"] = item.period;

  String payload;
  serializeJson(doc, payload);
  int code = http.POST(payload);
  http.end();

  if (code < 0) { Serial.printf("[HTTP] single fail, code=%d\n", code); return false; }
  return httpIsSuccess(code);               // C5
}

bool batchUploadReadings(ReadingItem arr[], int num)
{
  if (WiFi.status() != WL_CONNECTED || num <= 0) return false;

  WiFiClientSecure client;
  client.setInsecure();
  HTTPClient http;
  http.setTimeout(6000);                    // C6
  String url = API_BASE + "/api/readings/batch";
  http.begin(client, url);
  http.addHeader("Content-Type", "application/json");

  static DynamicJsonDocument doc(16384);    // S1
  doc.clear();
  JsonArray arrJson = doc["readings"].to<JsonArray>();

  for (int i = 0; i < num; i++)
  {
    if (arr[i].time_iso[0] == '\0') continue;
    JsonObject obj = arrJson.add<JsonObject>();
    obj["point_id"] = arr[i].point_id;
    obj["location"] = arr[i].location;
    obj["time"]     = arr[i].time_iso;
    obj["db"]       = arr[i].db;
    if (arr[i].period[0] != '\0') obj["period"] = arr[i].period;
  }

  if (arrJson.size() == 0)
  {
    http.end();
    Serial.println("[Batch] all invalid, keep cache");   // C2
    return false;
  }

  String payload;
  serializeJson(doc, payload);
  int code = http.POST(payload);
  http.end();

  if (!httpIsSuccess(code))                 // C5
  {
    Serial.printf("[HTTP] batch fail, code=%d\n", code);
    return false;
  }
  return true;
}

// ---------- 环形缓存（C1） ----------
void pushCache(const ReadingItem& item)
{
  if (item.time_iso[0] == '\0') { Serial.println("[Cache] skip empty time"); return; }

  if (xSemaphoreTake(cacheMutex, pdMS_TO_TICKS(100)) != pdTRUE)
  {
    Serial.println("[Cache] mutex timeout, drop reading");
    return;
  }

  int idx = (cacheHead + cacheCnt) % MAX_CACHE_NUM;
  if (cacheCnt >= MAX_CACHE_NUM)
  {
    cacheHead = (cacheHead + 1) % MAX_CACHE_NUM;   // 覆盖最旧
    Serial.println("[Cache] full, overwrite oldest");
  }
  else
  {
    cacheCnt++;
  }
  cacheBuf[idx] = item;

  xSemaphoreGive(cacheMutex);
}

void flushCache()
{
  ReadingItem* local = NULL;
  int n = 0;
  int headSnapshot = 0;

  if (xSemaphoreTake(cacheMutex, pdMS_TO_TICKS(100)) != pdTRUE)
  {
    Serial.println("[Cache] flush: mutex timeout");
    return;
  }
  n = cacheCnt;
  headSnapshot = cacheHead;
  if (n > 0)
  {
    local = (ReadingItem*)malloc(sizeof(ReadingItem) * n);
    if (!local) { xSemaphoreGive(cacheMutex); Serial.println("[Cache] malloc failed"); return; }
    for (int i = 0; i < n; i++)
      local[i] = cacheBuf[(headSnapshot + i) % MAX_CACHE_NUM];
  }
  xSemaphoreGive(cacheMutex);

  if (n <= 0 || !local) return;

  bool ok = batchUploadReadings(local, n);
  free(local);

  if (ok)
  {
    if (xSemaphoreTake(cacheMutex, pdMS_TO_TICKS(100)) == pdTRUE)
    {
      if (cacheHead == headSnapshot && cacheCnt >= n)
      {
        cacheHead = (cacheHead + n) % MAX_CACHE_NUM;
        cacheCnt -= n;
        Serial.printf("[Cache] flush success, remain %d\n", cacheCnt);
      }
      else
      {
        // 发送期间被覆盖：保留缓存，宁可重发不丢
        Serial.println("[Cache] head changed during send, keep cache");
      }
      xSemaphoreGive(cacheMutex);
    }
    else Serial.println("[Cache] flush ok but mutex timeout");
  }
  else Serial.println("[Cache] flush failed, retry later");
}

int readCacheCnt()
{
  int v = 0;
  if (xSemaphoreTake(cacheMutex, pdMS_TO_TICKS(50)) == pdTRUE) { v = cacheCnt; xSemaphoreGive(cacheMutex); }
  return v;
}

// ---------- 上报任务 ----------
void uploadTask(void *pvParam)
{
  ReadingItem item;
  uint32_t backoffMs = 0;
  uint32_t lastFlushMs = 0;

  for (;;)
  {
    if (xQueueReceive(uploadQueue, &item, portMAX_DELAY) == pdTRUE)
    {
      if (!uploadSingleReading(item)) pushCache(item);

      uint32_t now = millis();
      if (now - lastFlushMs >= backoffMs)
      {
        lastFlushMs = now;
        int before = readCacheCnt();
        flushCache();
        int after = readCacheCnt();
        if (after >= before && before > 0)
        {
          backoffMs = (backoffMs == 0) ? 5000 : backoffMs * 2;
          if (backoffMs > 60000) backoffMs = 60000;
        }
        else backoffMs = 0;
      }

      vTaskDelay(pdMS_TO_TICKS(1));
    }
  }
}

// ---------- I2S / Leq ----------
bool readInstantLp(float& outLp, uint32_t& outSamplesRead)
{
  int16_t sample[FRAME_SAMPLES];
  uint32_t bytesToRead = sizeof(sample);
  uint32_t bytesRead = 0;

  esp_err_t ret = i2s_read(I2S_PORT, sample, bytesToRead, &bytesRead, pdMS_TO_TICKS(100));
  if (ret != ESP_OK || bytesRead == 0) return false;
  if (bytesRead % sizeof(int16_t) != 0)  return false;

  uint32_t samplesRead = bytesRead / sizeof(int16_t);
  outSamplesRead = samplesRead;
  if (samplesRead == 0) return false;

  float sumSq = 0.0f;
  for (uint32_t i = 0; i < samplesRead; i++)
  {
    float s = sample[i] / 32768.0f;
    sumSq += s * s;
  }
  float rms = sqrtf(sumSq / samplesRead);
  if (rms < 1e-6f) rms = 1e-6f;

  float lp = 20.0f * log10f(rms) + CALIB_OFFSET;
  outLp = lp;
  lastLp = lp;
  return true;
}

void updateLeq()
{
  float lp;
  uint32_t samplesRead;
  if (!readInstantLp(lp, samplesRead)) return;

  float deltaT = (float)samplesRead / SAMPLE_RATE;
  float energy = powf(10.0f, lp / 10.0f);

  energySum += energy * deltaT;
  sumT      += deltaT;

  if (sumT >= LEQ_DURATION)
  {
    lastLeq = 10.0f * log10f(energySum / sumT);
    leqValid = true;
    leqDirty = true;                         // C3
    Serial.printf("[Leq] window, lastLeq=%.2f, sumT=%.3f\n", lastLeq, sumT);
    energySum = 0.0f;
    sumT = 0.0f;
  }
}

// ---------- Web ----------
void handleApiData()
{
  String t = getISO8601Time();
  StaticJsonDocument<384> doc;
  doc["point_id"] = POINT_ID;
  doc["location"] = LOCATION;
  doc["time"]     = t;
  doc["leqValid"] = leqValid;
  if (leqValid) doc["db"] = lastLeq;
  doc["lp"] = lastLp;

  String payload;
  serializeJson(doc, payload);
  server.send(200, "application/json", payload);
}

void handleRoot()
{
  server.send(200, "text/plain",
      "EchoGrid Node\n/api/data 获取噪声JSON\n"
      "Leq: by accumulated sample time window\n"
      "Cache: ring buffer; Upload: separate RTOS task");
}

// ---------- setup / loop ----------
void setup()
{
  Serial.begin(115200);
  delay(300);

  WiFi.setSleep(false);
  configTime(8 * 3600, 0, "ntp.aliyun.com", "ntp1.aliyun.com");

  uploadQueue = xQueueCreate(QUEUE_DEPTH, sizeof(ReadingItem));
  cacheMutex  = xSemaphoreCreateMutex();
  if (!uploadQueue || !cacheMutex) Serial.println("[Init] queue/mutex failed!");

  xTaskCreate(uploadTask, "uploadTask", 8192, NULL, 1, NULL);

  esp_err_t i2sRet = i2s_driver_install(I2S_PORT, &i2s_config, 0, NULL);
  if (i2sRet != ESP_OK) Serial.printf("[I2S] install fail: %s\n", esp_err_to_name(i2sRet));

  i2sRet = i2s_set_pin(I2S_PORT, &pin_config);
  if (i2sRet != ESP_OK) Serial.printf("[I2S] pin fail: %s\n", esp_err_to_name(i2sRet));

  i2s_zero_dma_buffer(I2S_PORT);

  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.print("[WiFi] connecting...");
  uint32_t wifiStart = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - wifiStart < WIFI_CONN_TIMEOUT)
  { delay(200); Serial.print("."); }

  if (WiFi.status() == WL_CONNECTED)
    Serial.println("\n[WiFi] IP: " + WiFi.localIP().toString());
  else
    Serial.println("\n[WiFi] timeout, continue offline");

  server.on("/api/data", handleApiData);
  server.on("/", handleRoot);
  server.begin();
  Serial.println("[Web] started");
}

void loop()
{
  server.handleClient();
  updateLeq();
  wifiServiceTick();

  // C3: 每个窗口最多上报一次
  if (leqDirty && millis() - lastUploadTick > UPLOAD_INTERVAL)
  {
    lastUploadTick = millis();

    String t = getISO8601Time();
    if (t.length() > 0)
    {
      ReadingItem item;
      copyToBuf(item.point_id, sizeof(item.point_id), String(POINT_ID));
      copyToBuf(item.location, sizeof(item.location), String(LOCATION));
      copyToBuf(item.time_iso, sizeof(item.time_iso), t);
      copyToBuf(item.period,   sizeof(item.period),   String(""));
      item.db = lastLeq;

      if (xQueueSend(uploadQueue, &item, 0) != pdTRUE)
      {
        Serial.println("[Queue] full, push cache");
        pushCache(item);
      }
      else leqDirty = false;
    }
    else Serial.println("[Upload] skip, NTP not ready");
  }
}
