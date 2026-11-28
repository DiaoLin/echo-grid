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
const float CALIB_OFFSET = 120.0f;

const int SCK_PIN = 26;
const int WS_PIN  = 25;
const int SD_PIN  = 22;

const i2s_port_t I2S_PORT = I2S_NUM_0;

const uint32_t WIFI_CONN_TIMEOUT        = 10000;
const uint32_t WIFI_RECONNECT_INTERVAL  = 5000;
const uint32_t UPLOAD_INTERVAL          = 5000;
const int MAX_CACHE_NUM = 50;
const int QUEUE_DEPTH   = 8;
const int UPLOAD_TASK_STACK = 12288;
// =============================================================

// ===================== 证书 =====================
static const char ROOT_CA_PEM[] = R"(
-----BEGIN CERTIFICATE-----
MIIE2zCCAsOgAwIBAgIRAKICU/FfJpHAXcHOE7m8yk4wDQYJKoZIhvcNAQELBQAw
LjELMAkGA1UEBhMCVVMxDTALBgNVBAoTBElTUkcxEDAOBgNVBAMTB1Jvb3QgWVIw
HhcNMjUwOTAzMDAwMDAwWhcNMjgwOTAyMjM1OTU5WjAzMQswCQYDVQQGEwJVUzEW
MBQGA1UEChMNTGV0J3MgRW5jcnlwdDEMMAoGA1UEAxMDWVIxMIIBIjANBgkqhkiG
9w0BAQEFAAOCAQ8AMIIBCgKCAQEAoVi8X2xCYgMXvJxNPKp/oF13UMgmPABB07VC
LNDtoXmt9luEZNJSBV10VyT1Pz6LD8Zq1d2gc43WNl1AdRrj4sEnazbOiz0nPpmG
Bp2hui49oZtDIY6wdKeZAi5BbNU20CH6RSBBMLSQ9cXrH8dxdv4PAJ45ssGML68U
SE3BsjC2a6cAN9L5CgXVIQi5tfNiTPoFZZ3S0OlXqLmmtdV95udWAb5b6e/F49Di
CsH0Y00Ag72BVIb1hzynmKe+X0mERBTtsb3BwmpV9ipeBjMLoR/D9cHxHQCWoi5l
TmXwY015J5rGelz1nZjJuxc2kioaX29XJBnhMkP531rSdG5uMwIDAQABo4HuMIHr
MA4GA1UdDwEB/wQEAwIBhjATBgNVHSUEDDAKBggrBgEFBQcDATASBgNVHRMBAf8E
CDAGAQH/AgEAMB0GA1UdDgQWBBQfLzW+RhSCzUCxrnksVXj699Ro+zAfBgNVHSME
GDAWgBTe51tg0CJtQCh9Pw0B/qS1UrRRlDAyBggrBgEFBQcBAQQmMCQwIgYIKwYB
BQUHMAKGFmh0dHA6Ly95ci5pLmxlbmNyLm9yZy8wEwYDVR0gBAwwCjAIBgZngQwB
AgEwJwYDVR0fBCAwHjAcoBqgGIYWaHR0cDovL3lyLmMubGVuY3Iub3JnLzANBgkq
hkiG9w0BAQsFAAOCAgEA0+zvMq3kHig1ddTmmm+RibTr9/RpX7k4buanMMRqbV/y
IvP82zAHN3mvaw+cASuVsdpd0ikjhr4hnhJQLQOzOp2ccKrsdGOAgo0vddeISFAq
EWEV4lmUM3vFF796up+bSgmJ1u6RupDCMxDgF8M3eLvGuj6L0lu3zkQ0KuQLnKxL
tB0oQqn1Idg5CuuGpMvQzk29Pa3D/qHurc0EIM9SxukQuJqq63lxsYyRQFU8yMBO
hq1w5LbfaWNRrz1uklOfI/pYkAb2E2MTZrAMQkBIE2S8Jt1F8gRc96o/xOsrgvSk
a84AisX6xq1lz1Z7jGvrnXc4TMcjxZTjiTaihcYI1JIXZiLtEMSCa5l3cu8YWd6z
dLRQlqRdclVjuQfNHawRJ6GWlkK0QJosivTKwdBw3KxEtzGo8yMHERbsy57gP1UX
HOMcmZYQC0gtyR3SxfenIM/MxC3Ia2Ypab/kQ/CTnlIn2KQ5JUC6NYrGCbhFN9bp
5lKJStEwCUnLpntcrXk5XVDCNv/5RyWpRThkGOV7GetKkQ0qAY8hCzWK6oqnAhDZ
cjlYVdWfqOw3DIOX6EDNBgAqHarRVxyF9QZdOaXSyPJ0ueD2BYJEBgaCGQ8rAaU/
Qc123V5LTXDZW4CcsPBDyhy4v+c8hClAyw/IkJlfBqxB9D+/wvIMHgECZ4ptP6o=
-----END CERTIFICATE-----
)";
// =============================================================

WebServer server(80);

float energySum = 0.0f;
float sumT      = 0.0f;

float lastLeq = 0.0f;
float lastLp  = 0.0f;
bool  leqValid = false;
bool  leqDirty = false;      

struct ReadingItem {
  char  point_id[16];
  char  location[64];
  char  time_iso[40];
  float db;
  char  period[16];
  uint32_t seq;
};

QueueHandle_t     uploadQueue  = NULL;
SemaphoreHandle_t cacheMutex   = NULL;

ReadingItem cacheBuf[MAX_CACHE_NUM];
int cacheHead = 0;
int cacheCnt  = 0;

ReadingItem uploadBatchBuf[MAX_CACHE_NUM];
uint32_t readingSeq = 1;
uint32_t g_maxUploadSeq = 0; // 记录已成功发送的最大序列号

uint32_t lastUploadTick = 0;
static bool ntpLogPrinted = false;
static uint32_t lastNtpWarnTick = 0;

static WiFiClientSecure g_tlsClient;
static bool g_tlsReady = false;

static void tlsEnsureReady()
{
  if (g_tlsReady) return;
  g_tlsClient.setCACert(ROOT_CA_PEM);   
  g_tlsReady = true;
}

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
  time_t now = time(nullptr);
  struct tm timeinfo;
  localtime_r(&now, &timeinfo); 

  if (timeinfo.tm_year < (2020 - 1900))
  {
    return "";
  }
  char buf[32];
  strftime(buf, sizeof(buf), "%Y-%m-%dT%H:%M:%S+08:00", &timeinfo);
  return String(buf);
}

void copyToBuf(char* dst, size_t dstSize, const String& s)
{
  if (dstSize == 0) return;
  strncpy(dst, s.c_str(), dstSize - 1);
  dst[dstSize - 1] = '\0';
}

void wifiServiceTick()
{
  static uint32_t reconnectTimer = 0;
  wl_status_t st = WiFi.status();
  if (st == WL_CONNECTED) return;
  if (st != WL_DISCONNECTED && st != WL_CONNECT_FAILED && st != WL_NO_SSID_AVAIL) return;

  if (millis() - reconnectTimer > WIFI_RECONNECT_INTERVAL)
  {
    reconnectTimer = millis();
    Serial.println("[WiFi] reconnecting...");
    WiFi.begin(WIFI_SSID, WIFI_PASS);
  }
}

// ---------- HTTP ----------
enum UploadStatus {
  UPLOAD_OK,      
  UPLOAD_RETRY,   
  UPLOAD_DROP     
};

UploadStatus checkHttpStatus(int code, const char* tag)
{
  if (code >= 200 && code < 300) return UPLOAD_OK;
  if (code >= 400 && code < 500 && code != 408 && code != 429) {
    Serial.printf("[HTTP] %s drop (fatal client error), code=%d\n", tag, code);
    return UPLOAD_DROP;
  }
  Serial.printf("[HTTP] %s fail, retry later, code=%d\n", tag, code);
  return UPLOAD_RETRY;
}

UploadStatus uploadSingleReading(const ReadingItem& item)
{
  if (WiFi.status() != WL_CONNECTED) return UPLOAD_RETRY;
  if (item.time_iso[0] == '\0')     return UPLOAD_DROP;

  tlsEnsureReady();
  HTTPClient http;
  http.setTimeout(6000);
  http.setReuse(true);
  String url = API_BASE + "/api/readings";
  
  if (!http.begin(g_tlsClient, url))
  {
    Serial.println("[HTTP] single begin fail, reset tls client");
    g_tlsClient.stop(); // 只在 begin 失败时重置
    return UPLOAD_RETRY;
  }
  http.addHeader("Content-Type", "application/json");

  StaticJsonDocument<512> doc;
  doc["point_id"] = item.point_id;
  doc["location"] = item.location;
  doc["time"]     = item.time_iso;
  doc["db"]       = item.db;
  doc["seq"]      = item.seq;
  if (item.period[0] != '\0') doc["period"] = item.period;

  String payload;
  serializeJson(doc, payload);
  int code = http.POST(payload);
  http.end();

  return checkHttpStatus(code, "single");
}

UploadStatus batchUploadReadings(ReadingItem arr[], int num)
{
  if (WiFi.status() != WL_CONNECTED || num <= 0) return UPLOAD_RETRY;

  tlsEnsureReady();
  HTTPClient http;
  http.setTimeout(6000);
  http.setReuse(true);
  String url = API_BASE + "/api/readings/batch";
  
  if (!http.begin(g_tlsClient, url))
  {
    Serial.println("[HTTP] batch begin fail, reset tls client");
    g_tlsClient.stop(); // 只在 begin 失败时重置
    return UPLOAD_RETRY;
  }
  http.addHeader("Content-Type", "application/json");

  static DynamicJsonDocument doc(16384);
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
    obj["seq"]      = arr[i].seq;
    if (arr[i].period[0] != '\0') obj["period"] = arr[i].period;
  }

  if (arrJson.size() == 0)
  {
    http.end();
    return UPLOAD_DROP; 
  }

  String payload;
  serializeJson(doc, payload);
  int code = http.POST(payload);
  http.end();

  return checkHttpStatus(code, "batch");
}

// ---------- 环形缓存 ----------
void pushCache(const ReadingItem& item)
{
  if (item.time_iso[0] == '\0') { Serial.println("[Cache] skip empty time"); return; }
  if (xSemaphoreTake(cacheMutex, pdMS_TO_TICKS(100)) != pdTRUE) return;

  int idx = (cacheHead + cacheCnt) % MAX_CACHE_NUM;
  if (cacheCnt >= MAX_CACHE_NUM)
  {
    // 【修复 2】检查是否覆盖了还没上传成功的旧数据
    if (cacheBuf[cacheHead].seq > g_maxUploadSeq) {
      Serial.printf("[Cache warn] overwrite unsent seq %u (> maxUploadSeq %u)\n", cacheBuf[cacheHead].seq, g_maxUploadSeq);
    }
    
    cacheHead = (cacheHead + 1) % MAX_CACHE_NUM;
    Serial.println("[Cache] full, overwrite oldest");
  }
  else cacheCnt++;

  cacheBuf[idx] = item;
  xSemaphoreGive(cacheMutex);
}

bool flushCache()
{
  int n = 0;
  if (xSemaphoreTake(cacheMutex, pdMS_TO_TICKS(100)) != pdTRUE) return false;
  
  n = cacheCnt;
  if (n > 0)
  {
    for (int i = 0; i < n; i++) uploadBatchBuf[i] = cacheBuf[(cacheHead + i) % MAX_CACHE_NUM];
  }
  xSemaphoreGive(cacheMutex);

  if (n <= 0) return true;

  UploadStatus status = batchUploadReadings(uploadBatchBuf, n);

  if (status == UPLOAD_OK || status == UPLOAD_DROP)
  {
    uint32_t newMaxSeq = 0;
    for(int i=0;i<n;i++){
      if(uploadBatchBuf[i].seq > newMaxSeq) newMaxSeq = uploadBatchBuf[i].seq;
    }
    if(newMaxSeq > g_maxUploadSeq) g_maxUploadSeq = newMaxSeq;

    if (xSemaphoreTake(cacheMutex, pdMS_TO_TICKS(100)) == pdTRUE)
    {
      int popped = 0;
      while (cacheCnt > 0) {
        if (cacheBuf[cacheHead].seq <= g_maxUploadSeq) {
          cacheHead = (cacheHead + 1) % MAX_CACHE_NUM;
          cacheCnt--;
          popped++;
        } else {
          break;
        }
      }
      
      if (status == UPLOAD_DROP) {
        Serial.printf("[Cache] dropped %d toxic entries\n", popped);
      } else {
        Serial.printf("[Cache] flush success, remain %d\n", cacheCnt);
      }
      xSemaphoreGive(cacheMutex);
    }
    return true; 
  }
  
  Serial.println("[Cache] flush failed, keep cache for retry");
  return false;
}

// ---------- 上报任务 ----------
void uploadTask(void *pvParam)
{
  ReadingItem item;
  uint32_t failStreak  = 0;
  uint32_t backoffMs   = 0;
  uint32_t lastFlushMs = 0;

  for (;;)
  {
    BaseType_t got = xQueueReceive(uploadQueue, &item, pdMS_TO_TICKS(1000));
    
    bool anySuccess = false;
    
    if (got == pdTRUE)
    {
      UploadStatus singleStatus = uploadSingleReading(item);
      
      if (singleStatus == UPLOAD_RETRY) {
        pushCache(item);
      } else if (singleStatus == UPLOAD_DROP) {
        Serial.println("[Queue] dropped toxic single entry");
      } else if (singleStatus == UPLOAD_OK) {
        if(item.seq > g_maxUploadSeq) g_maxUploadSeq = item.seq;
        anySuccess = true;
      }
    }

    uint32_t now = millis();
    if (now - lastFlushMs >= backoffMs)
    {
      lastFlushMs = now;
      if (flushCache()) anySuccess = true;
    }

    if (anySuccess)
    {
      failStreak = 0;
      backoffMs  = 0;
    }
    else if (got == pdTRUE) // 确实处理了数据且全部失败，退避加倍
    {
      failStreak++;
      uint32_t shift = (failStreak > 4) ? 4 : failStreak;
      backoffMs = 5000u * (1u << (shift - 1));
      if (backoffMs > 60000u) backoffMs = 60000u;
      Serial.printf("[Backoff] failStreak=%lu, next flush in %lu ms\n",
                    (unsigned long)failStreak, (unsigned long)backoffMs);
    }
    
    taskYIELD();
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
    leqDirty = true;
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
  
  // 【修复 3】添加 CORS 支持，允许前端看板跨域获取数据
  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.send(200, "application/json", payload);
}

void handleRoot()
{
  server.send(200, "text/plain",
      "EchoGrid Node\n/api/data 获取噪声JSON\n"
      "Leq: by accumulated sample time window\n"
      "Cache: ring buffer + seq幂等; Upload: Core0 RTOS task, periodic flush");
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
  
  if (!uploadQueue || !cacheMutex) {
    Serial.println("[Init] queue/mutex failed! HALT");
    while (1) {
      delay(500);
      Serial.print(".");
    }
  }

  xTaskCreatePinnedToCore(uploadTask, "uploadTask", UPLOAD_TASK_STACK, NULL, 2, NULL, 0);

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
      item.seq = readingSeq++;

      if (xQueueSend(uploadQueue, &item, 0) != pdTRUE)
      {
        Serial.println("[Queue] full, push cache");
        pushCache(item);
      }
      leqDirty = false;
    }
    else
    {
      if(millis() - lastNtpWarnTick > 5000){
        Serial.println("[Upload] skip, NTP not ready");
        lastNtpWarnTick = millis();
      }
    }
  }
}
