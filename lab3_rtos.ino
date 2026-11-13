#include <WiFi.h>
#include <PubSubClient.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <ArduinoJson.h>

const char* ssid        = "Wokwi-GUEST";
const char* password    = "";
const char* mqtt_server = "broker.emqx.io";
const int   mqtt_port   = 1883;
const char* TOPIC_DATA    = "aquaculture/ks2026/data";
const char* TOPIC_CONTROL = "aquaculture/ks2026/control";

#define TRIG_PIN  5
#define ECHO_PIN  18
#define ONE_WIRE  4
#define PUMP_PIN  26   // PWM điều khiển công suất bơm
#define VALVE_PIN 27   // Van thoát (ON/OFF)

const float TANK_HEIGHT = 200.0;  // cm

struct SensorData { float distance; float level; float temp; };

QueueHandle_t dataQueue;    // đầy đủ dữ liệu -> Task truyền data
QueueHandle_t alertQueue;   // chỉ mực nước    -> Task cảnh báo

volatile bool manualMode  = false;
volatile int  pumpPercent = 0;
volatile bool valveOpen   = false;

WiFiClient espClient;
PubSubClient mqtt(espClient);
OneWire oneWire(ONE_WIRE);
DallasTemperature ds18b20(&oneWire);

void setPump(int percent) {
  pumpPercent = percent;
  uint32_t duty = map(percent, 0, 100, 0, 255);
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWrite(PUMP_PIN, duty);
#else
  ledcWrite(0, duty);
#endif
}

void setValve(bool open) {
  valveOpen = open;
  digitalWrite(VALVE_PIN, open ? HIGH : LOW);
}

float readDistanceCm() {
  digitalWrite(TRIG_PIN, LOW);  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH); delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  long us = pulseIn(ECHO_PIN, HIGH, 30000);
  if (us == 0) return -1;            // không có tín hiệu phản hồi
  return us * 0.0343 / 2.0;
}

void taskReadData(void* pv) {
  SensorData d;
  for (;;) {
    d.distance = readDistanceCm();
    ds18b20.requestTemperatures();
    d.temp = ds18b20.getTempCByIndex(0);
    if (d.distance > 0) {
      d.level = TANK_HEIGHT - d.distance;
      Serial.printf("Muc nuoc: %.1f cm (khoang cach: %.1f cm) | Nhiet do: %.1f C\n",
                    d.level, d.distance, d.temp);
      xQueueSend(dataQueue, &d, 0);
      xQueueOverwrite(alertQueue, &d.level);
    }
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

bool runFor(uint32_t ms) {          // false nếu bị ngắt bởi chế độ MANUAL
  uint32_t t0 = millis();
  while (millis() - t0 < ms) {
    if (manualMode) return false;
    vTaskDelay(pdMS_TO_TICKS(100));
  }
  return true;
}

void taskAlert(void* pv) {
  float level;
  for (;;) {
    if (xQueueReceive(alertQueue, &level, pdMS_TO_TICKS(1000)) != pdTRUE) continue;
    if (manualMode) continue;

    if (level >= 150) {                // 1.5 m - 2 m
      setPump(100);
      if (runFor(20000)) setPump(0);
    } else if (level >= 100) {         // 1 m - 1.5 m
      setPump(50);
      if (runFor(10000)) setPump(0);
    } else if (level < 50) {           // < 0.5 m
      setValve(true);
      if (runFor(10000)) setValve(false);
    }                                  // 0.5 m - 1 m: không tác động
    xQueueReset(alertQueue);           // bỏ dữ liệu cũ trong lúc thực thi
  }
}

void onMessage(char* topic, byte* payload, unsigned int len) {
  Serial.printf("Nhan lenh: %.*s\n", (int)len, (const char*)payload);
  StaticJsonDocument<200> doc;
  if (deserializeJson(doc, payload, len)) { Serial.println("Loi JSON"); return; }
  const char* mode = doc["mode"] | "";
  if (strcmp(mode, "AUTO") == 0) {
    manualMode = false;
    setPump(0); setValve(false);
    return;
  }
  manualMode = true;
  if (doc.containsKey("pump"))  setPump(doc["pump"]);
  if (doc.containsKey("valve")) setValve(doc["valve"]);
}

void taskSendData(void* pv) {
  SensorData d;
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    Serial.println("Dang ket noi WiFi...");
    vTaskDelay(pdMS_TO_TICKS(500));
  }
  Serial.println("WiFi OK");
  mqtt.setServer(mqtt_server, mqtt_port);
  mqtt.setCallback(onMessage);

  for (;;) {
    if (!mqtt.connected()) {
      String id = "esp32-ks2026-" + String(random(0xffff), HEX);
      if (mqtt.connect(id.c_str())) {
        Serial.println("MQTT OK");
        mqtt.subscribe(TOPIC_CONTROL);
      } else {
        Serial.printf("MQTT loi, rc=%d\n", mqtt.state());
        vTaskDelay(pdMS_TO_TICKS(2000));
        continue;
      }
    }
    if (xQueueReceive(dataQueue, &d, pdMS_TO_TICKS(100)) == pdTRUE) {
      StaticJsonDocument<200> doc;
      doc["level"] = d.level;
      doc["temp"]  = d.temp;
      doc["mode"]  = manualMode ? "MANUAL" : "AUTO";
      doc["pump"]  = pumpPercent;
      doc["valve"] = valveOpen;
      char buf[200];
      serializeJson(doc, buf);
      mqtt.publish(TOPIC_DATA, buf);
    }
    mqtt.loop();
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(VALVE_PIN, OUTPUT);
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcAttach(PUMP_PIN, 1000, 8);
#else
  ledcSetup(0, 1000, 8);
  ledcAttachPin(PUMP_PIN, 0);
#endif
  setPump(0); setValve(false);
  ds18b20.begin();

  dataQueue  = xQueueCreate(10, sizeof(SensorData));
  alertQueue = xQueueCreate(1,  sizeof(float));

  xTaskCreatePinnedToCore(taskSendData, "SendData", 8192, NULL, 1, NULL, 0); // Core 0
  xTaskCreatePinnedToCore(taskReadData, "ReadData", 4096, NULL, 2, NULL, 1); // Core 1
  xTaskCreatePinnedToCore(taskAlert,    "Alert",    4096, NULL, 3, NULL, 1); // Core 1
}

void loop() { vTaskDelay(portMAX_DELAY); }
