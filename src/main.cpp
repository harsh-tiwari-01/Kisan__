

////////////////////////  SMART  FARM  (KISAN__) ////////////////////////
#include "secrets.h"

#if USE_HTTPS
#define TS_ENABLE_SSL
#endif

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <DHT.h>
#include <ThingSpeak.h>
#include <Preferences.h>
#include <esp_task_wdt.h>
#include <esp_system.h>
#include <math.h>
#include <string.h>
#include <time.h>

#if USE_HTTPS
#include "root_ca.h"   
#endif

namespace {
constexpr char FW_VERSION[] = "1.1.1";
constexpr uint32_t THINGSPEAK_CHANNEL_ID = 3548473UL;

constexpr uint8_t SOIL_PIN = 34;
constexpr uint8_t DHT_PIN = 15;
constexpr uint8_t PIR_PIN = 27;
constexpr uint8_t PUMP_PIN = 26;
constexpr uint8_t GREEN_LED_PIN = 25;
constexpr uint8_t RED_LED_PIN = 33;
constexpr uint8_t BUZZER_PIN = 32;
constexpr uint8_t LASER_PIN = 4;

constexpr int DRY_VALUE = 4095;
constexpr int WET_VALUE = 0;
constexpr int DRY_LIMIT_PERCENT = 30;
constexpr int WET_LIMIT_PERCENT = 60;

constexpr uint32_t PUMP_MAX_MS = 600000UL;
constexpr uint32_t PIR_WARMUP_MS = 5000UL;
constexpr uint32_t ALERT_MS = 10000UL;
constexpr uint32_t ALERT_GAP_MS = 30000UL;
constexpr uint32_t UPLOAD_INTERVAL_MS = 20000UL;
constexpr uint32_t WIFI_RETRY_MS = 5000UL;
constexpr uint32_t SOIL_SAMPLE_INTERVAL_MS = 100UL;
constexpr uint8_t SOIL_SAMPLE_COUNT = 8;

constexpr uint32_t WDT_TIMEOUT_S = 30;                 
constexpr uint8_t UPLOAD_FAILS_RECONNECT = 3;          
constexpr uint8_t UPLOAD_FAILS_RESTART = 15;          
constexpr uint32_t WIFI_OFFLINE_RESTART_MS = 600000UL; 

#if USE_DHT11
constexpr uint8_t DHT_TYPE = DHT11;
#else
constexpr uint8_t DHT_TYPE = DHT22;
#endif

DHT dht(DHT_PIN, DHT_TYPE);

#if USE_HTTPS
WiFiClientSecure client;
#else
WiFiClient client;
#endif

Preferences prefs;

int pumpState = 0;
int pumpFault = 0;
int laserState = 0;
int buzzerState = 0;
int alertActive = 0;

bool hasAlerted = false;

uint32_t bootMs = 0;
uint32_t pumpStartMs = 0;
uint32_t alertStartMs = 0;
uint32_t lastUploadMs = 0;
uint32_t lastWifiAttemptMs = 0;
uint32_t lastSoilSampleMs = 0;

uint32_t soilSamples[SOIL_SAMPLE_COUNT] = {};
uint8_t soilSampleIndex = 0;
uint8_t soilSamplesCollected = 0;
int filteredSoilPercent = 0;

uint32_t pumpCycles = 0;
uint32_t totalPumpSec = 0;
uint8_t uploadFailCount = 0;
uint32_t wifiOfflineSinceMs = 0;
bool wifiWasOnline = false;


const char *resetReasonText() {
  switch (esp_reset_reason()) {
    case ESP_RST_POWERON:  return "power";
    case ESP_RST_SW:       return "software";
    case ESP_RST_PANIC:    return "panic";
    case ESP_RST_INT_WDT:  return "int_wdt";
    case ESP_RST_TASK_WDT: return "task_wdt";
    case ESP_RST_WDT:      return "wdt";
    case ESP_RST_BROWNOUT: return "brownout";
    default:               return "other";
  }
}

void logTime() {
  struct tm t;
  if (getLocalTime(&t, 0)) {
    Serial.printf("[%02d:%02d:%02d] ", t.tm_hour, t.tm_min, t.tm_sec);
  } else {
    Serial.printf("[up %lus] ", (unsigned long)(millis() / 1000UL));
  }
}

void watchdogBegin() {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  esp_task_wdt_config_t cfg = {};
  cfg.timeout_ms = WDT_TIMEOUT_S * 1000UL;
  cfg.idle_core_mask = 0;
  cfg.trigger_panic = true;
  esp_task_wdt_reconfigure(&cfg);
#else
  esp_task_wdt_init(WDT_TIMEOUT_S, true);
#endif
  esp_task_wdt_add(NULL);
}

void saveFault() {
  prefs.putUChar("fault", static_cast<uint8_t>(pumpFault));
}

void onWifiEvent(WiFiEvent_t event) {
  if (event == ARDUINO_EVENT_WIFI_STA_GOT_IP) {
    Serial.print("Wi-Fi connected, IP: ");
    Serial.println(WiFi.localIP());
  } else if (event == ARDUINO_EVENT_WIFI_STA_DISCONNECTED) {
    Serial.println("Wi-Fi disconnected");
  }
}

void pumpOff() {
  if (pumpState) {
    totalPumpSec += (millis() - pumpStartMs) / 1000UL;
    prefs.putUInt("runsec", totalPumpSec);
  }
  digitalWrite(PUMP_PIN, LOW);
  pumpState = 0;
}

void pumpOn() {
  if (pumpState || pumpFault) return;

  digitalWrite(PUMP_PIN, HIGH);
  pumpState = 1;
  pumpStartMs = millis();

  ++pumpCycles;
  prefs.putUInt("cycles", pumpCycles);

  Serial.println("PUMP ON");
}

void setLaser(bool on) {
  digitalWrite(LASER_PIN, on ? HIGH : LOW);
  laserState = on ? 1 : 0;
}

void setBuzzer(bool on) {
  if (on) {
    tone(BUZZER_PIN, 4000);
  } else if (buzzerState) {   
    noTone(BUZZER_PIN);
  }
  buzzerState = on ? 1 : 0;
}

void updateStatusLeds() {
  const bool problem = alertActive || pumpFault;
  digitalWrite(RED_LED_PIN, problem ? HIGH : LOW);
  digitalWrite(GREEN_LED_PIN, problem ? LOW : HIGH);
}

int readSoilPercent() {
  const int raw = analogRead(SOIL_PIN);

  soilSamples[soilSampleIndex] = static_cast<uint32_t>(raw);
  soilSampleIndex = (soilSampleIndex + 1) % SOIL_SAMPLE_COUNT;

  if (soilSamplesCollected < SOIL_SAMPLE_COUNT) {
    ++soilSamplesCollected;
  }

  uint32_t sum = 0;
  for (uint8_t i = 0; i < soilSamplesCollected; ++i) {
    sum += soilSamples[i];
  }

  const int averagedRaw = static_cast<int>(sum / soilSamplesCollected);
  const int denominator = DRY_VALUE - WET_VALUE;

  if (denominator <= 0) return 0;

  const int percent =
      (DRY_VALUE - averagedRaw) * 100 / denominator;

  filteredSoilPercent = constrain(percent, 0, 100);
  return filteredSoilPercent;
}

void connectWifi() {
  if (WiFi.status() == WL_CONNECTED) return;

  const uint32_t now = millis();
  if (now - lastWifiAttemptMs < WIFI_RETRY_MS) return;

  lastWifiAttemptMs = now;

  Serial.printf("Connecting to Wi-Fi: %s\n", SECRET_SSID);
  WiFi.mode(WIFI_STA);
  WiFi.begin(SECRET_SSID, SECRET_PASS);
}

void beginAnimalAlert() {
  alertActive = 1;
  hasAlerted = true;
  alertStartMs = millis();

  setBuzzer(true);
  setLaser(true);

  Serial.println("ANIMAL DETECTED");
}

void endAnimalAlert() {
  alertActive = 0;

  setBuzzer(false);
  setLaser(false);

  Serial.println("Animal alert cleared");
}

void uploadReading(
    int soil,
    float tempC,
    float humidity,
    int pir,
    bool temperatureValid,
    bool humidityValid) {
  if (strlen(SECRET_WRITE_API_KEY) != 16) {
    Serial.printf(
        "ThingSpeak upload skipped: Write API Key must be 16 characters, yours has %d\n",
        (int)strlen(SECRET_WRITE_API_KEY));
    return;
  }

  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("ThingSpeak upload skipped: Wi-Fi is offline");
    return;
  }

  
  ThingSpeak.setField(1, soil);
  ThingSpeak.setField(
      2, temperatureValid ? String(tempC, 1) : String());
  ThingSpeak.setField(
      3, humidityValid ? String(humidity, 1) : String());
  ThingSpeak.setField(4, pumpState);
  ThingSpeak.setField(5, 0);  
  ThingSpeak.setField(6, alertActive);
  ThingSpeak.setField(7, pumpFault);
  ThingSpeak.setField(8, pir);

  ThingSpeak.setStatus(
      String("fw=") + FW_VERSION +
      ",rst=" + resetReasonText() +
      ",rssi=" + WiFi.RSSI() +
      ",up=" + (millis() / 1000UL) + "s" +
      ",cycles=" + pumpCycles +
      ",runsec=" + totalPumpSec +
      ",laser=" + laserState +
      ",buzzer=" + buzzerState);

  const int result =
      ThingSpeak.writeFields(THINGSPEAK_CHANNEL_ID, SECRET_WRITE_API_KEY);

  if (result == 200) {
    Serial.println("ThingSpeak upload OK");
    uploadFailCount = 0;
  } else {
    Serial.printf("ThingSpeak upload failed (code %d)\n", result);
    if (uploadFailCount < 255) ++uploadFailCount;
  }
}


void checkConnectionHealth() {
  const uint32_t now = millis();
  const bool online = (WiFi.status() == WL_CONNECTED);

  if (online) {
    wifiWasOnline = true;
    wifiOfflineSinceMs = 0;
  } else {
    if (wifiOfflineSinceMs == 0) wifiOfflineSinceMs = now;
    if (now - wifiOfflineSinceMs >= WIFI_OFFLINE_RESTART_MS) {
      Serial.println("Wi-Fi offline too long: restarting");
      delay(100);
      ESP.restart();
    }
  }

  if (uploadFailCount >= UPLOAD_FAILS_RESTART) {
    Serial.println("Too many upload failures: restarting");
    delay(100);
    ESP.restart();
  }

  if (uploadFailCount >= UPLOAD_FAILS_RECONNECT && online) {
    Serial.println("Repeated upload failures: reconnecting Wi-Fi");
    WiFi.disconnect();
    lastWifiAttemptMs = 0;
    uploadFailCount = UPLOAD_FAILS_RECONNECT - 1;  
  }
}


void printStatus() {
  logTime();
  Serial.printf(
      "STATUS fw=%s soil=%d%% pump=%d fault=%d alert=%d cycles=%lu runsec=%lu "
      "rssi=%d fails=%d heap=%lu\n",
      FW_VERSION, filteredSoilPercent, pumpState, pumpFault, alertActive,
      (unsigned long)pumpCycles, (unsigned long)totalPumpSec,
      WiFi.RSSI(), uploadFailCount, (unsigned long)ESP.getFreeHeap());
}

void handleSerialCommands() {
  while (Serial.available() > 0) {
    const char c = static_cast<char>(Serial.read());

    if (c == 's') {
      printStatus();
    } else if (c == 'r') {
      pumpFault = 0;
      saveFault();
      Serial.println("Pump fault cleared");
    } else if (c == 'c') {
      pumpCycles = 0;
      totalPumpSec = 0;
      prefs.putUInt("cycles", 0);
      prefs.putUInt("runsec", 0);
      Serial.println("Pump statistics cleared");
    }
  }
}
}  
void setup() {
  Serial.begin(115200);
  bootMs = millis();

  pinMode(SOIL_PIN, INPUT);
  pinMode(PIR_PIN, INPUT);

  analogReadResolution(12);
  analogSetPinAttenuation(SOIL_PIN, ADC_11db);

  pinMode(PUMP_PIN, OUTPUT);
  pinMode(GREEN_LED_PIN, OUTPUT);
  pinMode(RED_LED_PIN, OUTPUT);
  pinMode(LASER_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);

  prefs.begin("smartfarm", false);
  pumpFault = prefs.getUChar("fault", 0);
  pumpCycles = prefs.getUInt("cycles", 0);
  totalPumpSec = prefs.getUInt("runsec", 0);

  pumpOff();
  setLaser(false);
  setBuzzer(false);

  dht.begin();

#if USE_HTTPS
  client.setCACert(THINGSPEAK_ROOT_CA);
#endif
  ThingSpeak.begin(client);

  WiFi.onEvent(onWifiEvent);
  WiFi.mode(WIFI_STA);
  connectWifi();

  configTime(19800, 0, "pool.ntp.org", "time.google.com");

  updateStatusLeds();
  watchdogBegin();

  Serial.println("SmartFarm ESP32 starting");
  Serial.printf("Firmware %s, reset reason: %s\n", FW_VERSION, resetReasonText());
  Serial.printf("ThingSpeak channel: %lu\n", (unsigned long)THINGSPEAK_CHANNEL_ID);
  Serial.println("Cloud upload interval: 20 seconds");
  Serial.println("Local pump and alert logic stays active if cloud is offline");
  Serial.println("Serial commands: s=status, r=clear pump fault, c=clear pump stats");
  if (pumpFault) {
    Serial.println("WARNING: pump fault restored from flash. Send 'r' to clear.");
  }
}

void loop() {
  esp_task_wdt_reset();  
  const uint32_t now = millis();

  if (now - lastSoilSampleMs >= SOIL_SAMPLE_INTERVAL_MS ||
      lastSoilSampleMs == 0) {
    lastSoilSampleMs = now;
    readSoilPercent();
  }

  const int soil = filteredSoilPercent;
  const int pir = digitalRead(PIR_PIN);

  if (!pumpFault) {
    if (!pumpState && soil < DRY_LIMIT_PERCENT) {
      pumpOn();
    } else if (pumpState && soil >= WET_LIMIT_PERCENT) {
      pumpOff();
    }
  }

  if (pumpState && now - pumpStartMs >= PUMP_MAX_MS) {
    pumpOff();
    pumpFault = 1;
    saveFault();  
    Serial.println("PUMP FAULT: maximum runtime exceeded");
  }

  if (now - bootMs >= PIR_WARMUP_MS) {
    if (pir == HIGH && !alertActive &&
        (!hasAlerted || now - alertStartMs >= ALERT_GAP_MS)) {
      beginAnimalAlert();
    }

    if (alertActive && now - alertStartMs >= ALERT_MS) {
      endAnimalAlert();
    }
  }

  updateStatusLeds();
  connectWifi();
  handleSerialCommands();
  checkConnectionHealth();

  if (now - lastUploadMs >= UPLOAD_INTERVAL_MS) {
    lastUploadMs = now;

    const float tempC = dht.readTemperature();
    const float humidity = dht.readHumidity();

    const bool temperatureValid =
        !isnan(tempC) && tempC >= -40.0f && tempC <= 80.0f;
    const bool humidityValid =
        !isnan(humidity) && humidity >= 0.0f && humidity <= 100.0f;

    if (!temperatureValid) {
      Serial.println(
          "DHT temperature invalid; publishing other available fields");
    }

    if (!humidityValid) {
      Serial.println(
          "DHT humidity invalid; publishing other available fields");
    }

    logTime();
    Serial.printf(
        "soil=%d%% temp=%.1fC humidity=%.1f%% pir=%d pump=%d alert=%d fault=%d\n",
        soil, tempC, humidity, pir, pumpState, alertActive, pumpFault);

    uploadReading(
        soil, tempC, humidity, pir, temperatureValid, humidityValid);
  }

  delay(10);
}