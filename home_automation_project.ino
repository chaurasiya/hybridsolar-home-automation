#include <WiFi.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <DHT.h>
#include <FirebaseESP32.h>
#include <time.h>
#include <esp_task_wdt.h>
#include <esp_system.h>
#include <math.h>

// ==========================
// Pin Configuration
// ==========================
#define GRID_PIN       14    
#define CONTACTOR_PIN  5     
#define FAN_PIN        23    
#define VOLTAGE_PIN    35    

#define DHTPIN         16
#define DHTTYPE        DHT22

const int GRID_ON_LEVEL = LOW;
const bool RELAY_ACTIVE_LOW = false;
const int RELAY_ON  = RELAY_ACTIVE_LOW ? LOW : HIGH;
const int RELAY_OFF = RELAY_ACTIVE_LOW ? HIGH : LOW;

LiquidCrystal_I2C lcd(0x27, 16, 2);
DHT dht(DHTPIN, DHTTYPE);

// ==========================
// Network & API (Your Originals Maintained)
// ==========================
const char* WIFI_SSID = "abhi-wifi-2.4G";
const char* WIFI_PASSWORD = "YOUR_WIFI_PASSWORD";

#define DATABASE_URL "YOUR_FIREBASE_DATABASE_URL"
#define API_KEY "YOUR_FIREBASE_API_KEY"

bool wifiConnected = false;
unsigned long lastWiFiReconnectAttempt = 0;
const unsigned long WIFI_RECONNECT_INTERVAL = 30000;
uint8_t wifiFailCount = 0;

FirebaseData fbdo;
FirebaseAuth auth;
FirebaseConfig config;
uint16_t firebaseReadFailCount = 0;
uint16_t firebaseWriteFailCount = 0;

// ==========================
// Battery Configuration
// ==========================
float calFactor = 20.05; 
const float LOW_BATTERY_CUTOFF = 24.0;
const float BATTERY_RECOVER    = 25.0;

const bool ALLOW_MANUAL_CONTACTOR_OVERRIDE = true;
const float MANUAL_OVERRIDE_MIN_VOLTAGE = 23.5; 

bool sensorFault = false;
float prevBatteryVoltage = 0.0; 
float filteredVoltage = 0.0; // [FIX] Initialized to 0.0
uint8_t adcFaultCount = 0;

// ==========================
// Timers & Delays
// ==========================
const float FAN_ON_TEMP  = 37.0;
const float FAN_OFF_TEMP = 35.5;
uint8_t dhtFailCount = 0;

const unsigned long BATTERY_READ_INTERVAL   = 1000;
const unsigned long DHT_READ_INTERVAL       = 2500;
const unsigned long LCD_PAGE_TIME           = 3000;
const unsigned long SERIAL_INTERVAL         = 1000;
const unsigned long FIREBASE_LIVE_INTERVAL  = 5000;
const unsigned long FIREBASE_HISTORY_INTERVAL = 60000; 
const unsigned long CONTROL_READ_INTERVAL   = 2000;
const unsigned long CONTROL_TIMEOUT         = 30000; 
const unsigned long CONTACTOR_DELAY         = 3000; 
const unsigned long NTP_SYNC_INTERVAL       = 86400000; 

const unsigned long MIN_OFF_TO_ON_INTERVAL  = 5000;  
const unsigned long MIN_ON_TO_OFF_INTERVAL  = 30000; 
const unsigned long THIRTY_DAYS_SECONDS     = 2592000; 

unsigned long lastBatteryRead = 0, lastDhtRead = 0, lastLcdUpdate = 0, lastSerialPrint = 0;
unsigned long lastFirebaseLiveUpdate = 0, lastFirebaseHistoryUpdate = 0;
unsigned long lastControlRead = 0, lastControlSuccess = 0;
unsigned long gridRestoredAt = 0, lastContactorSwitch = 0, lastNtpSync = 0;

const unsigned long GRID_ON_CONFIRM_TIME  = 5000;
const unsigned long GRID_OFF_CONFIRM_TIME = 5000;

bool rawGridOn = false, lastRawGridOn = false, stableGridOn = false, prevGridState = false;
unsigned long rawGridChangedAt = 0;

// ==========================
// Persistence (RTC Memory)
// ==========================
RTC_DATA_ATTR unsigned long powerCutTime = 0;
RTC_DATA_ATTR uint32_t contactorCycles = 0; 
RTC_DATA_ATTR uint32_t brownoutCount = 0;   

// ==========================
// System Runtime State
// ==========================
float batteryVoltage = 0.0, temperature = NAN, humidity = NAN;
bool batteryOk = false, contactorOn = false, fanOn = false;
bool controlFanManual = false, controlFanActive = false;
bool controlContactorManual = false, controlContactorActive = false;

int systemResetReason = 0;
unsigned long maxLoopTimeUs = 0;

byte lcdPage = 0;
byte batteryIcon[8] = { B01110, B10001, B10001, B11111, B11111, B11111, B11111, B00000 };

// ==========================
// Helper Functions
// ==========================
unsigned long getUptimeSeconds() { return millis() / 1000; }
bool isTimeValid() { return time(nullptr) > 1700000000; }
unsigned long getEpochOrUptime() { return isTimeValid() ? (unsigned long)time(nullptr) : getUptimeSeconds(); }

void safeRestart(const __FlashStringHelper* msg) {
  Serial.println(msg);
  digitalWrite(CONTACTOR_PIN, RELAY_OFF);
  digitalWrite(FAN_PIN, RELAY_OFF);
  delay(150);
  ESP.restart();
}

void printFirebaseError(const char* action) {
  Serial.print(F("[FIREBASE] "));
  Serial.print(action);
  Serial.print(F(" failed: "));
  Serial.println(fbdo.errorReason());
}

// ==========================
// Sensor Updates
// ==========================
float readBatteryVoltage() {
  uint32_t sum = 0;
  const uint8_t samples = 32; 
  for (uint8_t i = 0; i < samples; i++) {
    sum += analogReadMilliVolts(VOLTAGE_PIN); 
    delayMicroseconds(200); 
  }
  float avgMilliVolts = sum / float(samples);
  return (avgMilliVolts / 1000.0) * calFactor; 
}

void updateBatteryStatus() {
  unsigned long now = millis();
  if (now - lastBatteryRead < BATTERY_READ_INTERVAL) return;

  lastBatteryRead = now;
  float currentVolts = readBatteryVoltage();
  
  if (prevBatteryVoltage == 0.0) {
      prevBatteryVoltage = currentVolts; 
      filteredVoltage = currentVolts;
  }
  
  filteredVoltage = (filteredVoltage * 0.9) + (currentVolts * 0.1);
  currentVolts = filteredVoltage;
  
  float delta = fabs(currentVolts - prevBatteryVoltage);
  
  if (delta > 5.0) {
      adcFaultCount++;
      if (adcFaultCount >= 3) {
          sensorFault = true;
          prevBatteryVoltage = currentVolts; 
      }
  } else {
      adcFaultCount = 0;
      sensorFault = (currentVolts < 5.0 || currentVolts > 35.0);
      prevBatteryVoltage = currentVolts;
  }

  batteryVoltage = currentVolts;

  if (sensorFault) {
    batteryOk = false;
  } else {
    if (batteryVoltage < LOW_BATTERY_CUTOFF) batteryOk = false;
    else if (batteryVoltage >= BATTERY_RECOVER) batteryOk = true;
  }
}

void updateDhtStatus() {
  unsigned long now = millis();
  if (now - lastDhtRead < DHT_READ_INTERVAL) return;

  lastDhtRead = now;
  float newTemp = dht.readTemperature();
  float newHum  = dht.readHumidity();

  if (isnan(newTemp) || isnan(newHum)) {
    dhtFailCount++;
    // [FIX] Fast DHT Recovery
    if (dhtFailCount > 5) {
      temperature = NAN; 
      humidity = NAN;
      dht.begin();
      dhtFailCount = 0;
    }
  } else {
    dhtFailCount = 0;
    temperature = newTemp;
    humidity = newHum;
  }
}

void updateGridStatus() {
  rawGridOn = digitalRead(GRID_PIN) == GRID_ON_LEVEL;
  unsigned long now = millis();

  if (rawGridOn != lastRawGridOn) {
    lastRawGridOn = rawGridOn;
    rawGridChangedAt = now;
  }

  unsigned long stableTime = now - rawGridChangedAt;
  
  if (rawGridOn && stableTime >= GRID_ON_CONFIRM_TIME) {
    if (!stableGridOn) {
      stableGridOn = true;
      gridRestoredAt = millis(); 
    }
  }
  if (!rawGridOn && stableTime >= GRID_OFF_CONFIRM_TIME) {
    stableGridOn = false;
  }
}

// ==========================
// Network & Firebase Logic
// ==========================
void maintainWiFi() {
  bool currentWiFi = (WiFi.status() == WL_CONNECTED);

  if (wifiConnected && !currentWiFi) {
    controlFanManual = false;
    controlContactorManual = false;
    Serial.println(F("[NETWORK] WiFi lost! Forced AUTO mode."));
  }

  wifiConnected = currentWiFi;
  if (wifiConnected) {
    wifiFailCount = 0;
    return;
  }

  unsigned long now = millis();
  if (now - lastWiFiReconnectAttempt < WIFI_RECONNECT_INTERVAL) return;

  lastWiFiReconnectAttempt = now;
  wifiFailCount++;
  
  if (wifiFailCount > 20) {
    safeRestart(F("[NETWORK] WiFi stack dead. Rebooting system..."));
  }

  Serial.println(F("[NETWORK] Reconnecting WiFi..."));
  WiFi.disconnect(false);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
}

void enforceControlTimeout() {
  if (lastControlSuccess == 0) return;
  bool cloudUnavailable = !wifiConnected || !Firebase.ready();

  if (cloudUnavailable && millis() - lastControlSuccess > CONTROL_TIMEOUT) {
    controlFanManual = false;
    controlContactorManual = false;
    Serial.println(F("[CONTROL] Cloud timeout. Returned to AUTO mode."));
  }
}

void readFirebaseControls() {
  if (!wifiConnected || !Firebase.ready()) return;

  unsigned long now = millis();
  if (now - lastControlRead < CONTROL_READ_INTERVAL) return;
  lastControlRead = now;

  unsigned long fbStart = millis(); 
  
  if (Firebase.getJSON(fbdo, "/control")) {
      FirebaseJsonData jsonData;
      FirebaseJson& json = fbdo.jsonObject();

      json.get(jsonData, "fan/manual");
      if (jsonData.success) controlFanManual = jsonData.boolValue;

      json.get(jsonData, "fan/active");
      if (jsonData.success) controlFanActive = jsonData.boolValue;

      json.get(jsonData, "contactor/manual");
      if (jsonData.success) controlContactorManual = jsonData.boolValue;

      json.get(jsonData, "contactor/active");
      if (jsonData.success) controlContactorActive = jsonData.boolValue;

      lastControlSuccess = now;
      firebaseReadFailCount = 0;
  } else {
      firebaseReadFailCount++;
  }

  if (Firebase.getFloat(fbdo, "/settings/calFactor")) {
     float newCal = fbdo.floatData();
     if (newCal > 10.0 && newCal < 40.0) calFactor = newCal;
  }

  if (millis() - fbStart > 2000) Serial.println(F("[WARNING] Slow Firebase Read"));

  if (firebaseReadFailCount > 50) {
    safeRestart(F("[FIREBASE] Read Deadlock. Rebooting..."));
  }
}

// ==========================
// Hardware Control
// ==========================
void updateContactor() {
  bool gridSafe = stableGridOn && (millis() - gridRestoredAt >= CONTACTOR_DELAY);
  
  // [FIX] Emergency Hard Cutoff to protect battery from deep discharge
  if (!gridSafe && batteryVoltage < 23.5) {
      contactorOn = false;
      digitalWrite(CONTACTOR_PIN, RELAY_OFF);
      return; 
  }

  bool autoPowerAvailable = gridSafe || (batteryOk && !sensorFault);
  bool manualOverrideAllowed = gridSafe || (ALLOW_MANUAL_CONTACTOR_OVERRIDE && batteryVoltage >= MANUAL_OVERRIDE_MIN_VOLTAGE && !sensorFault);

  bool targetContactor = false;

  if (controlContactorManual) {
    targetContactor = controlContactorActive && manualOverrideAllowed;
  } else {
    targetContactor = autoPowerAvailable;
  }

  if (targetContactor != contactorOn) {
    unsigned long requiredDelay = targetContactor ? MIN_OFF_TO_ON_INTERVAL : MIN_ON_TO_OFF_INTERVAL;
    
    if (millis() - lastContactorSwitch > requiredDelay) {
      // [FIX] Count ONLY 'ON' transitions
      if (targetContactor && !contactorOn) {
          contactorCycles++;
      }
      contactorOn = targetContactor;
      lastContactorSwitch = millis();
    }
  }

  digitalWrite(CONTACTOR_PIN, contactorOn ? RELAY_ON : RELAY_OFF);
}

void updateExhaustFan() {
  if (controlFanManual) {
    fanOn = controlFanActive;
  } else {
    if (isnan(temperature)) fanOn = false;
    else if (!fanOn && temperature > FAN_ON_TEMP) fanOn = true;
    else if (fanOn && temperature <= FAN_OFF_TEMP) fanOn = false;
  }
  digitalWrite(FAN_PIN, fanOn ? RELAY_ON : RELAY_OFF);
}

// ==========================
// Displays & Firebase Write
// ==========================
void updateLcd() {
  unsigned long now = millis();
  if (now - lastLcdUpdate < LCD_PAGE_TIME) return;
  lastLcdUpdate = now;
  lcdPage = (lcdPage + 1) % 5;
  lcd.clear();

  if (lcdPage == 0) {
    lcd.setCursor(0, 0); lcd.print(F("GRID:")); lcd.print(stableGridOn ? F("ON ") : F("OFF"));
    lcd.setCursor(10, 0); lcd.print(F("R:")); lcd.print(rawGridOn ? F("ON") : F("OFF"));
    lcd.setCursor(0, 1); lcd.print(F("CNT:")); lcd.print(contactorOn ? F("ON ") : F("OFF"));
    lcd.setCursor(9, 1); lcd.print(batteryOk ? F("BAT OK") : F("BAT LOW"));
  } else if (lcdPage == 1) {
    lcd.setCursor(0, 0); lcd.print(F("BATTERY VOLTAGE"));
    lcd.setCursor(2, 1); 
    if (sensorFault) { lcd.print(F("SENSOR FAULT!")); } 
    else { lcd.write(byte(0)); lcd.print(F(" ")); lcd.print(batteryVoltage, 1); lcd.print(F("V")); }
  } else if (lcdPage == 2) {
    lcd.setCursor(2, 0); lcd.print(F("Temperature"));
    lcd.setCursor(0, 1);
    if (isnan(temperature)) lcd.print(F("Sensor Error"));
    else { lcd.print(temperature, 1); lcd.print(F(" C ")); lcd.setCursor(9, 1); lcd.print(fanOn ? F("FAN:ON") : F("FAN:OFF")); }
  } else if (lcdPage == 3) {
    lcd.setCursor(4, 0); lcd.print(F("Humidity"));
    lcd.setCursor(6, 1);
    if (isnan(humidity)) lcd.print(F("Sensor Err"));
    else { lcd.print(humidity, 0); lcd.print(F(" %")); }
  } else {
    lcd.setCursor(3, 0); lcd.print(F("WiFi Status"));
    lcd.setCursor(2, 1);
    if (wifiConnected) lcd.print(F("Connected"));
    else lcd.print(F("Offline Mode"));
  }
}

void updateSerial() {
  unsigned long now = millis();
  if (now - lastSerialPrint < SERIAL_INTERVAL) return;
  lastSerialPrint = now;
  Serial.printf("RAW_GRID=%s | STABLE_GRID=%s | BAT=%.2fV | BAT_OK=%s | TEMP=%.1fC | HUM=%.0f%% | FAN=%s | CNT=%s | WIFI=%s | HEAP=%u\n",
                rawGridOn ? "ON" : "OFF", stableGridOn ? "ON" : "OFF", batteryVoltage, batteryOk ? "YES" : "NO",
                temperature, humidity, fanOn ? "ON" : "OFF", contactorOn ? "ON" : "OFF",
                wifiConnected ? "ON" : "OFF", ESP.getMinFreeHeap());
}

void logGridEvent() {
  if (!wifiConnected || !Firebase.ready() || stableGridOn == prevGridState) return;
  if (!isTimeValid()) return; 

  prevGridState = stableGridOn;
  time_t eventTime = time(nullptr);
  unsigned long epochTime = (unsigned long)eventTime;
  struct tm *ptm = localtime(&eventTime);
  char timeString[16];
  strftime(timeString, sizeof(timeString), "%I:%M %p", ptm);

  FirebaseJson eventJson;
  if (!stableGridOn) {
    powerCutTime = epochTime;
    eventJson.set("powerCut", timeString);
    eventJson.set("restored", "--:--");
    eventJson.set("duration", "0 Min");
  } else {
    eventJson.set("restored", timeString);
    if (powerCutTime > 0 && epochTime >= powerCutTime) {
      char durBuffer[32];
      snprintf(durBuffer, sizeof(durBuffer), "%lu Min", (epochTime - powerCutTime) / 60);
      eventJson.set("duration", durBuffer);
    }
  }

  // Use writeSuccess logic for cleaner tracking
  bool writeSuccess = true;
  writeSuccess &= Firebase.updateNode(fbdo, "/status/gridEvents", eventJson);

  char path[64];
  snprintf(path, sizeof(path), "/logs/%lu", epochTime);
  FirebaseJson logJson;
  logJson.set("event", stableGridOn ? "POWER_ON" : "POWER_OFF");
  logJson.set("timestamp", (int)epochTime);
  
  writeSuccess &= Firebase.updateNode(fbdo, path, logJson);
  
  if (!writeSuccess) {
      firebaseWriteFailCount++;
  } else {
      firebaseWriteFailCount = 0;
  }
}

void sendLiveStatusToFirebase() {
  if (!wifiConnected || !Firebase.ready()) return;

  unsigned long timestamp = getEpochOrUptime();
  bool manualOverrideAllowed = stableGridOn || (ALLOW_MANUAL_CONTACTOR_OVERRIDE && batteryVoltage >= MANUAL_OVERRIDE_MIN_VOLTAGE && !sensorFault);

  FirebaseJson statusJson;
  statusJson.set("battery", batteryVoltage);
  if (!isnan(temperature)) statusJson.set("temperature", temperature);
  if (!isnan(humidity)) statusJson.set("humidity", humidity);
  statusJson.set("fan", fanOn);
  statusJson.set("contactor", contactorOn);
  statusJson.set("gridStatus", stableGridOn ? "ONLINE" : "OFFLINE");
  statusJson.set("timestamp", (int)timestamp);
  statusJson.set("sensorFault", sensorFault);

  FirebaseJson systemJson;
  systemJson.set("wifiSSID", WiFi.SSID());
  systemJson.set("wifiRSSI", WiFi.RSSI());
  systemJson.set("minFreeHeap", ESP.getMinFreeHeap()); 
  systemJson.set("uptimeSec", (int)getUptimeSeconds());
  systemJson.set("firmware", "v4.0.0-final");
  systemJson.set("timeValid", isTimeValid());
  systemJson.set("controlFanManual", controlFanManual);
  systemJson.set("controlContactorManual", controlContactorManual);
  systemJson.set("manualContactorOverrideAllowed", manualOverrideAllowed);
  systemJson.set("lastResetReason", systemResetReason);
  systemJson.set("brownoutCount", brownoutCount);
  systemJson.set("cycles", contactorCycles);
  systemJson.set("maxLoopUs", (int)maxLoopTimeUs);

  // [FIX] Avoid updating Root `/`, update `/status` and `/system` safely
  bool writeSuccess = true;
  writeSuccess &= Firebase.updateNode(fbdo, "/status", statusJson);
  writeSuccess &= Firebase.updateNode(fbdo, "/system", systemJson);

  if (writeSuccess) {
      firebaseWriteFailCount = 0;
      maxLoopTimeUs = 0; 
  } else {
      firebaseWriteFailCount++;
      printFirebaseError("Live Status Sync");
  }

  if (firebaseWriteFailCount > 50) {
      safeRestart(F("[FIREBASE] Write Deadlock. Rebooting..."));
  }
}

void sendHistoryToFirebase() {
  if (!wifiConnected || !Firebase.ready()) return;
  if (!isTimeValid()) return;

  unsigned long epochTime = (unsigned long)time(nullptr);
  char historyPath[64];
  snprintf(historyPath, sizeof(historyPath), "/history/%lu", epochTime);

  FirebaseJson historyJson;
  historyJson.set("battery", batteryVoltage);
  if (!isnan(temperature)) historyJson.set("temperature", temperature);
  if (!isnan(humidity)) historyJson.set("humidity", humidity);
  historyJson.set("grid", stableGridOn ? "ONLINE" : "OFFLINE");

  if (!Firebase.updateNode(fbdo, historyPath, historyJson)) {
      firebaseWriteFailCount++;
  } else {
      firebaseWriteFailCount = 0;
  }
}

// ==========================
// Setup
// ==========================
void setup() {
  Serial.begin(115200);
  delay(500);

  // [FIX] Correct Brownout API Mix-up
  esp_reset_reason_t resetReason = esp_reset_reason();
  if (resetReason == ESP_RST_BROWNOUT) brownoutCount++;
  systemResetReason = (int)resetReason;
  
  Serial.printf("\n[INIT] ESP32 Booting... Reset Reason: %d\n", systemResetReason);

  esp_task_wdt_init(30, true);
  esp_task_wdt_add(NULL);

  pinMode(GRID_PIN, INPUT_PULLUP);
  pinMode(CONTACTOR_PIN, OUTPUT);
  pinMode(FAN_PIN, OUTPUT);

  // Absolute Safe Boot
  digitalWrite(CONTACTOR_PIN, RELAY_OFF);
  digitalWrite(FAN_PIN, RELAY_OFF);
  contactorOn = false;

  analogReadResolution(12);
  analogSetPinAttenuation(VOLTAGE_PIN, ADC_11db);

  lcd.init();
  lcd.backlight();
  lcd.createChar(0, batteryIcon);
  dht.begin();

  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(true);
  WiFi.persistent(false);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  lcd.clear();
  lcd.setCursor(2, 0); lcd.print(F("Connecting"));
  lcd.setCursor(4, 1); lcd.print(F("WiFi..."));

  unsigned long wifiStart = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - wifiStart < 10000) {
    delay(250);
    Serial.print(".");
    esp_task_wdt_reset(); 
  }

  wifiConnected = (WiFi.status() == WL_CONNECTED);
  if (wifiConnected) {
    Serial.println(F("\n[NETWORK] WiFi connected."));
  } else {
    Serial.println(F("\n[NETWORK] Timeout. Operating in offline mode."));
  }

  configTime(19800, 0, "pool.ntp.org", "time.nist.gov");

  if (wifiConnected) {
    Serial.print(F("[NETWORK] Waiting for NTP Sync..."));
    for (int i = 0; i < 20; i++) {
      if (isTimeValid()) break;
      delay(500);
      Serial.print(".");
      esp_task_wdt_reset(); 
    }
    Serial.println();
    lastNtpSync = millis();
  }

  config.api_key = API_KEY;
  config.database_url = DATABASE_URL;

  if (wifiConnected) {
    Firebase.signUp(&config, &auth, "", "");
    Firebase.begin(&config, &auth);
    Firebase.reconnectWiFi(true);
  }

  lastLcdUpdate = millis() - LCD_PAGE_TIME;

  rawGridOn = digitalRead(GRID_PIN) == GRID_ON_LEVEL;
  lastRawGridOn = rawGridOn;
  stableGridOn = false; 
  prevGridState = false;
  rawGridChangedAt = millis();

  batteryVoltage = readBatteryVoltage();
  prevBatteryVoltage = batteryVoltage;
  filteredVoltage = batteryVoltage; 
  
  sensorFault = (batteryVoltage < 5.0 || batteryVoltage > 35.0);
  if (!sensorFault) { batteryOk = batteryVoltage >= BATTERY_RECOVER; }
  
  temperature = dht.readTemperature();
  humidity = dht.readHumidity();

  updateContactor();
  updateExhaustFan();

  Serial.println(F("[INIT] Setup complete. System Live."));
}

// ==========================
// Main Execution Loop
// ==========================
void loop() {
  unsigned long loopStart = micros();
  esp_task_wdt_reset(); 

  if (ESP.getMinFreeHeap() < 15000) {
      safeRestart(F("[SYSTEM] Critical Low Heap. Auto-Healing Reboot..."));
  }

  if (getUptimeSeconds() > THIRTY_DAYS_SECONDS) { 
      safeRestart(F("[SYSTEM] 30-Day Scheduled Maintenance Reboot."));
  }

  if (millis() - lastNtpSync > NTP_SYNC_INTERVAL) {
      configTime(19800, 0, "pool.ntp.org", "time.nist.gov");
      lastNtpSync = millis();
  }

  maintainWiFi();          
  enforceControlTimeout(); 

  updateGridStatus();
  logGridEvent();

  updateBatteryStatus();
  updateDhtStatus();
  readFirebaseControls();  

  updateContactor();       
  updateExhaustFan();

  updateLcd();
  updateSerial();

  unsigned long now = millis();

  if (now - lastFirebaseLiveUpdate >= FIREBASE_LIVE_INTERVAL) {
    lastFirebaseLiveUpdate = now;
    sendLiveStatusToFirebase();
  }

  if (now - lastFirebaseHistoryUpdate >= FIREBASE_HISTORY_INTERVAL) {
    lastFirebaseHistoryUpdate = now;
    sendHistoryToFirebase(); 
  }

  unsigned long loopTime = micros() - loopStart;
  if(loopTime > maxLoopTimeUs) maxLoopTimeUs = loopTime;

  yield(); 
}
