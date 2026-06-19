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
// GPIO5 is used through your transistor/resistor driver stage.
#define GRID_PIN       14
#define CONTACTOR_PIN  5
#define FAN_PIN        23
#define VOLTAGE_PIN    35

#define DHTPIN         16
#define DHTTYPE        DHT22

// Grid detector logic:
// LOW  = Grid ON
// HIGH = Grid OFF
const int GRID_ON_LEVEL = LOW;

// Relay logic:
// Set true if your relay driver is active LOW.
const bool RELAY_ACTIVE_LOW = false;
const int RELAY_ON  = RELAY_ACTIVE_LOW ? LOW : HIGH;
const int RELAY_OFF = RELAY_ACTIVE_LOW ? HIGH : LOW;

// ==========================
// LCD and DHT
// ==========================
LiquidCrystal_I2C lcd(0x27, 16, 2);
DHT dht(DHTPIN, DHTTYPE);

// ==========================
// Network & Firebase
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

// Reused Firebase objects to reduce repeated heap churn.
FirebaseJson fbStatusJson;
FirebaseJson fbSystemJson;
FirebaseJson fbHistoryJson;
FirebaseJson fbEventJson;
FirebaseJson fbLogJson;
FirebaseJsonData fbJsonData;

// ==========================
// Battery Configuration
// ==========================
float calFactor = 20.05;

const float LOW_BATTERY_CUTOFF = 24.0;
const float BATTERY_RECOVER    = 25.0;
const float HARD_BATTERY_CUTOFF = 23.5;

const bool ALLOW_MANUAL_CONTACTOR_OVERRIDE = true;
const float MANUAL_OVERRIDE_MIN_VOLTAGE = 23.5;

bool sensorFault = false;
float batteryVoltage = 0.0;         // Protection/control voltage.
float displayVoltage = 0.0;         // Smoothed voltage for LCD/Firebase only.
float prevBatteryVoltage = 0.0;
uint8_t adcFaultCount = 0;

// ==========================
// Temperature / Fan
// ==========================
const float FAN_ON_TEMP  = 37.0;
const float FAN_OFF_TEMP = 35.5;
uint8_t dhtFailCount = 0;

float temperature = NAN;
float humidity = NAN;

// ==========================
// Timing
// ==========================
const unsigned long BATTERY_READ_INTERVAL      = 1000;
const unsigned long DHT_READ_INTERVAL          = 2500;
const unsigned long LCD_PAGE_TIME              = 3000;
const unsigned long SERIAL_INTERVAL            = 1000;

// Cloud work is telemetry/remote-control only. Local relay logic must not depend on it.
const unsigned long FIREBASE_LIVE_INTERVAL     = 10000;
const unsigned long FIREBASE_SYSTEM_INTERVAL   = 60000;
const unsigned long FIREBASE_HISTORY_INTERVAL  = 300000;
const unsigned long CONTROL_READ_INTERVAL      = 5000;

// Remote manual control is allowed only while cloud updates remain fresh.
// If WiFi/Firebase disappears, the unit returns to local AUTO mode.
const unsigned long CONTROL_TIMEOUT            = 30000;

// Industrial-style power stability timing:
// Grid OFF must be detected reasonably quickly.
// Grid ON must remain stable before load is allowed.
const unsigned long GRID_OFF_CONFIRM_TIME      = 2000;
const unsigned long GRID_ON_CONFIRM_TIME       = 30000;

// Contactor anti-short-cycle:
// Protective OFF is immediate. ON is delayed after any OFF.
const unsigned long MIN_OFF_TO_ON_INTERVAL     = 180000;

const unsigned long NTP_SYNC_INTERVAL          = 86400000;
const unsigned long THIRTY_DAYS_SECONDS        = 2592000;

unsigned long lastBatteryRead = 0;
unsigned long lastDhtRead = 0;
unsigned long lastLcdUpdate = 0;
unsigned long lastSerialPrint = 0;
unsigned long lastFirebaseLiveUpdate = 0;
unsigned long lastFirebaseSystemUpdate = 0;
unsigned long lastFirebaseHistoryUpdate = 0;
unsigned long lastControlRead = 0;
unsigned long lastControlSuccess = 0;
unsigned long lastNtpSync = 0;

unsigned long rawGridChangedAt = 0;
unsigned long gridRestoredAt = 0;
unsigned long lastContactorOffAt = 0;
unsigned long lastContactorSwitch = 0;

// ==========================
// Persistence Across Deep Sleep / Some Resets
// ==========================
RTC_DATA_ATTR unsigned long powerCutTime = 0;
RTC_DATA_ATTR uint32_t contactorCycles = 0;
RTC_DATA_ATTR uint32_t brownoutCount = 0;

// ==========================
// Runtime State
// ==========================
bool rawGridOn = false;
bool lastRawGridOn = false;
bool stableGridOn = false;
bool prevGridState = false;

bool batteryOk = false;
bool contactorOn = false;
bool fanOn = false;

bool controlFanManual = false;
bool controlFanActive = false;
bool controlContactorManual = false;
bool controlContactorActive = false;

int systemResetReason = 0;
unsigned long maxLoopTimeUs = 0;

byte lcdPage = 0;
byte batteryIcon[8] = {
  B01110,
  B10001,
  B10001,
  B11111,
  B11111,
  B11111,
  B11111,
  B00000
};

// ==========================
// Helper Functions
// ==========================
unsigned long getUptimeSeconds()
{
  return millis() / 1000;
}

bool isTimeValid()
{
  // 1700000000 is a simple sanity check that NTP has provided a real epoch time.
  return time(nullptr) > 1700000000;
}

unsigned long getEpochOrUptime()
{
  return isTimeValid() ? (unsigned long)time(nullptr) : getUptimeSeconds();
}

void safeRestart(const __FlashStringHelper* msg)
{
  // Always de-energize outputs before rebooting so a software recovery cannot
  // leave the contactor or fan unintentionally ON during restart.
  Serial.println(msg);
  digitalWrite(CONTACTOR_PIN, RELAY_OFF);
  digitalWrite(FAN_PIN, RELAY_OFF);
  delay(150);
  ESP.restart();
}

void printFirebaseError(const char* action)
{
  Serial.print(F("[FIREBASE] "));
  Serial.print(action);
  Serial.print(F(" failed: "));
  Serial.println(fbdo.errorReason());
}

const char* getContactorReason()
{
  if (sensorFault) return "SENSOR_FAULT";
  if (batteryVoltage < HARD_BATTERY_CUTOFF) return "HARD_BATTERY_CUTOFF";
  if (controlContactorManual && contactorOn) return "MANUAL_ON";
  if (controlContactorManual && !contactorOn) return "MANUAL_OFF_OR_BLOCKED";
  if (contactorOn && stableGridOn) return "GRID_ON";
  if (contactorOn && batteryOk) return "BATTERY_OK";
  if (!contactorOn && !stableGridOn && !batteryOk) return "GRID_OFF_BATTERY_LOW";
  if (!contactorOn && millis() - lastContactorOffAt < MIN_OFF_TO_ON_INTERVAL) return "ON_DELAY_ACTIVE";
  return contactorOn ? "ON" : "OFF";
}

const char* getFanReason()
{
  if (controlFanManual && fanOn) return "MANUAL_ON";
  if (controlFanManual && !fanOn) return "MANUAL_OFF";
  if (isnan(temperature)) return "TEMP_SENSOR_ERROR";
  if (fanOn) return "TEMP_HIGH";
  return "TEMP_NORMAL";
}

void setupWatchdog()
{
  // Arduino ESP32 core 3.x uses the ESP-IDF 5 watchdog API.
  // Older Arduino ESP32 core 2.x uses the simpler timeout/panic arguments.
#if ESP_IDF_VERSION_MAJOR >= 5
  esp_task_wdt_config_t wdtConfig = {};
  wdtConfig.timeout_ms = 30000;
  wdtConfig.idle_core_mask = (1 << portNUM_PROCESSORS) - 1;
  wdtConfig.trigger_panic = true;

  esp_err_t initResult = esp_task_wdt_init(&wdtConfig);
#else
  esp_err_t initResult = esp_task_wdt_init(30, true);
#endif

  if (initResult != ESP_OK && initResult != ESP_ERR_INVALID_STATE)
  {
    Serial.print(F("[WDT] Init failed: "));
    Serial.println((int)initResult);
  }

  esp_err_t addResult = esp_task_wdt_add(NULL);
  if (addResult != ESP_OK && addResult != ESP_ERR_INVALID_STATE)
  {
    Serial.print(F("[WDT] Add current task failed: "));
    Serial.println((int)addResult);
  }
}

// ==========================
// Sensor Updates
// ==========================
// Reads the divided battery voltage from ESP32 ADC1.
// analogReadMilliVolts() uses the ESP32 ADC calibration path when available.
float readBatteryVoltage()
{
  uint32_t sum = 0;
  const uint8_t samples = 32;

  for (uint8_t i = 0; i < samples; i++)
  {
    sum += analogReadMilliVolts(VOLTAGE_PIN);
    delayMicroseconds(200);
  }

  float avgMilliVolts = sum / float(samples);
  return (avgMilliVolts / 1000.0) * calFactor;
}

void updateBatteryStatus()
{
  // Battery protection uses the latest averaged voltage, not the display filter,
  // so low-voltage cutoff remains responsive.
  unsigned long now = millis();
  if (now - lastBatteryRead < BATTERY_READ_INTERVAL) return;

  lastBatteryRead = now;

  float currentVolts = readBatteryVoltage();

  if (prevBatteryVoltage == 0.0)
  {
    prevBatteryVoltage = currentVolts;
    displayVoltage = currentVolts;
  }

  // Keep protection responsive, but still catch impossible ADC jumps.
  float delta = fabs(currentVolts - prevBatteryVoltage);
  if (delta > 5.0)
  {
    // A sudden impossible jump usually means ADC/sensor wiring noise.
    // Require several bad samples before declaring a sensor fault.
    adcFaultCount++;
    if (adcFaultCount >= 3)
    {
      sensorFault = true;
      prevBatteryVoltage = currentVolts;
    }
  }
  else
  {
    adcFaultCount = 0;
    sensorFault = (currentVolts < 5.0 || currentVolts > 35.0);
    prevBatteryVoltage = currentVolts;
  }

  batteryVoltage = currentVolts;

  // Display/cloud smoothing only. Do not use this for protection cutoff.
  displayVoltage = (displayVoltage * 0.9) + (batteryVoltage * 0.1);

  if (sensorFault)
  {
    batteryOk = false;
  }
  else if (batteryVoltage < LOW_BATTERY_CUTOFF)
  {
    batteryOk = false;
  }
  else if (batteryVoltage >= BATTERY_RECOVER)
  {
    batteryOk = true;
  }
}

void updateDhtStatus()
{
  // DHT22 is slow; reading it too often causes bad samples.
  // Keep last good readings until repeated failures prove a real sensor issue.
  unsigned long now = millis();
  if (now - lastDhtRead < DHT_READ_INTERVAL) return;

  lastDhtRead = now;

  float newTemp = dht.readTemperature();
  float newHum  = dht.readHumidity();

  if (isnan(newTemp) || isnan(newHum))
  {
    dhtFailCount++;
    if (dhtFailCount > 5)
    {
      temperature = NAN;
      humidity = NAN;
      fanOn = false;
      dht.begin();
      dhtFailCount = 0;
    }
  }
  else
  {
    dhtFailCount = 0;
    temperature = newTemp;
    humidity = newHum;
  }
}

void updateGridStatus()
{
  // Debounce/filter the grid detector in software.
  // OFF and ON have different confirmation windows because restoration must be
  // stable before heavy load is allowed back.
  rawGridOn = digitalRead(GRID_PIN) == GRID_ON_LEVEL;
  unsigned long now = millis();

  if (rawGridOn != lastRawGridOn)
  {
    lastRawGridOn = rawGridOn;
    rawGridChangedAt = now;
  }

  unsigned long stableTime = now - rawGridChangedAt;

  if (rawGridOn && stableTime >= GRID_ON_CONFIRM_TIME)
  {
    if (!stableGridOn)
    {
      stableGridOn = true;
      gridRestoredAt = now;
    }
  }

  if (!rawGridOn && stableTime >= GRID_OFF_CONFIRM_TIME)
  {
    stableGridOn = false;
  }
}

// ==========================
// Local Hardware Control
// ==========================
void setContactor(bool targetOn)
{
  // Single place that changes contactor state.
  // This keeps cycle counting and OFF timestamp tracking consistent.
  if (targetOn == contactorOn) return;

  if (targetOn)
  {
    contactorCycles++;
  }
  else
  {
    lastContactorOffAt = millis();
  }

  contactorOn = targetOn;
  lastContactorSwitch = millis();
  digitalWrite(CONTACTOR_PIN, contactorOn ? RELAY_ON : RELAY_OFF);
}

void updateContactor()
{
  // Local safety gates always win over Firebase/manual requests.
  bool gridStableEnough = stableGridOn;

  // Immediate protective cutoff. Do not delay OFF for safety decisions.
  if (sensorFault || batteryVoltage < HARD_BATTERY_CUTOFF)
  {
    setContactor(false);
    return;
  }

  bool autoPowerAvailable = gridStableEnough || batteryOk;

  // Manual cloud override is still bounded by local electrical safety.
  // It cannot force the contactor ON below the configured minimum voltage.
  bool manualOverrideAllowed =
    gridStableEnough ||
    (ALLOW_MANUAL_CONTACTOR_OVERRIDE &&
     batteryVoltage >= MANUAL_OVERRIDE_MIN_VOLTAGE &&
     !sensorFault);

  bool targetContactor = controlContactorManual
    ? (controlContactorActive && manualOverrideAllowed)
    : autoPowerAvailable;

  if (!targetContactor)
  {
    setContactor(false);
    return;
  }

  // Delay only ON transitions to avoid contactor chatter and unstable restoration.
  if (!contactorOn && millis() - lastContactorOffAt < MIN_OFF_TO_ON_INTERVAL)
  {
    digitalWrite(CONTACTOR_PIN, RELAY_OFF);
    return;
  }

  setContactor(true);
}

void updateExhaustFan()
{
  // Fan manual mode is less risky than contactor manual mode, but it still
  // falls back to AUTO when cloud control times out.
  if (controlFanManual)
  {
    fanOn = controlFanActive;
  }
  else if (isnan(temperature))
  {
    fanOn = false;
  }
  else if (!fanOn && temperature > FAN_ON_TEMP)
  {
    fanOn = true;
  }
  else if (fanOn && temperature <= FAN_OFF_TEMP)
  {
    // Hysteresis prevents relay chatter around the 37C threshold.
    fanOn = false;
  }

  digitalWrite(FAN_PIN, fanOn ? RELAY_ON : RELAY_OFF);
}

void runLocalControl()
{
  // This is the core control loop. It must be safe to run with no WiFi,
  // no Firebase, and invalid time.
  updateGridStatus();
  updateBatteryStatus();
  updateDhtStatus();
  updateContactor();
  updateExhaustFan();
}

// ==========================
// Network & Firebase
// ==========================
void maintainWiFi()
{
  // WiFi reconnect is intentionally non-fatal for control.
  // Local automation continues even if cloud connectivity is unavailable.
  bool currentWiFi = (WiFi.status() == WL_CONNECTED);

  if (wifiConnected && !currentWiFi)
  {
    controlFanManual = false;
    controlContactorManual = false;
    Serial.println(F("[NETWORK] WiFi lost. Forced AUTO mode."));
  }

  wifiConnected = currentWiFi;
  if (wifiConnected)
  {
    wifiFailCount = 0;
    return;
  }

  unsigned long now = millis();
  if (now - lastWiFiReconnectAttempt < WIFI_RECONNECT_INTERVAL) return;

  lastWiFiReconnectAttempt = now;
  wifiFailCount++;

  if (wifiFailCount > 20)
  {
    safeRestart(F("[NETWORK] WiFi stack recovery reboot."));
  }

  Serial.println(F("[NETWORK] Reconnecting WiFi..."));
  WiFi.disconnect(false);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
}

void enforceControlTimeout()
{
  // Remote commands expire if cloud updates stop arriving.
  // This prevents stale dashboard state from controlling real hardware forever.
  if (lastControlSuccess == 0) return;

  bool cloudUnavailable = !wifiConnected || !Firebase.ready();
  if (cloudUnavailable && millis() - lastControlSuccess > CONTROL_TIMEOUT)
  {
    controlFanManual = false;
    controlContactorManual = false;
    Serial.println(F("[CONTROL] Cloud timeout. Returned to AUTO mode."));
  }
}

void readFirebaseControls()
{
  // Pull remote manual commands from Firebase.
  // These values are requests only; updateContactor() still applies local safety.
  if (!wifiConnected || !Firebase.ready()) return;

  unsigned long now = millis();
  if (now - lastControlRead < CONTROL_READ_INTERVAL) return;
  lastControlRead = now;

  if (Firebase.getJSON(fbdo, "/control"))
  {
    FirebaseJson& json = fbdo.jsonObject();

    json.get(fbJsonData, "fan/manual");
    if (fbJsonData.success) controlFanManual = fbJsonData.boolValue;

    json.get(fbJsonData, "fan/active");
    if (fbJsonData.success) controlFanActive = fbJsonData.boolValue;

    json.get(fbJsonData, "contactor/manual");
    if (fbJsonData.success) controlContactorManual = fbJsonData.boolValue;

    json.get(fbJsonData, "contactor/active");
    if (fbJsonData.success) controlContactorActive = fbJsonData.boolValue;

    lastControlSuccess = now;
    firebaseReadFailCount = 0;
  }
  else
  {
    firebaseReadFailCount++;
  }

  if (Firebase.getFloat(fbdo, "/settings/calFactor"))
  {
    float newCal = fbdo.floatData();
    if (newCal > 10.0 && newCal < 40.0) calFactor = newCal;
  }

  if (firebaseReadFailCount > 50)
  {
    controlFanManual = false;
    controlContactorManual = false;
    firebaseReadFailCount = 0;
    Serial.println(F("[FIREBASE] Repeated read failures. Controls forced AUTO."));
  }
}

void logGridEvent()
{
  // Logs only state transitions, not every loop.
  // This keeps Firebase writes small while preserving the important events.
  if (!wifiConnected || !Firebase.ready()) return;
  if (!isTimeValid()) return;
  if (stableGridOn == prevGridState) return;

  prevGridState = stableGridOn;

  time_t eventTime = time(nullptr);
  unsigned long epochTime = (unsigned long)eventTime;
  struct tm *ptm = localtime(&eventTime);

  char timeString[16];
  strftime(timeString, sizeof(timeString), "%I:%M %p", ptm);

  fbEventJson.clear();

  if (!stableGridOn)
  {
    powerCutTime = epochTime;
    fbEventJson.set("powerCut", timeString);
    fbEventJson.set("restored", "--:--");
    fbEventJson.set("duration", "0 Min");
  }
  else
  {
    fbEventJson.set("restored", timeString);
    if (powerCutTime > 0 && epochTime >= powerCutTime)
    {
      char durBuffer[32];
      snprintf(durBuffer, sizeof(durBuffer), "%lu Min", (epochTime - powerCutTime) / 60);
      fbEventJson.set("duration", durBuffer);
    }
  }

  bool writeSuccess = Firebase.updateNode(fbdo, "/status/gridEvents", fbEventJson);

  char path[64];
  snprintf(path, sizeof(path), "/logs/%lu", epochTime);

  fbLogJson.clear();
  fbLogJson.set("event", stableGridOn ? "POWER_ON" : "POWER_OFF");
  fbLogJson.set("timestamp", (int)epochTime);
  fbLogJson.set("battery", batteryVoltage);
  fbLogJson.set("contactor", contactorOn);

  writeSuccess &= Firebase.updateNode(fbdo, path, fbLogJson);

  firebaseWriteFailCount = writeSuccess ? 0 : firebaseWriteFailCount + 1;
}

void sendLiveStatusToFirebase()
{
  // Fast-changing operational state for dashboards.
  // Safe to skip if WiFi/Firebase is down.
  if (!wifiConnected || !Firebase.ready()) return;

  unsigned long timestamp = getEpochOrUptime();

  fbStatusJson.clear();
  fbStatusJson.set("deviceOnline", true);
  fbStatusJson.set("wifi", wifiConnected);
  fbStatusJson.set("firebaseReady", Firebase.ready());
  fbStatusJson.set("battery", displayVoltage);
  fbStatusJson.set("batteryControlVoltage", batteryVoltage);
  if (!isnan(temperature)) fbStatusJson.set("temperature", temperature);
  if (!isnan(humidity)) fbStatusJson.set("humidity", humidity);
  fbStatusJson.set("fan", fanOn);
  fbStatusJson.set("fanReason", getFanReason());
  fbStatusJson.set("contactor", contactorOn);
  fbStatusJson.set("contactorReason", getContactorReason());
  fbStatusJson.set("gridStatus", stableGridOn ? "ONLINE" : "OFFLINE");
  fbStatusJson.set("rawGrid", rawGridOn ? "ONLINE" : "OFFLINE");
  fbStatusJson.set("timestamp", (int)timestamp);
  fbStatusJson.set("sensorFault", sensorFault);
  fbStatusJson.set("batteryOk", batteryOk);

  if (Firebase.updateNode(fbdo, "/status", fbStatusJson))
  {
    firebaseWriteFailCount = 0;
    Serial.println(F("[FIREBASE] /status updated."));
  }
  else
  {
    firebaseWriteFailCount++;
    printFirebaseError("Live Status Sync");
  }
}

void sendSystemToFirebase()
{
  // Lower-frequency health/KPI payload.
  // Heap, reset reason, loop time, and cycles are useful for long-term reliability.
  if (!wifiConnected || !Firebase.ready()) return;

  bool manualOverrideAllowed =
    stableGridOn ||
    (ALLOW_MANUAL_CONTACTOR_OVERRIDE &&
     batteryVoltage >= MANUAL_OVERRIDE_MIN_VOLTAGE &&
     !sensorFault);

  fbSystemJson.clear();
  fbSystemJson.set("wifiSSID", WiFi.SSID());
  fbSystemJson.set("wifiRSSI", WiFi.RSSI());
  fbSystemJson.set("freeHeap", ESP.getFreeHeap());
  fbSystemJson.set("minFreeHeap", ESP.getMinFreeHeap());
  fbSystemJson.set("uptimeSec", (int)getUptimeSeconds());
  fbSystemJson.set("firmware", "v4.1.0-reliable");
  fbSystemJson.set("timeValid", isTimeValid());
  fbSystemJson.set("wifi", wifiConnected);
  fbSystemJson.set("firebaseReady", Firebase.ready());
  fbSystemJson.set("controlFanManual", controlFanManual);
  fbSystemJson.set("controlContactorManual", controlContactorManual);
  fbSystemJson.set("manualContactorOverrideAllowed", manualOverrideAllowed);
  fbSystemJson.set("lastResetReason", systemResetReason);
  fbSystemJson.set("brownoutCount", brownoutCount);
  fbSystemJson.set("cycles", contactorCycles);
  fbSystemJson.set("maxLoopUs", (int)maxLoopTimeUs);

  if (Firebase.updateNode(fbdo, "/system", fbSystemJson))
  {
    firebaseWriteFailCount = 0;
    maxLoopTimeUs = 0;
  }
  else
  {
    firebaseWriteFailCount++;
    printFirebaseError("System Sync");
  }
}

void sendHistoryToFirebase()
{
  // Periodic trend data. Events are logged separately in logGridEvent().
  if (!wifiConnected || !Firebase.ready()) return;
  if (!isTimeValid()) return;

  unsigned long epochTime = (unsigned long)time(nullptr);

  char historyPath[64];
  snprintf(historyPath, sizeof(historyPath), "/history/%lu", epochTime);

  fbHistoryJson.clear();
  fbHistoryJson.set("battery", displayVoltage);
  fbHistoryJson.set("batteryControlVoltage", batteryVoltage);
  if (!isnan(temperature)) fbHistoryJson.set("temperature", temperature);
  if (!isnan(humidity)) fbHistoryJson.set("humidity", humidity);
  fbHistoryJson.set("grid", stableGridOn ? "ONLINE" : "OFFLINE");
  fbHistoryJson.set("fan", fanOn);
  fbHistoryJson.set("contactor", contactorOn);
  fbHistoryJson.set("fanReason", getFanReason());
  fbHistoryJson.set("contactorReason", getContactorReason());

  if (Firebase.updateNode(fbdo, historyPath, fbHistoryJson))
  {
    firebaseWriteFailCount = 0;
  }
  else
  {
    firebaseWriteFailCount++;
  }
}

void runCloudTasks()
{
  // All cloud work is grouped here so the main loop can clearly run:
  // local control -> cloud tasks -> local control again.
  maintainWiFi();
  enforceControlTimeout();

  // Read remote controls before writes, but local safety still gates all outputs.
  readFirebaseControls();
  logGridEvent();

  unsigned long now = millis();

  if (now - lastFirebaseLiveUpdate >= FIREBASE_LIVE_INTERVAL)
  {
    lastFirebaseLiveUpdate = now;
    sendLiveStatusToFirebase();
  }

  if (now - lastFirebaseSystemUpdate >= FIREBASE_SYSTEM_INTERVAL)
  {
    lastFirebaseSystemUpdate = now;
    sendSystemToFirebase();
  }

  if (now - lastFirebaseHistoryUpdate >= FIREBASE_HISTORY_INTERVAL)
  {
    lastFirebaseHistoryUpdate = now;
    sendHistoryToFirebase();
  }

  if (firebaseWriteFailCount > 50)
  {
    firebaseWriteFailCount = 0;
    Serial.println(F("[FIREBASE] Repeated write failures. Continuing local control."));
  }
}

// ==========================
// LCD and Serial
// ==========================
void updateLcd()
{
  // LCD is intentionally updated on a timer to avoid display work slowing
  // the relay control loop.
  unsigned long now = millis();
  if (now - lastLcdUpdate < LCD_PAGE_TIME) return;

  lastLcdUpdate = now;
  lcdPage = (lcdPage + 1) % 5;

  lcd.clear();

  if (lcdPage == 0)
  {
    lcd.setCursor(0, 0);
    lcd.print(F("GRID:"));
    lcd.print(stableGridOn ? F("ON ") : F("OFF"));

    lcd.setCursor(10, 0);
    lcd.print(F("R:"));
    lcd.print(rawGridOn ? F("ON") : F("OFF"));

    lcd.setCursor(0, 1);
    lcd.print(F("CNT:"));
    lcd.print(contactorOn ? F("ON ") : F("OFF"));

    lcd.setCursor(9, 1);
    lcd.print(batteryOk ? F("BAT OK") : F("BAT LOW"));
  }
  else if (lcdPage == 1)
  {
    lcd.setCursor(0, 0);
    lcd.print(F("BATTERY VOLT"));

    lcd.setCursor(0, 1);
    if (sensorFault)
    {
      lcd.print(F("SENSOR FAULT"));
    }
    else
    {
      lcd.write(byte(0));
      lcd.print(F(" "));
      lcd.print(displayVoltage, 1);
      lcd.print(F("V"));
    }
  }
  else if (lcdPage == 2)
  {
    lcd.setCursor(2, 0);
    lcd.print(F("Temperature"));

    lcd.setCursor(0, 1);
    if (isnan(temperature))
    {
      lcd.print(F("Sensor Error"));
    }
    else
    {
      lcd.print(temperature, 1);
      lcd.print(F(" C "));
      lcd.setCursor(9, 1);
      lcd.print(fanOn ? F("FAN:ON") : F("FAN:OFF"));
    }
  }
  else if (lcdPage == 3)
  {
    lcd.setCursor(4, 0);
    lcd.print(F("Humidity"));

    lcd.setCursor(4, 1);
    if (isnan(humidity))
    {
      lcd.print(F("Sensor Err"));
    }
    else
    {
      lcd.print(humidity, 0);
      lcd.print(F(" %"));
    }
  }
  else
  {
    lcd.setCursor(3, 0);
    lcd.print(F("WiFi Status"));

    lcd.setCursor(2, 1);
    lcd.print(wifiConnected ? F("Connected") : F("Offline Mode"));
  }
}

void updateSerial()
{
  // Serial diagnostics include current and minimum heap.
  // Current heap drives reboot decisions; minimum heap is a long-term KPI.
  unsigned long now = millis();
  if (now - lastSerialPrint < SERIAL_INTERVAL) return;

  lastSerialPrint = now;

  Serial.printf(
    "RAW_GRID=%s | STABLE_GRID=%s | BAT=%.2fV | BAT_OK=%s | TEMP=%.1fC | HUM=%.0f%% | FAN=%s | CNT=%s | WIFI=%s | HEAP=%u | MIN_HEAP=%u\n",
    rawGridOn ? "ON" : "OFF",
    stableGridOn ? "ON" : "OFF",
    batteryVoltage,
    batteryOk ? "YES" : "NO",
    temperature,
    humidity,
    fanOn ? "ON" : "OFF",
    contactorOn ? "ON" : "OFF",
    wifiConnected ? "ON" : "OFF",
    ESP.getFreeHeap(),
    ESP.getMinFreeHeap()
  );
}

// ==========================
// Setup
// ==========================
void setup()
{
  Serial.begin(115200);
  delay(500);

  esp_reset_reason_t resetReason = esp_reset_reason();
  if (resetReason == ESP_RST_BROWNOUT) brownoutCount++;
  systemResetReason = (int)resetReason;

  Serial.printf("\n[INIT] ESP32 Booting. Reset Reason: %d\n", systemResetReason);

  setupWatchdog();

  pinMode(GRID_PIN, INPUT_PULLUP);
  pinMode(CONTACTOR_PIN, OUTPUT);
  pinMode(FAN_PIN, OUTPUT);

  // Force outputs OFF before initializing network/cloud libraries.
  digitalWrite(CONTACTOR_PIN, RELAY_OFF);
  digitalWrite(FAN_PIN, RELAY_OFF);
  contactorOn = false;
  fanOn = false;
  lastContactorOffAt = millis();

  analogReadResolution(12);
  analogSetPinAttenuation(VOLTAGE_PIN, ADC_11db);

  lcd.init();
  lcd.backlight();
  lcd.createChar(0, batteryIcon);
  dht.begin();

  lcd.clear();
  lcd.setCursor(2, 0);
  lcd.print(F("SMART HOME"));
  lcd.setCursor(1, 1);
  lcd.print(F("Local First"));

  rawGridOn = digitalRead(GRID_PIN) == GRID_ON_LEVEL;
  lastRawGridOn = rawGridOn;
  stableGridOn = false;
  prevGridState = false;
  rawGridChangedAt = millis();

  batteryVoltage = readBatteryVoltage();
  displayVoltage = batteryVoltage;
  prevBatteryVoltage = batteryVoltage;
  sensorFault = (batteryVoltage < 5.0 || batteryVoltage > 35.0);
  batteryOk = !sensorFault && batteryVoltage >= BATTERY_RECOVER;

  temperature = dht.readTemperature();
  humidity = dht.readHumidity();

  updateContactor();
  updateExhaustFan();

  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(true);
  WiFi.persistent(false);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  lcd.clear();
  lcd.setCursor(2, 0);
  lcd.print(F("Connecting"));
  lcd.setCursor(4, 1);
  lcd.print(F("WiFi..."));

  unsigned long wifiStart = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - wifiStart < 10000)
  {
    esp_task_wdt_reset();
    // Keep local safety logic alive even during startup WiFi wait.
    runLocalControl();
    delay(250);
    Serial.print(".");
  }

  wifiConnected = (WiFi.status() == WL_CONNECTED);
  Serial.println(wifiConnected ? F("\n[NETWORK] WiFi connected.") : F("\n[NETWORK] Offline mode."));

  configTime(19800, 0, "pool.ntp.org", "time.nist.gov");

  if (wifiConnected)
  {
    Serial.print(F("[NETWORK] Waiting for NTP Sync..."));
    for (int i = 0; i < 20; i++)
    {
      esp_task_wdt_reset();
      // NTP wait must not pause local control.
      runLocalControl();
      if (isTimeValid()) break;
      delay(500);
      Serial.print(".");
    }
    Serial.println();
  }

  lastNtpSync = millis();

  config.api_key = API_KEY;
  config.database_url = DATABASE_URL;

  if (wifiConnected)
  {
    if (!Firebase.signUp(&config, &auth, "", ""))
    {
      Serial.println(F("[FIREBASE] Sign-up failed. Local control will continue."));
    }

    Firebase.begin(&config, &auth);
    Firebase.reconnectWiFi(true);
  }

  lastLcdUpdate = millis() - LCD_PAGE_TIME;
  lastFirebaseLiveUpdate = millis();
  lastFirebaseSystemUpdate = millis();
  lastFirebaseHistoryUpdate = millis();

  Serial.println(F("[INIT] Setup complete. System live."));
}

// ==========================
// Main Loop
// ==========================
void loop()
{
  // Main loop rule:
  // 1. Protect/control hardware locally.
  // 2. Attempt lower-priority cloud work.
  // 3. Re-run local control in case network calls were slow.
  unsigned long loopStart = micros();
  esp_task_wdt_reset();

  // Use current heap for the emergency check. Min heap is reported as KPI only.
  if (ESP.getFreeHeap() < 15000)
  {
    safeRestart(F("[SYSTEM] Current heap critically low. Rebooting."));
  }

  if (getUptimeSeconds() > THIRTY_DAYS_SECONDS)
  {
    safeRestart(F("[SYSTEM] 30-day scheduled maintenance reboot."));
  }

  if (millis() - lastNtpSync > NTP_SYNC_INTERVAL)
  {
    configTime(19800, 0, "pool.ntp.org", "time.nist.gov");
    lastNtpSync = millis();
  }

  // First local pass: always protect/control hardware before network work.
  runLocalControl();

  // Cloud is optional and lower priority.
  runCloudTasks();

  // Second local pass: recover quickly if a cloud call took time.
  runLocalControl();

  updateLcd();
  updateSerial();

  unsigned long loopTime = micros() - loopStart;
  if (loopTime > maxLoopTimeUs) maxLoopTimeUs = loopTime;

  yield();
}
