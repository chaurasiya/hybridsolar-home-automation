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
// LCD and DHT Setup
// ==========================
LiquidCrystal_I2C lcd(0x27, 16, 2);
DHT dht(DHTPIN, DHTTYPE);

// ==========================
// Network & Firebase Credentials
// ==========================
const char* WIFI_SSID = "abhi-wifi-2.4G";
const char* WIFI_PASSWORD = "YOUR_WIFI_PASSWORD"; // Update this

#define DATABASE_URL "YOUR_FIREBASE_DATABASE_URL" // Update this
#define API_KEY "YOUR_FIREBASE_API_KEY"           // Update this

// Firebase Authentication Credentials (Email/Password)
#define FIREBASE_EMAIL "chaurasiyap815@gmail.com"
#define FIREBASE_PASSWORD "YOUR_PASSWORD"         // Update this

bool wifiConnected = false;
bool previousWiFiState = false; // Tracks WiFi transitions for Firebase re-initialization
unsigned long lastWiFiReconnectAttempt = 0;
const unsigned long WIFI_RECONNECT_INTERVAL = 10000; // 10 seconds for faster recovery
uint8_t wifiFailCount = 0;

// --- NEW ADDITION: Firebase Background Recovery Timers ---
// These variables track the non-blocking interval for Firebase reconnection attempts.
unsigned long lastFirebaseReconnectAttempt = 0;
const unsigned long FIREBASE_RECONNECT_INTERVAL = 60000; // Attempt recovery every 60 seconds
// ---------------------------------------------------------

FirebaseData fbdo;
FirebaseAuth auth;
FirebaseConfig config;

uint16_t firebaseReadFailCount = 0;
uint16_t firebaseWriteFailCount = 0;

// Reused Firebase objects to prevent heap memory fragmentation.
FirebaseJson fbStatusJson;
FirebaseJson fbSystemJson;
FirebaseJson fbHistoryJson;
FirebaseJson fbEventJson;
FirebaseJson fbLogJson;
FirebaseJsonData fbJsonData;

// ==========================
// Battery Configuration
// ==========================
// Accurate calibration factor based on hardware multimeter testing
float calFactor = 13.40;

const float LOW_BATTERY_CUTOFF = 24.0;
const float BATTERY_RECOVER    = 25.0;
const float HARD_BATTERY_CUTOFF = 23.5;

const bool ALLOW_MANUAL_CONTACTOR_OVERRIDE = true;
const float MANUAL_OVERRIDE_MIN_VOLTAGE = 23.5;

bool sensorFault = false;
float batteryVoltage = 0.0;         // Instantaneous voltage for hardware protection.
float displayVoltage = 0.0;         // Smoothed voltage for LCD and Firebase display.
float prevBatteryVoltage = 0.0;
uint8_t adcFaultCount = 0;

// ==========================
// Temperature / Fan Control
// ==========================
const float FAN_ON_TEMP  = 37.0;
const float FAN_OFF_TEMP = 35.5;
uint8_t dhtFailCount = 0;

float temperature = NAN;
float humidity = NAN;

// ==========================
// Timing Intervals
// ==========================
const unsigned long BATTERY_READ_INTERVAL      = 1000;
const unsigned long DHT_READ_INTERVAL          = 2500;
const unsigned long LCD_PAGE_TIME              = 3000;
const unsigned long SERIAL_INTERVAL            = 1000;

// Cloud operations are for telemetry and remote-control only. 
// Local relay safety logic does not depend on cloud connectivity.
const unsigned long FIREBASE_LIVE_INTERVAL     = 10000;
const unsigned long FIREBASE_SYSTEM_INTERVAL   = 60000;
const unsigned long FIREBASE_HISTORY_INTERVAL  = 60000;
const unsigned long CONTROL_READ_INTERVAL      = 5000;

// Remote manual control is permitted only while cloud updates remain fresh.
// If WiFi/Firebase disconnects, the system auto-reverts to local AUTO mode.
const unsigned long CONTROL_TIMEOUT            = 30000;

// Industrial-grade power stability timing logic:
// Grid OFF must be detected quickly to protect loads.
// Grid ON must remain completely stable before reconnecting the load.
const unsigned long GRID_OFF_CONFIRM_TIME      = 2000;
const unsigned long GRID_ON_CONFIRM_TIME       = 30000;

// Contactor anti-short-cycle protection:
// Protective OFF is immediate. ON action is delayed after any OFF event.
const unsigned long MIN_OFF_TO_ON_INTERVAL     = 30000;

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
// Persistence Across Deep Sleep / Resets
// ==========================
// RTC memory retains data even if the ESP32 restarts during a power cut.
RTC_DATA_ATTR unsigned long powerCutTime = 0;
RTC_DATA_ATTR char lastPowerCutStr[16] = "--:--"; 
RTC_DATA_ATTR uint32_t contactorCycles = 0;
RTC_DATA_ATTR uint32_t brownoutCount = 0;

// ==========================
// Runtime State Variables
// ==========================
bool rawGridOn = false;
bool lastRawGridOn = false;
bool stableGridOn = false;
bool prevGridState = false;

// Offline event caching flags
bool pendingPowerOffLog = false;
bool pendingPowerOnLog = false;

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
  B01110, B10001, B10001, B11111,
  B11111, B11111, B11111, B00000
};

// ==========================
// Utility Functions
// ==========================
unsigned long getUptimeSeconds()
{
  return millis() / 1000;
}

bool isTimeValid()
{
  // 1700000000 serves as a baseline check to verify NTP has returned a valid epoch time.
  return time(nullptr) > 1700000000;
}

unsigned long getEpochOrUptime()
{
  return isTimeValid() ? (unsigned long)time(nullptr) : getUptimeSeconds();
}

void safeRestart(const __FlashStringHelper* msg)
{
  // Ensure outputs are de-energized before rebooting to prevent 
  // loads from remaining unintentionally active during the restart cycle.
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
  if (contactorOn && stableGridOn) return "GRID_ON";
  if (sensorFault) return "SENSOR_FAULT";
  if (batteryVoltage < HARD_BATTERY_CUTOFF) return "HARD_BATTERY_CUTOFF";
  if (controlContactorManual && contactorOn) return "MANUAL_ON";
  if (controlContactorManual && !contactorOn) return "MANUAL_OFF_OR_BLOCKED";
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
  // Arduino ESP32 core 3.x requires the ESP-IDF 5 watchdog API implementation.
  // Older core versions (2.x) utilize standard timeout/panic parameters.
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
// Sensor Telemetry Logic
// ==========================
float readBatteryVoltage()
{
  uint32_t sum = 0;
  const uint8_t samples = 32;

  // Average multiple ADC samples to mitigate hardware noise.
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
  unsigned long now = millis();
  if (now - lastBatteryRead < BATTERY_READ_INTERVAL) return;

  lastBatteryRead = now;

  float currentVolts = readBatteryVoltage();

  if (prevBatteryVoltage == 0.0)
  {
    prevBatteryVoltage = currentVolts;
    displayVoltage = currentVolts;
  }

  float delta = fabs(currentVolts - prevBatteryVoltage);
  if (delta > 5.0)
  {
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
    // Stricter safety threshold for a 24V system (limits: 10.0V to 35.0V)
    sensorFault = (currentVolts < 10.0 || currentVolts > 35.0);
    prevBatteryVoltage = currentVolts;
  }

  batteryVoltage = currentVolts;
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
// Local Hardware Actuation
// ==========================
void setContactor(bool targetOn)
{
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
  bool gridStableEnough = stableGridOn;

  if (gridStableEnough)
  {
    setContactor(true);
    return;
  }

  if (sensorFault || batteryVoltage < HARD_BATTERY_CUTOFF)
  {
    setContactor(false);
    return;
  }

  bool autoPowerAvailable = batteryOk;

  bool manualOverrideAllowed =
    ALLOW_MANUAL_CONTACTOR_OVERRIDE &&
    batteryVoltage >= MANUAL_OVERRIDE_MIN_VOLTAGE &&
    !sensorFault;

  bool targetContactor = controlContactorManual
    ? (controlContactorActive && manualOverrideAllowed)
    : autoPowerAvailable;

  if (!targetContactor)
  {
    setContactor(false);
    return;
  }

  if (!contactorOn && millis() - lastContactorOffAt < MIN_OFF_TO_ON_INTERVAL)
  {
    digitalWrite(CONTACTOR_PIN, RELAY_OFF);
    return;
  }

  setContactor(true);
}

void updateExhaustFan()
{
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
    fanOn = false;
  }

  digitalWrite(FAN_PIN, fanOn ? RELAY_ON : RELAY_OFF);
}

void runLocalControl()
{
  updateGridStatus();
  updateBatteryStatus();
  updateDhtStatus();
  updateContactor();
  updateExhaustFan();
}

// ==========================
// Network & Cloud Services
// ==========================
void maintainWiFi()
{
  bool currentWiFi = (WiFi.status() == WL_CONNECTED);

  // Force Firebase re-initialization if WiFi reconnects
  if (currentWiFi && !previousWiFiState)
  {
    Serial.println(F("[NETWORK] WiFi connection restored. Re-initializing Firebase..."));
    Firebase.begin(&config, &auth);
    Firebase.reconnectWiFi(true);
  }
  previousWiFiState = currentWiFi;

  if (wifiConnected && !currentWiFi)
  {
    controlFanManual = false;
    controlContactorManual = false;
    Serial.println(F("[NETWORK] WiFi connection lost. Forced AUTO fallback mode."));
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
    safeRestart(F("[NETWORK] WiFi stack recovery reboot initiated."));
  }

  Serial.println(F("[NETWORK] Attempting WiFi reconnection..."));
  WiFi.disconnect(false);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
}

// --- NEW ADDITION: Independent Background Firebase Recovery ---
// This function actively monitors the Firebase session. If the token expires 
// or the connection drops while WiFi is still connected, it forces a silent 
// background re-initialization without blocking the main local control loop.
void attemptFirebaseRecovery()
{
  if (wifiConnected && !Firebase.ready())
  {
    unsigned long now = millis();
    if (now - lastFirebaseReconnectAttempt >= FIREBASE_RECONNECT_INTERVAL)
    {
      lastFirebaseReconnectAttempt = now;
      Serial.println(F("[FIREBASE] Connection lost (Token expired or network drop). Attempting background recovery..."));
      
      Firebase.begin(&config, &auth);
      Firebase.reconnectWiFi(true);
    }
  }
}
// --------------------------------------------------------------

void enforceControlTimeout()
{
  if (lastControlSuccess == 0) return;

  bool cloudUnavailable = !wifiConnected || !Firebase.ready();
  if (cloudUnavailable && millis() - lastControlSuccess > CONTROL_TIMEOUT)
  {
    controlFanManual = false;
    controlContactorManual = false;
    Serial.println(F("[CONTROL] Cloud timeout elapsed. Reverted to AUTO mode."));
  }
}

void readFirebaseControls()
{
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
    Serial.println(F("[FIREBASE] Repeated read failures detected. Controls forced to AUTO."));
  }
}

void logGridEvent()
{
  // 1. Local State Tracking (Executes even if offline)
  if (stableGridOn != prevGridState)
  {
    prevGridState = stableGridOn;
    time_t eventTime = time(nullptr);
    struct tm *ptm = localtime(&eventTime);
    char timeString[16];
    strftime(timeString, sizeof(timeString), "%I:%M %p", ptm);

    if (!stableGridOn) {
      powerCutTime = (unsigned long)eventTime;
      // Store exact cut time in deep memory to survive unexpected reboots.
      strncpy(lastPowerCutStr, timeString, sizeof(lastPowerCutStr));
      pendingPowerOffLog = true;
    } else {
      pendingPowerOnLog = true;
    }
  }

  // 2. Cloud Synchronization (Executes only when online)
  if (!wifiConnected || !Firebase.ready() || !isTimeValid()) return;

  if (pendingPowerOffLog)
  {
    fbEventJson.clear();
    fbEventJson.set("powerCut", lastPowerCutStr);
    fbEventJson.set("restored", "--:--");
    fbEventJson.set("duration", "0 Min");
    Firebase.updateNode(fbdo, "/status/gridEvents", fbEventJson);

    fbLogJson.clear();
    fbLogJson.set("event", "POWER_OFF");
    fbLogJson.set("timestamp", (int)powerCutTime);
    fbLogJson.set("battery", batteryVoltage);
    fbLogJson.set("contactor", contactorOn);
    
    char path[64];
    snprintf(path, sizeof(path), "/logs/%lu", powerCutTime);
    Firebase.updateNode(fbdo, path, fbLogJson);
    
    pendingPowerOffLog = false; // Successfully logged, clear flag
  }

  if (pendingPowerOnLog)
  {
    unsigned long epochTime = (unsigned long)time(nullptr);
    struct tm *ptm = localtime((time_t*)&epochTime);
    char timeString[16];
    strftime(timeString, sizeof(timeString), "%I:%M %p", ptm);

    fbEventJson.clear();
    fbEventJson.set("powerCut", lastPowerCutStr);
    fbEventJson.set("restored", timeString);
    if (powerCutTime > 0 && epochTime >= powerCutTime)
    {
      char durBuffer[32];
      snprintf(durBuffer, sizeof(durBuffer), "%lu Min", (epochTime - powerCutTime) / 60);
      fbEventJson.set("duration", durBuffer);
    }
    else
    {
      fbEventJson.set("duration", "-- Min");
    }
    Firebase.updateNode(fbdo, "/status/gridEvents", fbEventJson);

    fbLogJson.clear();
    fbLogJson.set("event", "POWER_ON");
    fbLogJson.set("timestamp", (int)epochTime);
    fbLogJson.set("battery", batteryVoltage);
    fbLogJson.set("contactor", contactorOn);

    char path[64];
    snprintf(path, sizeof(path), "/logs/%lu", epochTime);
    Firebase.updateNode(fbdo, path, fbLogJson);

    pendingPowerOnLog = false; // Successfully logged, clear flag
  }
}

void sendLiveStatusToFirebase()
{
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
  }
  else
  {
    firebaseWriteFailCount++;
    printFirebaseError("Live Status Sync");
  }
}

void sendSystemToFirebase()
{
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
  fbSystemJson.set("firmware", "v4.2.0-industrial"); // Updated firmware version
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
  maintainWiFi();
  
  // --- NEW ADDITION: Call the background Firebase recovery check ---
  attemptFirebaseRecovery(); 
  // -----------------------------------------------------------------
  
  enforceControlTimeout();

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

  // Cloud freeze detection:  .
 if (firebaseWriteFailCount > 200)
{
    if (wifiConnected)
    {
        Serial.println(F("[FIREBASE] Too many write failures. Reinitializing Firebase..."));

        Firebase.begin(&config, &auth);
        Firebase.reconnectWiFi(true);
    }

    firebaseWriteFailCount = 50;
}
}

// ==========================
// User Interface (LCD & Serial)
// ==========================
void updateLcd()
{
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
  unsigned long now = millis();
  if (now - lastSerialPrint < SERIAL_INTERVAL) return;

  lastSerialPrint = now;

  // --- NEW ADDITION: Added FB=%s to monitor Firebase ready state in Serial ---
  Serial.printf(
    "RAW_GRID=%s | STABLE_GRID=%s | BAT=%.2fV | BAT_OK=%s | TEMP=%.1fC | HUM=%.0f%% | FAN=%s | CNT=%s | WIFI=%s | FB=%s | HEAP=%u | MIN_HEAP=%u\n",
    rawGridOn ? "ON" : "OFF",
    stableGridOn ? "ON" : "OFF",
    batteryVoltage,
    batteryOk ? "YES" : "NO",
    temperature,
    humidity,
    fanOn ? "ON" : "OFF",
    contactorOn ? "ON" : "OFF",
    wifiConnected ? "ON" : "OFF",
    Firebase.ready() ? "ON" : "OFF",
    ESP.getFreeHeap(),
    ESP.getMinFreeHeap()
  );
  // ---------------------------------------------------------------------------
}

// ==========================
// Main Initialization (Setup)
// ==========================
void setup()
{
  Serial.begin(115200);
  delay(500);

  esp_reset_reason_t resetReason = esp_reset_reason();
  if (resetReason == ESP_RST_BROWNOUT) brownoutCount++;
  systemResetReason = (int)resetReason;

  Serial.printf("\n[INIT] ESPESP32 Booting. Reset Reason Code: %d\n", systemResetReason);

  setupWatchdog();

  pinMode(GRID_PIN, INPUT_PULLUP);
  pinMode(CONTACTOR_PIN, OUTPUT);
  pinMode(FAN_PIN, OUTPUT);

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
  
  // Stricter safety limits applied upon boot
  sensorFault = (batteryVoltage < 10.0 || batteryVoltage > 35.0);
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
    runLocalControl();
    delay(250);
    Serial.print(".");
  }

  wifiConnected = (WiFi.status() == WL_CONNECTED);
  previousWiFiState = wifiConnected;
  Serial.println(wifiConnected ? F("\n[NETWORK] WiFi successfully connected.") : F("\n[NETWORK] Operating in offline fallback mode."));

  configTime(19800, 0, "pool.ntp.org", "time.nist.gov");

  if (wifiConnected)
  {
    Serial.print(F("[NETWORK] Waiting for NTP Time Synchronization..."));
    for (int i = 0; i < 20; i++)
    {
      esp_task_wdt_reset();
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
    // Configure Email/Password Authentication
    auth.user.email = FIREBASE_EMAIL;
    auth.user.password = FIREBASE_PASSWORD;

    Firebase.begin(&config, &auth);
    Firebase.reconnectWiFi(true);
    
    Serial.println(F("[FIREBASE] Initiating connection via Email/Password Auth..."));
  }

  lastLcdUpdate = millis() - LCD_PAGE_TIME;
  lastFirebaseLiveUpdate = millis();
  lastFirebaseSystemUpdate = millis();
  lastFirebaseHistoryUpdate = millis();

  Serial.println(F("[INIT] System initialization complete. Services are live."));
}

// ==========================
// Main Execution Loop
// ==========================
void loop()
{
  unsigned long loopStart = micros();
  esp_task_wdt_reset();

  // Updated: Increased critical heap threshold to 30,000 bytes for higher stability
  if (ESP.getFreeHeap() < 30000)
  {
    safeRestart(F("[SYSTEM] Critical memory shortage detected. Triggering safe reboot."));
  }

  if (getUptimeSeconds() > THIRTY_DAYS_SECONDS)
  {
    safeRestart(F("[SYSTEM] Executing scheduled 30-day maintenance reboot."));
  }

  if (millis() - lastNtpSync > NTP_SYNC_INTERVAL)
  {
    configTime(19800, 0, "pool.ntp.org", "time.nist.gov");
    lastNtpSync = millis();
  }

  runLocalControl();
  runCloudTasks();
  runLocalControl();

  updateLcd();
  updateSerial();

  unsigned long loopTime = micros() - loopStart;
  if (loopTime > maxLoopTimeUs) maxLoopTimeUs = loopTime;

  yield();
}
