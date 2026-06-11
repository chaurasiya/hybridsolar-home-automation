#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <DHT.h>

// ==========================
// Pin Configuration
// ==========================
#define GRID_PIN       27    // Grid detector input
#define CONTACTOR_PIN  23    // AC contactor relay
#define FAN_PIN        26    // Exhaust fan relay
#define VOLTAGE_PIN    35    // Battery voltage sensor

#define DHTPIN         2
#define DHTTYPE        DHT22

// LOW = Grid ON, HIGH = Grid OFF
const int GRID_ON_LEVEL = LOW;

// Set true if your relay modules are active LOW
const bool RELAY_ACTIVE_LOW = false;

const int RELAY_ON  = RELAY_ACTIVE_LOW ? LOW : HIGH;
const int RELAY_OFF = RELAY_ACTIVE_LOW ? HIGH : LOW;

// ==========================
// LCD and DHT
// ==========================
LiquidCrystal_I2C lcd(0x27, 16, 2);
DHT dht(DHTPIN, DHTTYPE);

// ==========================
// Battery Configuration
// ==========================
const float CAL_FACTOR = 13.55;

const float LOW_BATTERY_CUTOFF = 24.2;
const float BATTERY_RECOVER    = 24.6;

// ==========================
// Exhaust Fan Configuration
// Fan turns ON above 37C.
// Fan turns OFF below 35.5C to avoid relay chattering.
// ==========================
const float FAN_ON_TEMP  = 37.0;
const float FAN_OFF_TEMP = 35.5;

// ==========================
// Grid Filtering
// ==========================
const unsigned long GRID_ON_CONFIRM_TIME  = 5000;
const unsigned long GRID_OFF_CONFIRM_TIME = 300;

bool rawGridOn = false;
bool lastRawGridOn = false;
bool stableGridOn = false;

unsigned long rawGridChangedAt = 0;

// ==========================
// Runtime State
// ==========================
float batteryVoltage = 0.0;
float temperature = NAN;
float humidity = NAN;

bool batteryOk = false;
bool contactorOn = false;
bool fanOn = false;

unsigned long lastBatteryRead = 0;
unsigned long lastDhtRead = 0;
unsigned long lastLcdUpdate = 0;
unsigned long lastSerialPrint = 0;

const unsigned long BATTERY_READ_INTERVAL = 1000;
const unsigned long DHT_READ_INTERVAL     = 2500;
const unsigned long LCD_PAGE_TIME         = 3000;
const unsigned long SERIAL_INTERVAL       = 1000;

byte lcdPage = 0;

// ==========================
// Battery Icon
// ==========================
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
// Battery Voltage Reading
// ==========================
float readBatteryVoltage()
{
  uint32_t sum = 0;
  const uint8_t samples = 64;

  for (uint8_t i = 0; i < samples; i++)
  {
    sum += analogRead(VOLTAGE_PIN);
    delay(2);
  }

  float adc = sum / float(samples);
  float adcVoltage = (adc * 3.3) / 4095.0;

  return adcVoltage * CAL_FACTOR;
}

// ==========================
// Battery Status
// ==========================
void updateBatteryStatus()
{
  unsigned long now = millis();

  if (now - lastBatteryRead < BATTERY_READ_INTERVAL)
  {
    return;
  }

  lastBatteryRead = now;
  batteryVoltage = readBatteryVoltage();

  if (batteryVoltage < LOW_BATTERY_CUTOFF)
  {
    batteryOk = false;
  }
  else if (batteryVoltage >= BATTERY_RECOVER)
  {
    batteryOk = true;
  }
}

// ==========================
// DHT22 Reading
// ==========================
void updateDhtStatus()
{
  unsigned long now = millis();

  if (now - lastDhtRead < DHT_READ_INTERVAL)
  {
    return;
  }

  lastDhtRead = now;

  float newTemp = dht.readTemperature();
  float newHum  = dht.readHumidity();

  if (!isnan(newTemp))
  {
    temperature = newTemp;
  }

  if (!isnan(newHum))
  {
    humidity = newHum;
  }
}

// ==========================
// Grid Status
// ==========================
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
    stableGridOn = true;
  }

  if (!rawGridOn && stableTime >= GRID_OFF_CONFIRM_TIME)
  {
    stableGridOn = false;
  }
}

// ==========================
// AC Contactor Logic
// Contactor ON when grid is stable ON
// OR battery voltage is healthy.
// ==========================
void updateContactor()
{
  contactorOn = stableGridOn || batteryOk;

  digitalWrite(CONTACTOR_PIN, contactorOn ? RELAY_ON : RELAY_OFF);
}

// ==========================
// Exhaust Fan Logic
// ==========================
void updateExhaustFan()
{
  if (isnan(temperature))
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

// ==========================
// LCD Display
// ==========================
void updateLcd()
{
  unsigned long now = millis();

  if (now - lastLcdUpdate < LCD_PAGE_TIME)
  {
    return;
  }

  lastLcdUpdate = now;
  lcdPage = (lcdPage + 1) % 4;

  lcd.clear();

  if (lcdPage == 0)
  {
    lcd.setCursor(0, 0);
    lcd.print(F("GRID:"));
    lcd.print(stableGridOn ? F("ON ") : F("OFF"));

    lcd.setCursor(10, 0);
    lcd.print(F("R:"));
    lcd.print(rawGridOn ? F("ON") : F("OF"));

    lcd.setCursor(0, 1);
    lcd.print(F("CNT:"));
    lcd.print(contactorOn ? F("ON ") : F("OFF"));

    lcd.setCursor(9, 1);
    lcd.print(batteryOk ? F("BAT OK") : F("BATLOW"));
  }
  else if (lcdPage == 1)
  {
    lcd.setCursor(0, 0);
    lcd.print(F("BATTERY VOLT"));

    lcd.setCursor(0, 1);
    lcd.write(byte(0));
    lcd.print(F(" "));
    lcd.print(batteryVoltage, 1);
    lcd.print(F("V"));
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

      lcd.setCursor(10, 1);
      lcd.print(fanOn ? F("F:ON") : F("F:OF"));
    }
  }
  else
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
}

// ==========================
// Serial Debug
// ==========================
void updateSerial()
{
  unsigned long now = millis();

  if (now - lastSerialPrint < SERIAL_INTERVAL)
  {
    return;
  }

  lastSerialPrint = now;

  Serial.print(F("RAW_GRID="));
  Serial.print(rawGridOn ? F("ON") : F("OFF"));

  Serial.print(F(" | STABLE_GRID="));
  Serial.print(stableGridOn ? F("ON") : F("OFF"));

  Serial.print(F(" | BAT="));
  Serial.print(batteryVoltage, 2);
  Serial.print(F("V"));

  Serial.print(F(" | BAT_STATUS="));
  Serial.print(batteryOk ? F("OK") : F("LOW"));

  Serial.print(F(" | TEMP="));
  Serial.print(temperature);

  Serial.print(F("C | HUM="));
  Serial.print(humidity);

  Serial.print(F("% | CONTACTOR="));
  Serial.print(contactorOn ? F("ON") : F("OFF"));

  Serial.print(F(" | FAN="));
  Serial.println(fanOn ? F("ON") : F("OFF"));
}

// ==========================
// Setup
// ==========================
void setup()
{
  Serial.begin(115200);

  pinMode(GRID_PIN, INPUT_PULLUP);
  pinMode(CONTACTOR_PIN, OUTPUT);
  pinMode(FAN_PIN, OUTPUT);

  digitalWrite(CONTACTOR_PIN, RELAY_OFF);
  digitalWrite(FAN_PIN, RELAY_OFF);

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
  lcd.print(F("Load Control"));

  delay(2000);

  rawGridOn = digitalRead(GRID_PIN) == GRID_ON_LEVEL;
  lastRawGridOn = rawGridOn;
  rawGridChangedAt = millis();

  batteryVoltage = readBatteryVoltage();
  batteryOk = batteryVoltage >= LOW_BATTERY_CUTOFF;

  temperature = dht.readTemperature();
  humidity = dht.readHumidity();

  updateExhaustFan();
}

// ==========================
// Main Loop
// ==========================
void loop()
{
  updateGridStatus();
  updateBatteryStatus();
  updateDhtStatus();

  updateContactor();
  updateExhaustFan();

  updateLcd();
  updateSerial();
}