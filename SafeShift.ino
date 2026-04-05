/*
 * ============================================================
 *  SafeShift — Universal Industrial Vehicle Safety Kit
 *  Target: ESP32-S3-MINI-1
 *  Board:  ESP32S3 Dev Module (Arduino IDE)
 * ============================================================
 *
 *  SENSORS (from schematic):
 *    - 5× JSN-SR04T   Blind spot ultrasonic (PR1–PR5 connectors)
 *    - 2× RCWL-0516   Microwave motion (MWSENSOR_R1, MWSENSOR_F1)
 *    - 1× Microswitch  Door interlock (DOOR1)
 *    - 1× A3144        Hall effect seatbelt (T4 connector)
 *    - 1× DS18B20      Engine temperature (TS/VK3)
 *    - 1× ADXL345      Vibration (I2C)
 *    - 1× WCS1500      Current sensor (VM4 / CURRENTSENSOR1)
 *    - 1× Pressure transducer  Hydraulic (ADC pin)
 *
 *  OUTPUTS:
 *    - 1× Relay        Interlock (cuts ignition/lift)
 *    - 1× Buzzer       Audible alerts
 *    - 4× LEDs         Zone warning panel (G/Y/Y/R)
 *    - 1× Status LED   Board indicator (D3/R4)
 *
 *  CONNECTIVITY:
 *    - WiFi + MQTT     Dashboard telemetry
 *
 *  LIBRARIES REQUIRED (install via Arduino Library Manager):
 *    - PubSubClient       (MQTT)
 *    - OneWire            (DS18B20)
 *    - DallasTemperature  (DS18B20)
 *    - Adafruit ADXL345   (vibration)
 *    - Adafruit Unified Sensor
 *    - ArduinoJson        (payload serialisation)
 * ============================================================
 */

// ─── Core libraries ──────────────────────────────────────────
#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <Wire.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>

// ─── WiFi / MQTT credentials ─────────────────────────────────
#define WIFI_SSID       "YOUR_WIFI_SSID"
#define WIFI_PASSWORD   "YOUR_WIFI_PASSWORD"
#define MQTT_BROKER     "192.168.1.100"   // your broker IP
#define MQTT_PORT       1883
#define MQTT_CLIENT_ID  "RetroGuard_01"
#define MQTT_TOPIC_STATE    "retroguard/state"
#define MQTT_TOPIC_ALERT    "retroguard/alert"
#define MQTT_TOPIC_CMD      "retroguard/cmd"   // incoming commands

// ─── PIN DEFINITIONS (ESP32-S3-MINI-1) ───────────────────────
// Mapped from schematic connectors + standard GPIO layout

// JSN-SR04T ultrasonic sensors (PR1–PR5)
// Each sensor: TRIG + ECHO pair
#define US_TRIG_1   1    // PR1 — rear left
#define US_ECHO_1   2
#define US_TRIG_2   3    // PR2 — rear centre
#define US_ECHO_2   4
#define US_TRIG_3   5    // PR3 — rear right
#define US_ECHO_3   6
#define US_TRIG_4   7    // PR4 — side left
#define US_ECHO_4   8
#define US_TRIG_5   9    // PR5 — side right
#define US_ECHO_5   10

// RCWL-0516 microwave radar (MWSENSOR_R1, MWSENSOR_F1)
#define MW_REAR     11   // MWSENSOR_R1 OUT pin
#define MW_FRONT    12   // MWSENSOR_F1 OUT pin

// Door microswitch (DOOR1)
#define DOOR_PIN    13   // active LOW when door closed

// A3144 Hall effect seatbelt (T4 connector)
#define BELT_PIN    14   // active LOW when buckled (open-collector)

// DS18B20 temperature (TS/VK3 — 1-Wire)
#define ONE_WIRE_BUS 15

// ADXL345 vibration (I2C — SDA/SCL)
#define SDA_PIN     17   // ESP32-S3 default I2C
#define SCL_PIN     18

// WCS1500 current sensor (VM4 / CURRENTSENSOR1 — analog)
#define CURRENT_PIN 16   // ADC1 channel

// Hydraulic pressure transducer (analog — 0.5–4.5V → divider → 3.3V)
#define PRESSURE_PIN 6   // ADC — GPIO6 / ADC1_CH5
// Note: after 10k/20k voltage divider: 4.5V → ~3.0V safe for ESP32-S3

// Outputs
#define RELAY_PIN   33   // HIGH = relay energised = circuit closed = OPERABLE
#define BUZZER_PIN  34   // active HIGH
#define LED_GREEN   35   // zone clear
#define LED_AMBER1  36   // advisory (3–5m)
#define LED_AMBER2  37   // warning  (1.5–3m)
#define LED_RED     38   // critical (<1.5m)
#define LED_STATUS  21   // onboard status LED (D3)

// ─── THRESHOLDS ───────────────────────────────────────────────
#define MAX_PRESSURE_BAR    280.0f  // hydraulic overload limit
#define MAX_TEMP_C          105.0f  // engine overtemp limit
#define MAX_VIBRATION_G     1.8f    // bearing fault threshold
#define WARN_VIBRATION_G    1.2f    // early vibration warning
#define SERVICE_INTERVAL_H  250.0f  // hours between services
#define SERVICE_WARN_H      20.0f   // warn when this many hours remain

// Blind spot zones (cm)
#define ZONE_CRITICAL_CM    150
#define ZONE_WARNING_CM     300
#define ZONE_ADVISORY_CM    500

// Hydraulic ADC conversion constants
// Sensor: 0–400 bar → 0.5–4.5V. After 10k/20k divider: 0.33–3.0V
// ESP32-S3 ADC: 12-bit = 0–4095 for 0–3.3V
#define PRESSURE_V_MIN      0.33f   // V at 0 bar (after divider)
#define PRESSURE_V_MAX      3.0f    // V at 400 bar (after divider)
#define PRESSURE_MAX_BAR    400.0f

// WCS1500: 0–200A → 1.0–4.5V analog output (datasheet: 11mV/A + offset)
// Sensitivity: ~0.011 V/A,  zero-current output ~2.5V (5V supply, so 1.25V at 3.3V pin)
#define WCS1500_ZERO_V      1.25f
#define WCS1500_SENS        0.0066f  // V/A after divider (11mV/A × 0.6)

// ─── TIMING ───────────────────────────────────────────────────
#define LOOP_INTERVAL_MS        100   // main sensor poll interval
#define MQTT_PUBLISH_INTERVAL   500   // state publish interval
#define TEMP_READ_INTERVAL      2000  // DS18B20 read interval (slow sensor)
#define MQTT_RECONNECT_INTERVAL 5000  // reconnect attempt interval
#define BUZZER_BEEP_MS          100   // short beep duration
#define US_SPIKE_IGNORE_MS      800   // ignore ultrasonic right after trigger

// Alert debounce — same alert type won't repeat within this window
#define ALERT_DEBOUNCE_MS       3000

// ─── OBJECT INSTANCES ─────────────────────────────────────────
WiFiClient        wifiClient;
PubSubClient      mqttClient(wifiClient);
OneWire           oneWire(ONE_WIRE_BUS);
DallasTemperature tempSensor(&oneWire);
Adafruit_ADXL345_Unified adxl = Adafruit_ADXL345_Unified(12345);

// ─── STATE VARIABLES ──────────────────────────────────────────

// Sensor readings (updated each loop)
float   s_pressureBar   = 0.0f;
float   s_tempC         = 25.0f;
float   s_vibrationG    = 0.0f;
float   s_currentA      = 0.0f;
bool    s_doorClosed    = true;
bool    s_beltBuckled   = true;
bool    s_mwRear        = false;
bool    s_mwFront       = false;
int     s_usDist[5]     = {500, 500, 500, 500, 500}; // cm per sensor

// Derived states
bool    d_relayOn       = true;   // true = operable
bool    d_overload      = false;
bool    d_overtemp      = false;
bool    d_highVibration = false;
bool    d_motionRear    = false;
bool    d_motionFront   = false;
String  d_worstZone     = "CLEAR"; // CLEAR / ADVISORY / WARNING / CRITICAL

// Runtime
unsigned long startMs           = 0;
float         runtimeHours      = 232.4f; // pre-seeded for demo realism
float         hoursToService    = 0.0f;

// Timing trackers
unsigned long lastLoopMs        = 0;
unsigned long lastPublishMs     = 0;
unsigned long lastTempReadMs    = 0;
unsigned long lastMqttAttemptMs = 0;
unsigned long buzzerOffMs       = 0;
bool          buzzerActive      = false;

// Alert debounce map: maps alert key → last fired timestamp
struct AlertRecord {
  String key;
  unsigned long lastFiredMs;
};
#define MAX_ALERTS 20
AlertRecord alertHistory[MAX_ALERTS];
int alertCount = 0;

// Alert log (last 50, for MQTT publish)
#define MAX_LOG 50
struct AlertEntry {
  String level;   // CRITICAL / WARNING / INFO
  String message;
  String detail;
  String timestamp;
};
AlertEntry alertLog[MAX_LOG];
int logHead = 0;

// ─── FORWARD DECLARATIONS ─────────────────────────────────────
void connectWiFi();
void connectMQTT();
void mqttCallback(char* topic, byte* payload, unsigned int length);
void readUltrasonics();
float readUltrasonicCm(int trigPin, int echoPin);
void readPressure();
void readTemperature();
void readVibration();
void readCurrent();
void readDigitalSensors();
void runFirmwareLogic();
String getUltrasoundZone(int distCm);
String getWorstZone();
void updateOutputs();
void publishState();
void publishAlert(String level, String message, String detail);
bool isDebounced(String key);
void addToLog(String level, String message, String detail);
String buildTimestamp();
String floatToStr(float val, int decimals);

// ─── SETUP ────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\n=============================");
  Serial.println("  RetroGuard v1.0 Booting...");
  Serial.println("=============================");

  // Output pins
  pinMode(RELAY_PIN,   OUTPUT); digitalWrite(RELAY_PIN,   HIGH); // default: operable
  pinMode(BUZZER_PIN,  OUTPUT); digitalWrite(BUZZER_PIN,  LOW);
  pinMode(LED_GREEN,   OUTPUT); digitalWrite(LED_GREEN,   LOW);
  pinMode(LED_AMBER1,  OUTPUT); digitalWrite(LED_AMBER1,  LOW);
  pinMode(LED_AMBER2,  OUTPUT); digitalWrite(LED_AMBER2,  LOW);
  pinMode(LED_RED,     OUTPUT); digitalWrite(LED_RED,     LOW);
  pinMode(LED_STATUS,  OUTPUT); digitalWrite(LED_STATUS,  HIGH); // on during boot

  // Ultrasonic TRIG pins
  pinMode(US_TRIG_1, OUTPUT); digitalWrite(US_TRIG_1, LOW);
  pinMode(US_TRIG_2, OUTPUT); digitalWrite(US_TRIG_2, LOW);
  pinMode(US_TRIG_3, OUTPUT); digitalWrite(US_TRIG_3, LOW);
  pinMode(US_TRIG_4, OUTPUT); digitalWrite(US_TRIG_4, LOW);
  pinMode(US_TRIG_5, OUTPUT); digitalWrite(US_TRIG_5, LOW);

  // Ultrasonic ECHO pins
  pinMode(US_ECHO_1, INPUT);
  pinMode(US_ECHO_2, INPUT);
  pinMode(US_ECHO_3, INPUT);
  pinMode(US_ECHO_4, INPUT);
  pinMode(US_ECHO_5, INPUT);

  // Microwave radar
  pinMode(MW_REAR,  INPUT);
  pinMode(MW_FRONT, INPUT);

  // Digital sensors
  pinMode(DOOR_PIN, INPUT_PULLUP);  // LOW = closed (reed/microswitch)
  pinMode(BELT_PIN, INPUT_PULLUP);  // LOW = buckled (A3144 open-collector)

  // I2C for ADXL345
  Wire.begin(SDA_PIN, SCL_PIN);

  // DS18B20
  tempSensor.begin();
  Serial.printf("DS18B20 devices found: %d\n", tempSensor.getDeviceCount());

  // ADXL345
  if (!adxl.begin()) {
    Serial.println("[WARN] ADXL345 not found — check wiring");
  } else {
    adxl.setRange(ADXL345_RANGE_4_G);
    Serial.println("[OK] ADXL345 ready");
  }

  // ADC attenuation for pressure + current pins
  // ESP32-S3: use analogSetPinAttenuation for 0–3.3V range
  analogSetPinAttenuation(PRESSURE_PIN, ADC_11db);
  analogSetPinAttenuation(CURRENT_PIN,  ADC_11db);

  startMs = millis();

  // WiFi + MQTT
  connectWiFi();
  mqttClient.setServer(MQTT_BROKER, MQTT_PORT);
  mqttClient.setCallback(mqttCallback);
  mqttClient.setBufferSize(2048);
  connectMQTT();

  digitalWrite(LED_STATUS, LOW); // boot complete

  // Startup grace period — give sensors time to settle
  delay(1000);

  addToLog("INFO", "System online", "RetroGuard v1.0 ready");
  Serial.println("[OK] Boot complete. Entering main loop.");
}

// ─── MAIN LOOP ────────────────────────────────────────────────
void loop() {
  unsigned long now = millis();

  // MQTT keep-alive
  if (!mqttClient.connected()) {
    if (now - lastMqttAttemptMs > MQTT_RECONNECT_INTERVAL) {
      lastMqttAttemptMs = now;
      connectMQTT();
    }
  }
  mqttClient.loop();

  // Buzzer auto-off
  if (buzzerActive && now > buzzerOffMs) {
    digitalWrite(BUZZER_PIN, LOW);
    buzzerActive = false;
  }

  // Main sensor poll at LOOP_INTERVAL_MS
  if (now - lastLoopMs < LOOP_INTERVAL_MS) return;
  lastLoopMs = now;

  // Update runtime
  runtimeHours = 232.4f + (float)(now - startMs) / 3600000.0f;
  hoursToService = SERVICE_INTERVAL_H - runtimeHours;

  // ── Read all sensors ──
  readDigitalSensors();  // door, belt, microwave
  readUltrasonics();     // 5× JSN-SR04T
  readPressure();        // hydraulic transducer
  readCurrent();         // WCS1500

  // Temperature is slow (DS18B20 needs ~750ms conversion)
  if (now - lastTempReadMs > TEMP_READ_INTERVAL) {
    readTemperature();
    lastTempReadMs = now;
  }

  readVibration();       // ADXL345

  // ── Run safety/maintenance logic ──
  runFirmwareLogic();

  // ── Drive outputs ──
  updateOutputs();

  // ── MQTT publish ──
  if (now - lastPublishMs > MQTT_PUBLISH_INTERVAL) {
    publishState();
    lastPublishMs = now;
  }
}

// ─── SENSOR READERS ───────────────────────────────────────────

void readDigitalSensors() {
  // Door: microswitch — LOW when door closed (INPUT_PULLUP)
  s_doorClosed  = (digitalRead(DOOR_PIN) == LOW);

  // Belt: A3144 hall effect — LOW when magnet present = buckled (open-collector + pullup)
  s_beltBuckled = (digitalRead(BELT_PIN) == LOW);

  // RCWL-0516: HIGH when motion detected
  s_mwRear  = (digitalRead(MW_REAR)  == HIGH);
  s_mwFront = (digitalRead(MW_FRONT) == HIGH);
}

void readUltrasonics() {
  // Poll each sensor with 30ms stagger to avoid cross-triggering
  int trigPins[] = {US_TRIG_1, US_TRIG_2, US_TRIG_3, US_TRIG_4, US_TRIG_5};
  int echoPins[] = {US_ECHO_1, US_ECHO_2, US_ECHO_3, US_ECHO_4, US_ECHO_5};

  for (int i = 0; i < 5; i++) {
    float d = readUltrasonicCm(trigPins[i], echoPins[i]);
    if (d > 0 && d <= 500) {
      s_usDist[i] = (int)d;
    } else {
      s_usDist[i] = 500; // no echo = clear
    }
    delay(30); // stagger — prevents echo cross-talk between sensors
  }
}

float readUltrasonicCm(int trigPin, int echoPin) {
  // JSN-SR04T requires 20µs trigger (vs HC-SR04's 10µs)
  digitalWrite(trigPin, LOW);
  delayMicroseconds(4);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(20);  // JSN-SR04T spec
  digitalWrite(trigPin, LOW);

  // Timeout 25ms = ~4.25m max range
  long duration = pulseIn(echoPin, HIGH, 25000);
  if (duration == 0) return 0; // timeout = nothing detected

  return (float)duration / 58.0f; // convert µs to cm
}

void readPressure() {
  // Read raw ADC (12-bit, 0–4095)
  int raw = analogRead(PRESSURE_PIN);
  float voltage = (raw / 4095.0f) * 3.3f;

  // Convert back through divider to get transducer voltage
  // Divider: 10k + 20k → Vout = Vin × (20/30) → Vin = Vout × (30/20)
  float vTransducer = voltage * 1.5f;

  // Map 0.5–4.5V → 0–400 bar
  float bar = (vTransducer - 0.5f) / 4.0f * PRESSURE_MAX_BAR;
  s_pressureBar = max(0.0f, bar);

  // ESP32-S3 ADC nonlinearity correction (simplified Espressif lookup)
  // For production: use esp_adc_cal_characterize()
  // For prototype: clamp to physically plausible range
  s_pressureBar = constrain(s_pressureBar, 0.0f, PRESSURE_MAX_BAR);
}

void readTemperature() {
  // Request conversion (non-blocking would be better for production)
  tempSensor.requestTemperatures();
  float t = tempSensor.getTempCByIndex(0);
  if (t != DEVICE_DISCONNECTED_C && t > -50.0f && t < 200.0f) {
    s_tempC = t;
  }
}

void readVibration() {
  sensors_event_t event;
  adxl.getEvent(&event);

  // Magnitude of acceleration vector (g)
  float ax = event.acceleration.x / 9.81f;
  float ay = event.acceleration.y / 9.81f;
  float az = event.acceleration.z / 9.81f;

  // Subtract 1g gravity from Z (static component when mounted flat)
  az = az - 1.0f;

  // RMS magnitude
  s_vibrationG = sqrt(ax*ax + ay*ay + az*az);

  // Clamp to sane range
  s_vibrationG = constrain(s_vibrationG, 0.0f, 10.0f);
}

void readCurrent() {
  // WCS1500: analog 0–200A
  // Output: ~2.5V at 0A (on 5V supply). After divider to ESP32-S3: ~1.25V at 0A
  // Sensitivity: 11mV/A on 5V → ~6.6mV/A after divider
  int raw = analogRead(CURRENT_PIN);
  float voltage = (raw / 4095.0f) * 3.3f;
  float current = (voltage - WCS1500_ZERO_V) / WCS1500_SENS;
  s_currentA = max(0.0f, current);
  s_currentA = constrain(s_currentA, 0.0f, 200.0f);
}

// ─── FIRMWARE LOGIC ───────────────────────────────────────────

void runFirmwareLogic() {
  unsigned long now = millis();
  bool relayOk = true;

  // ── Safety checks ──

  // 1. Hydraulic overload (weight lock)
  if (s_pressureBar > MAX_PRESSURE_BAR) {
    d_overload = true;
    relayOk = false;
    if (!isDebounced("overload")) {
      publishAlert("CRITICAL", "Overload detected",
                   floatToStr(s_pressureBar, 0) + " bar (max " +
                   floatToStr(MAX_PRESSURE_BAR, 0) + " bar)");
      addToLog("CRITICAL", "Overload",
               floatToStr(s_pressureBar, 0) + " bar");
    }
  } else {
    d_overload = false;
  }

  // 2. Door interlock
  if (!s_doorClosed) {
    relayOk = false;
    if (!isDebounced("door_open")) {
      publishAlert("CRITICAL", "Door open", "Interlock active");
      addToLog("CRITICAL", "Door open", "Interlock active");
    }
  }

  // 3. Seatbelt interlock
  if (!s_beltBuckled) {
    relayOk = false;
    if (!isDebounced("belt_unbuckled")) {
      publishAlert("CRITICAL", "Seatbelt unbuckled", "Interlock active");
      addToLog("CRITICAL", "Seatbelt unbuckled", "Interlock active");
    }
  }

  // 4. Pedestrian detection (RCWL-0516)
  //    Motion confirmed from rear sensor = elevated danger
  d_motionRear  = s_mwRear;
  d_motionFront = s_mwFront;

  if (s_mwRear && !isDebounced("mw_rear")) {
    publishAlert("WARNING", "Motion detected — rear", "RCWL-0516 triggered");
    addToLog("WARNING", "Rear motion", "RCWL-0516");
  }
  if (s_mwFront && !isDebounced("mw_front")) {
    publishAlert("WARNING", "Motion detected — front", "RCWL-0516 triggered");
    addToLog("WARNING", "Front motion", "RCWL-0516");
  }

  // 5. Blind spot zones
  d_worstZone = getWorstZone();

  if (d_worstZone == "CRITICAL" && !isDebounced("blindspot_crit")) {
    publishAlert("CRITICAL", "Blind spot critical",
                 "Object <150cm detected");
    addToLog("CRITICAL", "Blind spot", "<150cm");
  } else if (d_worstZone == "WARNING" && !isDebounced("blindspot_warn")) {
    publishAlert("WARNING", "Blind spot warning",
                 "Object 150–300cm");
    addToLog("WARNING", "Blind spot", "150–300cm");
  }

  // ── Maintenance checks ──

  // 6. Engine overtemp
  d_overtemp = (s_tempC > MAX_TEMP_C);
  if (d_overtemp && !isDebounced("overtemp")) {
    publishAlert("WARNING", "Engine overtemp",
                 floatToStr(s_tempC, 1) + "°C (max " +
                 floatToStr(MAX_TEMP_C, 0) + "°C)");
    addToLog("WARNING", "Overtemp",
             floatToStr(s_tempC, 1) + "°C");
  }

  // 7. High vibration (bearing wear indicator)
  d_highVibration = (s_vibrationG > MAX_VIBRATION_G);
  bool warnVibration = (s_vibrationG > WARN_VIBRATION_G && !d_highVibration);

  if (d_highVibration && !isDebounced("vibration_crit")) {
    publishAlert("CRITICAL", "High vibration",
                 floatToStr(s_vibrationG, 2) + " g — check bearings");
    addToLog("CRITICAL", "High vibration",
             floatToStr(s_vibrationG, 2) + " g");
  } else if (warnVibration && !isDebounced("vibration_warn")) {
    publishAlert("WARNING", "Vibration elevated",
                 floatToStr(s_vibrationG, 2) + " g");
    addToLog("WARNING", "Vibration", floatToStr(s_vibrationG, 2) + " g");
  }

  // 8. Service due
  if (hoursToService < SERVICE_WARN_H && hoursToService > 0 &&
      !isDebounced("service_due")) {
    publishAlert("INFO", "Service due soon",
                 floatToStr(hoursToService, 1) + " hrs remaining");
    addToLog("INFO", "Service due",
             floatToStr(hoursToService, 1) + " hrs");
  }
  if (hoursToService <= 0 && !isDebounced("service_overdue")) {
    publishAlert("WARNING", "Service overdue",
                 "Past interval by " +
                 floatToStr(-hoursToService, 1) + " hrs");
    addToLog("WARNING", "Service overdue",
             floatToStr(-hoursToService, 1) + " hrs late");
  }

  // ── Commit relay state ──
  d_relayOn = relayOk;
}

String getUltrasoundZone(int distCm) {
  if (distCm <= 0 || distCm >= 500)  return "CLEAR";
  if (distCm < ZONE_CRITICAL_CM)     return "CRITICAL";
  if (distCm < ZONE_WARNING_CM)      return "WARNING";
  if (distCm < ZONE_ADVISORY_CM)     return "ADVISORY";
  return "CLEAR";
}

String getWorstZone() {
  int worstPriority = 0;
  String worst = "CLEAR";
  const char* zones[] = {"CLEAR", "ADVISORY", "WARNING", "CRITICAL"};

  for (int i = 0; i < 5; i++) {
    String z = getUltrasoundZone(s_usDist[i]);
    int pri = 0;
    if (z == "ADVISORY") pri = 1;
    else if (z == "WARNING")  pri = 2;
    else if (z == "CRITICAL") pri = 3;

    if (pri > worstPriority) {
      worstPriority = pri;
      worst = z;
    }
  }
  return worst;
}

// ─── OUTPUTS ──────────────────────────────────────────────────

void updateOutputs() {
  // ── Relay ──
  digitalWrite(RELAY_PIN, d_relayOn ? HIGH : LOW);

  // ── LED warning panel ──
  // Shows worst blind spot zone
  bool isClear    = (d_worstZone == "CLEAR");
  bool isAdvisory = (d_worstZone == "ADVISORY");
  bool isWarning  = (d_worstZone == "WARNING");
  bool isCritical = (d_worstZone == "CRITICAL");

  // Override all LEDs to RED if relay interlocked
  if (!d_relayOn) {
    digitalWrite(LED_GREEN,  LOW);
    digitalWrite(LED_AMBER1, LOW);
    digitalWrite(LED_AMBER2, LOW);
    digitalWrite(LED_RED,    HIGH);
  } else {
    digitalWrite(LED_GREEN,  isClear    ? HIGH : LOW);
    digitalWrite(LED_AMBER1, isAdvisory ? HIGH : LOW);
    digitalWrite(LED_AMBER2, isWarning  ? HIGH : LOW);
    digitalWrite(LED_RED,    isCritical ? HIGH : LOW);
  }

  // ── Status LED ── heartbeat every 1s
  digitalWrite(LED_STATUS, (millis() / 1000) % 2 == 0 ? HIGH : LOW);

  // ── Buzzer ──
  if (!buzzerActive) {
    if (!d_relayOn || isCritical || d_highVibration) {
      // Continuous buzzer for critical states
      digitalWrite(BUZZER_PIN, HIGH);
      buzzerActive = true;
      buzzerOffMs  = millis() + 500; // 500ms on
    } else if (isWarning || s_mwRear || s_mwFront) {
      // Short beep for warnings
      digitalWrite(BUZZER_PIN, HIGH);
      buzzerActive = true;
      buzzerOffMs  = millis() + BUZZER_BEEP_MS;
    }
  }
}

// ─── MQTT ─────────────────────────────────────────────────────

void publishState() {
  if (!mqttClient.connected()) return;

  StaticJsonDocument<2048> doc;

  doc["relay"]     = d_relayOn;
  doc["timestamp"] = buildTimestamp();

  // Safety
  JsonObject safety = doc.createNestedObject("safety");
  safety["pressure_bar"]          = round(s_pressureBar * 10) / 10.0;
  safety["pressure_pct"]          = round(s_pressureBar / MAX_PRESSURE_BAR * 1000) / 10.0;
  safety["pressure_status"]       = s_pressureBar > MAX_PRESSURE_BAR ? "CRITICAL" :
                                    s_pressureBar > MAX_PRESSURE_BAR * 0.85f ? "WARNING" : "OK";
  safety["door_closed"]           = s_doorClosed;
  safety["belt_buckled"]          = s_beltBuckled;
  safety["mw_rear"]               = s_mwRear;
  safety["mw_front"]              = s_mwFront;
  safety["worst_zone"]            = d_worstZone;

  JsonArray zones = safety.createNestedArray("blind_spots");
  const char* sensorNames[] = {"rear_left","rear_center","rear_right","side_left","side_right"};
  for (int i = 0; i < 5; i++) {
    JsonObject z = zones.createNestedObject();
    z["sensor"]      = sensorNames[i];
    z["distance_cm"] = s_usDist[i];
    z["zone"]        = getUltrasoundZone(s_usDist[i]);
  }

  // Maintenance
  JsonObject maint = doc.createNestedObject("maintenance");
  maint["temp_c"]           = round(s_tempC * 10) / 10.0;
  maint["temp_status"]      = s_tempC > MAX_TEMP_C ? "CRITICAL" :
                              s_tempC > 90.0f      ? "WARNING"  : "OK";
  maint["vibration_g"]      = round(s_vibrationG * 100) / 100.0;
  maint["vibration_status"] = s_vibrationG > MAX_VIBRATION_G ? "CRITICAL" :
                              s_vibrationG > WARN_VIBRATION_G ? "WARNING" : "OK";
  maint["current_a"]        = round(s_currentA * 10) / 10.0;
  maint["runtime_hrs"]      = round(runtimeHours * 10) / 10.0;
  maint["hours_to_service"] = round(hoursToService * 10) / 10.0;
  maint["service_status"]   = hoursToService <= 0 ? "OVERDUE" :
                              hoursToService < SERVICE_WARN_H ? "DUE_SOON" : "OK";

  // Alert log (last 10 for MQTT payload size)
  JsonArray alerts = doc.createNestedArray("alerts");
  int start = max(0, logHead - 10);
  for (int i = logHead - 1; i >= start; i--) {
    int idx = (i + MAX_LOG) % MAX_LOG;
    if (alertLog[idx].level.isEmpty()) continue;
    JsonObject a = alerts.createNestedObject();
    a["level"]     = alertLog[idx].level;
    a["message"]   = alertLog[idx].message;
    a["detail"]    = alertLog[idx].detail;
    a["timestamp"] = alertLog[idx].timestamp;
  }

  // Stats
  JsonObject stats = doc.createNestedObject("stats");
  stats["uptime_s"]      = (millis() - startMs) / 1000;
  stats["runtime_hrs"]   = round(runtimeHours * 10) / 10.0;

  // Interlock reasons
  JsonArray reasons = doc.createNestedArray("interlock_reasons");
  if (!d_relayOn) {
    if (d_overload)      reasons.add("Hydraulic overload");
    if (!s_doorClosed)   reasons.add("Door open");
    if (!s_beltBuckled)  reasons.add("Seatbelt unbuckled");
  }

  // Serialise and publish
  char buffer[2048];
  size_t n = serializeJson(doc, buffer);
  mqttClient.publish(MQTT_TOPIC_STATE, buffer, n);
}

void publishAlert(String level, String message, String detail) {
  if (!mqttClient.connected()) return;

  StaticJsonDocument<256> doc;
  doc["level"]     = level;
  doc["message"]   = message;
  doc["detail"]    = detail;
  doc["timestamp"] = buildTimestamp();

  char buffer[256];
  size_t n = serializeJson(doc, buffer);
  mqttClient.publish(MQTT_TOPIC_ALERT, buffer, n);

  Serial.printf("[ALERT] %s: %s — %s\n",
                level.c_str(), message.c_str(), detail.c_str());
}

void mqttCallback(char* topic, byte* payload, unsigned int length) {
  // Handle incoming commands from dashboard
  // e.g. {"cmd":"reset_service_counter"} or {"cmd":"set_threshold","key":"pressure","value":300}
  StaticJsonDocument<256> doc;
  DeserializationError err = deserializeJson(doc, payload, length);
  if (err) return;

  String cmd = doc["cmd"].as<String>();

  if (cmd == "reset_service_counter") {
    runtimeHours = 0.0f;
    startMs = millis();
    addToLog("INFO", "Service counter reset", "Manual reset");
    Serial.println("[CMD] Service counter reset");
  }
  else if (cmd == "set_threshold" && doc.containsKey("key")) {
    String key = doc["key"].as<String>();
    float  val = doc["value"].as<float>();
    // In production you'd update EEPROM/NVS here
    Serial.printf("[CMD] Threshold update: %s = %.1f\n", key.c_str(), val);
  }
}

// ─── CONNECTIVITY ─────────────────────────────────────────────

void connectWiFi() {
  Serial.printf("Connecting to WiFi: %s", WIFI_SSID);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 20) {
    delay(500);
    Serial.print(".");
    attempts++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.printf("\n[OK] WiFi connected. IP: %s\n",
                  WiFi.localIP().toString().c_str());
  } else {
    Serial.println("\n[WARN] WiFi failed — running offline");
  }
}

void connectMQTT() {
  if (WiFi.status() != WL_CONNECTED) return;

  Serial.printf("Connecting to MQTT broker %s...", MQTT_BROKER);
  if (mqttClient.connect(MQTT_CLIENT_ID)) {
    Serial.println(" connected");
    mqttClient.subscribe(MQTT_TOPIC_CMD);
    // Publish online announcement
    mqttClient.publish("retroguard/status", "{\"status\":\"online\"}");
  } else {
    Serial.printf(" failed (rc=%d)\n", mqttClient.state());
  }
}

// ─── HELPERS ──────────────────────────────────────────────────

bool isDebounced(String key) {
  unsigned long now = millis();

  // Search existing record
  for (int i = 0; i < alertCount; i++) {
    if (alertHistory[i].key == key) {
      if (now - alertHistory[i].lastFiredMs < ALERT_DEBOUNCE_MS) {
        return true;  // still within debounce window — suppress
      } else {
        alertHistory[i].lastFiredMs = now;  // refresh
        return false;
      }
    }
  }

  // New key — add record
  if (alertCount < MAX_ALERTS) {
    alertHistory[alertCount].key = key;
    alertHistory[alertCount].lastFiredMs = now;
    alertCount++;
  }
  return false;
}

void addToLog(String level, String message, String detail) {
  alertLog[logHead % MAX_LOG] = {level, message, detail, buildTimestamp()};
  logHead++;
  if (logHead > MAX_LOG) logHead = MAX_LOG; // cap at MAX_LOG
}

String buildTimestamp() {
  unsigned long s = (millis() - startMs) / 1000;
  char buf[9];
  snprintf(buf, sizeof(buf), "%02lu:%02lu:%02lu",
           (s / 3600) % 24, (s / 60) % 60, s % 60);
  return String(buf);
}

String floatToStr(float val, int decimals) {
  char buf[16];
  if (decimals == 0)      snprintf(buf, sizeof(buf), "%.0f", val);
  else if (decimals == 1) snprintf(buf, sizeof(buf), "%.1f", val);
  else if (decimals == 2) snprintf(buf, sizeof(buf), "%.2f", val);
  else                    snprintf(buf, sizeof(buf), "%.3f", val);
  return String(buf);
}

