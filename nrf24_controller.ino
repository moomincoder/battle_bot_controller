// controller_tx.ino
// ESP32-C3 handheld controller with:
//  - Two joysticks (tank drive)
//  - Throttle axis for ESC
//  - ARM switch, KILL switch, ESC ENABLE switch
//  - Smoother joystick filtering & exponential curves
//  - Receives telemetry via ACK payloads (battery, link info)

#include <Arduino.h>
#include <SPI.h>
#include <RF24.h>

// --------- PIN CONFIG (CONTROLLER) ---------
// Analog inputs
#define JOY_LEFT_Y_PIN    A0
#define JOY_RIGHT_Y_PIN   A1
#define ESC_THROTTLE_PIN  A2

// Switches (wired to GND, INPUT_PULLUP)
#define ARM_SWITCH_PIN        9
#define KILL_SWITCH_PIN       10
#define ESC_ENABLE_SWITCH_PIN 8

// nRF24L01 pins
#define RF_CE_PIN   2
#define RF_CSN_PIN  3

// Joystick / ADC config
#define ADC_MAX        4095
#define JOY_CENTER     2048
#define JOY_DEADZONE   200
#define MAX_SPEED      255
#define ESC_MAX        255

// Filtering & expo
const float JOY_FILTER_ALPHA = 0.25f;  // 0..1, higher = faster response
const float DRIVE_EXPO       = 0.4f;   // 0 = linear, 1 = full expo
const float THROTTLE_EXPO    = 0.4f;

// --------- RADIO ---------
RF24 radio(RF_CE_PIN, RF_CSN_PIN);
const byte address[6] = "CTR01";

struct ControlPacket {
  int16_t left;        // -255..255
  int16_t right;       // -255..255
  int16_t esc;         // 0..255
  uint8_t armed;       // 1/0
  uint8_t kill;        // 1/0
  uint8_t esc_enable;  // 1/0
};

// Telemetry from robot (ACK payload)
struct TelemetryPacket {
  float    batteryVolts;  // robot battery voltage
  uint32_t uptimeMs;      // robot uptime
  uint16_t packetsRx;     // packets received on robot
  uint8_t  rpdFlag;       // 1 if strong RF signal (nRF24 RPD)
};

// Filtered joystick state
float filtLeft = 0.0f;      // normalized -1..1
float filtRight = 0.0f;
float filtThrottle = 0.0f;  // 0..1

// --------- HELPERS ---------
float applyExpo(float x, float expo) {
  // x in [-1,1] or [0,1]; expo 0..1
  // Standard RC-style expo: mix linear & cubic
  return (1.0f - expo) * x + expo * x * x * x;
}

float readJoystickNorm(int pin, bool invert = false) {
  int raw = analogRead(pin);
  if (invert) raw = ADC_MAX - raw;

  int delta = raw - JOY_CENTER;
  if (abs(delta) < JOY_DEADZONE) return 0.0f;

  float norm;
  if (delta > 0) {
    norm = (float)(delta - JOY_DEADZONE) / (float)(ADC_MAX - JOY_CENTER - JOY_DEADZONE);
  } else {
    norm = (float)(delta + JOY_DEADZONE) / (float)(JOY_CENTER - JOY_DEADZONE);
  }
  norm = constrain(norm, -1.0f, 1.0f);
  return norm;
}

float readThrottleNorm(int pin, bool invert = false) {
  int raw = analogRead(pin);
  if (invert) raw = ADC_MAX - raw;
  float norm = (float)raw / (float)ADC_MAX;  // 0..1
  return constrain(norm, 0.0f, 1.0f);
}

void readAndFilterInputs(ControlPacket &pkt) {
  // Raw normalized inputs
  float leftRaw     = readJoystickNorm(JOY_LEFT_Y_PIN,  true);  // invert so up = forward
  float rightRaw    = readJoystickNorm(JOY_RIGHT_Y_PIN, true);
  float throttleRaw = readThrottleNorm(ESC_THROTTLE_PIN, false);

  // Low-pass filter
  filtLeft     += JOY_FILTER_ALPHA * (leftRaw - filtLeft);
  filtRight    += JOY_FILTER_ALPHA * (rightRaw - filtRight);
  filtThrottle += JOY_FILTER_ALPHA * (throttleRaw - filtThrottle);

  // Apply expo
  float leftExpo  = applyExpo(filtLeft, DRIVE_EXPO);
  float rightExpo = applyExpo(filtRight, DRIVE_EXPO);
  float thrExpo   = applyExpo(filtThrottle, THROTTLE_EXPO);  // 0..1-ish

  // Map to integer output
  pkt.left  = (int16_t)constrain(leftExpo  * MAX_SPEED, -MAX_SPEED, MAX_SPEED);
  pkt.right = (int16_t)constrain(rightExpo * MAX_SPEED, -MAX_SPEED, MAX_SPEED);
  pkt.esc   = (int16_t)constrain(thrExpo   * ESC_MAX,   0,          ESC_MAX);

  // Switches
  pkt.armed      = (digitalRead(ARM_SWITCH_PIN)        == LOW);
  pkt.kill       = (digitalRead(KILL_SWITCH_PIN)       == LOW);
  pkt.esc_enable = (digitalRead(ESC_ENABLE_SWITCH_PIN) == LOW);
}

// --------- SETUP ---------
void setup() {
  Serial.begin(115200);
  delay(500);

  pinMode(ARM_SWITCH_PIN, INPUT_PULLUP);
  pinMode(KILL_SWITCH_PIN, INPUT_PULLUP);
  pinMode(ESC_ENABLE_SWITCH_PIN, INPUT_PULLUP);

  if (!radio.begin()) {
    Serial.println("nRF24 init failed");
    return;
  }

  radio.setPALevel(RF24_PA_LOW);
  radio.setDataRate(RF24_1MBPS);
  radio.setRetries(3, 5);

  radio.enableAckPayload();
  radio.enableDynamicPayloads();

  radio.openWritingPipe(address);
  radio.stopListening();

  Serial.println("Controller radio ready");
}

// --------- LOOP ---------
void loop() {
  ControlPacket pkt;
  readAndFilterInputs(pkt);

  bool ok = radio.write(&pkt, sizeof(pkt));

  if (ok && radio.isAckPayloadAvailable()) {
    TelemetryPacket tele;
    radio.read(&tele, sizeof(tele));
    // Example debug output (optional)
    static uint32_t lastPrint = 0;
    uint32_t now = millis();
    if (now - lastPrint > 500) {
      Serial.print("Batt: ");
      Serial.print(tele.batteryVolts, 2);
      Serial.print(" V  Uptime: ");
      Serial.print(tele.uptimeMs / 1000);
      Serial.print("s  PacketsRx: ");
      Serial.print(tele.packetsRx);
      Serial.print("  RPD:");
      Serial.println(tele.rpdFlag ? "strong" : "weak");
      lastPrint = now;
    }
  }

  delay(20);  // ~50 Hz
}
