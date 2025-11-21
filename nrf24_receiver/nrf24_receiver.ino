// robot_rx.ino
// ESP32-C3 robot:
//  - Receives control over nRF24
//  - Tank drive via L298N
//  - Brushless weapon via ESC
//  - Telemetry back to controller via ACK payload
//  - LED link status, soft-stop, ramping, latched failsafe

#include <Arduino.h>
#include <SPI.h>
#include <RF24.h>
#include <ESP32Servo.h>

// --------- PIN CONFIG ---------
// L298N pins
#define IN1_PIN   7     // left motor dir A
#define IN2_PIN   8     // left motor dir B
#define ENA_PIN   9     // left motor PWM

#define IN3_PIN   18    // right motor dir A
#define IN4_PIN   19    // right motor dir B
#define ENB_PIN   20    // right motor PWM

// ESC pin
#define ESC_PIN      10
#define ESC_MIN_US   1000
#define ESC_MAX_US   2000

// Battery sense (voltage divider to ADC)
#define BATT_ADC_PIN A0
// Adjust these for your actual divider: Vbattery -> R1 -> ADC -> R2 -> GND
const float BATT_R1 = 100000.0f;  // ohms
const float BATT_R2 = 10000.0f;   // ohms

// PWM setup for motors
#define LEFT_PWM_CH   0
#define RIGHT_PWM_CH  1
#define PWM_FREQ      20000
#define PWM_RES       8   // 0..255

// Timing
#define FAILSAFE_MS     5000UL   // hard latched failsafe
#define LINK_DROP_MS     100UL   // "healthy link" threshold for LED + soft stop
#define LED_BLINK_MS     250UL

// Ramping
#define MOTOR_RAMP_STEP  5       // max speed change per loop
#define ESC_RAMP_STEP    2       // max ESC change per loop (0..255)

// LED for link status
#define LED_PIN          8       // onboard LED pin (change for your board)

// nRF24
#define RF_CE_PIN   2
#define RF_CSN_PIN  3

RF24 radio(RF_CE_PIN, RF_CSN_PIN);
const byte address[6] = "CTR01";

struct ControlPacket {
  int16_t left;
  int16_t right;
  int16_t esc;
  uint8_t armed;
  uint8_t kill;
  uint8_t esc_enable;  // weapon enable
};

struct TelemetryPacket {
  float    batteryVolts;
  uint32_t uptimeMs;
  uint16_t packetsRx;
  uint8_t  rpdFlag;
};

ControlPacket lastPkt = {0};
unsigned long lastPacketTime = 0;
bool failsafeLatched = false;
uint16_t packetsRx = 0;

Servo esc;

// Applied outputs (ramped)
int16_t currentLeft = 0;
int16_t currentRight = 0;
uint8_t currentEsc = 0;

// LED state
unsigned long lastLedToggle = 0;
bool ledState = false;

// --------- HELPERS ---------
int16_t rampToward(int16_t current, int16_t target, int16_t step) {
  if (current < target) {
    int16_t diff = target - current;
    return current + (diff > step ? step : diff);
  } else if (current > target) {
    int16_t diff = current - target;
    return current - (diff > step ? step : diff);
  }
  return current;
}

uint8_t rampToward8(uint8_t current, uint8_t target, uint8_t step) {
  if (current < target) {
    uint8_t diff = target - current;
    return current + (diff > step ? step : diff);
  } else if (current > target) {
    uint8_t diff = current - target;
    return current - (diff > step ? step : diff);
  }
  return current;
}

void stopMotorsHard() {
  digitalWrite(IN1_PIN, LOW);
  digitalWrite(IN2_PIN, LOW);
  digitalWrite(IN3_PIN, LOW);
  digitalWrite(IN4_PIN, LOW);
  ledcWrite(LEFT_PWM_CH, 0);
  ledcWrite(RIGHT_PWM_CH, 0);
  currentLeft = 0;
  currentRight = 0;
}

void stopESCHard() {
  currentEsc = 0;
  esc.writeMicroseconds(ESC_MIN_US);
}

void applyMotorOutputs(int16_t left, int16_t right) {
  // Left
  if (left == 0) {
    digitalWrite(IN1_PIN, LOW);
    digitalWrite(IN2_PIN, LOW);
    ledcWrite(LEFT_PWM_CH, 0);
  } else if (left > 0) {
    digitalWrite(IN1_PIN, HIGH);
    digitalWrite(IN2_PIN, LOW);
    ledcWrite(LEFT_PWM_CH, (uint8_t)constrain(left, 0, 255));
  } else {
    digitalWrite(IN1_PIN, LOW);
    digitalWrite(IN2_PIN, HIGH);
    ledcWrite(LEFT_PWM_CH, (uint8_t)constrain(-left, 0, 255));
  }

  // Right
  if (right == 0) {
    digitalWrite(IN3_PIN, LOW);
    digitalWrite(IN4_PIN, LOW);
    ledcWrite(RIGHT_PWM_CH, 0);
  } else if (right > 0) {
    digitalWrite(IN3_PIN, HIGH);
    digitalWrite(IN4_PIN, LOW);
    ledcWrite(RIGHT_PWM_CH, (uint8_t)constrain(right, 0, 255));
  } else {
    digitalWrite(IN3_PIN, LOW);
    digitalWrite(IN4_PIN, HIGH);
    ledcWrite(RIGHT_PWM_CH, (uint8_t)constrain(-right, 0, 255));
  }
}

float readBatteryVolts() {
  int raw = analogRead(BATT_ADC_PIN);
  float vAdc = (float)raw * 3.3f / 4095.0f;           // ESP32-C3 ADC ~0..3.3V
  float vBat = vAdc * (BATT_R1 + BATT_R2) / BATT_R2;  // divider back-calculation
  return vBat;
}

void sendTelemetry(uint8_t pipe) {
  TelemetryPacket tele;
  tele.batteryVolts = readBatteryVolts();
  tele.uptimeMs     = millis();
  tele.packetsRx    = packetsRx;
  tele.rpdFlag      = radio.testRPD() ? 1 : 0;  // coarse "strong signal" flag

  radio.writeAckPayload(pipe, &tele, sizeof(tele));
}

void updateStatusLED() {
  unsigned long now = millis();
  bool linkHealthy = (!failsafeLatched && (now - lastPacketTime <= LINK_DROP_MS));

  if (linkHealthy) {
    digitalWrite(LED_PIN, HIGH);
    ledState = true;
  } else {
    if (now - lastLedToggle >= LED_BLINK_MS) {
      ledState = !ledState;
      digitalWrite(LED_PIN, ledState ? HIGH : LOW);
      lastLedToggle = now;
    }
  }
}

// Compute target outputs based on lastPkt & link state
void computeTargets(int16_t &targetLeft, int16_t &targetRight, uint8_t &targetEsc) {
  unsigned long now = millis();
  unsigned long since = now - lastPacketTime;

  // Default to everything off
  targetLeft = 0;
  targetRight = 0;
  targetEsc = 0;

  // Latched failsafe: stay off until disarmed
  if (failsafeLatched) {
    if (lastPkt.armed == 0) {
      // Disarm clears latch
      failsafeLatched = false;
    }
    return;
  }

  // If no packets yet, stay at zero
  if (lastPacketTime == 0) {
    return;
  }

  // If we've exceeded hard failsafe time, latch
  if (since > FAILSAFE_MS) {
    failsafeLatched = true;
    stopMotorsHard();
    stopESCHard();
    return;
  }

  // If link has dropped beyond LINK_DROP_MS, we soft-stop (targets = 0)
  bool linkHealthy = (since <= LINK_DROP_MS);

  bool armed      = (lastPkt.armed == 1);
  bool kill       = (lastPkt.kill  == 1);
  bool escAllowed = (lastPkt.esc_enable == 1);

  if (!armed || kill) {
    // Disarmed or kill = soft-stop to zero (targets already 0)
    return;
  }

  if (linkHealthy) {
    // Follow commanded values
    targetLeft  = constrain(lastPkt.left,  -255, 255);
    targetRight = constrain(lastPkt.right, -255, 255);

    if (escAllowed) {
      int16_t escVal = constrain(lastPkt.esc, 0, 255);
      targetEsc = (uint8_t)escVal;
    }
  }
}

// --------- SETUP ---------
void setup() {
  Serial.begin(115200);
  delay(500);

  pinMode(IN1_PIN, OUTPUT);
  pinMode(IN2_PIN, OUTPUT);
  pinMode(IN3_PIN, OUTPUT);
  pinMode(IN4_PIN, OUTPUT);

  ledcSetup(LEFT_PWM_CH, PWM_FREQ, PWM_RES);
  ledcAttachPin(ENA_PIN, LEFT_PWM_CH);

  ledcSetup(RIGHT_PWM_CH, PWM_FREQ, PWM_RES);
  ledcAttachPin(ENB_PIN, RIGHT_PWM_CH);

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  pinMode(BATT_ADC_PIN, INPUT);

  esc.attach(ESC_PIN);
  stopESCHard();
  Serial.println("Arming ESC at min throttle...");
  delay(3000);  // ESC arming window

  if (!radio.begin()) {
    Serial.println("nRF24 init failed!");
    return;
  }

  radio.setPALevel(RF24_PA_LOW);
  radio.setDataRate(RF24_1MBPS);

  radio.enableAckPayload();
  radio.enableDynamicPayloads();

  radio.openReadingPipe(1, address);
  radio.startListening();

  Serial.println("Robot radio ready");
  lastPacketTime = 0;  // no packets yet
}

// --------- LOOP ---------
void loop() {
  unsigned long now = millis();

  // Handle incoming control packets
  uint8_t pipe;
  while (radio.available(&pipe)) {
    radio.read(&lastPkt, sizeof(lastPkt));
    lastPacketTime = now;
    packetsRx++;

    // Load telemetry as ACK payload for this pipe
    sendTelemetry(pipe);
  }

  // Compute targets based on latest packet & link state
  int16_t targetLeft, targetRight;
  uint8_t targetEsc;
  computeTargets(targetLeft, targetRight, targetEsc);

  // Ramp outputs toward targets
  currentLeft  = rampToward(currentLeft,  targetLeft,  MOTOR_RAMP_STEP);
  currentRight = rampToward(currentRight, targetRight, MOTOR_RAMP_STEP);
  currentEsc   = rampToward8(currentEsc,  targetEsc,   ESC_RAMP_STEP);

  // Apply outputs
  applyMotorOutputs(currentLeft, currentRight);

  int escPulse = map(currentEsc, 0, 255, ESC_MIN_US, ESC_MAX_US);
  esc.writeMicroseconds(escPulse);

  // Update LED (blink / solid)
  updateStatusLED();
}
