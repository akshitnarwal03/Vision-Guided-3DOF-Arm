#include <Servo.h>
#include <EEPROM.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

// Arduino Uno/Nano defaults. The earlier sweep sketch used D3 for base;
// change BASE_PIN here if your wiring still uses D3.
const uint8_t PINS[3] = {9, 10, 11};
const int HOME[3] = {90, 90, 90};
const int DEFAULT_MIN[3] = {60, 60, 60};
const int DEFAULT_MAX[3] = {120, 120, 120};
const int PULSE_MIN_US = 1000;
const int PULSE_MAX_US = 2000;
const unsigned long SERIAL_TIMEOUT_MS = 750;
const unsigned long MOTION_TICK_MS = 20;
const float MAX_SPEED_DEG_S = 35.0f;
const float MAX_ACCEL_DEG_S2 = 90.0f;

struct StoredLimits {
  uint16_t magic;
  uint8_t version;
  uint8_t low[3];
  uint8_t high[3];
  uint8_t checksum;
};

const uint16_t LIMIT_MAGIC = 0xA475;
const uint8_t LIMIT_VERSION = 1;
Servo motors[3];
int lowLimit[3], highLimit[3], target[3];
float position[3] = {90, 90, 90};
float velocity[3] = {0, 0, 0};
bool attached = false;
bool moving = false;
bool timedOut = false;
bool holding = true;
unsigned long lastMessageMs = 0;
unsigned long lastTickMs = 0;
char lineBuffer[96];
uint8_t lineLength = 0;
bool lineOverflow = false;

uint8_t checksum(const StoredLimits &record) {
  uint8_t value = 0x5A;
  value ^= (uint8_t)record.magic;
  value ^= (uint8_t)(record.magic >> 8);
  value ^= record.version;
  for (uint8_t i = 0; i < 3; ++i) {
    value ^= record.low[i];
    value ^= record.high[i];
  }
  return value;
}

bool validLimits(const StoredLimits &record) {
  if (record.magic != LIMIT_MAGIC || record.version != LIMIT_VERSION ||
      record.checksum != checksum(record)) return false;
  for (uint8_t i = 0; i < 3; ++i) {
    if (record.low[i] > HOME[i] || record.high[i] < HOME[i] ||
        record.low[i] >= record.high[i] || record.high[i] > 180) return false;
  }
  return true;
}

void loadLimits() {
  StoredLimits record;
  EEPROM.get(0, record);
  for (uint8_t i = 0; i < 3; ++i) {
    lowLimit[i] = validLimits(record) ? record.low[i] : DEFAULT_MIN[i];
    highLimit[i] = validLimits(record) ? record.high[i] : DEFAULT_MAX[i];
  }
}

void printAngles(const char *prefix) {
  Serial.print(prefix);
  Serial.print(" B:"); Serial.print((int)(position[0] + 0.5f));
  Serial.print(" S:"); Serial.print((int)(position[1] + 0.5f));
  Serial.print(" E:"); Serial.print((int)(position[2] + 0.5f));
}

void printLimits() {
  Serial.print("LIMITS B:"); Serial.print(lowLimit[0]); Serial.print(':'); Serial.print(highLimit[0]);
  Serial.print(" S:"); Serial.print(lowLimit[1]); Serial.print(':'); Serial.print(highLimit[1]);
  Serial.print(" E:"); Serial.print(lowLimit[2]); Serial.print(':'); Serial.println(highLimit[2]);
}

void printStatus() {
  printAngles("STATE");
  Serial.print(" MODE:");
  if (!attached) Serial.print("DETACHED");
  else if (timedOut) Serial.print("TIMEOUT");
  else if (moving) Serial.print("MOVING");
  else if (holding) Serial.print("HOLD");
  else Serial.print("IDLE");
  Serial.print(" TGT:"); Serial.print(target[0]); Serial.print(',');
  Serial.print(target[1]); Serial.print(','); Serial.println(target[2]);
}

void freezeMotion(bool timeout) {
  for (uint8_t i = 0; i < 3; ++i) {
    target[i] = (int)(position[i] + 0.5f);
    position[i] = target[i];
    velocity[i] = 0;
    if (attached) motors[i].write(target[i]);
  }
  moving = false;
  holding = !timeout;
  timedOut = timeout;
}

bool anglesInRange(const int values[3]) {
  for (uint8_t i = 0; i < 3; ++i) {
    if (values[i] < lowLimit[i] || values[i] > highLimit[i]) return false;
  }
  return true;
}

void setTargets(const int values[3]) {
  for (uint8_t i = 0; i < 3; ++i) target[i] = values[i];
  moving = true;
  holding = false;
  timedOut = false;
}

void handleLine(char *line) {
  int a, b, c, d, e, f;
  char extra;
  if (strcmp(line, "PING") == 0) {
    Serial.println("OK PONG");
  } else if (strcmp(line, "STATUS") == 0) {
    printStatus();
  } else if (strcmp(line, "LIMITS?") == 0) {
    printLimits();
  } else if (strcmp(line, "HOLD") == 0) {
    freezeMotion(false);
    Serial.println("OK HOLD");
  } else if (strcmp(line, "DETACH") == 0) {
    freezeMotion(false);
    for (uint8_t i = 0; i < 3; ++i) motors[i].detach();
    attached = false;
    Serial.println("OK DETACH");
  } else if (strcmp(line, "ATTACH") == 0) {
    if (!attached) {
      // No position sensor: operator must support the arm and verify its pose.
      for (uint8_t i = 0; i < 3; ++i) motors[i].attach(PINS[i], PULSE_MIN_US, PULSE_MAX_US);
      attached = true;
      for (uint8_t i = 0; i < 3; ++i) motors[i].write(target[i]);
    }
    freezeMotion(false);
    Serial.println("OK ATTACH");
  } else if (strcmp(line, "HOME") == 0) {
    if (!attached) {
      Serial.println("ERR DETACHED attach first");
      return;
    }
    else {
      setTargets(HOME);
      Serial.println("OK HOME"); // Accepted, not completed. Poll STATUS for MODE:IDLE.
    }
  } else if (sscanf(line, "SET B:%d S:%d E:%d %c", &a, &b, &c, &extra) == 3 &&
             strncmp(line, "SET ", 4) == 0) {
    const int values[3] = {a, b, c};
    if (!attached) {
      Serial.println("ERR DETACHED attach first");
      return;
    } else if (!anglesInRange(values)) {
      Serial.println("ERR LIMIT angle outside firmware limits");
      return;
    }
    else {
      setTargets(values);
      Serial.print("OK SET B:"); Serial.print(a);
      Serial.print(" S:"); Serial.print(b);
      Serial.print(" E:"); Serial.println(c);
    }
  } else if (sscanf(line, "LIMITS B:%d:%d S:%d:%d E:%d:%d %c",
                    &a, &b, &c, &d, &e, &f, &extra) == 6 &&
             strncmp(line, "LIMITS ", 7) == 0) {
    const int nextLow[3] = {a, c, e};
    const int nextHigh[3] = {b, d, f};
    bool valid = !moving && attached;
    for (uint8_t i = 0; i < 3; ++i) {
      if (nextLow[i] < 0 || nextHigh[i] > 180 || nextLow[i] >= nextHigh[i] ||
          nextLow[i] > HOME[i] || nextHigh[i] < HOME[i] ||
          nextLow[i] > position[i] || nextHigh[i] < position[i]) valid = false;
    }
    if (!valid) {
      Serial.println("ERR LIMITS stop motion; include home and current pose");
      return;
    }
    else {
      StoredLimits record;
      record.magic = LIMIT_MAGIC;
      record.version = LIMIT_VERSION;
      for (uint8_t i = 0; i < 3; ++i) {
        record.low[i] = nextLow[i]; record.high[i] = nextHigh[i];
        lowLimit[i] = nextLow[i]; highLimit[i] = nextHigh[i];
      }
      record.checksum = checksum(record);
      EEPROM.put(0, record);
      Serial.println("OK LIMITS");
    }
  } else {
    Serial.println("ERR SYNTAX unknown or malformed command");
    return;
  }
  lastMessageMs = millis();
}

void readSerial() {
  while (Serial.available()) {
    char ch = (char)Serial.read();
    if (ch == '\r') continue;
    if (ch == '\n') {
      if (lineOverflow) Serial.println("ERR SYNTAX line too long");
      else {
        lineBuffer[lineLength] = '\0';
        handleLine(lineBuffer);
      }
      lineLength = 0;
      lineOverflow = false;
    } else if (!lineOverflow) {
      if (lineLength < sizeof(lineBuffer) - 1) lineBuffer[lineLength++] = ch;
      else lineOverflow = true;
    }
  }
}

void updateMotion() {
  unsigned long now = millis();
  if (now - lastTickMs < MOTION_TICK_MS) return;
  float dt = (now - lastTickMs) / 1000.0f;
  lastTickMs = now;
  if (dt > 0.1f) dt = 0.1f;
  if (!moving || !attached) return;
  bool allDone = true;
  for (uint8_t i = 0; i < 3; ++i) {
    float error = target[i] - position[i];
    if (error > -0.05f && error < 0.05f &&
        velocity[i] > -0.1f && velocity[i] < 0.1f) {
      position[i] = target[i]; velocity[i] = 0;
    } else {
      allDone = false;
      float stopSpeed = sqrtf(2.0f * MAX_ACCEL_DEG_S2 * fabsf(error));
      float wanted = (error >= 0 ? 1.0f : -1.0f) * min(MAX_SPEED_DEG_S, stopSpeed);
      float change = constrain(wanted - velocity[i], -MAX_ACCEL_DEG_S2 * dt,
                               MAX_ACCEL_DEG_S2 * dt);
      velocity[i] += change;
      float next = position[i] + velocity[i] * dt;
      if ((error > 0 && next > target[i]) || (error < 0 && next < target[i])) {
        next = target[i]; velocity[i] = 0;
      }
      position[i] = constrain(next, (float)lowLimit[i], (float)highLimit[i]);
    }
    motors[i].write((int)(position[i] + 0.5f));
  }
  if (allDone) moving = false;
}

void setup() {
  Serial.begin(115200);
  loadLimits();
  for (uint8_t i = 0; i < 3; ++i) {
    target[i] = HOME[i];
    motors[i].attach(PINS[i], PULSE_MIN_US, PULSE_MAX_US);
    motors[i].write(HOME[i]);
  }
  attached = true;
  lastMessageMs = millis();
  lastTickMs = lastMessageMs;
  Serial.println("READY ARM1");
}

void loop() {
  readSerial();
  if (attached && !timedOut && millis() - lastMessageMs > SERIAL_TIMEOUT_MS) {
    freezeMotion(true);
  }
  updateMotion();
}
