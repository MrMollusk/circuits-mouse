#include <Wire.h>
#include <HijelHID_BLEMouse.h>

#define BOOT_BUTTON_PIN 0
const uint8_t IMU_ADDR = 0x68;   // AD0 tied to GND

// Mouse buttons
#define LEFT_BUTTON_PIN 33
#define RIGHT_BUTTON_PIN 32

// 1/2/4/8/C coded rotary switch pins
const int ENC_1 = 25;
const int ENC_2 = 26;
const int ENC_4 = 27;
const int ENC_8 = 14;

const int SCROLL = 15;

HijelBLEMouse mouse("ESP32 Mouse", "ESP32");

bool lastBootPressed = false;
unsigned long lastPrint = 0;

// Encoder state
int lastEncValue = -1;
unsigned long lastEncChangeMs = 0;

// Mouse button state
bool lastLeftMousePressed = false;
bool lastRightMousePressed = false;
bool lastPairedState = false;

void writeReg(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(IMU_ADDR);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

void RotationSensor(float& dx, float& dy) {
  float rotationVal = analogRead(SCROLL);
  dx = (dx*0.5f/4095.0f)*rotationVal;
  dy = (dy*0.5f/4095.0f)*rotationVal;
}

bool readBytes(uint8_t reg, uint8_t *buf, uint8_t len) {
  Wire.beginTransmission(IMU_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;

  uint8_t n = Wire.requestFrom((int)IMU_ADDR, (int)len);
  if (n != len) return false;

  for (uint8_t i = 0; i < len; i++) {
    if (!Wire.available()) return false;
    buf[i] = Wire.read();
  }
  return true;
}

uint8_t readReg(uint8_t reg) {
  uint8_t v = 0xFF;
  if (readBytes(reg, &v, 1)) return v;
  return 0xFF;
}

int16_t read16(uint8_t reg) {
  uint8_t b[2];
  if (!readBytes(reg, b, 2)) return 0;
  return (int16_t)((b[0] << 8) | b[1]);
}

static inline int clamp127(int v) {
  if (v > 127) return 127;
  if (v < -127) return -127;
  return v;
}

// Active-low because encoder common C is tied to GND
int readEncoder8421() {
  int b1 = (digitalRead(ENC_1) == LOW) ? 1 : 0;
  int b2 = (digitalRead(ENC_2) == LOW) ? 1 : 0;
  int b4 = (digitalRead(ENC_4) == LOW) ? 1 : 0;
  int b8 = (digitalRead(ENC_8) == LOW) ? 1 : 0;

  return b1 + 2 * b2 + 4 * b4 + 8 * b8;
}

// Returns +1, -1, or 0 for one encoder step
int readEncoderStep() {
  int current = readEncoder8421();

  if (lastEncValue < 0) {
    lastEncValue = current;
    return 0;
  }

  if (current == lastEncValue) return 0;

  unsigned long now = millis();
  if (now - lastEncChangeMs < 5) {
    return 0;
  }
  lastEncChangeMs = now;

  int diff = current - lastEncValue;

  // Wrap handling for 0..15
  if (diff == 1 || diff == -15) {
    lastEncValue = current;
    return +1;
  }
  if (diff == -1 || diff == 15) {
    lastEncValue = current;
    return -1;
  }

  // Ignore jumps/noise and resync
  lastEncValue = current;
  return 0;
}

void setup() {
  Serial.begin(115200);

  pinMode(BOOT_BUTTON_PIN, INPUT_PULLUP);

  pinMode(LEFT_BUTTON_PIN, INPUT_PULLUP);
  pinMode(RIGHT_BUTTON_PIN, INPUT_PULLUP);

  pinMode(ENC_1, INPUT_PULLUP);
  pinMode(ENC_2, INPUT_PULLUP);
  pinMode(ENC_4, INPUT_PULLUP);
  pinMode(ENC_8, INPUT_PULLUP);
  pinMode(SCROLL, INPUT);

  Wire.begin(21, 22);
  Wire.setClock(100000);

  // Wake IMU
  writeReg(0x6B, 0x00);
  delay(100);

  // Configure IMU
  writeReg(0x1C, 0x00);   // accel ±2g
  writeReg(0x1B, 0x00);   // gyro ±250 dps
  writeReg(0x1A, 0x04);   // DLPF
  delay(50);

  Serial.print("WHO_AM_I=0x");
  Serial.println(readReg(0x75), HEX);

  lastEncValue = readEncoder8421();
  Serial.print("Initial encoder value = ");
  Serial.println(lastEncValue);

  // Active-low buttons
  lastLeftMousePressed = (digitalRead(LEFT_BUTTON_PIN) == LOW);
  lastRightMousePressed = (digitalRead(RIGHT_BUTTON_PIN) == LOW);

  mouse.setLogLevel(HIDLogLevel::Normal);
  mouse.begin();
}

void loop() {
  bool paired = mouse.isPaired();

  if (!paired) {
    lastPairedState = false;
    delay(20);
    return;
  }

  // Sync button state once immediately after pairing
  if (!lastPairedState) {
    lastPairedState = true;

    lastLeftMousePressed = (digitalRead(LEFT_BUTTON_PIN) == LOW);
    lastRightMousePressed = (digitalRead(RIGHT_BUTTON_PIN) == LOW);

    mouse.setButton(MouseButton::Left, lastLeftMousePressed);
    mouse.setButton(MouseButton::Right, lastRightMousePressed);
  }

  // BOOT test move
  bool bootPressed = (digitalRead(BOOT_BUTTON_PIN) == LOW);
  if (bootPressed && !lastBootPressed) {
    delay(20);
    if (digitalRead(BOOT_BUTTON_PIN) == LOW) {
      Serial.println("BOOT pressed -> forced move");
      mouse.move(40, 0);
      delay(150);
      mouse.move(-40, 0);
    }
  }
  lastBootPressed = bootPressed;

  // Left/right mouse buttons
  bool leftPressed = (digitalRead(LEFT_BUTTON_PIN) == LOW);
  bool rightPressed = (digitalRead(RIGHT_BUTTON_PIN) == LOW);

  if (leftPressed != lastLeftMousePressed) {
    lastLeftMousePressed = leftPressed;
    mouse.setButton(MouseButton::Left, leftPressed);

    Serial.print("Left mouse button ");
    Serial.println(leftPressed ? "pressed" : "released");
  }

  if (rightPressed != lastRightMousePressed) {
    lastRightMousePressed = rightPressed;
    mouse.setButton(MouseButton::Right, rightPressed);

    Serial.print("Right mouse button ");
    Serial.println(rightPressed ? "pressed" : "released");
  }

  // IMU mouse movement
  int16_t ax = read16(0x3B);
  int16_t ay = read16(0x3D);
  int16_t az = read16(0x3F);

  float fax = ax / 16384.0f;
  float fay = ay / 16384.0f;
  float faz = az / 16384.0f;

  // Corrected mapping:
  // forward tilt -> up
  // back tilt -> down
  // left tilt -> left
  // right tilt -> right
  float dx = (int)(-fax * 35.0f);
  float dy = (int)( fay * 35.0f);

  if (abs(dx) < 2) dx = 0;
  if (abs(dy) < 2) dy = 0;

  dx = clamp127(dx);
  dy = clamp127(dy);

  if (dx != 0 || dy != 0) {
    RotationSensor(dx, dy);
    mouse.move(dx, dy);
  }

  // Encoder scroll
  int step = readEncoderStep();
  if (step != 0) {
    // Reverse sign here if scroll direction feels wrong
    mouse.addScroll(-step);

    Serial.print("Encoder value = ");
    Serial.print(lastEncValue);
    Serial.print(" step = ");
    Serial.print(step);
    Serial.print(" scroll sent = ");
    Serial.println(-step);
  }

  if (millis() - lastPrint > 250) {
    lastPrint = millis();
    Serial.print("ax=");
    Serial.print(fax, 3);
    Serial.print(" ay=");
    Serial.print(fay, 3);
    Serial.print(" az=");
    Serial.print(faz, 3);
    Serial.print(" enc=");
    Serial.print(lastEncValue);
    Serial.print(" left=");
    Serial.print(leftPressed ? 1 : 0);
    Serial.print(" right=");
    Serial.println(rightPressed ? 1 : 0);
  }

  delay(10);
}
