#include <Wire.h>
#include <HijelHID_BLEMouse.h>
#include <math.h>

#define BOOT_BUTTON_PIN 0
#define LEFT_BUTTON_PIN 33
#define RIGHT_BUTTON_PIN 32
#define TOUCH_SENSOR 4          // ESP32 touch pin
#define SENSITIVITY_PIN 15      // potentiometer

const uint8_t IMU_ADDR = 0x68;  // MPU6050, AD0 -> GND

// 1/2/4/8 coded rotary switch pins
const int ENC_1 = 25;
const int ENC_2 = 26;
const int ENC_4 = 27;
const int ENC_8 = 14;

// MPU6050 registers
#define MPU6050_REG_PWR_MGMT_1    0x6B
#define MPU6050_REG_CONFIG        0x1A
#define MPU6050_REG_GYRO_CONFIG   0x1B
#define MPU6050_REG_ACCEL_CONFIG  0x1C
#define MPU6050_REG_ACCEL_XOUT_H  0x3B
#define MPU6050_REG_WHO_AM_I      0x75

// Sensor scales
#define ACCEL_SCALE 16384.0f   // ±2g
#define GYRO_SCALE  131.0f     // ±250 dps

HijelBLEMouse mouse("ESP32 Mouse", "ESP32");

// BLE / button state
bool lastBootPressed = false;
bool lastLeftMousePressed = false;
bool lastRightMousePressed = false;
bool lastPairedState = false;

// Encoder state
int lastEncValue = -1;
unsigned long lastEncChangeMs = 0;

// Timing
unsigned long lastPrint = 0;
unsigned long lastLoopUs = 0;
unsigned long lastTouchMs = 0;

// Gyro offsets
float gyroOffsetX = 0.0f;
float gyroOffsetY = 0.0f;
float gyroOffsetZ = 0.0f;

// Neutral tilt at calibration
float neutralPitch = 0.0f;
float neutralRoll = 0.0f;

// Kalman filter tuning
const float Q_ANGLE   = 0.001f;
const float Q_BIAS    = 0.003f;
const float R_MEASURE = 0.03f;

// Motion tuning
const int TOUCH_THRESHOLD = 100;
const unsigned long TOUCH_HOLD_MS = 5000;
const float TILT_DEADZONE_DEG = 10.0f;

struct Kalman1D {
  float angle;
  float bias;
  float P00, P01, P10, P11;
};

Kalman1D kalPitch;
Kalman1D kalRoll;

void initKalman(Kalman1D &k, float startAngle) {
  k.angle = startAngle;
  k.bias = 0.0f;
  k.P00 = 0.0f;
  k.P01 = 0.0f;
  k.P10 = 0.0f;
  k.P11 = 0.0f;
}

float updateKalman(Kalman1D &k, float measuredAngle, float gyroRate, float dt) {
  // Predict
  float rate = gyroRate - k.bias;
  k.angle += dt * rate;

  k.P00 += dt * (dt * k.P11 - k.P01 - k.P10 + Q_ANGLE);
  k.P01 -= dt * k.P11;
  k.P10 -= dt * k.P11;
  k.P11 += Q_BIAS * dt;

  // Update
  float S = k.P00 + R_MEASURE;
  float K0 = k.P00 / S;
  float K1 = k.P10 / S;

  float y = measuredAngle - k.angle;
  k.angle += K0 * y;
  k.bias  += K1 * y;

  float P00_temp = k.P00;
  float P01_temp = k.P01;

  k.P00 -= K0 * P00_temp;
  k.P01 -= K0 * P01_temp;
  k.P10 -= K1 * P00_temp;
  k.P11 -= K1 * P01_temp;

  return k.angle;
}

void writeReg(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(IMU_ADDR);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
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

static inline int clamp127(int v) {
  if (v > 127) return 127;
  if (v < -127) return -127;
  return v;
}

float applyResponseCurve(float angleDeg) {
  float s = (angleDeg < 0) ? -1.0f : 1.0f;
  float a = fabs(angleDeg);

  // optional tiny deadzone
  if (a < 0.2f) return 0.0f;

  // power curve
  float out = 1.8f * powf(a, 1.35f);

  if (out > 127.0f) out = 127.0f;
  return s * out;
}

float readSensitivityScale() {
  int raw = analogRead(SENSITIVITY_PIN);   // 0..4095 on ESP32
  return ((0.25 * raw) / 4095.0f);
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

  if (diff == 1 || diff == -15) {
    lastEncValue = current;
    return +1;
  }
  if (diff == -1 || diff == 15) {
    lastEncValue = current;
    return -1;
  }

  lastEncValue = current;
  return 0;
}

bool readMPU6050(float &ax, float &ay, float &az, float &gx, float &gy, float &gz) {
  uint8_t b[14];
  if (!readBytes(MPU6050_REG_ACCEL_XOUT_H, b, 14)) return false;

  int16_t rawAx = (int16_t)((b[0] << 8) | b[1]);
  int16_t rawAy = (int16_t)((b[2] << 8) | b[3]);
  int16_t rawAz = (int16_t)((b[4] << 8) | b[5]);

  // b[6], b[7] are temperature, skip them

  int16_t rawGx = (int16_t)((b[8] << 8) | b[9]);
  int16_t rawGy = (int16_t)((b[10] << 8) | b[11]);
  int16_t rawGz = (int16_t)((b[12] << 8) | b[13]);

  ax = rawAx / ACCEL_SCALE;
  ay = rawAy / ACCEL_SCALE;
  az = rawAz / ACCEL_SCALE;

  gx = rawGx / GYRO_SCALE - gyroOffsetX;
  gy = rawGy / GYRO_SCALE - gyroOffsetY;
  gz = rawGz / GYRO_SCALE - gyroOffsetZ;

  return true;
}

void calibrateIMU() {
  Serial.println("Calibrating IMU... keep it still");

  const int samples = 300;
  float ax, ay, az, gx, gy, gz;

  float gyroSumX = 0.0f;
  float gyroSumY = 0.0f;
  float gyroSumZ = 0.0f;

  float pitchSum = 0.0f;
  float rollSum = 0.0f;

  for (int i = 0; i < samples; i++) {
    if (readMPU6050(ax, ay, az, gx, gy, gz)) {
      gyroSumX += gx;
      gyroSumY += gy;
      gyroSumZ += gz;

      // Forward/back tilt from ay
      float accelPitch = atan2f(ay, sqrtf(ax * ax + az * az)) * 180.0f / PI;

      // Horizontal axis sign inverted/fixed here
      float accelRoll = atan2f(ax, sqrtf(ay * ay + az * az)) * 180.0f / PI;

      pitchSum += accelPitch;
      rollSum += accelRoll;
    }
    delay(5);
  }

  gyroOffsetX = gyroSumX / samples;
  gyroOffsetY = gyroSumY / samples;
  gyroOffsetZ = gyroSumZ / samples;

  neutralPitch = pitchSum / samples;
  neutralRoll  = rollSum / samples;

  initKalman(kalPitch, neutralPitch);
  initKalman(kalRoll, neutralRoll);

  Serial.println("Calibration complete");
  Serial.print("gyro offsets: ");
  Serial.print(gyroOffsetX, 3); Serial.print(", ");
  Serial.print(gyroOffsetY, 3); Serial.print(", ");
  Serial.println(gyroOffsetZ, 3);

  Serial.print("neutral pitch/roll: ");
  Serial.print(neutralPitch, 3); Serial.print(", ");
  Serial.println(neutralRoll, 3);
}

int currentTouchTime; 

void setup() {
  Serial.begin(115200);

  pinMode(BOOT_BUTTON_PIN, INPUT_PULLUP);
  pinMode(LEFT_BUTTON_PIN, INPUT_PULLUP);
  pinMode(RIGHT_BUTTON_PIN, INPUT_PULLUP);

  pinMode(ENC_1, INPUT_PULLUP);
  pinMode(ENC_2, INPUT_PULLUP);
  pinMode(ENC_4, INPUT_PULLUP);
  pinMode(ENC_8, INPUT_PULLUP);

  pinMode(SENSITIVITY_PIN, INPUT);

  Wire.begin(21, 22);
  Wire.setClock(100000);

  writeReg(MPU6050_REG_PWR_MGMT_1, 0x00);
  delay(100);
  writeReg(MPU6050_REG_ACCEL_CONFIG, 0x00); // accel ±2g
  writeReg(MPU6050_REG_GYRO_CONFIG, 0x00);  // gyro ±250 dps
  writeReg(MPU6050_REG_CONFIG, 0x04);       // DLPF
  delay(50);

  Serial.print("WHO_AM_I = 0x");
  Serial.println(readReg(MPU6050_REG_WHO_AM_I), HEX);

  lastEncValue = readEncoder8421();
  Serial.print("Initial encoder value = ");
  Serial.println(lastEncValue);

  lastLeftMousePressed = (digitalRead(LEFT_BUTTON_PIN) == LOW);
  lastRightMousePressed = (digitalRead(RIGHT_BUTTON_PIN) == LOW);

  mouse.setLogLevel(HIDLogLevel::Normal);
  mouse.begin();

#if defined(ESP32)
  analogReadResolution(12);
  analogSetPinAttenuation(SENSITIVITY_PIN, ADC_11db);
#endif

  calibrateIMU();

  lastLoopUs = micros();
  lastTouchMs = 0;
  currentTouchTime = 0;
}


void loop() {
  bool paired = mouse.isPaired();

  if (!paired) {
    lastPairedState = false;
    delay(20);
    return;
  }

  if (!lastPairedState) {
    lastPairedState = true;

    lastLeftMousePressed = (digitalRead(LEFT_BUTTON_PIN) == LOW);
    lastRightMousePressed = (digitalRead(RIGHT_BUTTON_PIN) == LOW);

    mouse.setButton(MouseButton::Left, lastLeftMousePressed);
    mouse.setButton(MouseButton::Right, lastRightMousePressed);
  }

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

  // Touch gating: touched usually means lower value on ESP32 touchRead
  int touchVal = analogRead(TOUCH_SENSOR);
  Serial.print("Touch Sensor Value: ");
  Serial.println(touchVal);
  currentTouchTime = millis();

  if (touchVal > TOUCH_THRESHOLD) {
    lastTouchMs = millis();
  }
  bool movementEnabled = (millis() - lastTouchMs) < TOUCH_HOLD_MS;
  int changeInTouchTime = currentTouchTime - lastTouchMs;
  // Buttons
  bool leftPressed = (digitalRead(LEFT_BUTTON_PIN) == LOW);
  bool rightPressed = (digitalRead(RIGHT_BUTTON_PIN) == LOW);
  Serial.print("Change in Touch time");
  Serial.println(changeInTouchTime);


  if(changeInTouchTime <= TOUCH_HOLD_MS){
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

    // IMU + Kalman
    float ax, ay, az, gx, gy, gz;
    if (readMPU6050(ax, ay, az, gx, gy, gz)) {
      unsigned long nowUs = micros();
      float dt = (nowUs - lastLoopUs) / 1000000.0f;
      lastLoopUs = nowUs;

      if (dt <= 0.0f || dt > 0.05f) dt = 0.01f;

      float accelPitch = atan2f(ay, sqrtf(ax * ax + az * az)) * 180.0f / PI;

      // Horizontal axis inversion fixed here
      float accelRoll = atan2f(ax, sqrtf(ay * ay + az * az)) * 180.0f / PI;

      float filtPitch = updateKalman(kalPitch, accelPitch, gx, dt);

      // Roll uses inverted gyro sign to match the new accel roll sign
      float filtRoll = updateKalman(kalRoll, accelRoll, -gy, dt);

      if (movementEnabled) {
        float forwardBack = filtPitch - neutralPitch;
        float leftRight   = filtRoll - neutralRoll;

        float dx = applyResponseCurve(leftRight);
        float dy = applyResponseCurve(-forwardBack); // forward tilt -> cursor up

        float sens = readSensitivityScale();
        dx *= sens;
        dy *= sens;

        int mouseDx = clamp127((int)dx);
        int mouseDy = clamp127((int)dy);

        if (mouseDx != 0 || mouseDy != 0) {
          mouse.move(-mouseDx, -mouseDy);
        }
      }

      if (millis() - lastPrint > 250) {
        lastPrint = millis();

        Serial.print("touch=");
        Serial.print(touchVal);
        Serial.print(" active=");
        Serial.print(movementEnabled ? 1 : 0);
        Serial.print(" pitch=");
        Serial.print(filtPitch - neutralPitch, 2);
        Serial.print(" roll=");
        Serial.print(filtRoll - neutralRoll, 2);
        Serial.print(" sens=");
        Serial.print(readSensitivityScale(), 2);
        Serial.print(" enc=");
        Serial.println(lastEncValue);
      }
    }

    // Encoder scroll
    int step = readEncoderStep();
    if (step != 0) {
      mouse.addScroll(-step);

      Serial.print("Encoder value = ");
      Serial.print(lastEncValue);
      Serial.print(" step = ");
      Serial.print(step);
      Serial.print(" scroll sent = ");
      Serial.println(-step);
    }
  }

  delay(5);
}
