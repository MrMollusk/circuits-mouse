#include <Wire.h>
#include <Arduino.h>
#include <BleMouse.h>

byte Version[3];
int8_t x_data, y_data, z_data;
byte range = 0x00;
float divi = 16;
float x, y, z;

BleMouse bleMouse("AccelMouse", "ESP32", 100);
const int LEFT_CLICK = 18;
const int RIGHT_CLICK = 17;

static inline int8_t clamp127(int v) {
  if (v > 127) return 127;
  if (v < -127) return -127;
  return (int8_t)v;
}

void setup() {
  Serial.begin(9600);
  Wire.begin();
  pinMode(LEFT_CLICK, INPUT_PULLUP);
  pinMode(RIGHT_CLICK, INPUT_PULLUP);

  // accel config (as you had it)
  Wire.beginTransmission(0x0A);
  Wire.write(0x22);
  Wire.write(range);
  Wire.write(0x20);
  Wire.write(0x05);
  Wire.endTransmission();

  bleMouse.begin();   // pair from PC Bluetooth settings
}

void readAxis(uint8_t reg, byte &dst, int8_t &out) {
  Wire.beginTransmission(0x0A);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(0x0A, (uint8_t)1);

  if (Wire.available()) dst = Wire.read();
  out = ((int8_t)dst) >> 2;
}

void mouseFunction(float xVal, float yVal) {
  if (!bleMouse.isConnected()) return;

  int dx = (int)(xVal * 40.0f);
  int dy = (int)(yVal * 40.0f);

  // deadzone to stop jitter
  if (abs(dx) < 2) dx = 10;
  if (abs(dy) < 2) dy = 10;

  bleMouse.move(clamp127(dy), clamp127(dx), 0);
  delay(10);
}

void loop() {
  switch (range) {
    case 0x00: divi = 16; break;
    case 0x01: divi =  8; break;
    case 0x02: divi =  4; break;
    case 0x03: divi =  2; break;
    default: while (1) {}
  }

  readAxis(0x04, Version[0], x_data);
  readAxis(0x06, Version[1], y_data);
  readAxis(0x08, Version[2], z_data);

  //bleMouse.click(LEFT_CLICK);
  bool leftClickPressed = digitalRead(LEFT_CLICK);
  bool rightClickPressed = digitalRead(RIGHT_CLICK);

  Serial.print("Left-Click: ");
  Serial.print(leftClickPressed);

  Serial.print("Right-Click: ");
  Serial.print(rightClickPressed);

  if(leftClickPressed){
    bleMouse.click(MOUSE_LEFT);
  }
  
  if(rightClickPressed){
    bleMouse.click(MOUSE_RIGHT);
  }

  x = (float)x_data / divi;
  y = (float)y_data / divi;
  z = (float)z_data / divi;


  Serial.print("X="); Serial.print(x);
  Serial.print("  Y="); Serial.print(y);
  Serial.print("  Z="); Serial.println(z);

  mouseFunction(x, y);
  delay(10);
}
