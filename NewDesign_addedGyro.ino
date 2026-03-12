#include <Wire.h>
void setup() {
  Serial.begin(9600);
  delay(1000);  // give sensor time to boot
  Wire.begin(21, 22);
  Serial.println("Scanning...");
}

void loop() {
  for (byte addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      Serial.print("Device found at 0x");
      Serial.println(addr, HEX);
    }
  }
  Serial.println("Scan done");
  delay(3000);
}