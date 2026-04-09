#include <HijelHID_BLEMouse.h>

HijelBLEMouse mouse("ESP32 Mouse", "ESP32");
unsigned long lastPrint = 0;

void setup() {
  Serial.begin(115200);
  delay(500);

  mouse.setLogLevel(HIDLogLevel::Normal);

  // Uncomment ONCE, flash, pair fresh, then comment it again
  // mouse.clearBonds();

  mouse.begin();

  Serial.println("BLE mouse started");
}

void loop() {
  if (millis() - lastPrint > 1000) {
    lastPrint = millis();
    Serial.print("bonded=");
    Serial.print(mouse.isBonded());
    Serial.print(" connected=");
    Serial.print(mouse.isConnected());
    Serial.print(" paired=");
    Serial.println(mouse.isPaired());
  }

  if (mouse.isPaired()) {
    mouse.move(20, 0);
    delay(1000);
    mouse.move(-20, 0);
    delay(1000);
  } else {
    delay(20);
  }
}
