#include <HijelHID_BLEMouse.h>

HijelBLEMouse mouse("ESP32 Mouse", "ESP32");

unsigned long lastPrint = 0;

void setup() {
  Serial.begin(115200);
  delay(200);

  mouse.setLogLevel(HIDLogLevel::Verbose);
  // mouse.clearBonds();   // use once, then comment out
  mouse.begin();
}

void loop() {
  if (millis() - lastPrint > 1000) {
    lastPrint = millis();
    Serial.print("connected=");
    Serial.print(mouse.isConnected());
    Serial.print(" paired=");
    Serial.println(mouse.isPaired());
  }

  if (!mouse.isPaired()) {
    delay(20);
    return;
  }

  mouse.move(20, 0);
  delay(1000);
  mouse.move(-20, 0);
  delay(1000);
}
