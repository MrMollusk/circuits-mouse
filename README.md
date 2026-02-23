# circuits-mouse

## How to get working 

1. Download the zip file from [https://github.com/T-vK/ESP32-BLE-Mouse](https://github.com/T-vK/ESP32-BLE-Mouse)
3. In Arduino IDE, goto sketch -> include library -> Add .ZIP library
4. Goto `C:\Users\[YourName]\Documents\Arduino\libraries\ESP32_BLE_Mouse\BleMouse.cpp` Or wherever your libraries for arduino ide are stored.
5. In BleMouse.cpp, make these changes:
- `BLEDevice::init(bleMouseInstance->deviceName);` TO `BLEDevice::init(String(bleMouseInstance->deviceName.c_str()));`
- `bleMouseInstance->hid->manufacturer()->setValue(bleMouseInstance->deviceManufacturer);` TO `bleMouseInstance->hid->manufacturer()->setValue(String(bleMouseInstance->deviceManufacturer.c_str()));`
- Flash esp-32.ino, you will see red messages eg: Pragma once. These are NOT Error messages, they are the headers from the esp32 library.

## Pin Connections
### BMA220 (Gyro) -> ESP32-WROOM-32
- VCC -> 3.3V
- GND -> GND
- SCL -> D22
- SDA -> D21

  ## PINOUTS
  ### ESP32-WROOM
  <img width="1777" height="1039" alt="image" src="https://github.com/user-attachments/assets/1944ba7b-7eb2-41f2-b6fc-62b6960291d2" />

  ### BMA220
  <img width="474" height="474" alt="image" src="https://github.com/user-attachments/assets/55289410-c95c-4dc8-998e-da0ccb504720" />

###Rotation Sensor Pinout
<img width="399" height="660" alt="image" src="https://github.com/user-attachments/assets/268f45dd-1e70-472e-bbae-382b6f05db84" />


  ### Fritzing Diagram 17/02/2026
  <img width="572" height="673" alt="image" src="https://github.com/user-attachments/assets/cec71ba9-26b5-4753-91cb-3d84b76cd963" />

Cap values = 4.7nF

Res values = 1kOhm



<img width="592" height="679" alt="image" src="https://github.com/user-attachments/assets/1ccdd6a7-2b7a-4b71-abaa-57257322258f" />
