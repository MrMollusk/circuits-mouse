# circuits-mouse

## How to get working 

1. Download the zip file from `https://github.com/T-vK/ESP32-BLE-Mouse`
2. In Arduino IDE, goto sketch -> include library -> Add .ZIP library
3. Goto `C:\Users\[YourName]\Documents\Arduino\libraries\ESP32_BLE_Mouse\BleMouse.cpp` Or wherever your libraries for arduino ide are stored.
4. In BleMouse.cpp, make these changes:
- `BLEDevice::init(bleMouseInstance->deviceName);` TO `BLEDevice::init(String(bleMouseInstance->deviceName.c_str()));`
- `bleMouseInstance->hid->manufacturer()->setValue(bleMouseInstance->deviceManufacturer);` TO `bleMouseInstance->hid->manufacturer()->setValue(String(bleMouseInstance->deviceManufacturer.c_str()));`
- Flash code, you will see red messages eg: Pragma once. These are NOT Error messages, they are the headers from the esp32 library.


