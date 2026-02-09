#include <Wire.h>
#include <Arduino.h>
#include <Mouse.h>

byte Version[3];
int8_t x_data;
int8_t y_data;
int8_t z_data;
byte range=0x00;
float divi=16;
float x,y,z;
void setup()
{
  Serial.begin(9600);
  Wire.begin();
  Wire.beginTransmission(0x0A); // address of the accelerometer
  Mouse.begin();
  // range settings
  Wire.write(0x22); //register address
  Wire.write(range); //can be set at"0x00""0x01""0x02""0x03", refer to Datashhet on wiki
  // low pass filter
  Wire.write(0x20); //register address
  Wire.write(0x05); //can be set at"0x05""0x04"......"0x01""0x00", refer to Datashhet on wiki
  Wire.endTransmission();
}

void AccelerometerInit()
{
  Wire.beginTransmission(0x0A); // address of the accelerometer
  // reset the accelerometer
  Wire.write(0x04); // Y data
  Wire.endTransmission();
  Wire.requestFrom(0x0A,1);    // request 6 bytes from slave device #2
  while(Wire.available())    // slave may send less than requested
  {
    Version[0] = Wire.read(); // receive a byte as characte
  }
  x_data=(int8_t)Version[0]>>2;

  Wire.beginTransmission(0x0A); // address of the accelerometer
  // reset the accelerometer
  Wire.write(0x06); // Y data
  Wire.endTransmission();
  Wire.requestFrom(0x0A,1);    // request 6 bytes from slave device #2
  while(Wire.available())    // slave may send less than requested
  {
    Version[1] = Wire.read(); // receive a byte as characte
  }
  y_data=(int8_t)Version[1]>>2;

  Wire.beginTransmission(0x0A); // address of the accelerometer
  // reset the accelerometer
  Wire.write(0x08); // Y data
  Wire.endTransmission();
  Wire.requestFrom(0x0A,1);    // request 6 bytes from slave device #2
   while(Wire.available())    // slave may send less than requested
  {
    Version[2] = Wire.read(); // receive a byte as characte
  }
   z_data=(int8_t)Version[2]>>2;

   x=(float)x_data/divi;
   y=(float)y_data/divi;
   z=(float)z_data/divi;
   Serial.print("X=");
   Serial.print(x);         // print the character
   Serial.print("  ");
   Serial.print("Y=");
   Serial.print(y);         // print the character
   Serial.print("  ");
   Serial.print("Z=");           // print the character
   Serial.println(z);
   mouseFunction(x, y);
}

void mouseFunction(float xVal, float yVal){


  Mouse.move(xVal*90, yVal*90, 0);
  delay(10);
}

void loop()
{
  switch(range)  //change the data dealing method based on the range u've set
  {
  case 0x00:divi=16;  break;
  case 0x01:divi=8;  break;
  case 0x02:divi=4;  break;
  case 0x03:divi=2;  break;
  default: Serial.println("range setting is Wrong,range:from 0to 3.Please check!");while(1);
  }
  AccelerometerInit();
 delay(10);

}
