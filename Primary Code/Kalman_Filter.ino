#include <Wire.h>
#include <Arduino.h>
#include <BleMouse.h>
#include <BasicLinearAlgebra.h>
#include <Kalman.h>

using namespace BLA;

byte Version[3];

#define Nstate 3
#define Nobs 2

#define n_p 1
#define n_a 0.165

// model std (1/inertia)
#define m_p 0.1
#define m_s 0.1
#define m_a 0.8

unsigned long T;
float DT;

BLA::Matrix<Nobs> obs;
KALMAN<Nstate, Nobs> K;

//PINS
const int EncoderEight = 16;
const int EncoderFour = 4;
const int EncoderTwo = 2;
const int EncoderOne = 15;
const int SCROLL = 35;
const int LEFT_CLICK = 18;
const int RIGHT_CLICK = 17;

//Global Variables
byte range = 0x00;
float divi = 16;
float rotationVal;
int pastEncoderValue = 0;
BleMouse bleMouse("AccelMouse", "ESP32", 100);

void setup() {
  Serial.begin(9600);
  Wire.begin();

  //Initialising Pins
  pinMode(LEFT_CLICK, INPUT_PULLUP); //Using PULLUP to get rid of noise: https://electronics.stackexchange.com/questions/542260/why-is-my-pullup-resistor-more-noise-immune-than-a-pull-down
  pinMode(RIGHT_CLICK, INPUT_PULLUP);
  pinMode(EncoderEight, INPUT_PULLUP);
  pinMode(EncoderFour, INPUT_PULLUP);
  pinMode(EncoderTwo, INPUT_PULLUP);
  pinMode(EncoderOne, INPUT_PULLUP);

  //Wire. enables us to use I2C communication
  //The following lines initialise various parameters of the accelerometer
  Wire.beginTransmission(0x0A);
  Wire.write(0x22);
  Wire.write(range);
  Wire.write(0x20);
  Wire.write(0x05);
  Wire.endTransmission();

   K.F = {1.0, 0.0, 0.0,
		 0.0, 1.0, 0.0,
         0.0, 0.0, 1.0};

  // measurement matrix n the position (e.g. GPS) and acceleration (e.g. accelerometer)
  K.H = {1.0, 0.0, 0.0,
         0.0, 0.0, 1.0};
  // measurement covariance matrix
  K.R = {n_p*n_p,   0.0,
           0.0, n_a*n_a};
  // model covariance matrix
  K.Q = {m_p*m_p,     0.0,     0.0,
             0.0, m_s*m_s,     0.0,
			 0.0,     0.0, m_a*m_a};


  bleMouse.begin();   // pair from PC Bluetooth settings

  T = millis();
}

static inline int8_t clamp127(int v) { //This function just prevents the values from the accelerometer going over a certain value
  if (v > 127) return 127;
  if (v < -127) return -127;
  return (int8_t)v;
}

int EncoderValueFunction(){
  int eight = digitalRead(EncoderEight);
  int four = digitalRead(EncoderFour);
  int two = digitalRead(EncoderTwo);
  int one = digitalRead(EncoderOne);

  int scrollAmount = 0;
  int value = one + 2*two + 2*2*four + 2*2*2*eight; //Converting the encoder values from binary to decimal

  if(value > pastEncoderValue || (value == 0 && pastEncoderValue ==15)){
    scrollAmount = 1;
  }
  else if (value < pastEncoderValue || (value == 15 && pastEncoderValue == 0)){
    scrollAmount = -1;
  }

  pastEncoderValue = value;
  return scrollAmount;
}

void readAxis(uint8_t reg, byte &dst, int8_t &out) {
  Wire.beginTransmission(0x0A);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(0x0A, (uint8_t)1);

  if (Wire.available()) dst = Wire.read();
  out = ((int8_t)dst) >> 2;
}

void mouseFunction(float xVal, float yVal, int scroll) {
  if (!bleMouse.isConnected()) return;

  int dx = (int)(xVal * 40.0f);
  int dy = (int)(yVal * 40.0f);

  // deadzone to stop jitter
  if (abs(dx) < 10) dx = 0;
  if (abs(dy) < 10) dy = 0;

  bleMouse.move(clamp127(dy), clamp127(dx), scroll);
}

void RotationSensor() {
  rotationVal = analogRead(SCROLL);
  divi = (0.05f/4095.0f)*rotationVal; //divi is used to change the sensitivity of the system
}

void printInformation(bool leftClickPressed, bool rightClickPressed, float x, float y, float z){

  //Mouse Button Press
  Serial.print("Left-Click: ");
  Serial.print(leftClickPressed);
  Serial.print("Right-Click: ");
  Serial.print(rightClickPressed);

  
  Serial.print("Divi: ");
  Serial.println(divi);

  Serial.print("X="); Serial.print(x);
  Serial.print("  Y="); Serial.print(y);
  Serial.print("  Z="); Serial.println(z);

}

void buttonFunction(bool &leftClickPressed, bool &rightClickPressed){
  leftClickPressed = digitalRead(LEFT_CLICK);
  rightClickPressed = digitalRead(RIGHT_CLICK);

  if(!leftClickPressed){ //Using NOTleftClickPressed because using INPUT_PULLUP
    bleMouse.click(MOUSE_LEFT);
  }
  
  if(!rightClickPressed){ //Using NOTleftClickPressed because using INPUT_PULLUP
    bleMouse.click(MOUSE_RIGHT);
  }
}

void accelerometerFunction(float &x, float &y, float &z){
  int8_t x_data, y_data, z_data;

  readAxis(0x04, Version[0], x_data);
  readAxis(0x06, Version[1], y_data);
  readAxis(0x08, Version[2], z_data);

  x = (float)x_data / divi;
  y = (float)y_data / divi;
  z = (float)z_data / divi;
}

float xVelocity(float x, float prevX, float prevT){
  return (x-prevX)/(T - prevT);
}

float yVelocity(float y, float prevY, float prevT){
  if(T == prevT){
    return 0;
  }
  return (y-prevY)/(T - prevT);
}


void loop() {
  float x, y, z;
  bool leftClickPressed, rightClickPressed;


  DT = (millis() - T)/1000.0;
  T = millis();
   K.F = {1.0,  DT,  DT*DT/2,
		      0.0, 1.0,       DT,
          0.0, 0.0,      1.0};



  
  buttonFunction(leftClickPressed, rightClickPressed);
  int scroll = EncoderValueFunction();
  RotationSensor();
  accelerometerFunction(x, y, z);
  //printInformation(leftClickPressed, rightClickPressed, x, y, z);

  obs(0) = x;
  obs(1) = y;
  K.update(obs);

  Serial.println("Non-filtered values: ");
  Serial.print("X = ");
  Serial.println(x);
  Serial.print("Y = ");
  Serial.println(y, 6);

  Serial.println("Filtered values: ");
  Serial.print("X = ");
  Serial.println(K.x(0));
  Serial.print("Y = ");
  Serial.println(K.x(1));

  mouseFunction(K.x(0), K.x(1), scroll);

  delay(5);
}
