#define sensorLeft A0
#define sensorFront A1
#define sensorRight A2
#define button 4

#define encRight 2
#define encLeft 3

#define pwmLeft 10
#define pwmRight 9
#define in1Left 6
#define in2Left 7
#define in1Right 5
#define in2Right 8

#define volt A7
#define alarm 13

const bool DEBUG = true;

const int sensorMinValue = 60; // in sensor values
const int sensorMaxValue = 400; // in sensor values
const int sensorValuesPerMillimeter = 3;  // every 3 values is 1 millimeter. Voltage function is not linear, need to find proper function for normalization
const int sensorWallDetect = 370;  // in sensor values

const int _buttonDelay = 300; // to not count random signals

const int V = 100;

const float pi = 3.14;

const int wheelR = 23;  // mm
const int degreePerEncoder = 30;  // degrees
const float encoderPerMillimeter = 180 / ((pi * wheelR) * degreePerEncoder);  // length of sector is L = Pi * R * alpha / 180   ||  1 encoder = 23 millimeters 

volatile int countLeft;
volatile int countRight;

// varialbes to store values
bool dirLeft = true;
bool dirRight = true;

int vLeft = 0;
int vRight = 0;

int distanceLeft = 0;
int distanceRight = 0;
int distanceFront = 0;

bool isWallLeft = false;
bool isWallRight = false;
bool isWallFront = false;

byte pinIns[7] = {encLeft, encRight, sensorLeft, sensorRight, sensorFront, button, volt};
byte pinOuts[7] = {pwmLeft, pwmRight, in1Left, in2Left, in1Right, in2Right, alarm};

void setup() {
  for (int i = 0; i < 7; i++) {
    pinMode(pinIns[i], INPUT);
  }

  for (int i = 0; i < 7; i++) {
    pinMode(pinOuts[i], OUTPUT);
  }

  initMotors();

  EIMSK |= (1 << INT0) | ( 1 << INT1 );    // Enable both interrupts
  EICRA |= (1 << ISC11) | ( 1 << ISC10 );  // RISING edge INT1
  EICRA |= (1 << ISC01) | ( 1 << ISC00 );  // RISING edge INT0

  resetEncoders();
  Serial.begin(9600);
}

void loop() {
  checkVoltage();
  int distance = 141;  // millimeters
  forward(distance * encoderPerMillimeter, distance * encoderPerMillimeter);
  delay(2000);
}
