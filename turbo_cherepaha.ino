#define sensorLeft A0
#define sensorFront A1
#define sensorRight A2
#define button 4

#define encRight 2
#define encLeft 3

#define pwmLeft 10
#define pwmRight 9
#define in1Left 5
#define in2Left 8
#define in1Right 6
#define in2Right 7

#define volt A7
#define alarm 13

byte pinIns[7] = {encLeft, encRight, sensorLeft, sensorRight, sensorFront, button, volt};
byte pinOuts[7] = {pwmLeft, pwmRight, in1Left, in2Left, in1Right, in2Right, alarm};

// enable prints, prints should be formatted and also need to add delay in debug mode

const bool DEBUG = false;

// constants
const int sensorValuesPerMillimeter = 3;  // every 3 values is 1 millimeter. Voltage function is not linear, need to find proper function for normalization
const int sensorWallDetect = 150;  // in sensor values

const int _buttonDelay = 300; // to not count random signals

const int V = 100;

const float pi = 3.14;

const int wheelR = 23;  // mm
const int degreePerEncoder = 30;  // degrees
const float encoderPerMillimeter = 180 / ((pi * wheelR) * degreePerEncoder);  // length of sector is L = Pi * R * alpha / 180   ||  1 encoder = 23 millimeters 
const int encodersPerTankTurn = 180 / degreePerEncoder - 1; // each whell should turn on 180 degrees

const float kPEnc = 1;
const float kDEnc = 1;
const float kPSens = 0.2;
const float kDSens = 0.1;

int refDistanceLeft = 0;
int refDistanceRight = 0;

// varialbes to store values

int errOldEnc = 0;
int errOldSens = 0;

volatile int countLeft;
volatile int countRight;

bool dirLeft = true;
bool dirRight = true;

bool onStart = true;
bool onStop = false;

int vLeft = 0;
int vRight = 0;
int vLeftCur = 0;
int vRightCur = 0;

int acsSpeed = 20;

int distanceLeft = 0;
int distanceRight = 0;
int distanceFront = 0;

bool isWallLeft = false;
bool isWallRight = false;
bool isWallFront = false;

long prevClock = millis();
long loopInterval = 1;

// maze solving parameters

int mazeShapeX = 2;
int mazeShapeY = 2;

int startPositionX = 0;
int startPositionY = 0;

int finishPositionX = 1;
int finishPositionY = 0;

// finish can be 2x2

#define directionU 0
#define directionR 1
#define directionD 2
#define directionL 3

byte robotDirection = directionU;
// 4 directions, cahnges for 2 args: y, x
char  positionChanges[4][2] = {
  1, 0,
  0, 1,
  -1, 0,
  0, -1,
}

int maze[mazeShapeY][mazeShapeX] = {0};
byte walls[mazeShapeY][mazeShapeX] = {0}; // each wall value represent walls in form UNKNOWN bits: URDL | PRESENT bits: URDL
/*
 * Example
 * |
 * |
 * |_|
*/


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

  Serial.begin(9600);
  resetEncoders();
  initRefDistance();
}

void loop() {
  forward(3);
//  checkVoltage();
//  testSensors();
//  int distance = 145;  // millimeters
//  forward(distance * encoderPerMillimeter);
//  motorsStop();
//  delay(1000);
//  backward(distance * encoderPerMillimeter);
//  motorsStop();
//  delay(1000);
//  turnTank(true);
//  motorsStop();
//  delay(1000);
//  turnTank(false);
//  motorsStop();
//  delay(3000);
//testMotors();
  
}
