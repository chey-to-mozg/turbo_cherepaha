#include <ArduinoQueue.h>

#define sensorLeft A0
#define sensorFrontLeft A2
#define sensorFrontRight A3
#define sensorRight A1
#define button 10

#define encRight 0
#define encLeft 1

#define pwmLeft 9
#define pwmRight 6
#define inLeft 8
#define inRight 4

#define red 5
#define green 16
#define blue 7

byte pinIns[6] = {encLeft, encRight, sensorLeft, sensorRight, sensorFrontLeft, sensorFrontRight};
byte pinOuts[7] = {pwmLeft, pwmRight, inLeft, inRight, red, green, blue};

// enable prints, prints should be formatted and also need to add delay in debug mode

// fixme finish can be 2x2

#define directionU 0
#define directionR 1
#define directionD 2
#define directionL 3

const bool DEBUG = true;

const int _buttonDelay = 300; // to not count random signals

// ** maze parameters ** //

const int wallLength = 180;

const int mazeShapeY = 3;
const int mazeShapeX = 3;

const int8_t startPositionY = 2;
const int8_t startPositionX = 2;

const int8_t finishPositionY = 1;
const int8_t finishPositionX = 0;

int8_t robotPositionY = startPositionY;
int8_t robotPositionX = startPositionX;

uint8_t robotDirectionStart = directionU;

uint8_t robotDirection = robotDirectionStart;

// maze solving parameters

// 4 directions, cahnges for 2 args: y, x
int8_t  positionChanges[4][2] = {
  -1, 0,
  0, 1,
  1, 0,
  0, -1,
};

uint8_t maze[mazeShapeY][mazeShapeX] = {0};  // manhethen distance
bool visited[mazeShapeY][mazeShapeX] = {0};  // set flag for visited celklls during search
uint8_t walls[mazeShapeY][mazeShapeX] = {0}; // each wall value represent walls in form UNKNOWN bits: URDL | PRESENT bits: URDL
/*
 * Example
 * |   |
 * | | |
 * |0|_|
 * robot in position 0, 0 (marked as 0), for this case walls[0][0] = 1101|0101 (we have information about Up wall, Right wall, Left wall i.e. 1101, and only 2 walls marked is Left wall and Right wall i.e. 0101)
*/

/*
 * test maze 
 * 
10 9  8  7 8  9 10 11 12 13 14 
9  8  7  6 11 10 11 12 13 14 15 
8  7  6  5 4 11 12 13 14 15 16 
9  8  9  6 3  2 13 14 15 16 17 
10 9  8  7 12 1 14 15 16 17 18 
11 10 9 10 11 0 15 16 17 18 23 
12 11 10 11 16 15 16 17 18 19 22 
13 12 11 12 13 14 15 16 17 18 21 
14 13 12 13 14 17 16 17 18 19 20 
15 14 13 14 15 16 17 18 19 20 21 
16 15 14 15 16 17 18 19 20 21 22
 */


// ** sensors ** //

int sensorReads = 10;

//const int sensorValuesPerMillimeter = 3;  // every 3 values is 1 millimeter. Voltage function is not linear, need to find proper function for normalization
const int sensorSideWallDetect = 100;  // in millineters
const int sensorFrontWallDetect = 100;
const int sensorFrontWallTreshold = 35;

int refDistanceLeft = 0;
int refDistanceRight = 0;

// ** reference parameters ** //

const float pi = 3.14;
const int wheelR = 17;  // mm

const int robotOffset = 50;
const int distToCenter = wallLength / 2 - robotOffset;
const int smallTurnCircle = 45;
const int smallCircleDistance = pi * smallTurnCircle * 90 / 180;
const int bigTurnCircleLeft = 127;
const int bigTurnCircleRight = 129;
const int bigCircleDistanceLeft = pi * bigTurnCircleLeft * 90 / 180;
const int bigCircleDistanceRight = pi * bigTurnCircleRight * 90 / 180;
const int distCenterToWheel = 35;

// ** encoder calculations ** //

const int gearRatio = 50;
const int cycleRatio = 3;
const int totalCyclesPerWheel = gearRatio * cycleRatio;
const float degreePerEncoder = 360 / totalCyclesPerWheel;  // degrees

const float encoderPerMillimeter = 180 / ((pi * wheelR) * degreePerEncoder);  // length of sector is L = Pi * R * alpha / 180   ||  1 encoder = 23 millimeters 

const int encodersPerTankTurn = pi * distCenterToWheel * 90 / 180 * encoderPerMillimeter; // each wheel should move 90 degree sector
const int encodersPerCell = wallLength * encoderPerMillimeter;
const int encoderPerHalfCell = encodersPerCell / 2;
const int encodersToCenter = distToCenter * encoderPerMillimeter;
const int encodersPerSmallCircle = smallCircleDistance * encoderPerMillimeter;
const int encodersPerBigCircleLeft = bigCircleDistanceLeft * encoderPerMillimeter;
const int encodersPerBigCircleRight = bigCircleDistanceRight * encoderPerMillimeter;

// ** motor controls ** //

const int Vdefault = 60;
const int Vfast = 150;
const int VfastTurn = 100;
const int Vmin = 0;
const int Vmax = 255;

const int vToCorect = 50;
const int acsSpeed = 5;

const float kPEncDefault = 1;
const float kPEncExtra = 0;

float kPEnc = kPEncDefault;
const float kDEnc = 0;
const float kPSens = 0.2;
const float kDSens = 0.2;


// varialbes to store values
int V = Vdefault;

int errOldEnc = 0;
int errOldSens = 0;

volatile int countLeft;
volatile int countRight;

int targetLeft = 0;
int targetRight = 0;

bool dirLeft = false;
bool dirRight = false;

bool onStart = true;
bool onStop = false;

int vLeft = 0;
int vRight = 0;
int vLeftCur = 0;
int vRightCur = 0;

int distanceLeft = 0;
int distanceRight = 0;
int distanceFrontLeft = 0;
int distanceFrontRight = 0;
int distanceFront = 0;

bool isWallLeft = false;
bool isWallRight = false;
bool isWallFront = false;

bool isLeftButton = false;
bool isRightButton = false;

bool isStart = true;
bool isFinish = false;
bool isBreak = false;
bool isCenter = false;

long prevClock = millis();
long loopInterval = 1;

void printConfig() {
  Serial.print("encoderPerMillimeter: ");
  Serial.println(encoderPerMillimeter);

  Serial.print("encodersPerTankTurn: ");
  Serial.println(encodersPerTankTurn);
  
  Serial.print("encodersPerCell: ");
  Serial.println(encodersPerCell);
  
  Serial.print("encodersToCenter: ");
  Serial.println(encodersToCenter);

  Serial.print("encodersPerSmallCircle: ");
  Serial.println(encodersPerSmallCircle);

  Serial.print("encodersPerBigCircleLeft: ");
  Serial.println(encodersPerBigCircleLeft);

  Serial.print("encodersPerBigCircleRight: ");
  Serial.println(encodersPerBigCircleRight);

  Serial.print("kPEnc: ");
  Serial.println(kPEnc);

  Serial.print("kDEnc: ");
  Serial.println(kDEnc);

  Serial.print("kPSens: ");
  Serial.println(kPSens);

  Serial.print("kDSens: ");
  Serial.println(kDSens);
}

void waitToPrepare() {
  bool signal = false;
  while (true) {
    readSensors();
    if (isLeftButton) {
      digitalWrite(red, true);
      delay(1500);
      digitalWrite(red, false);
      break;
    }
    signal = !signal;
    digitalWrite(green, signal);
    delay(500);
  }
}

void pingOnError() {
  bool signal = false;
  while (true) {
    signal = !signal;
    digitalWrite(red, signal);
    digitalWrite(green, signal);
    digitalWrite(blue, signal);
    delay(1000);
  }
}

void pingOnFinish() {
  bool signal = false;
  while (countLeft < 75) {
    signal = !signal;
    digitalWrite(green, signal);
    delay(1000);
  }
}

void setup() {
  for (int i = 0; i < 6; i++) {
    pinMode(pinIns[i], INPUT);
  }
  for (int i = 0; i < 7; i++) {
    pinMode(pinOuts[i], OUTPUT);
  }
  pinMode(button, INPUT_PULLUP);

  initMotors();

  attachInterrupt(2, rightInterrupt, FALLING);
  attachInterrupt(3, leftInterrupt, FALLING);

  if (DEBUG) {
    Serial.begin(9600);
    while (!Serial) ;
  }
  
  printConfig();
  floodfill(false);
  visited[robotPositionY][robotPositionX] = true;
  waitToPrepare();
  initRefDistance();
  setWalls();
}

void test_path() {
  forward(encodersToCenter);
  forward(encodersPerCell);
  forward(encodersPerCell);
  motorsStop();
  turnTank(false);
  motorsStop();
  forward(encodersPerCell);
  motorsStop();
  waitToPrepare();
}

void loop() {
//  test_path();
//testMotors();
testSensors();
//  checkVoltage();
//  decideMove();
//  if (isBreak) {
//    pingOnError();
//  }
//  if (isFinish) {
//    V = Vfast;
//    while (true) {
//      pingOnFinish();
//      waitToStart();
//      runShort();
//    }
//  }
}
