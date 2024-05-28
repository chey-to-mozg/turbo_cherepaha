#include <ArduinoQueue.h>

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

// fixme finish can be 2x2

#define directionU 0
#define directionR 1
#define directionD 2
#define directionL 3

const bool DEBUG = true;

// maze solving parameters

const int mazeShapeY = 11;
const int mazeShapeX = 11;

const int8_t robotPositionYStart = 10;
const int8_t robotPositionXStart = 10;

const int8_t finishPositionY = 5;
const int8_t finishPositionX = 5;

int8_t robotPositionY = robotPositionYStart;
int8_t robotPositionX = robotPositionXStart;

const uint8_t robotDirectionStart = directionU;

uint8_t robotDirection = robotDirectionStart;
// 4 directions, cahnges for 2 args: y, x
int8_t  positionChanges[4][2] = {
  -1, 0,
  0, 1,
  1, 0,
  0, -1,
};

int8_t positionYHistory[100] = {0};
int8_t positionXHistory[100] = {0};
char ditectionHistory[100] = {0};
int historyIndex = 0;

int maze[mazeShapeY][mazeShapeX] = {0};
uint8_t walls[mazeShapeY][mazeShapeX] = {0}; // each wall value represent walls in form UNKNOWN bits: URDL | PRESENT bits: URDL
//int maze[mazeShapeY][mazeShapeX] = {5, 4, 5, 0, 3, 4, 1, 2, 5};
//uint8_t walls[mazeShapeY][mazeShapeX] = {251, 248, 50, 236, 241, 252, 243, 246, 213 }; // each wall value represent walls in form UNKNOWN bits: URDL | PRESENT bits: URDL
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

// constants
const int sensorValuesPerMillimeter = 3;  // every 3 values is 1 millimeter. Voltage function is not linear, need to find proper function for normalization
const int sensorSideWallDetect = 100;  // in millineters
const int sensorFrontWallDetect = 120;
const int sensorFrontWallTreshold = 35;

const int _buttonDelay = 300; // to not count random signals

const int Vdefault = 70;

const float pi = 3.14;

const int wheelR = 23;  // mm
const int degreePerEncoder = 30;  // degrees
const float encoderPerMillimeter = 180 / ((pi * wheelR) * degreePerEncoder);  // length of sector is L = Pi * R * alpha / 180   ||  1 encoder = 23 millimeters 
const int encodersPerTankTurn = (180 / degreePerEncoder); // each whell should turn on 180 degrees

const float kPEnc = 0;
const float kDEnc = 0;
const float kPSens = 0.2;
const float kDSens = 0.01;

int refDistanceLeft = 0;
int refDistanceRight = 0;

const int wallLength = 190;
const int encodersPerCell = wallLength * encoderPerMillimeter + 1;

const int robotOffset = 50;
const int distToCenter = wallLength / 2 - robotOffset;
const int encodersToCenter = distToCenter * encoderPerMillimeter;

const int encodersToCorrect = encodersToCenter * 2;
const int vToCorect = 50;

const int sensorReads = 15;

// varialbes to store values
int V = Vdefault;

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

int acsSpeed = 10;

int distanceLeft = 0;
int distanceRight = 0;
int distanceFront = 0;

bool isWallLeft = false;
bool isWallRight = false;
bool isWallFront = false;

bool isStart = true;
bool isFinish = false;
bool isBreak = false;

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
  
  Serial.print("encodersToCorrect: ");
  Serial.println(encodersToCorrect);
  
}

void waitToStart() {
  bool signal = false;
  while (true) {
    readSensors();
    if (distanceFront < 50) {
      signal = true;
      digitalWrite(alarm, signal);
      delay(1500);
      break;
    }
    signal = !signal;
    digitalWrite(alarm, signal);
    delay(500);
  }
}

void pingOnError() {
  bool signal = false;
  while (true) {
    signal = !signal;
    digitalWrite(alarm, signal);
    delay(1000);
  }
}

void pingOnFinish() {
  bool signal = false;
  while (countLeft < 3) {
    signal = !signal;
    digitalWrite(alarm, signal);
    delay(1000);
  }
}

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
  printConfig();
  initMaze();
  waitToStart();
  initRefDistance();
  waitToStart();
  setWalls();
}

void loop() {
  checkVoltage();
  decideMove();
  //printWalls();
  if (isBreak) {
    exit(1);
  }
  if (isFinish) {
    pingOnFinish();
    waitToStart();
    runShort();
    exit(0);
  }  
//  testSensors();
}
