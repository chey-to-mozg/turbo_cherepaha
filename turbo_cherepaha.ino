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
#define blue A4
#define white A5

byte pinIns[7] = {encLeft, encRight, sensorLeft, sensorRight, sensorFront, button, volt};
byte pinOuts[9] = {pwmLeft, pwmRight, in1Left, in2Left, in1Right, in2Right, alarm, blue, white};

#define directionU 0
#define directionR 1
#define directionD 2
#define directionL 3

const bool DEBUG = false; // no prints with DEBUG == false, to enable floodfill maps prints and disable motors set DEBUG to true

const int _buttonDelay = 300; // to not count random signals

// ** maze parameters ** //

const int wallLength = 182;

const int mazeShapeY = 11;
const int mazeShapeX = 11;

const int8_t robotPositionYStart = 10;
const int8_t robotPositionXStart = 10;
const int8_t finishPositionY = 5;
const int8_t finishPositionX = 5;

int8_t robotPositionY = robotPositionYStart;
int8_t robotPositionX = robotPositionXStart;

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

int maze[mazeShapeY][mazeShapeX] = {0};
bool visited[mazeShapeY][mazeShapeX] = {0};
uint8_t walls[mazeShapeY][mazeShapeX] = {0}; // each wall value represent walls in form UNKNOWN bits: URDL | PRESENT bits: URDL
/*
 * Example
 * |   |
 * | | |
 * |0|_|
 * robot in position 0, 0 (marked as 0), for this case walls[0][0] = 1101|0101 (we have information about Up wall, Right wall, Left wall i.e. 1101, 
 * and only 2 walls marked: Left wall and Right wall i.e. 0101)
*/

// ** sensors ** //

int sensorReads = 10;

const int sensorSideWallDetect = 80;  // in millineters
const int sensorFrontWallDetect = 135;
const int sensorFrontWallTreshold = 40; //

int refDistanceLeft = 0;
int refDistanceRight = 0;

// ** reference parameters ** //

const float pi = 3.14;
const int wheelR = 23;  // mm

const int robotOffset = 50;
const int distToCenter = wallLength / 2 - robotOffset;
const int smallTurnCircle = 45;
const int smallCircleDistance = pi * smallTurnCircle * 90 / 180;
const int bigTurnCircle = 130;
const int bigCircleDistance = pi * bigTurnCircle * 90 / 180;
const int distCenterToWheel = 39;

// ** encoder calculations ** //

const float degreePerEncoder = 1.2;  // degrees

const float encoderPerMillimeter = 180 / ((pi * wheelR) * degreePerEncoder);  // length of sector is L = Pi * R * alpha / 180

const int encodersPerTankTurn = pi * distCenterToWheel * 90 / 180 * encoderPerMillimeter; // each wheel should move 90 degree sector || Double check on wheels with another R
const int encodersPerCell = wallLength * encoderPerMillimeter;
const int encoderPerHalfCell = encodersPerCell / 2;
const int encodersToCenter = distToCenter * encoderPerMillimeter;
const int encodersPerSmallCircle = smallCircleDistance * encoderPerMillimeter;
const int encodersPerBigCircle = bigCircleDistance * encoderPerMillimeter;

// ** motor controls ** //

const int Vdefault = 80;
const int Vfast = 150;
int VfastForward = Vfast;
const int VfastTurn = 100;
const int Vmin = 0;
const int Vmax = 255;

const int vToCorect = Vdefault;
const int acsSpeed = 5;

const float kPEncDefault = 0.17;
const float kPEncExtra = 0.33;

float kPEnc = kPEncDefault;
const float kDEnc = 0.1;
const float kPSens = 0.9;
const float kDSens = 0.9;


// varialbes to store values
int V = Vdefault;

int errOldEnc = 0;
int errOldSens = 0;

volatile int countLeft;
volatile int countRight;

int targetLeft = 0;
int targetRight = 0;

bool dirLeft = true;
bool dirRight = true;

bool onStart = true;
bool onStop = false;

int vLeft = 0;
int vRight = 0;
int vLeftCur = 0;
int vRightCur = 0;

int distanceLeft = 0;
int distanceRight = 0;
int distanceFront = 0;

bool isWallLeft = false;
bool isWallRight = false;
bool isWallFront = false;

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

  Serial.print("kPEnc: ");
  Serial.println(kPEnc);

  Serial.print("kDEnc: ");
  Serial.println(kDEnc);

  Serial.print("kPSens: ");
  Serial.println(kPSens);

  Serial.print("kDSens: ");
  Serial.println(kDSens);
}

void waitToStart() {
  bool signal = false;
  int led = white;
  while (true) {
    readSensors();
    if (distanceFront < 50) {
      digitalWrite(led, true);
      delay(1500);
      digitalWrite(led, false);
      break;
    }
    if (countLeft > 75) {
      digitalWrite(led, false);
      if (led == white) {
        led = blue;
        robotDirectionStart = directionL;
        robotDirection = robotDirectionStart;
      }
      else {
        led = white;
        robotDirectionStart = directionU;
        robotDirection = robotDirectionStart;
      }
      resetEncoders();
    }
    signal = !signal;
    digitalWrite(led, signal);
    delay(500);
  }
  resetEncoders();
}

void pingOnError() {
  bool signal = false;
  while (true) {
    signal = !signal;
    digitalWrite(alarm, signal);
    digitalWrite(white, signal);
    digitalWrite(blue, signal);
    delay(1000);
  }
}

void pingOnFinish() {
  bool signal = false;
  while (countLeft < 75) {
    signal = !signal;
    digitalWrite(alarm, signal);
    delay(1000);
    if (countRight > 75) {
      if (VfastForward == Vfast) {
        VfastForward = VfastTurn;
      }
      else {
        VfastForward = Vfast;
      }
      digitalWrite(white, 1);
      delay(1000);
      digitalWrite(white, 0);
      resetEncoders();
    }
  }
}

void setup() {
  for (int i = 0; i < 7; i++) {
    pinMode(pinIns[i], INPUT);
  }

  for (int i = 0; i < 9; i++) {
    pinMode(pinOuts[i], OUTPUT);
  }

  initMotors();

  EIMSK |= (1 << INT0) | ( 1 << INT1 );    // Enable both interrupts
  EICRA |= (1 << ISC11) | ( 1 << ISC10 );  // RISING edge INT1
  EICRA |= (1 << ISC01) | ( 1 << ISC00 );  // RISING edge INT0

  Serial.begin(9600);
  
  printConfig();
  initMaze(true);
  visited[robotPositionY][robotPositionX] = true;
  waitToStart();
  initRefDistance();
  setWalls();
}

void loop() {
  checkVoltage();
  decideMove();
  if (isBreak) {
    pingOnError();
  }
  if (isFinish) {
    V = Vfast;
    while (true) {
      pingOnFinish();
      waitToStart();
      runShort();
    }
  }
}
