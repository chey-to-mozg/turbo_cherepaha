const int sensorLeft = A0;
const int sensorRight = A1;
const int sensorFrontLeft = A2;
const int sensorFrontRight = A3;

const int buttonPin = 6;
const int _buttonDelay = 300; // to not count random signals

bool isLeftWall() {
  return digitalRead(sensorLeft);
}

bool isRightWall() {
  return digitalRead(sensorRight);
}

bool isFrontWall() {
  return digitalRead(sensorFrontLeft) & digitalRead(sensorFrontRight);
}

bool buttonPressed() {
  if (digitalRead(buttonPin)){
    int curTime = millis();
    while (digitalRead(buttonPin)) {
      if (millis - curTime > _buttonDelay) {
        return true;
      }
    }
  }
  return false;
}
