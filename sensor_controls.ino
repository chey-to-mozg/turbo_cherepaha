int calcDistance(int sensor) {
  // FIXME try transform value using formula
  int normalizedValue = analogRead(sensor) - sensorMinValue;
  if (normalizedValue < 0) {
    normalizedValue = 0;
  }
  return sensorMaxValue - normalizedValue;
}

void calcDistanceLeft() {
  distanceLeft = calcDistance(sensorLeft);
}

void calcDistanceRight() {
  distanceRight = calcDistance(sensorRight);
}

void calcDistanceFront() {
  distanceFront = calcDistance(sensorFront);
}

bool checkLeftWall() {
  isWallLeft = distanceLeft < sensorWallDetect;
}

int checkRightWall() {
  isWallRight = distanceRight < sensorWallDetect;
}

int checkFrontWall() {
  isWallFront = distanceFront < sensorWallDetect;
}

void readSensors() {
  calcDistanceLeft();
  checkLeftWall();
  calcDistanceRight();
  checkRightWall();
  calcDistanceFront();
  checkFrontWall();
}


bool buttonPressed() {
  if (digitalRead(button)){
    int curTime = millis();
    while (digitalRead(button)) {
      if (millis() - curTime > _buttonDelay) {
        return true;
      }
    }
  }
  return false;
}

void testSensors() {
  Serial.print("left encoder: ");
  Serial.print(countLeft);
  Serial.print(" right encoder:");
  Serial.println(countRight);
  
  readSensors();
  Serial.print("left sensor: ");
  Serial.print(distanceLeft);
  Serial.print(" right sensor:");
  Serial.print(distanceRight);
  Serial.print(" front sensor: ");
  Serial.println(distanceFront);

  Serial.print("Button: ");
  Serial.println(buttonPressed());
}

void checkVoltage() {
  digitalWrite(alarm, (analogRead(volt) < 770 ? 1 : 0));
}
