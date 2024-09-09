const float MCU_VOLTAGE = 5.0;

int calcDistance(int sensor) {
  int rawData = 0;
  for (int i = 0; i < sensorReads; i++) {
    rawData += analogRead(sensor);
  }
  float normalizedData = rawData * MCU_VOLTAGE / (1023.0 * sensorReads);
  return (33.9 + -69.5*(normalizedData) + 62.3*pow(normalizedData,2) + -25.4*pow(normalizedData,3) + 3.83*pow(normalizedData,4)) * 10; // in millimeters
}

void calcDistanceLeft() {
  distanceLeft = calcDistance(sensorLeft);
}

void calcDistanceRight() {
  distanceRight = calcDistance(sensorRight);
}

void calcDistanceFront() {
  distanceFrontLeft = calcDistance(sensorFrontLeft);
  distanceFrontRight = calcDistance(sensorFrontRight);
  distanceFront = (distanceFrontLeft + distanceFrontRight) / 2;
}

bool checkLeftWall() {
  isWallLeft = distanceLeft < sensorSideWallDetect;
}

int checkRightWall() {
  isWallRight = distanceRight < sensorSideWallDetect;
}

int checkFrontWall() {
  isWallFront = distanceFront < sensorFrontWallDetect;
}

void readButtons() {
  int buttonLevel = analogRead(button);
  if (buttonLevel < 200) {
    if (buttonLevel < 100) {
      isRightButton = true;
    } 
    else {
      isLeftButton = true;
    }
  }
  else {
    isRightButton = false;
    isLeftButton = false;
  }
}

void readSensors() {
  readButtons();
  calcDistanceLeft();
  checkLeftWall();
  calcDistanceRight();
  checkRightWall();
  calcDistanceFront();
  checkFrontWall();
}

void initRefDistance() {
  int avgDistLeft = 0;
  int avgDistRight = 0;
  int counts = 10;
  for (int i = 0; i < counts; i++) {
    readSensors();
    avgDistLeft += distanceLeft;
    avgDistRight += distanceRight;
  }
  
  refDistanceLeft = avgDistLeft / counts;
  refDistanceRight = avgDistRight / counts;
  if (DEBUG) {
    Serial.print("distance ");
    Serial.print(avgDistLeft);
    Serial.print(" ");
    Serial.print(avgDistRight);
    Serial.print(" ");
    Serial.print(refDistanceLeft);
    Serial.print(" ");
    Serial.println(refDistanceRight);
  }
}

bool buttonPressed() {
  Serial.println(digitalRead(button));
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
  Serial.print(countRight);
  
  readSensors();
  Serial.print(" left sensor: ");
  Serial.print(distanceLeft);
  Serial.print(" right sensor:");
  Serial.print(distanceRight);
  Serial.print(" front sensor left: ");
  Serial.print(distanceFrontLeft);
  Serial.print(" front sensor right: ");
  Serial.print(distanceFrontRight);
  Serial.print(" front sensor: ");
  Serial.print(distanceFront);

  Serial.print(" Button left: ");
  Serial.println(isLeftButton);
  Serial.print(" Button right: ");
  Serial.println(isRightButton);
  delay(100);
}
