const float MCU_VOLTAGE = 5.0;

int calcDistance(int sensor) {
  int rawData = analogRead(sensor);
  float normalizedData = rawData * MCU_VOLTAGE / 1023.0;
  if (DEBUG) {
    Serial.print(rawData);
    Serial.print(" ");
    Serial.println(normalizedData);
  }
  return (33.9 + -69.5*(normalizedData) + 62.3*pow(normalizedData,2) + -25.4*pow(normalizedData,3) + 3.83*pow(normalizedData,4)) * 10; // in millimeters
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
  delay(100);
}

void checkVoltage() {
  digitalWrite(alarm, (analogRead(volt) < 770 ? 1 : 0));
}
