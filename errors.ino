int calcLoopDelay() {
  long curTime = millis();
  loopInterval = curTime - prevClock;
  prevClock = curTime;
}

float calcEncodersError() {
  int error = countLeft - countRight;
  
  float pTerm = error * kPEnc;
  float dTerm = (error - errOldEnc) * kDEnc;
  dTerm /= loopInterval;
  float totError = pTerm + dTerm;
  
  if (DEBUG) {
    Serial.print("enc err ");
    Serial.println(totError);
  }
  
  errOldEnc = error;
  return totError;
}

float calcSensorError() {
  readSensors();
  int error = 0;
  int errorLeft = distanceLeft - refDistanceLeft;
  int errorRight = distanceRight - refDistanceRight;
  
  if (isWallLeft && isWallRight) {
    error = errorLeft - errorRight;
  }
  else if (isWallLeft) {
    error = 2 * errorLeft;
  }
  else if (isWallRight) {
    error = -2 * errorRight;
  }
  
  float pTerm = error * kPSens;
  float dTerm = (error - errOldEnc) * kDSens;
  dTerm /= loopInterval;
  float totError = pTerm + dTerm;
  
  if (!DEBUG) {
    Serial.print("sens err ");
    Serial.print(" ");
    Serial.print(errorLeft);
    Serial.print(" ");
    Serial.print(errorRight);
    Serial.print(" ");
    Serial.print(error);
    Serial.print(" ");
    Serial.print(isWallRight);
    Serial.print(" ");
    Serial.println(totError);
  }
  
  errOldSens = error;
  return totError;
}

int calcError(bool calcSensors) {
  float encoderError = calcEncodersError();
  float sensorError = 0;
  
  if (calcSensors) {
    sensorError = calcSensorError();
  }
  
  float totError = encoderError + sensorError;
  
  if (DEBUG) {
    Serial.print("total err ");
    Serial.println(totError);
  }
  
  return totError;
}
