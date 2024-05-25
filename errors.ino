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
    Serial.print(" enc left ");
    Serial.print(countLeft);
    Serial.print(" enc right ");
    Serial.print(countRight);
    Serial.print(" enc err ");
    Serial.print(totError);
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
  
  if (DEBUG) {
    Serial.print(" err left ");
    Serial.print(errorLeft);
    Serial.print(" err right ");
    Serial.print(errorRight);
    Serial.print(" wall left ");
    Serial.print(isWallLeft);
    Serial.print(" wall right ");
    Serial.print(isWallRight);
    Serial.print(" sens error ");
    Serial.print(totError);
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
    Serial.print(" total err ");
    Serial.println(totError);
  }
  
  return totError;
}
