int calcLoopDelay() {
  long curTime = millis();
  loopInterval = curTime - prevClock;
  prevClock = curTime;
}

float calcEncodersError() {
  int error = (targetLeft - countLeft) - (targetRight - countRight);
  
  float pTerm = error * kPEnc;
  float dTerm = (error - errOldEnc) * kDEnc;
  dTerm /= loopInterval;
  float totError = pTerm + dTerm;
  
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

void correctSpeed(bool calcSensors) {
  float encoderError = 0;
  float sensorError = 0;
  
  vLeft = V;
  vRight = V;
  encoderError = calcEncodersError();
  
  if (calcSensors) {
    sensorError = calcSensorError();
  }
  
  float totError = sensorError - encoderError;

  vLeft -= totError;
  vRight += totError;
  
  if(DEBUG) {
    Serial.print(" totError ");
    Serial.println(totError);
  }
}
