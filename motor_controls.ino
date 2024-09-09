// right encoder interrupt
void rightInterrupt()
{
  int count = countRight;
  count++;
  countRight = count;
}

// left encoder interrupt
void leftInterrupt()
{
  int count = countLeft;
  count++;
  countLeft = count;
}

void initMotors() {
  dirLeft = false;
  dirRight = false;
  digitalWrite(inLeft, dirLeft);
  digitalWrite(inRight, dirRight);
}

void changeDirLeft() {
  dirLeft = !dirLeft;
  analogWrite(pwmLeft, 0);
  delay(1);
  digitalWrite(inLeft, dirLeft);
  analogWrite(pwmLeft, vLeft);
}

void changeDirRight() {
  dirRight = !dirRight;
  analogWrite(pwmRight, 0);
  delay(1);
  digitalWrite(inRight, dirRight);
  analogWrite(pwmRight, vRight);
}

void setV() {
  if (vLeft > Vmax) {
    vLeft = Vmax;
  }
  if (vLeft < Vmin) {
    vLeft = Vmin;
  }
  if (vRight > Vmax) {
    vRight = Vmax;
  }
  if (vRight < Vmin) {
    vRight = Vmin;
  }
  if (onStart) {
    if (vLeftCur < vLeft)
    {
      vLeftCur += acsSpeed;
    }
    if (vRightCur < vRight)
    {
      vRightCur += acsSpeed;
    }
    if (vLeftCur > vLeft) {
      vLeftCur = vLeft;
    }
    if (vRightCur > vRight) {
      vRightCur = vRight;
    }
    if (vLeftCur > vLeft || vRightCur > vRight) {
      onStart = false;
    }
  } 
  else {
    vLeftCur = vLeft;
    vRightCur = vRight;
  }
  if (DEBUG) {
    Serial.print(vLeftCur);
    Serial.print(" <> ");
    Serial.println(vRightCur);
  }
  analogWrite(pwmLeft, vLeftCur);
  analogWrite(pwmRight, vRightCur);
}

void resetEncoders() {
  countLeft = 0;
  countRight = 0;
}

bool checkEncoders(bool strict) {
  if (strict) {
    // both counters should reach goal
    return countLeft < targetLeft || countRight < targetRight;
  }
  else {
    if (distanceFront <= sensorFrontWallTreshold) {
      return false;
    }
    // in case one counter reach goal we can move futher
    return countLeft < targetLeft && countRight < targetRight;
  }
}

void move(int targetL, int targetR, bool calcSensors) {
  targetLeft = targetL;
  targetRight = targetR;
  int error = 0;
  readSensors();
  while (checkEncoders(false)) {
    correctSpeed(calcSensors);
    setV();
  }
  errOldEnc = 0;
  errOldSens = 0;
  resetEncoders();
}

void moveStrict(int targetL, int targetR) {
  targetLeft = targetL;
  targetRight = targetR;
  while (checkEncoders(true)) {
    correctSpeed(false);
    if (countLeft > targetLeft) {
      vLeft = 0;
    }
    if (countRight > targetRight) {
      vRight = 0;
    }
    setV();
  }
  errOldEnc = 0;
  errOldSens = 0;
  resetEncoders();
}

void _forward(int targetL, int targetR, bool checkSensors) {
  if (dirLeft) {
    changeDirLeft();
  }
  if (dirRight) {
    changeDirRight();
  }
  if (checkSensors) {
    move(targetL, targetR, checkSensors);
  }
  else {
    moveStrict(targetL, targetR);
  }
  
}

void forward(int target) {
  _forward(target, target, true);
}

void forwardCurve(int targetL, int targetR) {
  _forward(targetL, targetR, false);
}

void turnCurve(bool isLeft) {
  if (isLeft) {
    forwardCurve(encodersPerSmallCircle, encodersPerBigCircleLeft);
  }
  else {
    forwardCurve(encodersPerBigCircleRight, encodersPerSmallCircle);
  }
}

void backward(int target) {
  V = vToCorect;
  if (dirLeft) {
    changeDirLeft();
  }
  if (dirRight) {
    changeDirRight();
  }
  move(target, target, false);
  V = Vdefault;
}

void turnTank(bool isLeft) {
  if (isLeft) {
    if (!dirLeft) {
      changeDirLeft();
    }
    if (dirRight) {
      changeDirRight();
    }
  }
  else {
    if (dirLeft) {
      changeDirLeft();
    }
    if (!dirRight) {
      changeDirRight();
    }
  }
  moveStrict(encodersPerTankTurn, encodersPerTankTurn);
}

void motorsStop() {
  onStart = false;
  vLeft = 0;
  vRight = 0;
  setV();
  delay(100);
  resetEncoders();
}

void testMotors() {
  // different V
  for (int i = 0; i < 255; i++) {
    vLeft = i;
    vRight = i;
    setV();
    delay(10);
  }
  for (int i = 255; i > 0; i--) {
    vLeft = i;
    vRight = i;
    setV();
    delay(10);
  }
  
  // backward
  vLeft = 100;
  vRight = 100;
  changeDirLeft();
  changeDirRight();
  setV();
  delay(1000);
  
  // spining
  changeDirLeft();
  delay(1000);
  
  // another direction
  changeDirLeft();
  changeDirRight();
  delay(1000);
}
