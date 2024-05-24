// right encoder interrupt
ISR( INT0_vect )
{
  int count = countRight;
  count++;
  countRight = count;
}

// left encoder interrupt
ISR( INT1_vect )
{
  int count = countLeft;
  count++;
  countLeft = count;
}

void initMotors() {
  dirLeft = true;
  dirRight = true;
  digitalWrite(in1Left, !dirLeft);
  digitalWrite(in2Left, dirLeft);
  digitalWrite(in1Right, !dirRight);
  digitalWrite(in2Right, dirRight);
}

void changeDirLeft() {
  dirLeft = !dirLeft;
  analogWrite(pwmLeft, 0);
  delay(1);
  digitalWrite(in1Left, !dirLeft);
  digitalWrite(in2Left, dirLeft);
  analogWrite(pwmLeft, vLeft);
}

void changeDirRight() {
  dirRight = !dirRight;
  analogWrite(pwmRight, 0);
  delay(1);
  digitalWrite(in1Right, !dirRight);
  digitalWrite(in2Right, dirRight);
  analogWrite(pwmRight, vRight);
}

void setV() {
  if (vLeft > 255) {
    vLeft = 255;
  }
  if (vLeft < 0) {
    vLeft = 0;
  }
  if (vRight > 255) {
    vRight = 255;
  }
  if (vRight < 0) {
    vRight = 0;
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
    if (vLeftCur > vLeft || vRightCur > vRight) {
      vLeftCur = vLeft;
      vRightCur = vRight;
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

bool checkEncoders(int targetLeft, int targetRight) {
  if (DEBUG) {
    Serial.print(countLeft);
    Serial.print(" < ");
    Serial.print(targetLeft);
    Serial.print("  ");
    Serial.print(countRight);
    Serial.print(" < ");
    Serial.println(targetRight);
  }
  return countLeft < targetLeft && countRight < targetRight;
}

void move(int targetLeft, int targetRight, bool calcSensors) {
  onStart = true;
  resetEncoders();
  int error = 0;
  while (checkEncoders(targetLeft, targetRight)) {
    error = calcError(calcSensors);
    vLeft = V - error;
    vRight = V + error;
    setV();
  }
}

void forward(int target) {
  if (!dirLeft) {
    changeDirLeft();
  }
  if (!dirRight) {
    changeDirRight();
  }
  move(target, target, true);
}

void backward(int target) {
  if (dirLeft) {
    changeDirLeft();
  }
  if (dirRight) {
    changeDirRight();
  }
  move(target, target, true);
}

void turnTank(bool isLeft) {
  if (isLeft) {
    if (dirLeft) {
      changeDirLeft();
    }
    if (!dirRight) {
      changeDirRight();
    }
  }
  else {
    if (!dirLeft) {
      changeDirLeft();
    }
    if (dirRight) {
      changeDirRight();
    }
  }
  move(encodersPerTankTurn, encodersPerTankTurn, false);
}

void motorsStop() {
  onStart = false;
  vLeft = 0;
  vRight = 0;
  setV();
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
