uint8_t maskUnknownU = 0b10000000;
uint8_t maskUnknownR = 0b01000000;
uint8_t maskUnknownD = 0b00100000;
uint8_t maskUnknownL = 0b00010000;

uint8_t maskWallU = 0b00001000;
uint8_t maskWallR = 0b00000100;
uint8_t maskWallD = 0b00000010;
uint8_t maskWallL = 0b00000001;

// think on shifts
uint8_t dirToUnknownMasks[4][3] = {
  maskUnknownL, maskUnknownU, maskUnknownR, 
  maskUnknownU, maskUnknownR, maskUnknownD,
  maskUnknownR, maskUnknownD, maskUnknownL,
  maskUnknownD, maskUnknownL, maskUnknownU, 
};

uint8_t dirToWallMasks[4][3] = {
  maskWallL, maskWallU, maskWallR, 
  maskWallU, maskWallR, maskWallD,
  maskWallR, maskWallD, maskWallL,
  maskWallD, maskWallL, maskWallU, 
};

uint8_t wallMasks[4] = {maskWallU, maskWallR, maskWallD, maskWallL}; // mapped with positionChanges
uint8_t wallCheckedMasks[4] = {maskUnknownU, maskUnknownR, maskUnknownD, maskUnknownL}; // mapped with positionChanges

bool checkPosition(int8_t y, int8_t x) {
  return y >= 0 && y < mazeShapeY && x >= 0 && x < mazeShapeX;
}

void turnDirection(bool isLeft) {
  if (isLeft) {
    robotDirection = (robotDirection + 3) % 4;
  }
  else {
    robotDirection = (robotDirection + 1) % 4;
  }
  if (DEBUG) {
    Serial.print(robotDirection);
    Serial.print(" ");
  }
}

void movePosition() {
  int8_t deltaY = positionChanges[robotDirection][0];
  int8_t deltaX = positionChanges[robotDirection][1];
  robotPositionY += deltaY;
  robotPositionX += deltaX;
  if (DEBUG) {
    Serial.print("deltas ");
    Serial.print(deltaY);
    Serial.print(" ");
    Serial.print(deltaX);
    Serial.print(" ");
    Serial.print("robot position ");
    Serial.print(robotPositionY);
    Serial.print(" ");
    Serial.print(robotPositionX);
    Serial.println(" ");
  }
}

struct Pair {
  int8_t y;
  int8_t x;
};

ArduinoQueue<Pair> neighbours(4);

void calcNeighbours(int8_t y, int8_t x, bool strict = true) {
  while(!neighbours.isEmpty()) {
    neighbours.dequeue();
  }
  Pair pos;
  for(int dir = 0; dir < 4; dir++) {
    int8_t neighbourY = y + positionChanges[dir][0];
    int8_t neighbourX = x + positionChanges[dir][1];
    uint8_t isWall;
    if (strict) {
      isWall = walls[y][x] & wallMasks[dir];
    }
    else {
      isWall = 0;
    }
    if (checkPosition(neighbourY, neighbourX) && !isWall) {
      pos = {neighbourY, neighbourX};
      neighbours.enqueue(pos);
    }
  }
}

char directionToChar[4] = {'^', '>', 'v', '<'};

void initMaze() {
  for (int y = 0; y < mazeShapeY; y++) {
    for (int x = 0; x < mazeShapeX; x++) {
      maze[y][x] = -1;
    }
  }
  ArduinoQueue<Pair> positions(mazeShapeX * mazeShapeY);
  Pair pos;
  pos = {finishPositionY, finishPositionX};
  positions.enqueue(pos);
  
  maze[finishPositionY][finishPositionX] = 0;
  
  while (!positions.isEmpty()) {
    pos = positions.dequeue();
    int minNeighbour = mazeShapeX * mazeShapeY;
    calcNeighbours(pos.y, pos.x);
    while(!neighbours.isEmpty()) {
      Pair neighbour = neighbours.dequeue();
      int neighbValue = maze[neighbour.y][neighbour.x];
      if (neighbValue == -1){
        positions.enqueue(neighbour);
      }
      else {
        minNeighbour = min(minNeighbour, neighbValue);
      }
    }
    if (maze[pos.y][pos.x] == -1) {
      maze[pos.y][pos.x] = minNeighbour + 1;
    }
  } 
  if (DEBUG) {
    for (int y = 0; y < mazeShapeY; y++) {
      for (int x = 0; x < mazeShapeX; x++) {
        if (y == robotPositionY && x == robotPositionX) {
          Serial.print(directionToChar[robotDirection]);
        }
        else {
          Serial.print(maze[y][x]);
        }
        Serial.print(" ");
      }
      Serial.println("");
    }
  }
}

void floodfill() {
  if (robotPositionY == finishPositionY && robotPositionX == finishPositionX) {
    return;
  }
  ArduinoQueue<Pair> positions(mazeShapeX * mazeShapeY);
  Pair pos;
  pos = {robotPositionY, robotPositionX};
  positions.enqueue(pos);
  while (!positions.isEmpty()) {
    pos = positions.dequeue();
    int curVal = maze[pos.y][pos.x];
    if (curVal == 0) {
      continue;
    }
    if (DEBUG) {
      Serial.print("check ");
      Serial.print(pos.y);
      Serial.print(" ");
      Serial.println(pos.x);
    }
    int minNeighbour = mazeShapeX * mazeShapeY;
    calcNeighbours(pos.y, pos.x);
    while(!neighbours.isEmpty()) {
      Pair neighbour = neighbours.dequeue();
      if (DEBUG) {
        Serial.print(" check neighbor ");
        Serial.print(neighbour.y);
        Serial.print(" ");
        Serial.print(neighbour.x);
      }
      int neighbValue = maze[neighbour.y][neighbour.x];
      minNeighbour = min(minNeighbour, neighbValue);
      if (DEBUG) {
        Serial.print(" neighbValue ");
        Serial.println(neighbValue);
      }
    }
    if (curVal <= minNeighbour) {
      maze[pos.y][pos.x] = minNeighbour + 1;
      calcNeighbours(pos.y, pos.x, false);
      while(!neighbours.isEmpty()) {
        Pair neighbour = neighbours.dequeue();
        positions.enqueue(neighbour);
      }
    }
  }
  if (DEBUG) {
    for (int y = 0; y < mazeShapeY; y++) {
      for (int x = 0; x < mazeShapeX; x++) {
        if (y == robotPositionY && x == robotPositionX) {
          Serial.print(directionToChar[robotDirection]);
        }
        else {
          Serial.print(maze[y][x]);
        }
        Serial.print(" ");
      }
      Serial.println("");
    }
  }
}

void setWalls() {
  readSensors();
  uint8_t curCell = walls[robotPositionY][robotPositionX];
  uint8_t leftDirection = (robotDirection + 3) % 4;
  uint8_t rightDirection = (robotDirection + 1) % 4;
  uint8_t directions[3] = {leftDirection, robotDirection, rightDirection};
  bool detectedWalls[3] = {isWallLeft, isWallFront, isWallRight};
  for (int i = 0; i < 3; i++) {
    uint8_t dirUknownMask = dirToUnknownMasks[robotDirection][i];
    if (!(curCell & dirUknownMask)){
      curCell |= dirUknownMask;
      if (detectedWalls[i]) {
        curCell |= dirToWallMasks[robotDirection][i];
      }
      int8_t posNeighbourY = robotPositionY + positionChanges[directions[i]][0];
      int8_t posNeighbourX = robotPositionX + positionChanges[directions[i]][1];
      if (checkPosition(posNeighbourY, posNeighbourX)) {
        uint8_t neighbourCell = walls[posNeighbourY][posNeighbourX];
        uint8_t oppositeDirection = (directions[i] + 2) % 4;
        
        neighbourCell |= dirToUnknownMasks[oppositeDirection][1];
        if (detectedWalls[i]) {
          neighbourCell |= dirToWallMasks[oppositeDirection][1];
        }
        walls[posNeighbourY][posNeighbourX] = neighbourCell;
      }
    }
  }
  walls[robotPositionY][robotPositionX] = curCell;
  floodfill();
}

#define _moveFoward 0
#define _turnLeft 1
#define _turnRight 2
#define _start 3

ArduinoQueue<int> path(mazeShapeY * mazeShapeX * 2); // multiply by 2 for turns

void calcShortestPath() {
  path.enqueue(_start);
  int8_t curPosY = robotPositionYStart;
  int8_t curPosX = robotPositionXStart;
  int curVal = maze[curPosY][curPosX];
  uint8_t curDir = robotDirectionStart;
  
  while (curVal != 0) {
    calcNeighbours(curPosY, curPosX);
    int8_t targetDiffY = 0;
    int8_t targetDiffX = 0;
    uint8_t targetDirection = curDir;
    while(!neighbours.isEmpty()) {
      Pair neighbour = neighbours.dequeue();
      // copy paste
      int valDiff = curVal - maze[neighbour.y][neighbour.x];
      targetDiffY = neighbour.y - curPosY;
      targetDiffX = neighbour.x - curPosX;
      for (int i = 0; i < 4; i++) {
        if (positionChanges[i][0] == targetDiffY && positionChanges[i][1] == targetDiffX) {
          targetDirection = i;
          break;
        }
      }
      uint8_t forwardWall = walls[curPosY][curPosX] & wallMasks[targetDirection];
      uint8_t wallChecked = walls[curPosY][curPosX] & wallCheckedMasks[targetDirection];
      if ( valDiff == 1 && !(forwardWall) && wallChecked){
        // copy paste
        uint8_t directionDiff = (curDir + 4 - targetDirection) % 4;
        if (directionDiff == 0) {
          path.enqueue(_moveFoward);
        }
        if (directionDiff == 1) {
          path.enqueue(_turnLeft);
        }
        if (directionDiff == 3) {
          path.enqueue(_turnRight);
        }
        curPosY = neighbour.y;
        curPosX = neighbour.x;
        curVal = maze[neighbour.y][neighbour.x];
        curDir = targetDirection;
        Serial.print("curPosY ");
        Serial.print(curPosY);
        Serial.print(" curPosX ");
        Serial.print(curPosX);
        Serial.print(" curVal ");
        Serial.print(curVal);
        Serial.print(" curDir ");
        Serial.println(curDir);
        break;
      }
    }
  }
}

void decideMove() {
  if (robotPositionY == finishPositionY && robotPositionX == finishPositionX){
    motorsStop();
    if (DEBUG) {
      Serial.println("Finish");
    }
    isFinish = true;
    calcShortestPath();
    resetEncoders();
    return;
  }
  if (isStart) {
    if (DEBUG) {
      Serial.println("To center");
    }
    else {
      forward(encodersToCenter);
    }
    isStart = false;
  }
  int curVal = maze[robotPositionY][robotPositionX];
  int8_t deltaY = positionChanges[robotDirection][0];
  int8_t deltaX = positionChanges[robotDirection][1];
  int valDiff = curVal - maze[robotPositionY + deltaY][robotPositionX + deltaX];
  uint8_t forwardWall = walls[robotPositionY][robotPositionX] & wallMasks[robotDirection];
  if ( valDiff == 1 && !(forwardWall)){
    if (DEBUG) {
      Serial.println("Forward");
      waitToStart();
    }
    else {
      forward(encodersPerCell);
    }
    movePosition();
  }
  else {
    int8_t targetDiffY = 0;
    int8_t targetDiffX = 0;
    calcNeighbours(robotPositionY, robotPositionX);
    while(!neighbours.isEmpty()) {
      Pair neighbour = neighbours.dequeue();
      if (curVal - maze[neighbour.y][neighbour.x] == 1) {
        targetDiffY = neighbour.y - robotPositionY;
        targetDiffX = neighbour.x - robotPositionX;
        break;
      }
    }
    uint8_t targetDirection = robotDirection;
    for (int i = 0; i < 4; i++) {
      if (positionChanges[i][0] == targetDiffY && positionChanges[i][1] == targetDiffX) {
        targetDirection = i;
        break;
      }
    }
    uint8_t directionDiff = (robotDirection + 4 - targetDirection) % 4;
    if (directionDiff == 0) {
      if (DEBUG) {
        Serial.println("Break");
      }
      motorsStop();
      isBreak = true;
    }
    if (directionDiff == 1) {
      // turn left
      if (DEBUG) {
        Serial.println("Left");
        waitToStart();
      }
      else {
        turnTank(true);
        forward(encodersPerCell);
      }
      turnDirection(true);
      movePosition();
      setWalls();
    }
    if (directionDiff == 2) {
      // turn around
      if (DEBUG) {
        Serial.println("Around");
        waitToStart();
      }
      else {
        turnTank(true);
        turnTank(true);
        if (walls[robotPositionY][robotPositionX] & wallMasks[robotDirection])
        {
          backward(encodersToCorrect);
        }
      }
      turnDirection(true);
      turnDirection(true);
      isStart = true;
    }
    if (directionDiff == 3) {
      // turn right
      if (DEBUG) {
        Serial.println("Right");
        waitToStart();
      }
      else {
        turnTank(false);
        forward(encodersPerCell);  
      }
      turnDirection(false);
      movePosition();
    }
  }
  //motorsStop();
  setWalls();
}

void runShort() {
  while (!path.isEmpty()) {
    int dir = path.dequeue();
    if (dir == 0) {
      forward(encodersPerCell);
    }
    else if (dir == 1) {
      turnTank(true);
      forward(encodersPerCell);
    }
    else if (dir == 2) {
      turnTank(false);
      forward(encodersPerCell);
    }
    else if (dir == 3) {
      forward(encodersToCenter);
    }
  }
  motorsStop();
  exit(0);
}

void updateHistory() {
  positionYHistory[historyIndex] = robotPositionY;
  positionXHistory[historyIndex] = robotPositionX;
  ditectionHistory[historyIndex] = directionToChar[robotDirection];
  Serial.print("history update ");
  Serial.print(positionYHistory[historyIndex]);
  Serial.print(" ");
  Serial.print(positionXHistory[historyIndex]);
  Serial.print(" ");
  Serial.print(ditectionHistory[historyIndex]);
  Serial.println(" ");
  historyIndex += 1;
}

void printWalls() {
  Serial.println("=========================");
  for (int y = 0; y < mazeShapeY; y++) {
    for (int x = 0; x < mazeShapeX; x++) {
      Serial.print(walls[y][x]);
      Serial.print(" ");
    }
    Serial.println("");
  }
  Serial.println("=========================");
  for (int i = 0; i < historyIndex; i++) {
    Serial.print(positionYHistory[i]);
    Serial.print(" ");
  }
  Serial.println("");
  for (int i = 0; i < historyIndex; i++) {
    Serial.print(positionXHistory[i]);
    Serial.print(" ");
  }
  Serial.println("");
  for (int i = 0; i < historyIndex; i++) {
    Serial.print(ditectionHistory[i]);
    Serial.print(" ");
  }
}
