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
  visited[robotPositionY][robotPositionX] = true;
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

void calcNeighbours(int8_t y, int8_t x, bool strict) {
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

void initMaze(bool reset) {
  for (int y = 0; y < mazeShapeY; y++) {
    for (int x = 0; x < mazeShapeX; x++) {
      maze[y][x] = -1;
    }
  }

  bool _visited[mazeShapeY][mazeShapeX] = {0};
  ArduinoQueue<Pair> positions(mazeShapeX * mazeShapeY);
  
  calcNeighbours(finishPositionY, finishPositionX, !reset);
  while(!neighbours.isEmpty()) {
    Pair neighbour = neighbours.dequeue();
    positions.enqueue(neighbour);
  }
  maze[finishPositionY][finishPositionX] = 0;

  Pair pos;
  while (!positions.isEmpty()) {
    pos = positions.dequeue();
    if (_visited[pos.y][pos.x]) {
      continue;
    }
    _visited[pos.y][pos.x] = true;
    int minNeighbour = mazeShapeX * mazeShapeY;
    calcNeighbours(pos.y, pos.x, !reset);
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
    printMaps();
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
    int minNeighbour = mazeShapeX * mazeShapeY;
    calcNeighbours(pos.y, pos.x, true);
    while(!neighbours.isEmpty()) {
      Pair neighbour = neighbours.dequeue();
      int neighbValue = maze[neighbour.y][neighbour.x];
      minNeighbour = min(minNeighbour, neighbValue);
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
    printMaps();
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

#define _moveForward 0
#define _turnLeft 1
#define _turnRight 2
#define _start 3

uint8_t path[mazeShapeY * mazeShapeX] = {0};
uint16_t pathIndex = 0;
uint16_t pathLen = 0;

void calcShortestPath() {
  for (int y = 0; y < mazeShapeY; y++) {
    for (int x = 0; x < mazeShapeX; x++) {
      if (!visited[y][x]) {
        walls[y][x] = 255;
        maze[y][x] = -1;
      }
    }
  }
  
  initMaze(false);
  
  path[pathIndex] = _start;
  pathIndex += 1;
  
  int8_t curPosY = robotPositionYStart;
  int8_t curPosX = robotPositionXStart;
  int curVal = maze[curPosY][curPosX];
  uint8_t curDir = robotDirectionStart;
  
  while (curVal != 0) {
    calcNeighbours(curPosY, curPosX, true);
    int8_t targetDiffY = 0;
    int8_t targetDiffX = 0;
    uint8_t targetDirection = curDir;
    int minDiff = mazeShapeY * mazeShapeX;
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
      if ( valDiff == 1 && !forwardWall) {
        // copy paste
        uint8_t directionDiff = (curDir + 4 - targetDirection) % 4;
        if (directionDiff == 0) {
          if (DEBUG) {
            Serial.println("Forward");
          }
          path[pathIndex] = _moveForward;
        }
        else if (directionDiff == 1) {
          if (DEBUG) {
            Serial.println("Left");
          }
          path[pathIndex] = _turnLeft;
        }
        else if (directionDiff == 3) {
          if (DEBUG) {
            Serial.println("Right");
          }
          path[pathIndex] = _turnRight;
        }
        pathIndex += 1;
        curPosY = neighbour.y;
        curPosX = neighbour.x;
        curVal = maze[neighbour.y][neighbour.x];
        curDir = targetDirection;
        if (DEBUG) {
          Serial.print("curPosY ");
          Serial.print(curPosY);
          Serial.print(" curPosX ");
          Serial.print(curPosX);
          Serial.print(" curVal ");
          Serial.print(curVal);
          Serial.print(" curDir ");
          Serial.println(curDir);
        }
        break;
      }
    }
  }
  path[pathIndex] = _moveForward;
  pathIndex += 1;
  pathLen = pathIndex;
}

void decideMove() {
  if (robotPositionY == finishPositionY && robotPositionX == finishPositionX){
    if (DEBUG) {
      Serial.println("Finish");
    }
    else {
      forward(encoderPerHalfCell);
      motorsStop();
    }
    isFinish = true;
    calcShortestPath();
    return;
  }
  if (isStart) {
    onStart = true;
    if (DEBUG) {
      Serial.println("To center");
    }
    else {
      forward(encodersToCenter);
//      if (smoothSearch) {
//        isCenter = true;
//      }
    }
    onStart = false;
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
      if (isCenter) {
        forward(encoderPerHalfCell);
        isCenter = false;
      }
      else {
        forward(encodersPerCell);
      }
    }
    movePosition();
  }
  else {
    int8_t targetDiffY = 0;
    int8_t targetDiffX = 0;
    calcNeighbours(robotPositionY, robotPositionX, true);
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
    }
    if (directionDiff == 2) {
      // turn around
      if (DEBUG) {
        Serial.println("Around");
        waitToStart();
      }
      else {
        forward(encoderPerHalfCell);
        turnTank(true);
        turnTank(true);
        if (walls[robotPositionY][robotPositionX] & wallMasks[robotDirection])
        {
          backward(encoderPerHalfCell);
          isStart = true;
        }
      }
      turnDirection(true);
      turnDirection(true);
      
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
  setWalls();
}

void runShort() {
  onStart = true;
  sensorReads = 3;
  pathIndex = 0;
  while (pathIndex < pathLen) {
    uint8_t dir = path[pathIndex];

    if (pathIndex + 1 < pathLen && path[pathIndex + 1] != _moveForward) {
      V = VfastTurn;
    }
    else {
      V = VfastForward;
    }

    pathIndex += 1;
    
    if (dir == _moveForward) {
      if (isCenter) {
//        forward(encoderPerHalfCell);
        forward(encodersPerCell);
        isCenter = false;
      }
      else {
        forward(encodersPerCell);
      }
    }
    else if (dir == _turnLeft) {
      V = Vdefault;
//      turnCurve(true);
       turnTank(true);
       forward(encodersPerCell);
    }
    else if (dir == _turnRight) {
      V = Vdefault;
//      turnCurve(false);
      turnTank(false);
       forward(encodersPerCell);
    }
    else if (dir == _start) {
      forward(encodersToCenter);
      isCenter = true;
      onStart = false;
    }
  }
  V = Vdefault;
  forward(encodersPerCell);
  motorsStop();
}

char directionToChar[4] = {'^', '>', 'v', '<'};

void printMaps() {
  Serial.println("=========================");
  for (int y = 0; y < mazeShapeY; y++) {
    for (int x = 0; x < mazeShapeX; x++) {
      if (walls[y][x] / 10 < 1) {
        Serial.print("  ");
      }
      else if (walls[y][x] / 100 < 1) {
        Serial.print(" ");
      }
      Serial.print(walls[y][x]);
      Serial.print(" ");
    }
    Serial.println("");
  }
  Serial.println("=========================");
  for (int y = 0; y < mazeShapeY; y++) {
    for (int x = 0; x < mazeShapeX; x++) {
      Serial.print(visited[y][x]);
      Serial.print(" ");
    }
    Serial.println("");
  }
  Serial.println("=========================");
  for (int y = 0; y < mazeShapeY; y++) {
    for (int x = 0; x < mazeShapeX; x++) {
      if (maze[y][x] / 10 < 1) {
        Serial.print("  ");
      }
      else if (maze[y][x] / 100 < 1) {
        Serial.print(" ");
      }
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
  Serial.println("=========================");
}


//#define _moveForward 0
//#define _turnLeft 1
//#define _turnRight 2
//#define _start 3
// test fast move
void setPath() {
  path[0] = 3;
  path[1] = 0;
  path[2] = 0;
  path[3] = 0;
  path[4] = 2;
  path[5] = 0;
  path[6] = 0;
  path[7] = 1;
  path[8] = 2;
  path[9] = 0;
  path[10] = 0;
  path[11] = 1;
  path[12] = 1;
  pathLen = 13;
}
