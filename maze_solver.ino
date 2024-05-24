sizeX = 2
sizeY = 2

maze[sizeY][sizeX] = {0}
walls[sizeY][sizeX] = {0}

curY = 0
curX = 0

finishY = 0
finishX = 0

durU = 0
dirR = 1
dirD = 2
dirL = 3

changes = {
  1, 0,
  0, 1,
  -1, 0,
  0, -1,
}

robotDir = dirU

cellsQueue = {}

maskUnchekU = 0x10000000
maskUnchekR = 0x01000000
maskUnchekD = 0x00100000
maskUnchekL = 0x00010000

maskWallU = 0x00001000
maskWallR = 0x00000100
maskWallD = 0x00000010
maskWallL = 0x00000001

// think on shifts
dirToUncheckMasks = {
  maskUnchekL, maskUnchekU, maskUnchekR, 
  maskUnchekU, maskUnchekR, maskUnchekD,
  maskUnchekR, maskUnchekD, maskUnchekL,
  maskUnchekD, maskUnchekL, maskUnchekU, 
}

dirToUncheckMasks = {
  maskWallL, maskWallU, maskWallR, 
  maskWallU, maskWallR, maskWallD,
  maskWallR, maskWallD, maskWallL,
  maskWallD, maskWallL, maskWallU, 
}

void setWalls() {
  mask
}
