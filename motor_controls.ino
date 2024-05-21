const int LeftEncA = 2;
const int RightEncA = 3;

const int lefrMotorDir = 7;
const int leftMotorPwm = 9;
const int rightMotorDir = 8;
const int rightMotorPwm = 10;

bool leftDir = true;
bool rightDir = true;

volatile int leftCount;
volatile int rightCount;

// left encoder interrupt
ISR( INT0_vect )
{
  int count = leftCount;
  if (leftDir) {
    count++;
  } else {
    count--;
  }
  leftCount = count;
}

// right encoder interrupt
ISR( INT1_vect )
{
  int count = rightCount;
  if (rightDir) {
    count++;
  } else {
    count--;
  }
  rightCount = count;
}
