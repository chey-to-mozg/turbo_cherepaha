void setup() {
  pinMode(LeftEncA, INPUT);
//  pinMode(LeftEncB, INPUT);
  pinMode(RightEncA, INPUT);
//  pinMode(RightEncB, INPUT);

  pinMode(sensorLeft, INPUT);
  pinMode(sensorRight, INPUT);
  pinMode(sensorFrontLeft, INPUT);
  pinMode(sensorFrontRight, INPUT);
  pinMode(buttonPin, INPUT);

  EIMSK |= (1 << INT0) | ( 1 << INT1 );    // Enable both interrupts
  EICRA |= (1 << ISC11) | ( 1 << ISC10 );  // RISING edge INT1
  EICRA |= (1 << ISC01) | ( 1 << ISC00 );  // RISING edge INT0

  Serial.begin(9600);
}

void test_sensors() {
  Serial.print("left encoder: ")
  Serial.print(leftCount);
  Serial.print(" right encoder:");
  Serial.println(rightCount);
  
  Serial.print("left sensor: ")
  Serial.print(isLeftWall());
  Serial.print(" right sensor:");
  Serial.print(isRightWall());
  Serial.print("front sensor: ")
  Serial.println(sFrontWall());

  Serial.print("Button: ")
  Serial.println(buttonPressed());
}

void loop() {
  test_sensors();
  delay(100);
}
