void setup() {
//  pinMode(A0, INPUT);
  pinMode(A1, INPUT);
//  pinMode(10, INPUT);
//  pinMode(11, INPUT);
//  pinMode(12, INPUT);
//  pinMode(13  , INPUT);
  Serial.begin(9600);
}

void loop() {
//  int leftSensor = analogRead(A0);
  int rightSensor = analogRead(A1);
//  int enc1 = digitalRead(10);
//  int enc2 = digitalRead(11);
//  int enc3 = digitalRead(12);
//  int enc4 = digitalRead(13);
//  Serial.print(leftSensor);
//  Serial.print(" ");
  Serial.println(rightSensor);
//  Serial.print(enc1);
//  Serial.print(" ");
//  Serial.print(enc2);
//  Serial.print(" ");
//  Serial.print(enc3);
//  Serial.print(" ");
//  Serial.println(enc4);
  delay(500);
}
