#include <Servo.h>

int potPos;
int servoPos;

Servo s1;


void setup() {
  // put your setup code here, to run once:
  s1.attach(9);
}

void loop() {
  // put your main code here, to run repeatedly:
  potPos = analogRead(A0);

  servoPos = map(potPos, 0, 1023, 20, 160);

  s1.write(servoPos);
  delay(2000);
}
