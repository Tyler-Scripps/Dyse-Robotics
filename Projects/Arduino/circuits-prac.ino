
int potPos;

void setup() {
  Serial.begin(9600); // init serial connection with computer
  pinMode(13, OUTPUT); // declare pin 13 as output

}

void loop() {

  potPos = analogRead(A0); // value of potentiometer 0-1023

  Serial.println(potPos);
  
  digitalWrite(13, HIGH);

  delay(potPos * 1.5); // potentiometer controls delay duration

  digitalWrite(13, LOW);

  delay(potPos * 1.5);

}
