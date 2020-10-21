int photoresister = 0; // brightness of room
int threshold = 750; // if photoresister reads below led will turn on

void setup() {
  Serial.begin(9600);
  pinMode(13, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  photoresister = analogRead(A0);

  Serial.println(photoresister);

  if(photoresister < threshold)
  {
    digitalWrite(13, HIGH);
  }
  else
  {
    digitalWrite(13, LOW);
  }
  delay(1000);
}
