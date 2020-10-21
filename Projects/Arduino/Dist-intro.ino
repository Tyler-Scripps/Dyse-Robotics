const int trig = 11;
const int echo = 12;
const int led = 9;

float prevDist = 0;
float dist = 0;

void setup() {
  // put your setup code here, to run once:
  pinMode(led, OUTPUT);
  pinMode(trig, OUTPUT);
  pinMode(echo, INPUT);
  
}

float getDistance()
{
  digitalWrite(trig, HIGH);
  delay(100);
  digitalWrite(trig, LOW);
  int echoTime = pulseIn(echo, HIGH);
  return echoTime / 148.0;
}

void loop() {
  // put your main code here, to run repeatedly:
  prevDist = dist;
  dist = getDistance();
  if((dist - prevDist) > 5)
  {
    analogWrite(led, 100);
  }
  else
  {
    analogWrite(led, 0);
  }
}
