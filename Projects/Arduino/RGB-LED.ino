int photores = 0;
int potPos = 0;
int thresh = 750;

int r = 9;
int g = 10;
int b = 11;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);

  pinMode(r, OUTPUT);
  pinMode(g, OUTPUT);
  pinMode(b, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  
  photores = analogRead(A0);
  potPos = analogRead(A1);
  Serial.print("PhotoResistor: ");
  Serial.println(photores);
  Serial.print("Potentiometer: ");
  Serial.println(potPos);

  int r_mask = ((1023 - potPos) % 10);
  int g_mask = (potPos % 5);
  int b_mask = (potPos % 3);
  if(photores < thresh)
  {
    analogWrite(r, r_mask);
    analogWrite(g, g_mask);
    analogWrite(b, b_mask);
    delay(1000);
  }
}
