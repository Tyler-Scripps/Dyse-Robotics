#include <Wire.h>

int data = 13;

void setup() {
  // put your setup code here, to run once:
  Wire.begin(0x04);
  Wire.onRequest(sendData);
}

void sendData()
{
  Wire.write(data);
}

void loop() {
  // put your main code here, to run repeatedly:

}
