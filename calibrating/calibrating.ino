#include <Servo.h>

int throttle;

void setup()
{
  Serial.begin(9600);
  DDRA |= B11110000;
}

void loop()
{
  if (Serial.available() > 0)
  {
    throttle = Serial.parseInt();

    while (Serial.available() > 0)
    {
      Serial.read();
    }
  }

  PORTA |= B11110000;
  delayMicroseconds(throttle);
  PORTA &= B00001111;
  delayMicroseconds(throttle);
  Serial.println(throttle);
}
