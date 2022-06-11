#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

RF24 radio(7, 8);

const uint64_t address = 0xE8E8F0F0E1LL;

struct QuadData {
  byte throttle;
  byte pitch;
  byte roll;
  byte yaw;
};

QuadData quad_data;

void reset_data() {
  quad_data.throttle = 0;
  quad_data.pitch = 127;
  quad_data.roll = 127;
  quad_data.yaw = 127;
}

int map_joystick_values(int val, int lower, int middle, int upper, bool reverse)
{
  val = constrain(val, lower, upper);
  if ( val < middle )
    val = map(val, lower, middle, 0, 128);
  else
    val = map(val, middle, upper, 128, 255);
  return ( reverse ? 255 - val : val );
}

void setup()
{
  Serial.begin(57600);
  radio.begin();
  radio.setAutoAck(false);
  radio.setPALevel(RF24_PA_LOW);
  radio.setChannel(110);
  radio.openWritingPipe(address);
  radio.setDataRate(RF24_250KBPS);
  radio.stopListening();
  reset_data();
}


void loop()
{
  quad_data.throttle = map_joystick_values( analogRead(A0), 0, 501, 1024, false );
  quad_data.yaw      = map_joystick_values( analogRead(A1), 0, 506, 1024, false );
  quad_data.pitch    = map_joystick_values( analogRead(A2), 0, 517, 1024, false );
  quad_data.roll     = map_joystick_values( analogRead(A3), 0, 510, 1024, false );

  radio.write(&quad_data, sizeof(QuadData));
}

void printJoystickValues() {
  Serial.print(analogRead(A0));
  Serial.print(' ');
  Serial.print(analogRead(A1));
  Serial.print(' ');
  Serial.print(analogRead(A2));
  Serial.print(' ');
  Serial.println(analogRead(A3));
}
