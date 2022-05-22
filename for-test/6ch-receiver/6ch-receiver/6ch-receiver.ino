#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <Servo.h>

RF24 radio(8, 9);

Servo channel_1;
Servo channel_2;
Servo channel_3;
Servo channel_4;

int ch1_pulse = 0;
int ch2_pulse = 0;
int ch3_pulse = 0;
int ch4_pulse = 0;

const uint64_t address = 0xE8E8F0F0E1LL;

unsigned long int last_recv_time;

struct QuadData {
  byte throttle;
  byte pitch;
  byte roll;
  byte yaw;
};

QuadData quad_data;

void reset_data() {
  quad_data.throttle = 127;
  quad_data.pitch = 127;
  quad_data.roll = 127;
  quad_data.yaw = 127;
}

void set_ppm_values() {
  ch1_pulse = map(quad_data.throttle, 0, 255, 1000, 2000);
  ch2_pulse = map(quad_data.pitch,    0, 255, 1000, 2000);
  ch3_pulse = map(quad_data.roll,     0, 255, 1000, 2000);
  ch4_pulse = map(quad_data.yaw,      0, 255, 1000, 2000);
}

void setup() {
  Serial.begin(57600);
  
  radio.begin();
  radio.setAutoAck(false);
  radio.setPALevel(RF24_PA_LOW);
  radio.setChannel(110);
  radio.openReadingPipe(1, address);
  radio.setDataRate(RF24_250KBPS);
  radio.startListening();

  channel_1.attach(4);
  channel_2.attach(5);
  channel_3.attach(6);
  channel_4.attach(7);

  reset_data();
}


void loop() {
  receive_data();

  unsigned long int now = millis();
  if (now - last_recv_time > 1000) {
    reset_data();
  }

  set_ppm_values();

  channel_1.writeMicroseconds(ch1_pulse);  
  channel_2.writeMicroseconds(ch2_pulse);  
  channel_3.writeMicroseconds(ch3_pulse);  
  channel_4.writeMicroseconds(ch4_pulse);  
  
  print_data();
}

void receive_data()
{
  while ( radio.available() ) {
    radio.read(&quad_data, sizeof(QuadData));
    last_recv_time = millis();
  }
}

void print_data() {
  Serial.print("Throttle: ");
  Serial.print(ch1_pulse);
  Serial.print(" Pitch: ");
  Serial.print(ch2_pulse);
  Serial.print(" Roll: ");
  Serial.print(ch3_pulse);
  Serial.print(" Yaw: ");
  Serial.println(ch4_pulse);
}
