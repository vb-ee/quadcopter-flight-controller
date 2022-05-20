//////Include Libraries
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>


RF24 radio(8, 9);

#define channel_number 4  //set the number of channels
#define sigPin 2  //set PPM signal output pin on the arduino
#define PPM_FrLen 27000  //set the PPM frame length in microseconds (1ms = 1000µs)
#define PPM_PulseLen 400  //set the pulse length

const uint64_t address = 0xE8E8F0F0E1LL;

int ppm[channel_number];

unsigned long last_recv_time;

struct QuadData {
  byte throttle;
  byte pitch;
  byte roll;
  byte yaw;
  //  byte land;
  //  byte alt;
};

QuadData sent_data;

void reset_data() {
  sent_data.throttle = 0;
  sent_data.pitch = 127;
  sent_data.roll = 127;
  sent_data.yaw = 127;
  //  sent_data.land = 0;
  //  sent_data.alt = 0;
  set_ppm_values();
}

void set_ppm_values() {
  ppm[0] = map(sent_data.throttle, 0, 255, 1000, 2000);
  ppm[1] = map(sent_data.pitch,      0, 255, 1000, 2000);
  ppm[2] = map(sent_data.roll,    0, 255, 1000, 2000);
  ppm[3] = map(sent_data.yaw,     0, 255, 1000, 2000);
  //  ppm[4] = map(sent_data.land, 0, 1, 1000, 2000);
  //  ppm[5] = map(sent_data.alt, 0, 1, 1000, 2000);
}


void setup_ppm() {
  pinMode(sigPin, OUTPUT);
  digitalWrite(sigPin, 0);  //set the PPM signal pin to the default state (off)

  cli();
  TCCR1A = 0; // set entire TCCR1 register to 0
  TCCR1B = 0;

  OCR3B = 100;  // compare match register (not very important, sets the timeout for the first interrupt)
  TCCR1B |= (1 << WGM12);  // turn on CTC mode
  TCCR1B |= (1 << CS11);  // 8 prescaler: 0,5 microseconds at 16mhz
  TIMSK1 |= (1 << OCIE1A); // enable timer compare interrupt
  sei();
}

void setup() {
  reset_data();
  setup_ppm();
  Serial.begin(115200);
  radio.begin();
  radio.setAutoAck(false);
  radio.setPALevel(RF24_PA_LOW);
  radio.setChannel(110);
  radio.openReadingPipe(1, address);
  radio.setDataRate(RF24_250KBPS);
  radio.startListening();
}


void loop() {
  recieve_data();

  unsigned long now = millis();
  if (now - last_recv_time > 1000) {
    reset_data();
  }

  set_ppm_values();
  print_data();
}

void recieve_data()
{
  while ( radio.available() ) {
    radio.read(&sent_data, sizeof(QuadData));
    last_recv_time = millis();
  }
}

void print_data() {
  Serial.print("Throttle: ");
  Serial.print(ppm[0]);
  Serial.print(" Pitch: ");
  Serial.print(ppm[1]);
  Serial.print(" Roll: ");
  Serial.print(ppm[2]);
  Serial.print(" Yaw: ");
  Serial.println(ppm[3]);
  //  Serial.print(" Landing: ");
  //  Serial.print(ppm[4]);
  //  Serial.print(" Altitutde: ");
  //  Serial.println(ppm[5]);
}

#define clockMultiplier 2 // set this to 2 if you are using a 16MHz arduino, leave as 1 for an 8MHz arduino

ISR(TIMER1_COMPA_vect) { 
  static boolean state = true;

  TCNT1 = 0;

  if ( state ) {
    //end pulse
    PORTE &= B00010000; // turn pin 2 off. Could also use: digitalWrite(sigPin,0)
    OCR3B = PPM_PulseLen * clockMultiplier;
    state = false;
  }
  else {
    //start pulse
    static byte cur_chan_numb;
    static unsigned int calc_rest;

    PORTE |= B00010000; // turn pin 2 on. Could also use: digitalWrite(sigPin,1)
    state = true;

    if (cur_chan_numb >= channel_number) {
      cur_chan_numb = 0;
      calc_rest += PPM_PulseLen;
      OCR3B = (PPM_FrLen - calc_rest) * clockMultiplier;
      calc_rest = 0;
    }
    else {
      OCR3B = (ppm[cur_chan_numb] - PPM_PulseLen) * clockMultiplier;
      calc_rest += ppm[cur_chan_numb];
      cur_chan_numb++;
    }
  }
}
