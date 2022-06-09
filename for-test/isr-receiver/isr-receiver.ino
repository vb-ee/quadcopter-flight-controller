byte last_channel_1, last_channel_2, last_channel_3, last_channel_4;
unsigned long timer_channel_1, timer_channel_2, timer_channel_3, timer_channel_4, esc_timer, esc_loop_timer;
unsigned long timer_1, timer_2, timer_3, timer_4, current_time;
volatile int receiver_input[4];

void setup() {
  Serial.begin(57600);
  DDRA |= B11110000; // Set 26, 27, 28 and 29 pins as outputs
  PCICR  |= (1 << PCIE0);  // Set PCIE0 to enable PCMSK0 scan
  PCMSK0 |= (1 << PCINT4); // Set PCINT0 (digital input 10) to trigger an interrupt on state change
  PCMSK0 |= (1 << PCINT5); // Set PCINT1 (digital input 11) to trigger an interrupt on state change
  PCMSK0 |= (1 << PCINT6); // Set PCINT2 (digital input 12)to trigger an interrupt on state change
  PCMSK0 |= (1 << PCINT7); // Set PCINT3 (digital input 13)to trigger an interrupt on state change
}

void loop() {
  delay(4);
//  PORTA |= B11110000;
//  delayMicroseconds(1000);
//  PORTA &= B00001111;
//  delayMicroseconds(3000);
  print_signals();
}

ISR(PCINT0_vect) {
  current_time = micros();
  //Channel 1=========================================
  if (PINB & B00010000) {                                                   //Is input 10 high?
    if (last_channel_1 == 0) {                                              //Input 10 changed from 0 to 1.
      last_channel_1 = 1;                                                   //Remember current input state.
      timer_1 = current_time;                                               //Set timer_1 to current_time.
    }
  }
  else if (last_channel_1 == 1) {                                           //Input 10 is not high and changed from 1 to 0.
    last_channel_1 = 0;                                                     //Remember current input state.
    receiver_input[0] = current_time - timer_1;                             //Channel 1 is current_time - timer_1.
  }
  //Channel 2=========================================
  if (PINB & B00100000 ) {                                                  //Is input 11 high?
    if (last_channel_2 == 0) {                                              //Input 11 changed from 0 to 1.
      last_channel_2 = 1;                                                   //Remember current input state.
      timer_2 = current_time;                                               //Set timer_2 to current_time.
    }
  }
  else if (last_channel_2 == 1) {                                           //Input 11 is not high and changed from 1 to 0.
    last_channel_2 = 0;                                                     //Remember current input state.
    receiver_input[1] = current_time - timer_2;                             //Channel 2 is current_time - timer_2.
  }
  //Channel 3=========================================
  if (PINB & B01000000 ) {                                                  //Is input 12 high?
    if (last_channel_3 == 0) {                                              //Input 12 changed from 0 to 1.
      last_channel_3 = 1;                                                   //Remember current input state.
      timer_3 = current_time;                                               //Set timer_3 to current_time.
    }
  }
  else if (last_channel_3 == 1) {                                           //Input 12 is not high and changed from 1 to 0.
    last_channel_3 = 0;                                                     //Remember current input state.
    receiver_input[2] = current_time - timer_3;                             //Channel 3 is current_time - timer_3.

  }
  //Channel 4=========================================
  if (PINB & B10000000 ) {                                                  //Is input 13 high?
    if (last_channel_4 == 0) {                                              //Input 13 changed from 0 to 1.
      last_channel_4 = 1;                                                   //Remember current input state.
      timer_4 = current_time;                                               //Set timer_4 to current_time.
    }
  }
  else if (last_channel_4 == 1) {                                           //Input 13 is not high and changed from 1 to 0.
    last_channel_4 = 0;                                                     //Remember current input state.
    receiver_input[3] = current_time - timer_4;                             //Channel 4 is current_time - timer_4.
  }
}

void print_signals() {
  Serial.print("Roll:");
  if(receiver_input[0] - 1480 < 0)Serial.print("<<<");
  else if(receiver_input[0] - 1520 > 0)Serial.print(">>>");
  else Serial.print("-+-");
  Serial.print(receiver_input[0]);

  Serial.print("  Pitch:");
  if(receiver_input[1] - 1480 < 0)Serial.print("<<<");
  else if(receiver_input[1] - 1520 > 0)Serial.print(">>>");
  else Serial.print("-+-");
  Serial.print(receiver_input[1]);

  Serial.print("  Throttle:");
  if(receiver_input[2] - 1480 < 0)Serial.print("^^^");
  else if(receiver_input[2] - 1520 > 0)Serial.print("vvv");
  else Serial.print("-+-");
  Serial.print(receiver_input[2]);

  Serial.print("  Yaw:");
  if(receiver_input[3] - 1480 < 0)Serial.print("<<<");
  else if(receiver_input[3] - 1520 > 0)Serial.print(">>>");
  else Serial.print("-+-");
  Serial.println(receiver_input[3]);

}
