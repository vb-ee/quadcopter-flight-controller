#include <Wire.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

RF24 radio(8, 9);

#define th 0
#define pt 1
#define rl 2
#define yw 3

#define channel_number 4
#define sigPin 2
#define PPM_FrLen 27000
#define PPM_PulseLen 400

#define clockMultiplier 2


const uint64_t nrf24_address = 0xE8E8F0F0E1LL;

int ppm[channel_number];

unsigned long last_recv_time;

struct QuadData {
  byte throttle;
  byte pitch;
  byte roll;
  byte yaw;
};

QuadData sent_data;

float accel_x, accel_y, accel_z;
float gyro_x, gyro_y, gyro_z;
float gyro_roll, gyro_pitch, gyro_yaw;
float gyro_x_avg, gyro_y_avg, gyro_z_avg;
float accel_vec_mag;
boolean set_gyro_angle;
float pitch_angle, roll_angle, yaw_angle;
float measured_pitch, measured_roll, measured_yaw;
float acc_pitch_angle, acc_roll_angle;
unsigned long int loop_timer, time_diff;
int temp;
unsigned long int esc1, esc2, esc3, esc4;
float roll_setpoint, pitch_setpoint, yaw_setpoint;
float kp_roll = 1.2;
float ki_roll = 0.05;
float kd_roll = 18;
float kp_pitch = kp_roll;
float ki_pitch = ki_roll;
float kd_pitch = kd_roll;
float kp_yaw = 3;
float ki_yaw = 0.02;
float kd_yaw = 0;
float roll_pid, pitch_pid, yaw_pid;
float pitch_error, roll_error, yaw_error;
float pitch_pid_i, roll_pid_i, yaw_pid_i;
float previous_pitch_error, previous_roll_error, previous_yaw_error;
int throttle;
int battery_voltage;
float pid_max = 400;
int analog_battery = A0;
int led_pin = 4;
int led_pin1 = 5;


void setup() {
  Wire.begin();
  TWBR = 12; // Set I2C clock frequency to 400kHz
  Serial.begin(57600);
  
  set_nrf24(nrf24_address);
  
  DDRB |= B11110000; // Set 4, 5, 6, 7 pins as outputs
  pinMode(led_pin, OUTPUT);
  
  digitalWrite(led_pin, HIGH);

  set_mpu6050();

  for (int i = 0; i < 1250 ; i++) { //Wait 5 seconds before continuing.
    PORTB |= B11110000;             //Set digital ports 4, 5, 6 and 7 high.
    delayMicroseconds(1000);                                                
    PORTB &= B00001111;             //Set digital ports 4, 5, 6 and 7 low.
    delayMicroseconds(3000);                                                
  }
  
  calibrate_gyro();
  
  setup_ppm();
  reset_data();
  
  battery_voltage = (analogRead(0) + 65) * 1.2317;

  loop_timer = micros();
  
  digitalWrite(led_pin, LOW);
}


void loop() {   
  get_from_rf();

  read_mpu();

  calculate_angles();

  calculate_setpoints();

  state_checking();

  calculate_pid();

  pid_control();

  //  compensate_battery();

  //  print_info();

  //  print_data();

  run_motors();
}

void set_nrf24(uint64_t address) {
  radio.begin();
  radio.setAutoAck(false);
  radio.setPALevel(RF24_PA_LOW);
  radio.setChannel(110);
  radio.openReadingPipe(1, address);
  radio.setDataRate(RF24_250KBPS);
  radio.startListening();

}

void set_mpu6050() {
  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();

  Wire.beginTransmission(0x68);
  Wire.write(0x1B);
  Wire.write(0x08);
  Wire.endTransmission();

  Wire.beginTransmission(0x68);
  Wire.write(0x1C);
  Wire.write(0x10);
  Wire.endTransmission();

  //Random register check to see if the values are written correct
  Wire.requestFrom(0x68, 1);                                         
  while (Wire.available() < 1);                                      
  if (Wire.read() != 0x08) {                                         
    digitalWrite(12, HIGH);                                          
    while (1) delay(10);                                              
  }

  Wire.beginTransmission(0x68);
  Wire.write(0x1A);
  Wire.write(0x03);
  Wire.endTransmission();
}


void read_mpu() {
  Wire.beginTransmission(0x68);
  Wire.write(0x3B);
  Wire.endTransmission();
  Wire.requestFrom(0x68, 14);
  while (Wire.available() < 14);
  accel_x = Wire.read() << 8 | Wire.read();
  accel_y = Wire.read() << 8 | Wire.read();
  accel_z = Wire.read() << 8 | Wire.read();
  temp = Wire.read() << 8 | Wire.read();
  gyro_x = Wire.read() << 8 | Wire.read();
  gyro_y = Wire.read() << 8 | Wire.read();
  gyro_z = Wire.read() << 8 | Wire.read();
}


void calibrate_gyro() {
  for (int i = 0; i < 2000; i++) {
    if (i % 15 == 0) digitalWrite(12, !digitalRead(12));  // Change the led status to indicate calibration.
    read_mpu();
    gyro_x_avg += gyro_x;
    gyro_y_avg += gyro_y;
    gyro_z_avg += gyro_z;

    // Preventing esc beeps during calibration
    PORTB |= B11110000; // Set pins 4, 5, 6, 7 to high 
    delayMicroseconds(1000);
    PORTB &= B00001111; // Set pins 4, 5, 6, 7 to low
    delay(3);
  }

  gyro_x_avg /= 2000;
  gyro_y_avg /= 2000;
  gyro_z_avg /= 2000;
}


void calculate_gyro_angles() {
  gyro_x -= gyro_x_avg;
  gyro_y -= gyro_y_avg;
  gyro_z -= gyro_z_avg;

  pitch_angle += gyro_y * 0.0000763;
  roll_angle += gyro_x * 0.0000763;

  pitch_angle -= roll_angle * sin(gyro_z * 0.00000133);
  roll_angle += pitch_angle * sin(gyro_z * 0.00000133);
}


void calculate_accel_angles() {
  accel_vec_mag = sqrt((accel_x * accel_x) + (accel_y * accel_y) + (accel_z * accel_z));

  if (abs(accel_x) < accel_vec_mag) {
    acc_pitch_angle = asin(accel_x / accel_vec_mag) * -57.296;
  }
  if (abs(accel_y) < accel_vec_mag) {
    acc_roll_angle = asin(accel_y / accel_vec_mag) * 57.296;
  }
}


void calculate_angles() {
  calculate_gyro_angles();
  calculate_accel_angles();

  if (set_gyro_angle) {
    pitch_angle = pitch_angle * 0.9996 + acc_pitch_angle * 0.0004;
    roll_angle = roll_angle * 0.9996 + acc_roll_angle * 0.0004;
  }
  else {
    reset_gyro_angles();
    set_gyro_angle = true;
  }

  measured_pitch = measured_pitch * 0.9 + pitch_angle * 0.1;
  measured_roll = measured_roll * 0.9 + roll_angle * 0.1;
  measured_yaw = -gyro_z / 65.5;

  gyro_pitch = 0.7 * gyro_pitch + 0.3 * gyro_y / 65.5;
  gyro_roll = 0.7 * gyro_roll + 0.3 * gyro_x / 65.5;
  gyro_yaw = 0.7 * gyro_yaw + 0.3 * gyro_z / 65.5;
}


void reset_gyro_angles() {
  pitch_angle = acc_pitch_angle;
  roll_angle = acc_roll_angle;
}


void calculate_setpoints() {
  pitch_setpoint = calculate_setpoint(measured_pitch, ppm[pt]);
  roll_setpoint = calculate_setpoint(measured_roll, ppm[rl]);
  yaw_setpoint = calculate_yaw_setpoint(ppm[yw], ppm[th]);
}


float calculate_setpoint(float out_angle, int channel_pulse) {
  float level_adjust = out_angle * 15;
  float setpoint = 0;

  if (channel_pulse > 1508) {
    setpoint = channel_pulse - 1508;
  } else if (channel_pulse <  1492) {
    setpoint = channel_pulse - 1492;
  }

  setpoint -= level_adjust;
  setpoint /= 3;

  return setpoint;
}

float calculate_yaw_setpoint(int yaw_pulse, int throttle_pulse) {
  float setpoint = 0;

  if (throttle_pulse > 1050) {
    setpoint = calculate_setpoint(0, yaw_pulse);
  }
  return setpoint;
}


void reset_pid_values() {
  pitch_error = 0;
  roll_error = 0;
  yaw_error = 0;

  pitch_pid_i = 0;
  roll_pid_i = 0;
  yaw_pid_i = 0;

  previous_pitch_error = 0;
  previous_roll_error = 0;
  previous_yaw_error = 0;
}


void state_checking() {
  if (ppm[th] < 1020)stop_all();
  if (ppm[th] < 1050 && ppm[th] >= 1020) {
    reset_pid_values();
    reset_gyro_angles();
  }
}


void stop_all() {
  esc1 = 1000;
  esc2 = 1000;
  esc3 = 1000;
  esc4 = 1000;
}


void calculate_pid() {
  roll_error = gyro_roll - roll_setpoint;
  roll_pid_i += ki_roll * roll_error;
  if (roll_pid_i > pid_max)roll_pid_i = pid_max;
  else if (roll_pid_i < -pid_max)roll_pid_i = -pid_max;
  roll_pid = kp_roll * roll_error + roll_pid_i + kd_roll * (roll_error - previous_roll_error);
  if (roll_pid > pid_max)roll_pid = pid_max;
  else if (roll_pid < -pid_max)roll_pid = -pid_max;
  previous_roll_error = roll_error;

  pitch_error = gyro_pitch - pitch_setpoint;
  pitch_pid_i += ki_pitch * pitch_error;
  if (pitch_pid_i > pid_max)pitch_pid_i = pid_max;
  else if (pitch_pid_i < -pid_max)pitch_pid_i = -pid_max;
  pitch_pid = kp_pitch * pitch_error + pitch_pid_i + kd_pitch * (pitch_error - previous_pitch_error);
  if (pitch_pid > pid_max)pitch_pid = pid_max;
  else if (pitch_pid < -pid_max)pitch_pid = -pid_max;
  previous_pitch_error = pitch_error;

  yaw_error = gyro_yaw - yaw_setpoint;
  yaw_pid_i += ki_yaw * yaw_error;
  if (yaw_pid_i > pid_max)yaw_pid_i = pid_max;
  else if (yaw_pid_i < -pid_max)yaw_pid_i = -pid_max;
  yaw_pid = kp_yaw * yaw_error + yaw_pid_i + kd_yaw * (yaw_error - previous_yaw_error);
  if (yaw_pid > pid_max)yaw_pid = pid_max;
  else if (yaw_pid < -pid_max)yaw_pid = -pid_max;
  previous_yaw_error = yaw_error;
}


void pid_control() {
  throttle = ppm[th];
  if (throttle > 1800)throttle = 1800;
  if (throttle >= 1050) {
    esc1 = throttle - roll_pid + pitch_pid + yaw_pid;
    esc2 = throttle + roll_pid + pitch_pid - yaw_pid;
    esc3 = throttle - roll_pid - pitch_pid - yaw_pid;
    esc4 = throttle + roll_pid - pitch_pid + yaw_pid;

    if (esc1 < 1100) esc1 = 1100;
    if (esc2 < 1100) esc2 = 1100;
    if (esc3 < 1100) esc3 = 1100;
    if (esc4 < 1100) esc4 = 1100;

    if (esc1 > 2000) esc1 = 2000;
    if (esc2 > 2000) esc2 = 2000;
    if (esc3 > 2000) esc3 = 2000;
    if (esc4 > 2000) esc4 = 2000;
  }
}


void run_motors() {
  if (micros() - loop_timer > 4050)digitalWrite(led_pin, HIGH);
  while (micros() - loop_timer < 4000);
  loop_timer = micros();

  PORTB |= B11110000;

  while (PORTB >= 16) {
    time_diff = micros();
    time_diff -= loop_timer;
    if (time_diff >= esc1) PORTB &= B11101111;
    if (time_diff >= esc2) PORTB &= B11011111;
    if (time_diff >= esc3) PORTB &= B10111111;
    if (time_diff >= esc4) PORTB &= B01111111;
  }
}


bool battery_connected() {
  battery_voltage = battery_voltage * 0.92 + (analogRead(0) + 65) * 0.09853;
  return battery_voltage < 1240 && battery_voltage > 800;
}

void compensate_battery() {
  if (battery_voltage < 1000 && battery_voltage > 600)digitalWrite(led_pin, HIGH);
  if (battery_connected()) {
    esc1 += esc1 * ((1240 - battery_voltage) / (float) 3500);
    esc2 += esc2 * ((1240 - battery_voltage) / (float) 3500);
    esc3 += esc3 * ((1240 - battery_voltage) / (float) 3500);
    esc4 += esc4 * ((1240 - battery_voltage) / (float) 3500);
  }
}



void reset_data() {
  sent_data.throttle = 0;
  sent_data.pitch = 127;
  sent_data.roll = 127;
  sent_data.yaw = 127;
  set_ppm_values();
}

void set_ppm_values() {
  ppm[th] = map(sent_data.throttle, 0, 255, 1000, 2000);
  ppm[pt] = map(sent_data.pitch,      0, 255, 1000, 2000);
  ppm[rl] = map(sent_data.roll,    0, 255, 1000, 2000);
  ppm[yw] = map(sent_data.yaw,     0, 255, 1000, 2000);
}

void setup_ppm() {
  pinMode(sigPin, OUTPUT);
  digitalWrite(sigPin, 0);

  cli();
  TCCR1A = 0;
  TCCR1B = 0;

  OCR3B = 100;
  TCCR1B |= (1 << WGM12);
  TCCR1B |= (1 << CS11);
  TIMSK1 |= (1 << OCIE1A);
  sei();
}

void recieve_data()
{
  while ( radio.available() ) {
    radio.read(&sent_data, sizeof(QuadData));
    last_recv_time = millis();
  }
}

void get_from_rf() {
  recieve_data();

  unsigned long now = millis();
  if (now - last_recv_time > 1000) {
    reset_data();
  }

  set_ppm_values();
}

void print_info() {
  Serial.print("Loop time: ");
  Serial.println(micros() - loop_timer);
  //    Serial.print(" Pitch: ");
  //    Serial.print(measured_pitch);
  //    Serial.print(" Roll: ");
  //    Serial.print(measured_roll);
  //    Serial.print(" esc1: ");
  //    Serial.print(esc1);
  //    Serial.print(" esc2: ");
  //    Serial.print(esc2);
  //    Serial.print(" esc3: ");
  //    Serial.print(esc3);
  //    Serial.print(" esc4: ");
  //    Serial.println(esc4);
}

void print_data() {
  Serial.print("Loop time: ");
  Serial.print(micros() - loop_timer);
  Serial.print(" Throttle: ");
  Serial.print(ppm[th]);
  Serial.print(" Pitch: ");
  Serial.print(ppm[pt]);
  Serial.print(" Roll: ");
  Serial.print(ppm[rl]);
  Serial.print(" Yaw: ");
  Serial.println(ppm[yw]);
  //  Serial.print(" Stop: ");
  //  Serial.println(ppm[st]);
}


ISR(TIMER1_COMPA_vect) {
  static boolean state = true;

  TCNT1 = 0;

  if ( state ) {
    PORTE &= B00010000;
    OCR3B = PPM_PulseLen * clockMultiplier;
    state = false;
  }
  else {
    static byte cur_chan_numb;
    static unsigned int calc_rest;

    PORTE |= B00010000;
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
