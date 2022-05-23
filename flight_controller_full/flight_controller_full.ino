#include <Wire.h>

#define throt 0
#define pitch 1
#define roll 2
#define yaw 3

#define stopped 0
#define starting 1
#define started 2

#define refresh_rate 250

volatile int receiver_input[4];
volatile int pulse_length[4];
volatile unsigned long timer_1, timer_2, timer_3, timer_4, current_time;
volatile byte last_channel_1, last_channel_2, last_channel_3, last_channel_4;

float accel_x, accel_y, accel_z; 
float gyro_x, gyro_y, gyro_z;
float gyro_x_avg, gyro_y_avg, gyro_z_avg;
float acc_pitch_angle, acc_roll_angle;

float gyro_roll, gyro_pitch, gyro_yaw;
float pitch_angle, roll_angle, yaw_angle;
float measured_pitch, measured_roll, measured_yaw;

int temp;
float accel_vec_mag;
boolean gyro_angle_set;

unsigned long int loop_timer, time_diff;
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
int led_pin = 2;
int counter;

int status = stopped;

void setup() {
  Wire.begin();
  TWBR = 12; // Set I2C clock frequency to 400kHz
  Serial.begin(57600);
   
  DDRB |= B11110000; // Set 4, 5, 6, 7 pins as outputs
  pinMode(led_pin, OUTPUT); // Change this line to port manipulation
  
  digitalWrite(led_pin, HIGH);

  set_mpu6050();

  for (int i = 0; i < 1250 ; i++) { //Wait 5 seconds before continuing.
    PORTB |= B11110000;             //Set digital ports 4, 5, 6 and 7 high.
    delayMicroseconds(1000);                                                
    PORTB &= B00001111;             //Set digital ports 4, 5, 6 and 7 low.
    delayMicroseconds(3000);                                                
  }
  
  calibrate_gyro();

  PCICR  |= (1 << PCIE0);  // Set PCIE0 to enable PCMSK0 scan
  PCMSK0 |= (1 << PCINT4); // Set PCINT4 (digital input 10) to trigger an interrupt on state change
  PCMSK0 |= (1 << PCINT5); // Set PCINT5 (digital input 11) to trigger an interrupt on state change
  PCMSK0 |= (1 << PCINT6); // Set PCINT6 (digital input 12)to trigger an interrupt on state change
  PCMSK0 |= (1 << PCINT7); // Set PCINT7 (digital input 13)to trigger an interrupt on state change

  while (pulse_length[throt] < 990 || pulse_length[throttle] > 1020 || pulse_length[yaw] < 1400) {
      pulse_length[throt] = convert_receiver_input(3);                 //Convert the actual receiver signals for throttle to the standard 1000 - 2000us
      pulse_length[yaw] = convert_receiver_input(4);                 //Convert the actual receiver signals for yaw to the standard 1000 - 2000us
    counter++;                                                               //While waiting increment start whith every loop.
    //We don't want the esc's to be beeping annoyingly. So let's give them a 1000us puls while waiting for the receiver inputs.
    PORTB |= B11110000;                                                     //Set digital poort 4, 5, 6 and 7 high.
    delayMicroseconds(1000);                                                //Wait 1000us.
    PORTB &= B00001111;                                                     //Set digital poort 4, 5, 6 and 7 low.
    delay(3);                                                               //Wait 3 milliseconds before the next loop.
    if (counter == 125) {                                                     //Every 125 loops (500ms).
      digitalWrite(led_pin, !digitalRead(led_pin));                                   //Change the led status.
      counter = 0;                                                            //Start again at 0.
    }
  }
    
  battery_voltage = (analogRead(0) + 65) * 1.2317;

  loop_timer = micros();
  
  digitalWrite(led_pin, LOW);
}


void loop() {   
  read_mpu();

  calculate_angles();

  calculate_setpoints();

  calculate_pid();
  
  if (is_started()) {

  pid_control();

// For now we use static power supply not battery, so we dont need this function
//  compensate_battery(); 
  }


  run_motors();
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

  convert_receiver_signals();
  
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
    if (i % 15 == 0) digitalWrite(led_pin, !digitalRead(led_pin));  // Change the led status to indicate calibration.
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

  pitch_angle += (gyro_y / (refresh_rate * 65.5));
  roll_angle += (gyro_x / (refresh_rate * 65.5));

  pitch_angle -= roll_angle * sin(gyro_z * (PI / (refresh_rate * 65.5 * 180)));
  roll_angle += pitch_angle * sin(gyro_z * (PI / (refresh_rate * 65.5 * 180)));
}


void calculate_accel_angles() {
  accel_vec_mag = sqrt((accel_x * accel_x) + (accel_y * accel_y) + (accel_z * accel_z));

  if (abs(accel_x) < accel_vec_mag) {
    acc_pitch_angle = asin((float)accel_x / accel_vec_mag) * -(180 / PI);
  }
  if (abs(accel_y) < accel_vec_mag) {
    acc_roll_angle = asin((float)accel_y / accel_vec_mag) * (180 / PI);
  }
}


void calculate_angles() {
  calculate_gyro_angles();
  calculate_accel_angles();

  if (gyro_angle_set) {
    pitch_angle = pitch_angle * 0.9996 + acc_pitch_angle * 0.0004;
    roll_angle = roll_angle * 0.9996 + acc_roll_angle * 0.0004;
  }
  else {
    reset_gyro_angles();
    gyro_angle_set = true;
  }

//  measured_pitch = measured_pitch * 0.9 + pitch_angle * 0.1;
//  measured_roll = measured_roll * 0.9 + roll_angle * 0.1;
//  measured_yaw = -gyro_z / 65.5;

  gyro_pitch = 0.7 * gyro_pitch + 0.3 * gyro_y / 65.5;
  gyro_roll = 0.7 * gyro_roll + 0.3 * gyro_x / 65.5;
  gyro_yaw = 0.7 * gyro_yaw + 0.3 * gyro_z / 65.5;
}


void reset_gyro_angles() {
  pitch_angle = acc_pitch_angle;
  roll_angle = acc_roll_angle;
}


void calculate_setpoints() {
  pitch_setpoint = calculate_setpoint(measured_pitch, pulse_length[pitch]);
  roll_setpoint = calculate_setpoint(measured_roll, pulse_length[roll]);
  yaw_setpoint = calculate_yaw_setpoint(pulse_length[yaw], pulse_length[throt]);
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

bool is_started() {
  // When left stick is moved in the bottom left corner
  if (status == stopped && pulse_length[yaw] < 1050 && pulse_length[throt] < 1050) {
    status = starting;
  }

  // When left stick is moved back in the center position
  if (status == starting && pulse_length[yaw] > 1450 && pulse_length[throt] < 1050) {
    status = started;

    // Reset PID controller's variables to prevent bump start
    reset_pid_values();

    reset_gyro_angles();
  }

  // When left stick is moved in the bottom right corner
  if (status == started && pulse_length[yaw] > 1950 && pulse_length[throt] < 1050) {
    status = stopped;
    // Make sure to always stop motors when status is STOPPED
    stop_all();
  }
 return status == started;  
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
  throttle = pulse_length[throt];
  if (throttle > 1800)throttle = 1800;

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

int convert_receiver_input(byte channel) {
      
}

void convert_receiver_signals() {
  pulse_length[throt] = convert_receiver_input(throt);
  pulse_length[pitch] = convert_receiver_input(pitch);
  pulse_length[roll] = convert_receiver_input(roll);
  pulse_length[yaw] = convert_receiver_input(yaw);
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
  Serial.print(pulse_length[throt]);
  Serial.print(" Pitch: ");
  Serial.print(pulse_length[pitch]);
  Serial.print(" Roll: ");
  Serial.print(pulse_length[roll]);
  Serial.print(" Yaw: ");
  Serial.println(pulse_length[yaw]);
}
