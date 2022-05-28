#include <Wire.h>

// gyro data
float accel_x, accel_y, accel_z;
float gyro_x, gyro_y, gyro_z;

// offsets
float gyro_x_avg, gyro_y_avg, gyro_z_avg = 0;

// magnitude of acceleration vector
float accel_vec_mag;

boolean set_gyro_angle = false;

// angles
float pitch_angle, roll_angle, pitch_out, roll_out, yaw_angle;
float acc_pitch_angle, acc_roll_angle, gyro_yaw;

// time taken per loop
long loop_timer;

// temperature data
int temp;


void setup() {
  Serial.begin(57600);
  TWBR = 12;
  Wire.begin();
  set_mpu();
  calc_gyro_offsets();
  loop_timer = micros();
}


void loop() {

  read_mpu();
//  Serial.println(micros() - loop_timer);
// subtracting offsets from raw data to get correct value
  gyro_x -= gyro_x_avg;
  gyro_y -= gyro_y_avg;
  gyro_z -= gyro_z_avg;

// calculating angles from raw data  
  pitch_angle += gyro_x * 0.0000611;
  roll_angle += gyro_y * 0.0000611;

// swapping the angles in case if the state is changed
  pitch_angle += roll_angle * sin(gyro_z * 0.000001066);
  roll_angle -= pitch_angle * sin(gyro_z * 0.000001066);
////  gyro_yaw = gyro_yaw * 0.7 + 0.1 * gyro_z / 65.5;
//
// calculating angles from raw acceleration values
  accel_vec_mag = sqrt((accel_x * accel_x) + (accel_y * accel_y) + (accel_z * accel_z));
  acc_pitch_angle = asin(accel_y / accel_vec_mag) * 57.296;
  acc_roll_angle = asin(accel_x / accel_vec_mag) * -57.296;
//
//  if (set_gyro_angle) {
//// using complementary filter to reduce the noise
//    pitch_angle = pitch_angle * 0.9996 + acc_pitch_angle * 0.0004;
//    roll_angle = roll_angle * 0.9996 + acc_roll_angle * 0.0004;
//  }
//  else {
//// setting initial angle values
//    pitch_angle = acc_pitch_angle;
//    roll_angle = acc_roll_angle;
//    set_gyro_angle = true;
//  }

// getting more steady angle values  
//  pitch_out = 0.9 * pitch_out + 0.1 * pitch_angle;
//  roll_out = 0.9 * roll_out + 0.1 * roll_angle;

  print_angles();
    print_raw();

// looptime is set to 4000 but can be changed according to the implementation
  while (micros() - loop_timer < 4000);
  loop_timer = micros();
}

void set_mpu() {
  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();
//  Wire.beginTransmission(0x68);
//  Wire.write(0x37);
//  Wire.write(0x02); //
//  Wire.endTransmission();
//  Wire.beginTransmission(0x68);
//  Wire.write(0x6A);
//  Wire.write(0x00);
//  Wire.endTransmission();
  Wire.beginTransmission(0x68);
  Wire.write(0x1B); 
  Wire.write(0b00001000); // gyro sensitivity is set to 65.5 and can be adjusted here, see the datasheet for more info
  Wire.endTransmission();
  Wire.beginTransmission(0x68);
  Wire.write(0x1C);
  Wire.write(0b00010000);// acceleration sensitivity is set to 8192 and can be adjusted here, see the datasheet for more info
  Wire.endTransmission();
  Wire.beginTransmission(0x68);                        //Start communication with the MPU-6050.
  Wire.write(0x1A);                                            //We want to write to the CONFIG register (1A hex).
  Wire.write(0x03);                                            //Set the register bits as 00000011 (Set Digital Low Pass Filter to ~43Hz).
  Wire.endTransmission();                                      //End the transmission with the gyro.
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


void calc_gyro_offsets() {
  for (int i = 0; i < 2000; i++) {
    read_mpu();
    gyro_x_avg += gyro_x;
    gyro_y_avg += gyro_y;
    gyro_z_avg += gyro_z;
    delay(3);
  }

  gyro_x_avg /= 2000;
  gyro_y_avg /= 2000;
  gyro_z_avg /= 2000;

}


void print_angles() {
  Serial.print("Pitch angle = ");
  Serial.print(pitch_angle);
  Serial.print(" Roll angle = ");
  Serial.print(roll_angle);
}

void print_raw() {
//  Serial.print("gyro_x = ");
//  Serial.print(gyro_x);
//  Serial.print(" gyro_y = ");
//  Serial.print(gyro_y);
  Serial.print(" accel_x = ");
  Serial.print(acc_pitch_angle);
  Serial.print(" accel_y = ");
  Serial.println(acc_roll_angle);

}
