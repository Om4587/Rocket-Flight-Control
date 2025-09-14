<<<<<<< HEAD
#include<cmath>
#include<Wire.h>
float RateRoll, RatePitch, RateYaw;
float AngleRoll, AnglePitch;
float AccX, AccY, AccZ;
float AccZInertial;
float VelocityVertical;

float LoopTimer;
void gyro_signals(void){
  Wire.beginTransmission(0x68);
  Wire.write(0x1A);
  Wire.write(0x05);
  Wire.endTransmission();
  Wire.beginTransmission(0x68);
  Wire.write(0x1C);
  Wire.write(0x10);
  Wire.endTransmission();
  Wire.beginTransmission(0x68);
  Wire.write(0x3B);
  Wire.endTransmission();
  Wire.requestFrom(0x68,6);
  int16_t AccXLSB = Wire.read() << 8 | Wire.read();
  int16_t AccYLSB = Wire.read() << 8 | Wire.read();
  int16_t AccZLSB = Wire.read() << 8 | Wire.read();

  Wire.beginTransmission(0x68);
  Wire.write(0x1B);
  Wire.write(0x8);
  Wire.endTransmission();
  Wire.beginTransmission(0x68);
  Wire.write(0x43);
  Wire.endTransmission();
  Wire.requestFrom(0x68,6);
  
  int16_t GyroX = Wire.read() << 8 | Wire.read();
  int16_t GyroY = Wire.read() << 8 | Wire.read();
  int16_t GyroZ = Wire.read() << 8 | Wire.read();

  RateRoll = (float)GyroX/65.5;
  RatePitch = (float)GyroY/65.5;
  RateYaw = (float)GyroZ/65.5;

  AccX = (float)AccXLSB / 4096;
  AccY = (float)AccYLSB / 4096 - 5.7;
  AccZ = (float)AccZLSB / 4096 + 0.15;

  AngleRoll = atan(AccY / sqrt(AccX*AccX+AccZ*AccZ))*1/(3.142/180);
  AnglePitch = atan(AccX / sqrt(AccY*AccY+AccZ*AccZ))*1/(3.142/180);

}
void setup() {
  Serial.begin(57600);
  pinMode(19,OUTPUT);
  digitalWrite(19,HIGH);
  Wire.setClock(400000);
  Wire.begin();
  delay(250);
  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();

}

void loop() {
  gyro_signals();

  // Time step
  float dt = (micros() - LoopTimer) / 1e6;
  LoopTimer = micros();

  // --- Gyro integration (degrees/sec * seconds) ---
  float gyroRollChange  = RateRoll * dt;   // RateRoll from gyro
  float gyroPitchChange = RatePitch * dt;  // RatePitch from gyro

  // --- Accelerometer angles ---
  float accRoll  = atan2(AccY, sqrt(AccX*AccX + AccZ*AccZ)) * 180.0 / M_PI;
  float accPitch = atan2(AccX, sqrt(AccY*AccY + AccZ*AccZ)) * 180.0 / M_PI;

  // --- Complementary filter ---
  float alpha = 0.98;  // trust gyro 98%, accel 2%
  AngleRoll  = alpha * (AngleRoll  + gyroRollChange)  + (1 - alpha) * accRoll;
  AnglePitch = alpha * (AnglePitch + gyroPitchChange) + (1 - alpha) * accPitch;

  // --- Use filtered roll/pitch for inertial Z ---
  float rollRad  = AngleRoll * M_PI / 180.0;
  float pitchRad = AnglePitch * M_PI / 180.0;

  AccZInertial = -sin(pitchRad) * AccX
               + sin(rollRad) * cos(pitchRad) * AccY
               + cos(rollRad) * cos(pitchRad) * AccZ;

  // Remove gravity and convert to cm/s²
  AccZInertial = (AccZInertial - 1.0) * 9.81 * 100.0;

  // Integrate to velocity
  VelocityVertical += AccZInertial * dt;

  Serial.println(VelocityVertical);
}
=======
#include<cmath>
#include<Wire.h>
float RateRoll, RatePitch, RateYaw;
float AngleRoll, AnglePitch;
float AccX, AccY, AccZ;
float AccZInertial;
float VelocityVertical;

float LoopTimer;
void gyro_signals(void){
  Wire.beginTransmission(0x68);
  Wire.write(0x1A);
  Wire.write(0x05);
  Wire.endTransmission();
  Wire.beginTransmission(0x68);
  Wire.write(0x1C);
  Wire.write(0x10);
  Wire.endTransmission();
  Wire.beginTransmission(0x68);
  Wire.write(0x3B);
  Wire.endTransmission();
  Wire.requestFrom(0x68,6);
  int16_t AccXLSB = Wire.read() << 8 | Wire.read();
  int16_t AccYLSB = Wire.read() << 8 | Wire.read();
  int16_t AccZLSB = Wire.read() << 8 | Wire.read();

  Wire.beginTransmission(0x68);
  Wire.write(0x1B);
  Wire.write(0x8);
  Wire.endTransmission();
  Wire.beginTransmission(0x68);
  Wire.write(0x43);
  Wire.endTransmission();
  Wire.requestFrom(0x68,6);
  
  int16_t GyroX = Wire.read() << 8 | Wire.read();
  int16_t GyroY = Wire.read() << 8 | Wire.read();
  int16_t GyroZ = Wire.read() << 8 | Wire.read();

  RateRoll = (float)GyroX/65.5;
  RatePitch = (float)GyroY/65.5;
  RateYaw = (float)GyroZ/65.5;

  AccX = (float)AccXLSB / 4096;
  AccY = (float)AccYLSB / 4096 - 5.7;
  AccZ = (float)AccZLSB / 4096 + 0.15;

  AngleRoll = atan(AccY / sqrt(AccX*AccX+AccZ*AccZ))*1/(3.142/180);
  AnglePitch = atan(AccX / sqrt(AccY*AccY+AccZ*AccZ))*1/(3.142/180);

}
void setup() {
  Serial.begin(57600);
  pinMode(19,OUTPUT);
  digitalWrite(19,HIGH);
  Wire.setClock(400000);
  Wire.begin();
  delay(250);
  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();

}

void loop() {
  gyro_signals();

  // Time step
  float dt = (micros() - LoopTimer) / 1e6;
  LoopTimer = micros();

  // --- Gyro integration (degrees/sec * seconds) ---
  float gyroRollChange  = RateRoll * dt;   // RateRoll from gyro
  float gyroPitchChange = RatePitch * dt;  // RatePitch from gyro

  // --- Accelerometer angles ---
  float accRoll  = atan2(AccY, sqrt(AccX*AccX + AccZ*AccZ)) * 180.0 / M_PI;
  float accPitch = atan2(AccX, sqrt(AccY*AccY + AccZ*AccZ)) * 180.0 / M_PI;

  // --- Complementary filter ---
  float alpha = 0.98;  // trust gyro 98%, accel 2%
  AngleRoll  = alpha * (AngleRoll  + gyroRollChange)  + (1 - alpha) * accRoll;
  AnglePitch = alpha * (AnglePitch + gyroPitchChange) + (1 - alpha) * accPitch;

  // --- Use filtered roll/pitch for inertial Z ---
  float rollRad  = AngleRoll * M_PI / 180.0;
  float pitchRad = AnglePitch * M_PI / 180.0;

  AccZInertial = -sin(pitchRad) * AccX
               + sin(rollRad) * cos(pitchRad) * AccY
               + cos(rollRad) * cos(pitchRad) * AccZ;

  // Remove gravity and convert to cm/s²
  AccZInertial = (AccZInertial - 1.0) * 9.81 * 100.0;

  // Integrate to velocity
  VelocityVertical += AccZInertial * dt;

  Serial.println(VelocityVertical);
}
>>>>>>> a49214930fa1aebd007e29ae480992e6a367d6f1
