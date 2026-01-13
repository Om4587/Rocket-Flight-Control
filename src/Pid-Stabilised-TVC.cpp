#include <Wire.h>
#include <Servo.h>

Servo Xservo, Yservo;

// -- Pins--
const int servoPinX = 6;   // Pitch axis servo
const int servoPinY = 7;   // Roll axis servo
const int buzzerPin = 8;

const int ledRed    = 1;   // Danger tilt
const int ledYellow = 3;   // Jitter / movement
const int ledBlue   = 4;   // No movement

// -- IMU State --
float AccX, AccY, AccZ;
float RateRoll, RatePitch;
float AngleRoll = 0, AnglePitch = 0;  
const float gyroScale = 131.0f;

uint32_t prevMicros;
const float alpha = 0.92f;

// -- Filtering --
float smoothRoll = 0, smoothPitch = 0;
const float LPF = 0.40f;

// -- PID --
float Kp = 2.8, Ki = 0.02, Kd = 1.2;
float integralRoll = 0, prevErrorRoll = 0;
float integralPitch = 0, prevErrorPitch = 0;
const float ERROR_DEADBAND = 0.6f;

// -- Servo Motion --
float servoX = 90, servoY = 90;
float lastWriteX = 90, lastWriteY = 90;
float servoXfiltered = 90, servoYfiltered = 90;

const float SERVO_SMOOTH = 0.10f;
const float SERVO_MIN_DELTA = 0.5f;

const float EXTREME_THRESHOLD = 75.0f;   // Y-axis danger tilt


// ---- IMU READ ----
void readMPU(float &dt)
{
  Wire.beginTransmission(0x68);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom((uint8_t)0x68, (uint8_t)14, (bool)true);

  int16_t ax = (Wire.read() << 8) | Wire.read();
  int16_t ay = (Wire.read() << 8) | Wire.read();
  int16_t az = (Wire.read() << 8) | Wire.read();
  Wire.read(); Wire.read();
  int16_t gx = (Wire.read() << 8) | Wire.read();
  int16_t gy = (Wire.read() << 8) | Wire.read();
  Wire.read(); Wire.read();

  AccX = ax / 16384.0f;
  AccY = ay / 16384.0f;
  AccZ = az / 16384.0f;

  RateRoll  = gx / gyroScale;
  RatePitch = gy / gyroScale;

  uint32_t now = micros();
  dt = (now - prevMicros) * 1e-6f;
  prevMicros = now;
  if (dt <= 0 || dt > 0.05f) dt = 0.01f;

  // ---- Angle math ----
  float accPitch = atan2f(AccZ, sqrtf(AccX*AccX + AccY*AccY)) * 180.0f / PI;
  float accRoll  = atan2f(-AccX, sqrtf(AccY*AccY + AccZ*AccZ)) * 180.0f / PI;

  AnglePitch = alpha * (AnglePitch + RateRoll  * dt) + (1-alpha)*accPitch;
  AngleRoll  = alpha * (AngleRoll  + RatePitch * dt) + (1-alpha)*accRoll;

  // LPF smoothing
  smoothPitch += LPF * (AnglePitch - smoothPitch);
  smoothRoll  += LPF * (AngleRoll  - smoothRoll);

  AnglePitch = smoothPitch;
  AngleRoll  = smoothRoll;
}


// ---- PID ----
float computePID(float target, float angle, float &integral, float &prevError, float dt)
{
  float error = target - angle;
  if (fabs(error) < ERROR_DEADBAND) error = 0;

  integral += error * dt;
  integral = constrain(integral, -50, 50);

  float derivative = (error - prevError) / dt;
  derivative = constrain(derivative, -400, 400);

  prevError = error;
  return Kp*error + Ki*integral + Kd*derivative;
}


// ---- SETUP ----
void setup()
{
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(400000);

  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();

  Xservo.attach(servoPinX); // Pitch stabilizer
  Yservo.attach(servoPinY); // Roll stabilizer

  pinMode(buzzerPin, OUTPUT);
  pinMode(ledRed, OUTPUT);
  pinMode(ledYellow, OUTPUT);
  pinMode(ledBlue, OUTPUT);

  prevMicros = micros();

  Serial.println("LED states: Blue=Still, Yellow=Jitter, Red=Danger");
  Serial.println("Pitch,Roll,ServoX,ServoY,Buzz");
}


// ---- LOOP ----
void loop()
{
  float dt;
  readMPU(dt);

  // ---- PID ----
  float outPitch = computePID(0, AnglePitch, integralPitch, prevErrorPitch, dt);
  float outRoll  = computePID(0, AngleRoll,  integralRoll,  prevErrorRoll,  dt);

  // Opposite stabilizing direction
  outPitch = -outPitch;

  // ---- Servo commands ----
  float desiredX = constrain(90 - outPitch, 0, 180);
  float desiredY = constrain(90 - outRoll,  0, 180);

  servoXfiltered += (desiredX - servoXfiltered) * 0.42f;
  servoYfiltered += (desiredY - servoYfiltered) * 0.42f;

  servoX += (servoXfiltered - servoX) * SERVO_SMOOTH;
  servoY += (servoYfiltered - servoY) * SERVO_SMOOTH;

  if (fabs(servoX - lastWriteX) >= SERVO_MIN_DELTA) {
    Xservo.write((int)servoX);
    lastWriteX = servoX;
  }
  if (fabs(servoY - lastWriteY) >= SERVO_MIN_DELTA) {
    Yservo.write((int)servoY);
    lastWriteY = servoY;
  }

  // ===== LED + BUZZER STATES =====
  float absPitch = fabs(AnglePitch);
  float absRoll  = fabs(AngleRoll);

  float motion = fabs(RateRoll) + fabs(RatePitch);

  bool dangerTilt = (absPitch > EXTREME_THRESHOLD);
  bool noMovement = (motion < 1.5f);            // perfectly still
  bool jitter     = (motion >= 1.5f && motion < 15.0f);

  // reset LEDs
  digitalWrite(ledRed, HIGH);
  digitalWrite(ledYellow, LOW);
  digitalWrite(ledBlue, LOW);

  if (dangerTilt) {
    digitalWrite(ledRed, LOW);
    digitalWrite(buzzerPin, HIGH);
  }
  else if (noMovement) {
    digitalWrite(ledBlue, HIGH);
    digitalWrite(buzzerPin, LOW);
  }
  else if (jitter) {
    digitalWrite(ledYellow, HIGH);
    digitalWrite(buzzerPin, LOW);
  }
  else {
    // fast motion but not danger — LEDs off
    digitalWrite(buzzerPin, LOW);
  }

  // plotting
  Serial.print(AnglePitch,2); Serial.print(",");
  Serial.print(AngleRoll,2);  Serial.print(",");
  Serial.print(servoX,2);     Serial.print(",");
  Serial.print(servoY,2);     Serial.print(",");
  Serial.println(dangerTilt ? 1 : 0);

  delay(8);
}
