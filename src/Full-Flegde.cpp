#include<Wire.h>
#include<Servo.h>
#include<Adafruit_Sensor.h>
#include<Adafruit_BMP280.h>
Adafruit_BMP280 bmp; 
int servoPin = 4; //X servo
int servoPin2 = 5; //Y servo
Servo X;
Servo Y;

float RateRoll,RatePitch, RateYaw;
float RateCalibrationRoll, RateCalibrationPitch, RateCalibrationYaw;
int RateCalibrationNumber;
float AccX, AccY, AccZ;
float AccZInertial;
float AngleRoll, AnglePitch;
uint32_t LoopTimer;
uint16_t dig_T1,dig_P1;
int16_t dig_T2, dig_T3, dig_P2, dig_P3, dig_P4, dig_P5;
int16_t dig_P6, dig_P7, dig_P8, dig_P9;

float AltitudeBarometer, AltitudeBarometerStartUp;
float KalmanAngleRoll = 0, KalmanUncertainityAngleRoll = 2*2;
float KalmanAnglePitch = 0, KalmanUncertainityAnglePitch = 2*2;
float Kalman1DOutput[] = {0,0};

void kalman_1d(float KalmanState, float KalmanUncertainity, float KalmanInput, float KalmanMeasurement){
  KalmanState = KalmanState + 0.004*KalmanInput;
  KalmanUncertainity = KalmanUncertainity + 0.004*0.004*4*4;
  float KalmanGain = KalmanUncertainity * 1/(1*KalmanUncertainity + 3*3);
  KalmanState = KalmanState + KalmanGain*(KalmanMeasurement - KalmanState);
  KalmanUncertainity = (1 - KalmanGain)* KalmanUncertainity;
  Kalman1DOutput[0] = KalmanState;
  Kalman1DOutput[1] = KalmanUncertainity;
}
#include<BasicLinearAlgebra.h>
using namespace BLA;
float AltitudeKalman, VelocityVerticalKalman;
BLA::Matrix<2,2>F; BLA::Matrix<2,1>G;
BLA::Matrix<2,2>P; BLA::Matrix<2,2>Q;
BLA::Matrix<2,1>S; BLA::Matrix<1,2>H;
BLA::Matrix<2,2>I; BLA::Matrix<1,1>Acc;
BLA::Matrix<2,1>K; BLA::Matrix<1,1>R;
BLA::Matrix<1,1>L; BLA::Matrix<1,1>M;

void kalman_2d(void){
  Acc = {AccZInertial};
  S = F*S + G*Acc;
  P = F * P * (~F) + Q;
  Matrix<1,1> L = H * P * (~H) + R;
  bool success = Invert(L); // Invert L in place
  if (!success) {
    K = {0, 0}; // Handle non-invertible case
  } else {
    K = P * (~H) * L; // Compute Kalman gain
  }
  M = {AltitudeBarometer};
  S = S+K*(M-H*S);
  AltitudeKalman = S(0,0);
  VelocityVerticalKalman = S(1,0);
  P = (I-K*H)*P;

}
void barometer_signals(){
  Wire.beginTransmission(0x76);
  Wire.write(0xF7);
  Wire.endTransmission();
  Wire.requestFrom(0x76,6);
  uint32_t press_msb = Wire.read();
  uint32_t press_lsb = Wire.read();
  uint32_t press_xlsb = Wire.read();
  uint32_t temp_msb = Wire.read();
  uint32_t temp_lsb = Wire.read();
  uint32_t temp_xlsb = Wire.read();

  unsigned long int adc_P = (press_msb << 12) | (press_lsb << 4) | (press_xlsb >> 4);
  unsigned long int adc_T = (temp_msb << 12) | (temp_lsb << 4) | (temp_xlsb >> 4);

  signed long int var1, var2;
  var1  = ((((adc_T>>3) - ((signed long int)dig_T1<<1))) * ((signed long int)dig_T2)) >> 11;
  var2  = (((((adc_T>>4) - ((signed long int)dig_T1)) * ((adc_T>>4) - ((signed long int)dig_T1))) >> 12) *  ((signed long int)dig_T3)) >> 14;

  signed long int t_fine = var1 + var2;

  unsigned long int p;
  var1 = (((signed long int)t_fine)>>1) - (signed long int)64000;
  var2 = (((var1>>2)*(var1>>2))>>11)*((signed long int)dig_P6);
  var2 = var2 + ((var1*((signed long int)dig_P5))<<1);
  var2 = (var2>>2) + (((signed long int)dig_P4)<<16);

  var1 = (((dig_P3*(((var1>>2)*(var1>>2))>>13))>>3)+((((signed long int)dig_P2)*var1)>>1))>>18;
  var1 = ((((32768+var1))*((signed long int)dig_P1))>>15);

  if(var1 == 0) {p=0;}
  p = (((unsigned long int)(((signed long int)1048576)-adc_P)-(var2>>12)))*3125;
  if(p<0x80000000) {p = (p << 1) / ((unsigned long int)var1);}
  else{p = (p / (unsigned long int)var1)*2;}
  var1 = (((signed long int)(p>>2))*((signed long int)dig_P8))>>13;
  p = (unsigned long int)((signed long int)p + ((var1 + var2 + dig_P7)>>4));

  double pressure = (double)p/100;
  AltitudeBarometer = 44330*(1-pow(pressure/1013.25,1/5.255))*100; 
}

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
  AccY = (float)AccYLSB / 4096 - 5.7+1;
  AccZ = (float)AccZLSB / 4096 + 0.15;

  AngleRoll = atan(AccY / sqrt(AccX*AccX+AccZ*AccZ))*1/(3.142/180);
  AnglePitch = atan(AccX / sqrt(AccY*AccY+AccZ*AccZ))*1/(3.142/180);
}
void setup() {
  Serial.begin(57600);
  Serial.println(F("BMP280 test"));
  if (!bmp.begin(0x76)) {   
    Serial.println(F("Could not find BMP280 sensor!"));
    while (1);
  }
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     
                  Adafruit_BMP280::SAMPLING_X2,     
                  Adafruit_BMP280::SAMPLING_X16,    
                  Adafruit_BMP280::FILTER_X16,      
                  Adafruit_BMP280::STANDBY_MS_500); 

  pinMode(19,OUTPUT);
  digitalWrite(19,HIGH);
  Wire.setClock(400000);
  Wire.begin();
  delay(250);
  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();

  Wire.beginTransmission(0x76);
  Wire.write(0x74);
  Wire.write(0x57);
  Wire.endTransmission();
  Wire.beginTransmission(0x76);
  Wire.write(0xF5);
  Wire.write(0x14);
  Wire.endTransmission();
  uint8_t data[24], i=0;
  Wire.beginTransmission(0x76);
  Wire.write(0x88);
  Wire.endTransmission();
  Wire.requestFrom(0x76, 24);
  while(Wire.available()){
    data[i] = Wire.read();
    i++;
  }
  dig_T1 = (data[1] << 8) | data[0];
  dig_T2 = (data[3] << 8) | data[2];
  dig_T3 = (data[5] << 8) | data[4];
  dig_P1 = (data[7] << 8) | data[6];
  dig_P2 = (data[9] << 8) | data[8];
  dig_P3 = (data[11] << 8) | data[10];
  dig_P4 = (data[13] << 8) | data[12];
  dig_P5 = (data[15] << 8) | data[14];
  dig_P6 = (data[17] << 8) | data[16];
  dig_P7 = (data[19] << 8) | data[18];
  dig_P8 = (data[21] << 8) | data[20];
  dig_P9 = (data[23] << 8) | data[22];
  delay(250);

  for(RateCalibrationNumber = 0; RateCalibrationNumber < 2000; RateCalibrationNumber++)
  {
    gyro_signals();
    RateCalibrationRoll += RateRoll;
    RateCalibrationPitch += RatePitch;
    RateCalibrationYaw += RateYaw;
    barometer_signals();
    AltitudeBarometerStartUp+= AltitudeBarometer;
    delay(1);
  }
  AltitudeBarometerStartUp /= 2000;

  F = {1, 0.004, 0, 1}; // State transition matrix (example)
  G = {0.5*0.004*0.004, 0.004};
  P = {0, 0, 0, 0}; // Covariance matrix
  H = {1, 0}; // Observation matrix
  Q = (G * ~G) * (10.0f * 10.0f); // Process noise covariance (computed in setup)
  R = {30*30}; // Measurement noise covariance (30*30)
  S = {0, 0}; // State vector
  I = {1, 0, 0, 1};
  
  LoopTimer = micros();
  RateCalibrationRoll /= 2000;
  RateCalibrationPitch /= 2000;
  RateCalibrationYaw /= 2000;
  LoopTimer = micros();
  
  X.attach(servoPin);
  Y.attach(servoPin2);
}
void loop(){
  gyro_signals();
  RateRoll -= RateCalibrationRoll;
  RatePitch -= RateCalibrationPitch;
  RateYaw -= RateCalibrationYaw;

  AccZInertial = -sin(AnglePitch * (PI / 180)) * AccX + cos(AnglePitch * (PI / 180)) * sin(AngleRoll * (PI / 180)) * AccY;
  AccZInertial = (AccZInertial-1)*9.81*100;

  barometer_signals();
  AltitudeBarometer -= AltitudeBarometerStartUp;
  kalman_2d();

  kalman_1d(KalmanAngleRoll, KalmanUncertainityAngleRoll, RateRoll, AngleRoll);
  KalmanAngleRoll = Kalman1DOutput[0];
  KalmanUncertainityAngleRoll = Kalman1DOutput[1];
  kalman_1d(KalmanAnglePitch, KalmanUncertainityAnglePitch, RatePitch, AnglePitch);
  KalmanAnglePitch = Kalman1DOutput[0];
  KalmanUncertainityAnglePitch = Kalman1DOutput[1];

  //Accelerometer
  Serial.print("Ax ");
  Serial.print(AccX);
  Serial.print(" ");
  Serial.print("Ay ");
  Serial.print(AccY);
  Serial.print(" ");
  Serial.print("Az ");
  Serial.println(AccZ);
  
  //Angles
  Serial.print("RateRoll[*]= ");
  Serial.print(KalmanAngleRoll);
  Serial.print(" ");
  Serial.print("RatePitch[*]= ");
  Serial.println(KalmanAnglePitch);

  //Alt and Vert velocity(Kalman 2D)
  Serial.print(AltitudeKalman);
  Serial.print(" ");
  Serial.println(VelocityVerticalKalman);

  //BMP280 data
  Serial.print(bmp.readTemperature());
  Serial.print(",");         
  Serial.print(bmp.readPressure()/100); 
  Serial.print(",");
  Serial.println(bmp.readAltitude(1013.23)); 
  while (micros() - LoopTimer < 4000);
  LoopTimer = micros();
  if(AccX>=0.50){
    for(int posY =75; posY<=0; posY++){
      Y.write(posY);
      
      delay(1);
    }
  }
  else{
    Y.write(90);
    X.write(75);
  }
  if(AccX<0){
    for(int posY =160; posY>=75; posY--){
      Y.write(posY);
      
      delay(1);
  }
  }
  else{
    Y.write(90);
    X.write(75);
  }
  if(AccZ>=0.50){
    for(int posZ = 0; posZ <=30; posZ++){
      X.write(posZ);
      delay(1);
    }
  }
    else
    {
      Y.write(90);
      X.write(75);
    }
  if(AccZ<= -0.50){
    for(int posZ = 160; posZ>=160; posZ--){
      X.write(posZ);
      delay(1);
    }
  }
    else
    {
      Y.write(90);
      X.write(75);
    }
  }