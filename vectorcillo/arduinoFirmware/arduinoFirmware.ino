// Firmware for the "sensor reader" board in a racer robot

// Gyroscope code based on:
// MPU-6050 Short Example Sketch by Arduino User JohnChi

#include<Wire.h>
#include <QTRSensors.h>

#define COS45 0.70710678118

/*ARRAY IR POLOLU*/
#define IR1 2
#define IR2 3
#define IR3 4
#define IR4 5
#define IR5 6
#define IR6 7
#define IR7 8
#define IR8 9
#define TIMEOUT 2000
#define NUM_IR_SENSORS 8
/*****************/


#define NUM_DIST_SENSORS 3
#define DIST_5_PIN      A4
#define DIST_4_PIN      A3
#define DIST_3_PIN      A2
#define DIST_2_PIN      A1
#define DIST_1_PIN      A0
unsigned int defAnalogPIN[5] = {DIST_1_PIN, DIST_2_PIN, DIST_3_PIN, DIST_4_PIN, DIST_5_PIN};



unsigned int line_pos_pre = 0;
unsigned long last_time_line = 0, last_time_IMU = 0, last_time_distance = 0, last_time_sent = 0;
float line_derivate = 0;
unsigned int line_pos = 0;
unsigned int line_values[NUM_IR_SENSORS];
float dist_values[NUM_DIST_SENSORS];
float dist_min_values[NUM_DIST_SENSORS] = {1000};
float dist_last_values[NUM_DIST_SENSORS] = {0};
float dist_derivate_values[NUM_DIST_SENSORS] = {0};

QTRSensorsRC qtrrc((unsigned char[]) {
  IR1, IR2, IR3, IR4, IR5, IR6, IR7, IR8
}, NUM_IR_SENSORS, TIMEOUT, QTR_NO_EMITTER_PIN); // IR emitter is always ON


const int MPU = 0x68; // I2C address of the MPU-6050

#define MPU6050_PWR_MGMT_1    0x6B
#define MPU6050_ACCEL_CONFIG  0x1C
#define MPU6050_GYRO_CONFIG   0x1B
#define MPU6050_SMPLRT_DIV    0x19
#define MPU6050_CONFIG        0x1A

int16_t AcX = 0, AcY = 0, GyZ = 0;
float max_accel = 0, max_accel_lateral = 0;
int16_t AcXoffset, AcYoffset, GyZoffset;
float GyZ_integral;


void IMUwriteReg(byte reg, byte val) {
  Wire.begin();
  Wire.beginTransmission(MPU);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission(true);
}

bool readIMU(int16_t *AcX, int16_t *AcY, int16_t *GyZ) {
  int16_t new_AcX, new_AcY, new_GyZ;
  bool updated;
  Wire.beginTransmission(MPU);
  Wire.write(0x3B);  // starting with register 0x3D (ACCEL_YOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 14, true); // request a total of 14 registers
  new_AcX = Wire.read() << 8 | Wire.read(); // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
  new_AcY = Wire.read() << 8 | Wire.read(); // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  for (int i = 0; i < 8; i++) Wire.read(); // Discard 0x3F-0x46
  new_GyZ = Wire.read() << 8 | Wire.read(); // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)

  updated = (*AcX != new_AcX) || (*AcY != new_AcY) || (*GyZ != new_GyZ); // Check if value has changed

  *AcX = new_AcX;
  *AcY = new_AcY;
  *GyZ = new_GyZ;
  return updated;
}

void init_IMU() {
  Serial.println(F("Calibrating IMU..."));
  for (int i = 0; i < 3; i++) { // Reset the IMU a few times
    IMUwriteReg(MPU6050_PWR_MGMT_1, bit(7) );  // DEVICE_RESET to 1 (D7=1)
    delay(100);
  }

  IMUwriteReg(MPU6050_PWR_MGMT_1, bit(0) | bit(1) ); // set clock source to Z Gyro (D0=D1=1, D2=0) and set SLEEP to zero (D6=0, wakes up the MPU-6050)

  IMUwriteReg(MPU6050_ACCEL_CONFIG, bit(3) | bit(4) ); // set sensitivity to +-16G (D3=1, D4=1) and disable high pass filter (D0,D1,D2=0)

  IMUwriteReg(MPU6050_GYRO_CONFIG, bit(3) | bit(4) ); // set sensitivity to +-2000deg/s (D3=1, D4=1)

  IMUwriteReg(MPU6050_SMPLRT_DIV, 0 ); // set sampling rate to 1khz (1khz / (1 + 0) = 1000 Hz)

  IMUwriteReg(MPU6050_CONFIG, bit(0) | bit(5) ); // disable digital low pass filter (D0=D1=D2=0) and EXT_SYNC to GYRO_ZOUT (D3=D4=0, D5=1)

  delay(1000);

  // Measure IMU sensor offsets (robot must remain still)
  AcXoffset = 0;
  AcYoffset = 0;
  GyZoffset = 0;
  for (int i = 0; i < 10; i++) {
    readIMU(&AcX, &AcY, &GyZ);
    AcXoffset += AcX;
    AcYoffset += AcY;
    GyZoffset += GyZ;
    delay(10);
  }
  AcXoffset /= 10;
  AcYoffset /= 10;
  GyZoffset /= 10;
  Serial.println(F("IMU offsets:"));
  Serial.println(AcXoffset);
  Serial.println(AcYoffset);
  Serial.println(GyZoffset);
}





void calibrateIR(int time) {
  int i;
  Serial.println(F("Calibrating line sensor... please sweep the sensor across a black line a few times"));
  //ledOn();
  qtrrc.resetCalibration();
  unsigned long targetTime = millis() + 5000;
  while (millis() < targetTime) {
    /*qtrrc.read(line_values);
      for(i=0; i<NUM_IR_SENSORS; i++){
        Serial.print(line_values[i]);
        Serial.print(" ");
      }
      Serial.println();*/
    qtrrc.calibrate(); // reads all sensors at 2500 us per read (i.e. ~25 ms per call)
  }
  //ledOff();
  //print results
  Serial.println(F("Results\nMinimum: "));
  for (i = 0; i < NUM_IR_SENSORS; i++) {
    Serial.print(qtrrc.calibratedMinimumOn[i]);
    Serial.print(F(" "));
  }
  Serial.println();
  Serial.print(F("Maximum: "));
  for (i = 0; i < NUM_IR_SENSORS; i++) {
    Serial.print(qtrrc.calibratedMaximumOn[i]);
    Serial.print(F(" "));
  }
  Serial.println();
}


float analogReadAverage(int pin, int samples) {
  float result = 0;
  for (int i = 0; i < samples; i++) {
    result += analogRead(pin);
    delayMicroseconds(100);
  }
  return result / (float)samples;
}

float getDistanceCM(int pin) {
  float res = 4419.36 / ((float)analogReadAverage(pin, 4) - 32.736); // Convert ADC value to centimeters
  if (res < 0 || res > 150) res = 150;
  return res;
}


void setup() {
  delay(400);

  Serial.begin(250000);
  Serial.println(F("Starting up sensor board..."));

  init_IMU();

  calibrateIR(5);

  GyZ_integral = 0;
}




void sendValues2ESP() {
  int i;
  //Serial.print(micros()/1000000.);
  //Serial.print(",");
  //Serial.print(F("Gyr: "));
  Serial.print((GyZ_integral / 3000.) * 180.);
  Serial.print(F(" "));
  
  //Serial.print(F("maxAccXY: "));
  Serial.print(max_accel);
  Serial.print(F(" "));
  Serial.print(max_accel_lateral);
  Serial.print(F(" "));
  
  //Serial.print(F("Line: "));
  Serial.print(line_pos);
  Serial.print(F(" "));
  
  //Serial.print(F("LineDt: "));
  Serial.print(line_derivate);
  Serial.print(F(" "));
  
  //Serial.print(F("minDist: "));
  for (i = 0; i < NUM_DIST_SENSORS; i++) {
    Serial.print(dist_min_values[i]);
    Serial.print(F(" "));
  }

  //Serial.print(F(" DistDt: "));
  for (i = 0; i < NUM_DIST_SENSORS; i++) {
    Serial.print(dist_derivate_values[i]);
    Serial.print(F(" "));
  }
  /*for(i=0; i<NUM_IR_SENSORS; i++){
      Serial.print(line_values[i]);
      Serial.print(" ");
    }*/
  Serial.println();
}


void loop() {
  int i;
  bool sendValues = false;
  bool changed = readIMU(&AcX, &AcY, &GyZ);// check if value has been updated
  unsigned long current = micros();
  float dt = ((float)(current - last_time_IMU)) / 1000000.;
  if (changed) {
    if (dt > 0) {
      GyZ_integral += (GyZ - GyZoffset) * dt;
      last_time_IMU = current;
    }
    float accel = (AcX - AcXoffset) * COS45 + (AcY - AcYoffset) * COS45; // The sensor is rotated 45 degrees, we need to account for that
    float accel_lateral = (AcX - AcXoffset) * COS45 - (AcY - AcYoffset) * COS45;
    if (abs(accel) > abs(max_accel)) max_accel = accel;
    if (abs(accel_lateral) > abs(max_accel_lateral)) max_accel_lateral = accel_lateral;
  }

  line_pos = qtrrc.readLine(line_values);
  current = micros();
  dt = ((float)(current - last_time_line)) / 1000000.;
  if (dt > 0.1) {
    line_derivate = ((float)line_pos - (float)line_pos_pre) / dt;
    line_pos_pre = line_pos;
    last_time_line = current;
  }

  for (i = 0; i < NUM_DIST_SENSORS; i++) {
    dist_values[i] = getDistanceCM(defAnalogPIN[i]);
    if (dist_values[i] < dist_min_values[i]) dist_min_values[i] = dist_values[i];
  }
  current = micros();
  dt = ((float)(current - last_time_distance)) / 1000000.;
  if (dt > 0.1) {
    for (i = 0; i < NUM_DIST_SENSORS; i++) dist_derivate_values[i] = (dist_values[i] - dist_last_values[i]) / dt;
    for (i = 0; i < NUM_DIST_SENSORS; i++) dist_last_values[i] = dist_values[i];
    last_time_distance = current;
  }

  current = micros();
  dt = ((float)(current - last_time_sent)) / 1000000.;
  if (dt > 0.05) {//Serial.available() > 0) { // RX pin is dead :-(
    sendValues2ESP();
    //while (Serial.available() > 0) Serial.read();
    for (i = 0; i < NUM_DIST_SENSORS; i++) dist_min_values[i] = 1000;
    max_accel = 0;
    max_accel_lateral = 0;
    last_time_sent = current;
  }

}

