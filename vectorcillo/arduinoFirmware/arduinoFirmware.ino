// Firmware for the "sensor reader" board in a racer robot

// Gyroscope code based on:
// MPU-6050 Short Example Sketch by Arduino User JohnChi

#include<Wire.h>
#include <QTRSensors.h>

const int MPU=0x68;  // I2C address of the MPU-6050

#define MPU6050_PWR_MGMT_1    0x6B
#define MPU6050_ACCEL_CONFIG  0x1C
#define MPU6050_GYRO_CONFIG   0x1B
#define MPU6050_SMPLRT_DIV    0x19
#define MPU6050_CONFIG        0x1A

void init_IMU() {
  for(int i=0; i<3; i++) { // Reset the IMU a few times
    IMUwriteReg(MPU6050_PWR_MGMT_1, bit(7) );  // DEVICE_RESET to 1 (D7=1)
    delay(100);
  }

  IMUwriteReg(MPU6050_PWR_MGMT_1, bit(0) | bit(1) ); // set clock source to Z Gyro (D0=D1=1, D2=0) and set SLEEP to zero (D6=0, wakes up the MPU-6050)

  IMUwriteReg(MPU6050_ACCEL_CONFIG, bit(3) | bit(4) ); // set sensitivity to +-16G (D3=1, D4=1) and disable high pass filter (D0,D1,D2=0)

  IMUwriteReg(MPU6050_GYRO_CONFIG, bit(3) | bit(4) ); // set sensitivity to +-2000deg/s (D3=1, D4=1)

  IMUwriteReg(MPU6050_SMPLRT_DIV, 0 ); // set sampling rate to 1khz (1khz / (1 + 0) = 1000 Hz)

  IMUwriteReg(MPU6050_CONFIG, bit(0) | bit(5) ); // disable digital low pass filter (D0=D1=D2=0) and EXT_SYNC to GYRO_ZOUT (D3=D4=0, D5=1)
}


//#define LED_RED_PIN     13
//#define LED_GREEN_PIN   9

#define BUZZER_PIN      8

//#define M1A_PIN         11
//#define M2A_PIN         10
//#define M4A_PIN         5
//#define M3A_PIN         6

#define BUTTON_PIN      7

//#define LINE_R_PIN      2
//#define LINE_M_PIN      3
//#define LINE_L_PIN      4


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
unsigned int defAnalogPIN[5]={DIST_1_PIN,DIST_2_PIN,DIST_3_PIN,DIST_4_PIN,DIST_5_PIN};




int16_t AcY, GyZ;
int16_t AcYoffset, GyZoffset;
float AcY_integral, GyZ_integral;

unsigned long prev_ts;

void IMUwriteReg(byte reg, byte val) {
  Wire.begin();
  Wire.beginTransmission(MPU);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission(true);
}



QTRSensorsRC qtrrc((unsigned char[]) {IR1, IR2, IR3, IR4, IR5, IR6, IR7, IR8}, NUM_IR_SENSORS, TIMEOUT, QTR_NO_EMITTER_PIN); //emisor siempre encendido

void calibrateIR( int time, bool printflag ){
  int i;
  //ledOn();
  for (i = 0; i < 40*time; i++)
  {
    qtrrc.calibrate();       // reads all sensors 10 times at 2500 us per read (i.e. ~25 ms per call)
  }
  //ledOff();
  //print results
  if(printflag)Serial.println("Resultados\nMinimos:");
  for (i = 0; i < NUM_IR_SENSORS; i++){
    if(printflag)Serial.print(qtrrc.calibratedMinimumOn[i]);
    if(printflag)Serial.print(" ");
  }
  if(printflag)Serial.println();
  if(printflag)Serial.println("Maximos:");
  for (i = 0; i < NUM_IR_SENSORS; i++){
    if(printflag)Serial.print(qtrrc.calibratedMaximumOn[i]);
    if(printflag)Serial.print(" ");
  }
  if(printflag)Serial.println();
}


void setup() {
  delay(400);

  //init_motor_pins();
  //init_button_pin();
  //pinMode(LED_RED_PIN, OUTPUT);
  //pinMode(LED_GREEN_PIN, OUTPUT);

  //digitalWrite(LED_RED_PIN, HIGH);

  init_IMU();

  Serial.begin(115200);
  delay(1000);

  // Measure IMU sensor offsets (robot must remain still)
  AcYoffset = 0;
  GyZoffset = 0;
  for(int i=0; i<10; i++) {
    readIMU(&AcY, &GyZ);
    AcYoffset += AcY;
    GyZoffset += GyZ;
  }
  AcYoffset /= 10;
  GyZoffset /= 10;
  Serial.println("Offset:");
  Serial.println(AcYoffset);
  Serial.println(GyZoffset);


  //digitalWrite(LED_RED_PIN, LOW);
  //digitalWrite(LED_GREEN_PIN, HIGH);

  //while(!button_is_pressed());

  prev_ts = millis();

  AcY_integral = 0;
  GyZ_integral = 0;
}

void readIMU(int16_t *AcY, int16_t *GyZ) {
  Wire.beginTransmission(MPU);
  Wire.write(0x3D);  // starting with register 0x3D (ACCEL_YOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU,12,true);  // request a total of 12 registers
  *AcY = Wire.read()<<8|Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  for(int i=0; i<8; i++) Wire.read(); // Discard 0x3F-0x46
  *GyZ = Wire.read()<<8 | Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
}



float linear_motion_big = 0;
unsigned long contTs = 0;
unsigned int line_pos_pre=0;
unsigned long last_time_ir=0;
double line_derivate=0;
unsigned int line_pos=0;
unsigned int values[NUM_IR_SENSORS];
unsigned int dist_value[NUM_DIST_SENSORS];
unsigned int dist_min_values[NUM_DIST_SENSORS]={2000};


inline void sendValues2ESP(){
    int i;
    //Serial.print(micros()/1000000.);
    //Serial.print(",");
    Serial.print(line_pos);
    Serial.print(",");
    Serial.print(line_derivate);
    Serial.print(",");
    for(i=0;i<NUM_DIST_SENSORS;i++){
        Serial.print(dist_value[i]);
        Serial.print(",");
    }
    for(i=0;i<NUM_IR_SENSORS;i++){
        Serial.print(values[i]);
        Serial.print(",");
    }
    Serial.println("");
}

void loop() {
  int i;
  readIMU(&AcY, &GyZ);
  unsigned long ts = micros();
  float dt = ((float)(ts-prev_ts))/1000000.;
  if(dt > 0) { // Ensures the integration only over sensible sampling intervals
    GyZ_integral += (GyZ-GyZoffset)*dt;
  }
  prev_ts = ts;

  if( (micros()-last_time_ir) > 1000){ //samplin IR min 1ms
    line_pos = qtrrc.readLine(values);
    line_derivate = (1.*line_pos - 1.*line_pos_pre)/((micros()-last_time_ir)/1000000.) ;
    last_time_ir=micros();
  }



  for(i=0;i=NUM_DIST_SENSORS;i++){
    unsigned int read=analogRead(defAnalogPIN[i]);
    if(read < dist_min_values[i] ){
        dist_min_values[i]=read;
    }
  }

  if(Serial.available()>0){
    sendValues2ESP();
    while(Serial.available()>0)Serial.read();
  }

}

