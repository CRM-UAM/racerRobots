
#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <ESP8266WebServer.h>
#include <ESP8266mDNS.h>
#include <ESP8266HTTPUpdateServer.h>
#include <WiFiUdp.h>

#include "credentials.h"
//const char ssid[] = "XXXX"; // network SSID (name)
//const char pass[] = "XXXX"; // network password

ESP8266WebServer httpServer(80);
ESP8266HTTPUpdateServer httpUpdater;
WiFiServer telnet_server(23);
WiFiClient telnet;

// Go to http://192.168.4.1 in a web browser
// connected to this access point to update it

#define PIN_SPEED_A 12
#define PIN_SPEED_B 13

#define PIN_DIR_A 15
#define PIN_DIR_B 4

#define PIN_SENSOR_R 14
#define PIN_SENSOR_L 16

#define PIN_BUTTON 0

bool buttonIsPressed() {
  return !digitalRead(PIN_BUTTON);
}

void turnOffMotors() {
  analogWrite(PIN_SPEED_A, 0);
  analogWrite(PIN_SPEED_B, 0);

  analogWrite(PIN_DIR_A, 0);
  analogWrite(PIN_DIR_B, 0);
}


void steering(int dir) {
  if (dir > 1000) dir = 400;
  if (dir < -1000) dir = -400;
  if (dir > 0) {
    analogWrite(PIN_DIR_A, dir);
    analogWrite(PIN_DIR_B, 0);
  } else {
    analogWrite(PIN_DIR_A, 0);
    analogWrite(PIN_DIR_B, -dir);
  }
  delay(1); // Yield some CPU cycles to ESP8266 (to avoid watchdog restarts)
}

//void setDir(int dir) {
  /*if (abs(dir) < 50) {
    while (!digitalRead(PIN_SENSOR_R) || !digitalRead(PIN_SENSOR_L)) {
      while (!digitalRead(PIN_SENSOR_R)) steering(500);
      while (!digitalRead(PIN_SENSOR_L)) steering(-500);
    }
  } else *//*if (dir > 0) {
    while (!digitalRead(PIN_SENSOR_R) || digitalRead(PIN_SENSOR_L)) steering(500);
  } else if (dir < 0) {
    while (digitalRead(PIN_SENSOR_R) || !digitalRead(PIN_SENSOR_L)) steering(-500);
  }
  steering(dir);
}*/


float gyro, maxAcc, maxAccLateral, line, lineDt, dir, dirDt, minDistFront, minDistL, minDistR, distDtFront, distDtL, distDtR;
int dirLeft=0, dirRight=0, dirIntegral=0;

void empty_sensor_queue() {
  while(Serial.available() > 0) Serial.read();
  while (Serial.read() != '\n');
}

void read_from_sensors() {
  gyro = Serial.parseFloat();
  maxAcc = Serial.parseFloat();
  maxAccLateral = Serial.parseFloat();
  line = Serial.parseFloat();
  lineDt = Serial.parseFloat();
  dir = Serial.parseFloat();
  dirDt = Serial.parseFloat();
  minDistFront = Serial.parseFloat();
  minDistL = Serial.parseFloat();
  minDistR = Serial.parseFloat();
  distDtFront = Serial.parseFloat();
  //distDtL = Serial.parseFloat();
  //distDtR = Serial.parseFloat();
}


void setup() {
  Serial.begin(115200);
  pinMode(PIN_BUTTON, INPUT);

  pinMode(PIN_SENSOR_L, INPUT_PULLUP);
  pinMode(PIN_SENSOR_R, INPUT_PULLUP);

  pinMode(PIN_SPEED_A, OUTPUT);
  pinMode(PIN_SPEED_B, OUTPUT);

  pinMode(PIN_DIR_A, OUTPUT);
  pinMode(PIN_DIR_B, OUTPUT);

  analogWriteFreq(20000); //

  turnOffMotors();

  delay(1000);

  Serial.begin(115200);

  Serial.print("\nConfiguring access point...");
  WiFi.softAP(ssid, pass);

  IPAddress myIP = WiFi.softAPIP();
  Serial.print("AP IP address: ");
  Serial.println(myIP);
  httpUpdater.setup(&httpServer);
  httpServer.begin();
  Serial.println("HTTP server started");

  delay(1000);

  telnet_server.begin();
  //telnet_server.setNoDelay(true);
  Serial.println("Telnet server started");

  Serial.println("Ready");

  steering(600);
  delay(1000);
  empty_sensor_queue();
  read_from_sensors();
  dirRight = dir;

  steering(-600);
  delay(1000);
  empty_sensor_queue();
  read_from_sensors();
  dirLeft = dir;
  
  steering(0);
}







int i=0;
void loop() {
  if (telnet_server.hasClient()) {
    if (telnet) telnet.stop();
    telnet = telnet_server.available();
  }
  
  if(Serial.available() > 0) {
    while (Serial.read() != '\n');
    read_from_sensors();
    float dirError = map(dir, dirLeft,dirRight, -1000,1000) - lineDt*0.1;
    if (telnet && telnet.connected() && i > 20) {
      telnet.println(dirError);
      i=0;
    }
    i++;
    dirIntegral += dirError;
    steering(-dirError);
    //steering(map(line, 7000, 0, -900, 900));
  }

  httpServer.handleClient();

  if (buttonIsPressed()) {
    analogWrite(PIN_SPEED_B, 0);
    delay(200);
    if (buttonIsPressed()) analogWrite(PIN_SPEED_B, 500);
    delay(500);
  } /*else {
    analogWrite(PIN_SPEED_A, 0);
    analogWrite(PIN_SPEED_B, 0);
    analogWrite(PIN_DIR_A, 0);
    analogWrite(PIN_DIR_B, 0);

  }*/
}
