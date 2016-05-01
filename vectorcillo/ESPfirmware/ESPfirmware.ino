
#include <ESP8266WiFi.h>
#include <WiFiClient.h> 
#include <ESP8266WebServer.h>
#include <ESP8266mDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>

#include "credentials.h"
//const char ssid[] = "XXXX"; // network SSID (name)
//const char pass[] = "XXXX"; // network password

ESP8266WebServer server(80);

/* Just a little test message.  Go to http://192.168.4.1 in a web browser
 * connected to this access point to see it.
 */
void handleRoot() {
	server.send(200, "text/html", "<h1>You are connected</h1>");
}

#define PIN_SPEED_A 12
#define PIN_SPEED_B 13

#define PIN_DIR_A 15
#define PIN_DIR_B 4

#define PIN_SENSOR_R 14
#define PIN_SENSOR_L 16

#define PIN_BUTTON 0

void setup() {
  pinMode(PIN_BUTTON, INPUT);
  
  pinMode(PIN_SENSOR_L, INPUT_PULLUP);
  pinMode(PIN_SENSOR_R, INPUT_PULLUP);
  
  pinMode(PIN_SPEED_A, OUTPUT);
  pinMode(PIN_SPEED_B, OUTPUT);

  pinMode(PIN_DIR_A, OUTPUT);
  pinMode(PIN_DIR_B, OUTPUT);


  analogWrite(PIN_SPEED_A, 0);
  analogWrite(PIN_SPEED_B, 0);
  
  analogWrite(PIN_DIR_A, 0);
  analogWrite(PIN_DIR_B, 0);

  
	delay(1000);

  
	Serial.begin(115200);
	Serial.println();
	Serial.print("Configuring access point...");
	/* You can remove the password parameter if you want the AP to be open. */
	WiFi.softAP(ssid, pass);

	IPAddress myIP = WiFi.softAPIP();
	Serial.print("AP IP address: ");
	Serial.println(myIP);
	server.on("/", handleRoot);
	server.begin();
	Serial.println("HTTP server started");

  ArduinoOTA.onStart([]() {
    Serial.println("Start");
  });
  ArduinoOTA.onEnd([]() {
    Serial.println("\nEnd");
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
    else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
    else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
    else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
    else if (error == OTA_END_ERROR) Serial.println("End Failed");
  });
  ArduinoOTA.begin();
  Serial.println("Ready");
}

void loop() {
  ArduinoOTA.handle();
	server.handleClient();
  if(!digitalRead(PIN_BUTTON)) {
    analogWrite(PIN_DIR_B, 300);
    delay(1000);
    analogWrite(PIN_DIR_B, 0);
    analogWrite(PIN_DIR_A, 300);
    delay(1000);
    analogWrite(PIN_SPEED_B, 500);
    delay(3000);
  } else {
    analogWrite(PIN_SPEED_A, 0);
    analogWrite(PIN_SPEED_B, 0);
    analogWrite(PIN_DIR_A, 0);
    analogWrite(PIN_DIR_B, 0);
    
  }
}
