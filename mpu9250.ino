#include "Wire.h"
#include <SPI.h>
#include <WiFi.h>
#include "mpu9250.h"
#include "mpu9250const.h"
#include "tmp117.h"

const char* ssid = "aithinker";
const char* ppsd = "aithinker888";
const char* hostname = "ball_001";
//const char* ssid = "partytime";
//const char* ppsd = "qwertyuiop";

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);

//  CmelodyMPU9250.I2Cscan();

  // We start by connecting to a WiFi network

  Serial.println();
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid, ppsd);
  
  while (WiFi.status() != WL_CONNECTED) {
      delay(500);
      Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected.");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());

  WiFi.softAPsetHostname(hostname);
//  WiFi.setHostname("Station_Taichi");
  Serial.println(WiFi.getHostname());
  CmelodyMPU9250.begin();
  CTmpDemo.begin();
  
}

void loop() {
//  CmelodyMPU9250.updatesensorbuf();
  CmelodyMPU9250.updatebuf();
  delay(1000);
}
