
#include <SPI.h>
#include <WiFiNINA.h>
#include <WiFiUdp.h>
#include <RTCZero.h>
#include <I2CSoilMoistureSensor.h>
#include <Wire.h>
#include <ThingSpeak.h>

RTCZero rtc;

#include "secret.h"
//Wifi
char ssid[] = SECRET_SSID;    //network SSID
char pass[] = SECRET_PASS;    //network password (use for WPA, or use as key for WEP)

int status = WL_IDLE_STATUS;
// Initialize the Wifi client library
WiFiClient client;

// server address:
//char server[] = "sensorscanada.ca";
char server[] = "https://api.thingspeak.com";
//IPAddress server(64,131,82,241);
#define PORT 80 //port we comunicate on


unsigned long lastConnectionTime = 0;            // last time you connected to the server, in milliseconds
const unsigned long postingInterval = 20L * 2000L; // delay between updates, in milliseconds

//Sensors
// Define our pins.
const int An_S1_In = A0;
//const int An_S1_Power = 12;

const int SENSOR_DELAY = 10; // in milliseconds

I2CSoilMoistureSensor Sensor_i2c_1;

//these should be in the secret file but I put them here so you all can get at them
#define Thing_Read_Key "9ZNNL6X3JJH1M1LV"
#define Thing_Write_Key "ZFDRBL1QJVBYYO2G"

// ThingSpeak information
char* writeAPIKey = Thing_Write_Key;
char* readAPIKey = Thing_Read_Key; 
const long channelID = 824844;

const int Array_Size = 10;

void setup() {
  // put your setup code here, to run once:
  int Time_Stamp[4];
  Serial.begin(9600);
  wifi_connect();
  //Clock

  //pin for analog sensor may be moved to an external function later
  pinMode(An_S1_In, INPUT);
  //set up i2c sensor
  I2C_sensor_setup();
}

void loop() {
  int sum_Analog = 0;
  int count = 0;
  float result_Analog;
  float result_I2C;

  float sum_I2C_humidity = 0;
  float I2C_Temp;
  float I2C_Light;
  while(1){
       // if there's incoming data from the net connection.
    // send it out the serial port.  This is for debugging
    // purposes only: REMOVE FROM FINAL VERSION
    while (client.available()) {
      char c = client.read();
      Serial.write(c);
    }
    //float Analog_Average;
    //float I2C_average;
    // if enough time has passed connect again:
    if (millis() - lastConnectionTime > postingInterval) {
       //get the average of out values
      result_Analog= (float) sum_Analog/count;
      sum_Analog=0;
      result_I2C= (float) sum_I2C_humidity/count;
      sum_I2C_humidity=0;
      
      //I2C sensor reads no averaging
      I2C_Temp = Sensor_i2c_1.getTemperature()/(float)10;
      I2C_Light = Sensor_i2c_1.getLight(true);

      
      lastConnectionTime = millis();
      Serial.print("Sensor Value: ");
      Serial.println(An_Sensor_read());
      Serial.print("Sensor Value ave: ");
      Serial.println(result_Analog);
      Serial.print("Count: ");
      Serial.println(count);
      count=0;
      I2C_sensor_read();
      ThingSpeak.begin(client);
      int  writeSuccess = ThingSpeak.writeField(channelID, 2,result_Analog, writeAPIKey);
      writeSuccess = ThingSpeak.writeField(channelID, 1,result_I2C, writeAPIKey);
      writeSuccess = ThingSpeak.writeField(channelID, 3,I2C_Temp, writeAPIKey);
      writeSuccess = ThingSpeak.writeField(channelID, 4,I2C_Light, writeAPIKey);   
    }
    //things we average
    //Analog
    sum_Analog = sum_Analog + An_Sensor_read();

    //I2C
    sum_I2C_humidity = sum_I2C_humidity+Sensor_i2c_1.getCapacitance();
    
    count++;
    delay(4000);
    
  }
 
}

void wifi_connect(){
  if (WiFi.status() == WL_NO_SHIELD) {
    Serial.println("WiFi shield not present");
    // don't continue:
    while (true);
  }

  // attempt to connect to WiFi network:
  while ( status != WL_CONNECTED) {
    Serial.print("Attempting to connect to SSID: ");
    Serial.println(ssid);
    // Connect to WPA/WPA2 network.
    status = WiFi.begin(ssid, pass);

    // wait 10 seconds for connection:
    delay(10000);
  }
  printWiFiStatus();  // you're connected now, so print out the status:
  return;
  
}

void printWiFiStatus() {
  // print the SSID of the network you're attached to:
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

  // print your WiFi shield's IP address:
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);

  // print the received signal strength:
  long rssi = WiFi.RSSI();
  Serial.print("signal strength (RSSI):");
  Serial.print(rssi);
  Serial.println(" dBm");
}


int An_Sensor_read(){
   int value=0;
    // Take reading
    value=(analogRead(An_S1_In));
    return value; 
}

void I2C_sensor_setup(){
    //Sensor Code
  
  Wire.begin();
  // reset sensor
  Sensor_i2c_1.begin();
  delay(1000); // give some time to boot up
  Serial.print("I2C Soil Moisture Sensor Address: ");
  Serial.println(Sensor_i2c_1.getAddress(),HEX);
  Serial.print("Sensor Firmware version: ");
  Serial.println(Sensor_i2c_1.getVersion(),HEX);
  Serial.println();
}

//this function needs to return values. Probably a string or we could pass it a pointer to edit
void I2C_sensor_read(){
  Serial.print("Soil Moisture Capacitance: ");
  Serial.print(Sensor_i2c_1.getCapacitance()); //read capacitance register
  Serial.print(", Temperature: ");
  Serial.print(Sensor_i2c_1.getTemperature()/(float)10); //temperature register
  Serial.print(", Light: ");
  Serial.println(Sensor_i2c_1.getLight(true)); //request light measurement, wait and read light register
  Sensor_i2c_1.sleep(); // available since FW 2.3 
}


