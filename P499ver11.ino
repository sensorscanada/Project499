
#include <SPI.h>
#include <WiFiNINA.h>
#include <WiFiUdp.h>
#include <RTCZero.h>
#include <I2CSoilMoistureSensor.h>
#include <Wire.h>

RTCZero rtc;

#include "secret.h"
//Wifi
char ssid[] = SECRET_SSID;    //network SSID
char pass[] = SECRET_PASS;    //network password (use for WPA, or use as key for WEP)

int status = WL_IDLE_STATUS;
// Initialize the Wifi client library
WiFiClient client;

// server address:
char server[] = "sensorscanada.ca";
//IPAddress server(64,131,82,241);
#define PORT 80 //port we comunicate on


unsigned long lastConnectionTime = 0;            // last time you connected to the server, in milliseconds
const unsigned long postingInterval = 20L * 1000L; // delay between updates, in milliseconds

const int GMT = 7; //change this to adapt it to your time zone note this will be negative

//Sensors
// Define our pins.
const int An_S1_In = A0;
//const int An_S1_Power = 12;

const int SENSOR_DELAY = 10; // in milliseconds

I2CSoilMoistureSensor Sensor_i2c_1;

void setup() {
  // put your setup code here, to run once:
  int Time_Stamp[4];
  Serial.begin(9600);
  wifi_connect();
  //Clock
  rtc.begin();
  rtc.setEpoch(get_epoch(10));
  set_rtc();//set the correct time

  //pin for analog sensor may be moved to an external function later
  pinMode(An_S1_In, INPUT);
  //set up i2c sensor
  I2C_sensor_setup();

}

void loop() {

  // if there's incoming data from the net connection.
  // send it out the serial port.  This is for debugging
  // purposes only: REMOVE FROM FINAL VERSION
  while (client.available()) {
    char c = client.read();
    Serial.write(c);
  }
  // if ten seconds have passed since your last connection,
  // then connect again and send data:
  if (millis() - lastConnectionTime > postingInterval) {
    //httpRequest();
    lastConnectionTime = millis();
    Serial.print("Sensor Value: ");
    Serial.println(An_Sensor_read());
  }
  
  //printDate();
  //printTime();
  //Serial.println();
  delay(1000);

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

unsigned long get_epoch(int maxTries){//get time from internets
  unsigned long epoch;
  int numberOfTries = 0;
  do {
    epoch = WiFi.getTime();
    numberOfTries++;
  }
  while ((epoch == 0) && (numberOfTries < maxTries));

  if (numberOfTries == maxTries) {
    Serial.print("NTP unreachable!!");
    return(0);
  }
  else {
    Serial.print("Epoch received: ");
    Serial.println(epoch);
    Serial.println();
    return(epoch);
  }
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

void set_rtc(){
  int temp_Hours = rtc.getHours();
  int temp_Day = rtc.getDay();
  int temp_Month = rtc.getMonth();
  int temp_Year = rtc.getYear();
  if (temp_Hours<GMT){
      temp_Hours+=(24-GMT);
      if(temp_Day=1){
        switch (temp_Month){
          case 1:
            temp_Month=12;
            temp_Day=31;
            --temp_Year;
            break;
          case 2:
          case 4:
          case 6:
          case 8:
          case 9:
          case 11:
            temp_Day=31;
            temp_Month--;
            break;
          case 3:
            temp_Day=29;
            temp_Month--;
            break;
          case 5:
          case 7:
          case 10:
            temp_Day=30;
            temp_Month--;
            break;
          default:
            break;
          }
      }
  }else temp_Hours -= GMT;

  rtc.setHours(temp_Hours);
  rtc.setDay(temp_Day);
  rtc.setMonth(temp_Month);
  rtc.setYear(temp_Year);
  printDate();
  printTime();
  
}

void printTime()
{
  print2digits(rtc.getHours());
  Serial.print(":");
  print2digits(rtc.getMinutes());
  Serial.print(":");
  print2digits(rtc.getSeconds());
  Serial.println();
}

void printDate()
{
  Serial.print(rtc.getDay());
  Serial.print("/");
  Serial.print(rtc.getMonth());
  Serial.print("/");
  Serial.print(rtc.getYear());

  Serial.print(" ");
}

void print2digits(int number) {
  if (number < 10) {
    Serial.print("0");
  }
  Serial.print(number);
}

// this method makes a HTTP connection to the server:
void httpRequest() {
  // close any connection before send a new request.
  // This will free the socket on the Nina module
  client.stop();

  // if there's a successful connection:
  if (client.connect(server, PORT)) {
    Serial.println("connecting...");
    // send the HTTP PUT request:
    client.println("GET /index.html HTTP/1.1");
    client.println("Host: sensorscanada.ca");
    client.println("User-Agent: ArduinoWiFi/1.1");
    client.println("Connection: close");
    client.println();

    // note the time that the connection was made:
    lastConnectionTime = millis();
  } else {
    // if you couldn't make a connection:
    Serial.println("connection failed");
  }
}



int An_Sensor_read(){
   // Turn sensor ON and wait a moment.
   int value=0;
    //digitalWrite(An_S1_Power, HIGH);
    //delay(SENSOR_DELAY);

    // Take reading, send to plotter.
    value=(analogRead(An_S1_In));
    // Turn sensor OFF again.
    //digitalWrite(An_S1_Power, LOW);
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

void I2C_sensor_read(){
  Serial.print("Soil Moisture Capacitance: ");
  Serial.print(Sensor_i2c_1.getCapacitance()); //read capacitance register
  Serial.print(", Temperature: ");
  Serial.print(Sensor_i2c_1.getTemperature()/(float)10); //temperature register
  Serial.print(", Light: ");
  Serial.println(Sensor_i2c_1.getLight(true)); //request light measurement, wait and read light register
  Sensor_i2c_1.sleep(); // available since FW 2.3 
}


