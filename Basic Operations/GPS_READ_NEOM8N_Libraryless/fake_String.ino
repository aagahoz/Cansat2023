#include <Wire.h>
#include <SoftwareSerial.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>

// XBee Pin Definitions
const int rxPin = 0;//normalde 7
const int txPin = 1;//normalde 8
SoftwareSerial xbee(rxPin, txPin);  // XBee communication

Adafruit_BMP280 bmp;
int TeamID = 1007;
String MissionTime = "12:34:56.23";
int PacketCount = 21;
String Mode = "S";
String State = "HS_RELEASE";
float Altitude = 108.2;
String HS_DEPLOYED = "P";
String PC_DEPLOYED = "C";
String MAST_RAISED = "M";
float TEMPERATURE = 32.1;
float VOLTAGE = 2.5;
String GPS_TIME = "15:06:33";
float GPS_ALTITUDE = 105.5;
float GPS_LATITUDE = 170.6868;
float GPS_LONGITUDE = 42.5674;
int GPS_SATS = 3;
float TILT_X = 5.32;
float TILT_Y = 15.21;
String CMD_ECHO = "ST";
int PRESSURE = 10231;



void setup() {
  // Start serial communication
  Serial.begin(9600);
  Wire.begin();
  //while (!Serial) continue;
  
  // Start XBee communication
  xbee.begin(9600);
}

void loop() {
  PacketCount++;
  String str = String(TeamID) + "," + MissionTime + "," + String(PacketCount) + "," + Mode + "," + State + "," + String(Altitude) + "," + HS_DEPLOYED + "," + PC_DEPLOYED + "," + MAST_RAISED + "," + String(TEMPERATURE) + "," + String(PRESSURE) + "," + String(VOLTAGE) + "," + GPS_TIME + "," + String(GPS_ALTITUDE) + "," + String(GPS_LATITUDE) + "," + String(GPS_LONGITUDE) + "," + String(GPS_SATS) + "," + String(TILT_X) + "," + String(TILT_Y) + "," + CMD_ECHO;
  //float temperature = bmp.readTemperature();
  xbee.println(str);
  Serial.println(str);
  delay(1000);
}
