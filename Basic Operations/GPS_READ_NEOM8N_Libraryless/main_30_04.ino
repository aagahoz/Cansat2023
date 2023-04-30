#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <Adafruit_BMP280.h>
#include <SoftwareSerial.h>
#include <RTClib.h>
RTC_DS3231 rtc;
Adafruit_BMP280 bmp;
char t[32];

const int rxPin = 7;
const int txPin = 8;
int a=0;
SoftwareSerial xbee(rxPin, txPin);  // XBee communication
SoftwareSerial gpsSerial(0, 1); // RX, TX


Adafruit_BNO055 bno = Adafruit_BNO055(55);

typedef struct gpgga
{
  char sentenceIdentifier[7];   //1
  float gpsTime;           //2
  double latitude;        //3
  char latitudeDir;       //4
  double longtitude;        //5
  char longtitudeDir;       //6
  char fix_quality;       //7
  int numberOfSatellites;     //8
  float horizontal_dilution;    //9
  double altitude;        //10
}GPGGA;


GPGGA gpsDatas; 

int getCommaCount(String str);
void parseGPGGA(String str, GPGGA& gpgga);
void setup(void)
{
  Serial.begin(9600);
  gpsSerial.begin(9600);
  delay(1000); 
  gpsSerial.println("$PUBX,40,GGA,0,1,0,0,0,0*"); // Sadece GNGGA mesajını iste
  Wire.begin();
  rtc.begin();
  rtc.adjust(DateTime(F(__DATE__),F(__TIME__)));
  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }

  delay(1000);

  // Initialize BMP280 sensor
  if (!bmp.begin(0x76)) {
    Serial.println("Could not find a valid BMP280 sensor, check wiring!");
    while (1);
  }
  xbee.begin(9600);
}
void loop(void)
{

//  sensors_event_t event;
//  bno.getEvent(&event);
//  Serial.print("X: ");
//  Serial.print(event.orientation.x, 4);
//  Serial.print("\tY: ");
//  Serial.println(event.orientation.y, 4);
//  float temperature = bmp.readTemperature();
//  float pressure = bmp.readPressure() / 100.0F;
//  // Print sensor readings to serial monitor
//  Serial.print("Temperature = ");
//  Serial.print(temperature);
//  Serial.println(" C");
//  Serial.print("Pressure = ");
//  Serial.print(pressure);
//  Serial.println(" hPa");
//  Serial.print(F("Approx altitude = "));
//  Serial.print(bmp.readAltitude(1013.25)); /* Adjusted to local forecast! */
//  Serial.println(" m");
//
//  DateTime now = rtc.now();
//  sprintf(t, "%02d:%02d:%02d %02d/%02d/%02d", now.hour(), now.minute(), now.second(), now.day(), now.month(), now.year());  
//  Serial.print(F("Date/Time: "));
//  Serial.println(t);
//  //delay(2000);

  if (gpsSerial.available()) { // GPS modülünden veri var mı kontrol et
    String message = gpsSerial.readStringUntil('\n'); // Gelen veriyi oku
    if (message.substring(0, 6) == "$GNGGA") { // Eğer GNGGA mesajıysa

      int virgulCount = getCommaCount(message);
      if (virgulCount == 14)
      {
          parseGPGGA(message, gpsDatas);
          Serial.print("Latitude: ");
          Serial.println(gpsDatas.latitude);
          Serial.print("Latitude Direction: ");
          Serial.println(gpsDatas.latitudeDir);
          Serial.print("Longitude: ");
          Serial.println(gpsDatas.longtitude);
          Serial.print("Longitude Direction: ");
          Serial.println(gpsDatas.longtitudeDir);
          Serial.print("GPS Time: ");
          Serial.println(gpsDatas.gpsTime);
          Serial.print("Fix Quality: ");
          Serial.println(gpsDatas.fix_quality);
          Serial.print("Number of Satellites: ");
          Serial.println(gpsDatas.numberOfSatellites);
          Serial.print("Horizontal Dilution: ");
          Serial.println(gpsDatas.horizontal_dilution);
          Serial.print("Altitude: ");
          Serial.println(gpsDatas.altitude);
          Serial.println();
          Serial.println();
         
      }
    }
  }
  
//  // Send sensor readings to XBee
//  xbee.print("Temperature = ");
//  xbee.print(temperature);
//  xbee.println("C");
//  xbee.print("Pressure = ");
//  xbee.print(pressure);
//  xbee.println(" hPa");
}

int getCommaCount(String str) {
  int sayac = 0;
  for (int i = 0; i < str.length(); i++) {
    if (str.charAt(i) == ',') {
      sayac++;
    }
  }
  return sayac;
}


void parseGPGGA(String str, GPGGA& gpgga) {
  int pos = 0;
  for (int i = 0; i < 10; i++) {
    int nextPos = str.indexOf(',', pos);
    String valueStr = str.substring(pos, nextPos);
    pos = nextPos + 1;
    switch (i) {
      case 0:
        strcpy(gpgga.sentenceIdentifier, valueStr.c_str());
        break;
      case 1:
        gpgga.gpsTime = valueStr.toFloat();
        break;
      case 2:
        gpgga.latitude = valueStr.toFloat();
        break;
      case 3:
        gpgga.latitudeDir = valueStr.charAt(0);
        break;
      case 4:
        gpgga.longtitude = valueStr.toFloat();
        break;
      case 5:
        gpgga.longtitudeDir = valueStr.charAt(0);
        break;
      case 6:
        gpgga.fix_quality = valueStr.charAt(0);
        break;
      case 7:
        gpgga.numberOfSatellites = valueStr.toInt();
        break;
      case 8:
        gpgga.horizontal_dilution = valueStr.toFloat();
        break;
      case 9:
        gpgga.altitude = valueStr.toFloat();
        break;
    }
  }
}
