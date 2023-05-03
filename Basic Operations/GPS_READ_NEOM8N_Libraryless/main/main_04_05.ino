#include <IntervalTimer.h>
#include <SoftwareSerial.h>
#include <Wire.h>               // BMP280
#include <Adafruit_BMP280.h>    // BMP280
#include <RTClib.h>             // RTC
#include <Adafruit_Sensor.h>    // BNO055
#include <Adafruit_BNO055.h>    // BNO055
#include <utility/imumaths.h>   // BNO055

IntervalTimer GpsTimer, I2CTimer, SendDataTimer, Timer4 ;

// Sensors Variables Declaration
int TEAM_ID = 1007;
int PACKETCOUNT, PRESSURE ;
String MODE, STATE, HS_DEPLOYED, PC_DEPLOYED, MAST_RAISED, CMD_ECHO;
float ALTITUDE, TEMPERATURE, VOLTAGE, TILT_X, TILT_Y;
char MISSION_TIME[32];

 // GPS
  typedef struct gpgga
{
  char SENTENCEIDENTIFIER[7];   //1
  float GPS_TIME;           //2
  double GPS_LATITUDE;        //3
  char LATUTE_DIR;       //4
  double GPS_LONGITUDE;        //5
  char LONGTITUDE_DIR;       //6
  char FIX_QUALITY;       //7
  int GPS_SATS;     //8
  float HORIZANTAL_DILUTION;    //9
  double GPS_ALTITUDE;        //10
}GPGGA;

  int getCommaCount(String str);
  void parseGPGGA(String str, GPGGA& gpgga);
  SoftwareSerial gpsSerial(0, 1);
  GPGGA gpsDatas;

  
Adafruit_BMP280 bmp;
RTC_DS3231 rtc;
Adafruit_BNO055 bno = Adafruit_BNO055(55);

void setup() {
  Serial.begin(9600);
  gpsSerial.begin(9600);
  Wire.begin();
  bmp.begin(0x76);
  rtc.begin();
  bno.begin();
  rtc.adjust(DateTime(F(__DATE__),F(__TIME__)));
  // Timer Declaration
  GpsTimer.begin(GpsTimer_ISR             , 100000); 
  GpsTimer.priority(200); 
  I2CTimer.begin(I2CTimer_ISR             , 1000000);  
  I2CTimer.priority(100);
  SendDataTimer.begin(SendDataTimer_ISR   , 1000000); 
  SendDataTimer.priority(150);
  Timer4.begin(Timer4_ISR                 , 1000000);
  delay(1000);
  gpsSerial.println("$PUBX,40,GGA,0,1,0,0,0,0*");
}

void loop() {
  // Ana işlemler burada yapılabilir
 
}

void GpsTimer_ISR() {
  if (gpsSerial.available()) { // GPS modülünden veri var mı kontrol et
    String message = gpsSerial.readStringUntil('\n'); // Gelen veriyi oku
    if (message.substring(0, 6) == "$GNGGA") { // Eğer GNGGA mesajıysa

      int virgulCount = getCommaCount(message);
      if (virgulCount == 14)
      {
          parseGPGGA(message, gpsDatas);
//          Serial.print("Latitude: ");
//          Serial.println(gpsDatas.GPS_LATITUDE);
//          Serial.print("Latitude Direction: ");
//          Serial.println(gpsDatas.LATUTE_DIR);
//          Serial.print("Longitude: ");
//          Serial.println(gpsDatas.GPS_LONGITUDE);
//          Serial.print("Longitude Direction: ");
//          Serial.println(gpsDatas.LONGTITUDE_DIR);
//          Serial.print("GPS Time: ");
//          Serial.println(gpsDatas.GPS_TIME);
//          Serial.print("Fix Quality: ");
//          Serial.println(gpsDatas.FIX_QUALITY);
//          Serial.print("Number of Satellites: ");
//          Serial.println(gpsDatas.GPS_SATS);
//          Serial.print("Horizontal Dilution: ");
//          Serial.println(gpsDatas.HORIZANTAL_DILUTION);
//          Serial.print("Altitude: ");
//          Serial.println(gpsDatas.GPS_ALTITUDE);
//          Serial.println();
//          Serial.println();
      }
    }
  }
}

void I2CTimer_ISR() {
  // BMP280
   TEMPERATURE = bmp.readTemperature();
   PRESSURE    = bmp.readPressure() / 100.0F;
   ALTITUDE    = bmp.readAltitude(1013.25); // 1013.25 Referance pressure
  
//  Serial.print ("TEMPERATURE : ");
//  Serial.println(TEMPERATURE);
//  Serial.print ("PRESSURE : ");
//  Serial.println(PRESSURE);
//  Serial.print ("ALTITUDE : ");
//  Serial.println(ALTITUDE);

  
  // RTC
  DateTime now = rtc.now();
  sprintf(MISSION_TIME, "%02d:%02d:%02d %02d/%02d/%02d", now.hour(), now.minute(), now.second(), now.day(), now.month(), now.year());
  
//  Serial.print(F("DATE / TIME: "));
//  Serial.println(MISSION_TIME);

  // BNO055
  sensors_event_t event;
  bno.getEvent(&event);
  TILT_X= event.orientation.x;
  TILT_Y= event.orientation.y;
  
//  Serial.print("X: ");
//  Serial.print(TILT_X);
//  Serial.print("\tY: ");
//  Serial.print(TILT_Y);
//  Serial.println("");
   
}

void SendDataTimer_ISR() {


 String DATA_ALL = String(TEAM_ID) + "," + String(MISSION_TIME) + "," + String(PACKETCOUNT) + "," + MODE+ "," + STATE+ "," + String(ALTITUDE) + "," + HS_DEPLOYED + "," + PC_DEPLOYED + "," + MAST_RAISED + "," + String(TEMPERATURE) + "," + String(PRESSURE) + "," + String(VOLTAGE) + "," + String(gpsDatas.GPS_TIME) + "," + String(gpsDatas.GPS_ALTITUDE) + "," + String(gpsDatas.GPS_LATITUDE) + "," + String(gpsDatas.GPS_LONGITUDE) + "," + String(gpsDatas.GPS_SATS) + "," + String(TILT_X) + "," + String(TILT_Y) + "," + CMD_ECHO;
 Serial.print("DATA : ");
 Serial.println(DATA_ALL);

  
}

void Timer4_ISR() {

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
        strcpy(gpgga.SENTENCEIDENTIFIER, valueStr.c_str());
        break;
      case 1:
        gpgga.GPS_TIME = valueStr.toFloat();
        break;
      case 2:
        gpgga.GPS_LATITUDE = valueStr.toFloat();
        break;
      case 3:
        gpgga.LATUTE_DIR = valueStr.charAt(0);
        break;
      case 4:
        gpgga.GPS_LONGITUDE = valueStr.toFloat();
        break;
      case 5:
        gpgga.LONGTITUDE_DIR = valueStr.charAt(0);
        break;
      case 6:
        gpgga.FIX_QUALITY = valueStr.charAt(0);
        break;
      case 7:
        gpgga.GPS_SATS = valueStr.toInt();
        break;
      case 8:
        gpgga.HORIZANTAL_DILUTION = valueStr.toFloat();
        break;
      case 9:
        gpgga.GPS_ALTITUDE = valueStr.toFloat();
        break;
    }
  }
}
