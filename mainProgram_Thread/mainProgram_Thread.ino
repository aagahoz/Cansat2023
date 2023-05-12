#include <Thread.h>

#include <SoftwareSerial.h>
#include <Wire.h>             // BMP280
#include <Adafruit_BMP280.h>  // BMP280
// #include <RTClib.h>           // RTC
#include <Adafruit_Sensor.h>  // BNO055
#include <Adafruit_BNO055.h>  // BNO055
#include <utility/imumaths.h> // BNO055
#include <Servo.h>


// THREADS
Thread bmpThread = Thread();
Thread rtcThread = Thread();
Thread gpsThread = Thread();
Thread bnoThread = Thread();

Thread comReceiverThread = Thread();
Thread comSenderThread = Thread();

Thread servoThread = Thread();
Thread flagThread = Thread();

Thread mainThread = Thread();

Thread ledThread = Thread();

Thread SdCardThread = Thread();


// Variables
int TEAM_ID = 1007;
int PACKETCOUNT, PRESSURE;
String MODE, STATE, HS_DEPLOYED, PC_DEPLOYED, MAST_RAISED, CMD_ECHO;
float ALTITUDE, TEMPERATURE, VOLTAGE, TILT_X, TILT_Y;
char MISSION_TIME[32];
char incomingChar;
const int Xbee_rx = 7;
const int Xbee_tx = 8;
// EKSTRALAR
String DATA_XBEE_COMING = "";
String DATA_INCOMING_COMMANDS = "";
// GPS
typedef struct gpgga
{
    char SENTENCEIDENTIFIER[7]; // 1
    float GPS_TIME;             // 2
    double GPS_LATITUDE;        // 3
    char LATUTE_DIR;            // 4
    double GPS_LONGITUDE;       // 5
    char LONGTITUDE_DIR;        // 6
    char FIX_QUALITY;           // 7
    int GPS_SATS;               // 8
    float HORIZANTAL_DILUTION;  // 9
    double GPS_ALTITUDE;        // 10
} GPGGA;
GPGGA gpsDatas;


// Object Decleration
SoftwareSerial xbee(Xbee_rx, Xbee_tx);
Adafruit_BMP280 bmp;
// RTC_DS3231 rtc;
Adafruit_BNO055 bno = Adafruit_BNO055(55);
SoftwareSerial gpsSerial(0, 1); 
Servo myservo;  // Servo motor nesnesi oluştur
int pos = 0;    // Başlangıç pozisyonunu belirle



int getCommaCount(String str);
void parseGPGGA(String str, GPGGA &gpgga);


void setup() 
{

  pinMode(13, OUTPUT);
  Wire.begin();
  bmp.begin(0x76);
//  rtc.begin();
  bno.begin();
//  rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  xbee.begin(9600);
  gpsSerial.begin(9600);
  myservo.attach(3);  // Servo motoru pin 9'a bağla


  bmpThread.onRun(bmpCallback);
  bmpThread.setInterval(100);

  rtcThread.onRun(rtcCallback);
  rtcThread.setInterval(100);

  gpsThread.onRun(gpsCallback);
  gpsThread.setInterval(100);

  bnoThread.onRun(bnoCallback);
  bnoThread.setInterval(100);

  comSenderThread.onRun(comSenderCallback);
  comSenderThread.setInterval(100);

  comReceiverThread.onRun(comReceiverCallback);
  comReceiverThread.setInterval(100);
//
//  servoThread.onRun(servoCallback);
//  servoThread.setInterval(100);
//
//  flagThread.onRun(flagCallback);
//  flagThread.setInterval(100);
//
//  mainThread.onRun(mainCallback);
//  mainThread.setInterval(100);
//
//  ledThread.onRun(ledCallback);
//  ledThread.setInterval(100);
//
//  SdCardThread.onRun(SdCardCallback);
//  SdCardThread.setInterval(100);
//

}

void loop() 
{
  if(bmpThread.shouldRun())
    bmpThread.run();

  if(rtcThread.shouldRun())
    rtcThread.run();

  if(gpsThread.shouldRun())
    gpsThread.run();

  if(bnoThread.shouldRun())
    bnoThread.run();

  if(comSenderThread.shouldRun())
    comSenderThread.run();
    
  if(comReceiverThread.shouldRun())
    comReceiverThread.run();

//  if(servoThread.shouldRun())
//    servoThread.run();
//  
//  if(flagThread.shouldRun())
//    flagThread.run();
//
//  if(mainThread.shouldRun())
//    mainThread.run();
//
//  if(ledThread.shouldRun())
//    ledThread.run();
//
//  if(SdCardThread.shouldRun())
//    SdCardThread.run();
}


void bmpCallback()
{
  TEMPERATURE = bmp.readTemperature();
  PRESSURE = bmp.readPressure() / 100.0F;
  ALTITUDE = bmp.readAltitude(1013.25); // 1013.25 Referance pressure

  String data = "TEMPERATURE: " + String(TEMPERATURE) + " -- " + "PRESSURE: " + String(PRESSURE) + " -- " + "ALTITUDE: " + String(ALTITUDE);
  Serial.println(data);
    
  Serial.print("1--> BMP Callback is running: ");
  Serial.println(millis());
}


void rtcCallback()
{
//  DateTime now = rtc.now();
//  sprintf(MISSION_TIME, "%02d:%02d:%02d %02d/%02d/%02d", now.hour(), now.minute(), now.second(), now.day(), now.month(), now.year());

//    Serial.print(F("DATE / TIME: "));
//    Serial.println(MISSION_TIME);

    
  Serial.print("2--> RTC Callback is running: ");
  Serial.println(millis());
}


void gpsCallback()
{
  if (gpsSerial.available())
  {                                                     // GPS modülünden veri var mı kontrol et
      String message = gpsSerial.readStringUntil('\n'); // Gelen veriyi oku
      if (message.substring(0, 6) == "$GNGGA")
      { // Eğer GNGGA mesajıysa

          int virgulCount = getCommaCount(message);
          if (virgulCount == 14)
          {
              parseGPGGA(message, gpsDatas);
              Serial.print("Latitude: ");
              Serial.println(gpsDatas.GPS_LATITUDE);
              Serial.print("Latitude Direction: ");
              Serial.println(gpsDatas.LATUTE_DIR);
              Serial.print("Longitude: ");
              Serial.println(gpsDatas.GPS_LONGITUDE);
              Serial.print("Longitude Direction: ");
              Serial.println(gpsDatas.LONGTITUDE_DIR);
              Serial.print("GPS Time: ");
              Serial.println(gpsDatas.GPS_TIME);
              Serial.print("Fix Quality: ");
              Serial.println(gpsDatas.FIX_QUALITY);
              Serial.print("Number of Satellites: ");
              Serial.println(gpsDatas.GPS_SATS);
              Serial.print("Horizontal Dilution: ");
              Serial.println(gpsDatas.HORIZANTAL_DILUTION);
              Serial.print("Altitude: ");
              Serial.println(gpsDatas.GPS_ALTITUDE);
              Serial.println();
              Serial.println();
          }
      }
  }
  Serial.print("3--> GPS Callback is running: ");
  Serial.println(millis());
}


void bnoCallback()
{
  sensors_event_t event;
  bno.getEvent(&event);
  TILT_X = event.orientation.x;
  TILT_Y = event.orientation.y;

  Serial.println("X: " + String(TILT_X) + "\tY: " + String(TILT_Y));
  
  Serial.print("4--> BNO055 Callback is running: ");
  Serial.println(millis());
}

void comSenderCallback()
{
  String DATA_ALL = String(TEAM_ID) + "," + String(MISSION_TIME) + "," + String(PACKETCOUNT) + "," + MODE + "," + STATE + "," + String(ALTITUDE) + "," + HS_DEPLOYED + "," + PC_DEPLOYED + "," + MAST_RAISED + "," + String(TEMPERATURE) + "," + String(PRESSURE) + "," + String(VOLTAGE) + "," + String(gpsDatas.GPS_TIME) + "," + String(gpsDatas.GPS_ALTITUDE) + "," + String(gpsDatas.GPS_LATITUDE) + "," + String(gpsDatas.GPS_LONGITUDE) + "," + String(gpsDatas.GPS_SATS) + "," + String(TILT_X) + "," + String(TILT_Y) + "," + CMD_ECHO;
  xbee.println(DATA_ALL);
  
  Serial.print("5--> Commuinication Sender Callback is running: ");
  Serial.println(millis());
}

void comReceiverCallback()
{
  if (xbee.available())
  {
      String incoming_data = xbee.readStringUntil('\n');
      Serial.println("3_Timer --- XBEE is available");

      Serial.println("Gelen Ham Veri -->  " + incoming_data + "   ;  " + "Boyutu -->  " + incoming_data.length() + "   ;  " + "Son -2 Char -->" + incoming_data[incoming_data.length() - 1]);

      if (incoming_data.length() == 12)
      {
          if (incoming_data[0] == '!')
          {
              if (incoming_data[incoming_data.length() - 1] == '+')
              {
                  Serial.println("Alınan Fix Data : " + incoming_data);
              }
          }
      }
  }
  
  Serial.print("6--> Commuinication Receiver Callback is running: ");
  Serial.println(millis());
}

void servoCallback()
{
  // 0'dan 180'e kadar gidip gel
  for (pos = 0; pos <= 180; pos += 1) {
    myservo.write(pos);              // Servo'yu yeni pozisyona gönder
    delay(15);                       // Servo'nun yeni pozisyona ulaşması için bekleyin
  }
  for (pos = 180; pos >= 0; pos -= 1) {
    myservo.write(pos);              // Servo'yu yeni pozisyona gönder
    delay(15);                       // Servo'nun yeni pozisyona ulaşması için bekleyin
  }
  Serial.print("7--> Servo Callback is running: ");
  Serial.println(millis());
}

void flagCallback()
{
  Serial.print("8--> Flag Callback is running: ");
  Serial.println(millis());
}

void mainCallback()
{
  Serial.print("9--> Main Callback is running: ");
  Serial.println(millis());
}

void ledCallback()
{
  static bool ledStatus = false;
  ledStatus = !ledStatus;

  digitalWrite(13, ledStatus);

  Serial.print("10--> LED Callback is running: ");
  Serial.println(millis());
}

void SdCardCallback()
{


  Serial.print("11--> SD Card Callback is running: ");
  Serial.println(millis());
}



int getCommaCount(String str)
{
    int sayac = 0;
    for (unsigned int i = 0; i < str.length(); i++)
    {
        if (str.charAt(i) == ',')
        {
            sayac++;
        }
    }
    return sayac;
}

void parseGPGGA(String str, GPGGA &gpgga)
{
    int pos = 0;
    for (int i = 0; i < 10; i++)
    {
        int nextPos = str.indexOf(',', pos);
        String valueStr = str.substring(pos, nextPos);
        pos = nextPos + 1;
        switch (i)
        {
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
