#include <SoftwareSerial.h>

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

int getCommaCount(String str);
void parseGPGGA(String str, GPGGA& gpgga);

SoftwareSerial gpsSerial(0, 1); // RX, TX pimleri

GPGGA gpsDatas; 


void setup() {
  Serial.begin(9600); // Seri haberleşme hızı
  gpsSerial.begin(9600); // GPS modülüne seri haberleşme başlat
  delay(1000); // 1 saniye bekle
  gpsSerial.println("$PUBX,40,GGA,0,1,0,0,0,0*"); // Sadece GNGGA mesajını iste

  
}

void loop() {
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
} //Sadece GNGGA nın çekildiği kodlar. ilk satırı sorunlu çekiyor


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
