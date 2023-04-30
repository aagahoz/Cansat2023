#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>

Adafruit_BMP280 bmp;

// Değiştirilmesi gereken parametreler
float seaLevelPressure = 1013.25;  // deniz seviyesi basıncı (hektopaskal)
float currentAltitude = 0.0;      // bulunduğunuz yerin yüksekliği (metre)

void setup() {
  Serial.begin(9600);
  if (!bmp.begin(0x76)) {
    Serial.println("BMP280 sensörü bulunamadı, bağlantıları kontrol edin!");
    while (1);
  }
  // BMP280 sensörüyle ilk okuma yapılır
  float currentPressure = bmp.readPressure() / 100.0F; // basınç değeri (hektopaskal)
  currentAltitude = 44330 * (1.0 - pow(currentPressure / seaLevelPressure, 0.1903)); // yükseklik (metre)
}

void loop() {
  Serial.print(F("Temperature = "));
    Serial.print(bmp.readTemperature());
    Serial.println(" C");

    Serial.print(F("Pressure = "));
    Serial.print(bmp.readPressure());
    Serial.println(" Pa");
  float pressure = bmp.readPressure() / 100.0F; // basınç değeri (hektopaskal)
  float altitude = 44330*(1.0 - pow(pressure / seaLevelPressure, 0.1903)); // yükseklik (metre)
  float altitudeFromSeaLevel = altitude - currentAltitude; // deniz seviyesine göre yükseklik (metre)
  Serial.print("Yükseklik: ");
  Serial.print(altitudeFromSeaLevel);
  Serial.println(" metre.");
  delay(1000);
}//bmp280 0 metre referans kodu
