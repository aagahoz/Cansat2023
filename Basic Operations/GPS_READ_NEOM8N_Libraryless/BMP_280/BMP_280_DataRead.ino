#include <Wire.h>
#include <Adafruit_BMP280.h>
Adafruit_BMP280 bmp;
void setup() {
  Serial.begin(9600);
  if (!bmp.begin(0x76)) {
    Serial.println("Could not find a valid BMP280 sensor, check wiring!");
    while (1);
  }
}
void loop() {
  float temperature = bmp.readTemperature();
  float pressure = bmp.readPressure() / 100.0F;
  float altitude = bmp.readAltitude(1013.25);  // Referans basıncı 1013.25 hPa (deniz seviyesi basıncı)
  // Print sensor readings to serial monitor
  Serial.print("Temperature = ");
  Serial.print(temperature);
  Serial.println(" C");
  Serial.print("Pressure = ");
  Serial.print(pressure);
  Serial.println(" hPa");
  Serial.print("Altitude = ");
  Serial.print(altitude);
  Serial.println(" m");
  delay(1000);
}
