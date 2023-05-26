/*          TASK DURUMLARI
 *  RTC okuma                 +
 *  BMP280 okuma              +
 *  Telemetri gonderimi       +
 *  Telemetri Alma            -
 *  IMU Okuma.                -
 *  GPS Okuma                 -
 *  Servo Control             -
 *  DC Motor Control          -
 *  SD Card telemetri yazma   -
 *  Analog Pil okuma          -

 
 RTC PinOut
    Vcc         ->   3.3V
    Ground      ->   Ground
    SDA         ->   21     
    SCL         ->   22

 BMP280 PinOut
    Vcc         ->   3.3V
    Ground      ->   Ground
    SDA         ->   21     
    SCL         ->   22

 Serial PinOut
    3.3V          ->  3.3V
    Ground      ->  Ground
    TX          ->  Tx
    RX          ->  Rx

 IMU PinOut
    Vcc         ->   3.3V
    Ground      ->   Ground
    SDA         ->   21     
    SCL         ->   22

 GPS PinOut
    3.3V          ->  3.3V
    Ground      ->  Ground
    TX          ->  13
    RX          ->  14

 Servo PinOut
    Vcc         ->   Regulator +
    Ground      ->   Ground
    PWM Sinyal  ->   12

 DC Motor PinOut
    Vcc         ->   ?
    Ground      ->   ?
    PWM Sinyal  ->   ?

 SD CARD PinOut   
    Vcc         ->   5V
    Ground      ->   Ground
    MOSI        ->   23     
    MISO        ->   19
    CS          ->   5     
    CLK         ->   18

 Analog Pil Gerilimi PinOut
    Analog Pin  ->  ? - 34
    GND         ->  Ground

 */


// I2C
#include <Wire.h>
// BMP280
#include <Adafruit_BMP280.h>
// RTC
#include <RTClib.h>
// IMU
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
// GPS
#include <HardwareSerial.h>
// SD Card
#include "FS.h"
#include "SD.h"
#include <SPI.h>
#define SD_CS 5
// Servo
#include <Servo.h>


// Thread Çekirdek Configrasyonu
#if CONFIG_FREERTOS_UNICORE
#define ARDUINO_RUNNING_CORE 0
#else
#define ARDUINO_RUNNING_CORE 1
#endif

#ifndef LED_BUILTIN
#define LED_BUILTIN 13
#endif

struct TelemetryStruct {
  const String team_id = "1007";
  String mission_time = "0";
  String packet_count = "0";
  String mode = "0";
  String state = "0";
  String altitude = "0";
  String hs_deployed = "0";
  String pc_deployed = "0";
  String mast_raised = "0";
  String temp = "0";
  String pressure = "0";
  String volt = "0";
  String gps_time = "0";
  String gps_altitude = "0";
  String gps_latitude = "0";
  String gps_longitude = "0";
  String gps_sats = "0";
  String tilt_x = "0";
  String tilt_y = "0";
  String cmd_echo = "0";
  String accelX = "0";
  String accelY = "0";
  String accelZ = "0";
};

// Kütüphaneler
Adafruit_BMP280 bmp;
RTC_DS1307 newRTC;
Adafruit_MPU6050 mpu;
HardwareSerial SerialGPS(1);
Servo servo;


// Global Degerler
TelemetryStruct TelemetryObject;
String XBee_Incoming_Packet[10];

bool ServoState = false;
bool DcState = false;
const int servoPin = 25;
const int voltagePin = 34;

bool is_sd_card_mounted = true;
bool is_sd_card_attached = true;
bool is_sd_card_initialize = true;

String XBee_Payload_Telemetry;




void TaskRTC(void* pvParameters) {
  (void)pvParameters;

  for (;;) {
    bool rtc_find_state = newRTC.begin();
    bool rtc_running_state = newRTC.isrunning();

    if (rtc_find_state && rtc_running_state) {
      DateTime now = newRTC.now();
      TelemetryObject.gps_time = String(now.day()) + "," + String(now.month()) + "," + String(now.year()) + ";" + String(now.hour()) + "," + String(now.minute()) + "," + String(now.second());
      Serial.println("RTC--> " + TelemetryObject.gps_time);
    } else {
      Serial.println("RTC--> RTC ERROR!");
      TelemetryObject.gps_time = ",,;,,";
    }
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

void TaskBMP280(void* pvParameters) {
  (void)pvParameters;

  for (;;) {
    float temperature = bmp.readTemperature();
    float pressure = bmp.readPressure();
    float altitude = bmp.readAltitude(1017.5);

    TelemetryObject.temp = String(temperature);
    TelemetryObject.pressure = String(pressure);
    TelemetryObject.altitude = String(altitude);

    Serial.println("1- BMP280--> " + TelemetryObject.pressure + "," + TelemetryObject.temp + "," + TelemetryObject.altitude);
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

void TaskXBeeSend(void* pvParameters) {
  (void)pvParameters;

  for (;;) {
    XBee_Payload_Telemetry = TelemetryObject.team_id + "," + TelemetryObject.mission_time + "," + TelemetryObject.packet_count + "," + TelemetryObject.mode + "," + TelemetryObject.state + "," + TelemetryObject.altitude + "," + TelemetryObject.hs_deployed + "," + TelemetryObject.pc_deployed + "," + TelemetryObject.mast_raised + "," + TelemetryObject.temp + "," + TelemetryObject.pressure + "," + TelemetryObject.volt + "," + TelemetryObject.gps_time + "," + TelemetryObject.gps_altitude + "," + TelemetryObject.gps_latitude + "," + TelemetryObject.gps_longitude + "," + TelemetryObject.gps_sats + "," + TelemetryObject.tilt_x + "," + TelemetryObject.tilt_y + "," + TelemetryObject.cmd_echo + "," + TelemetryObject.accelX + "," + TelemetryObject.accelY + "," + TelemetryObject.accelZ;



    Serial.print("2- XBEE SEND PAYLOAD -->");
    Serial.println(XBee_Payload_Telemetry);

    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

void TaskXBeeReceive(void* pvParameters) {
  (void)pvParameters;

  for (;;) {

    if (Serial.available()) {
      String incoming_data = Serial.readStringUntil('\n');

      Serial.println("Gelen Ham Veri -->  " + incoming_data + "   ;  " + "Boyutu -->  " + incoming_data.length() + "   ;  " + "Son -2 Char -->" + incoming_data[incoming_data.length() - 1]);

      if (incoming_data.length() == 12) {
        if (incoming_data[0] == '!') {
          if (incoming_data[incoming_data.length() - 1] == '+') {
            Serial.println("Alınan Fix Data : " + incoming_data);
          }
        }
      }
    }


    Serial.print("3- XBEE Receive PAYLOAD -->");
    for (int i = 0; i < 10; i++) {
      Serial.print(XBee_Incoming_Packet[i]);
      Serial.print(" ");
    }
    Serial.println();

    vTaskDelay(pdMS_TO_TICKS(900));
  }
}

void TaskIMU(void* pvParameters) {
  (void)pvParameters;

  for (;;) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    float gyroX = g.gyro.x;
    float gyroY = g.gyro.y;
    float gyroZ = g.gyro.z;
    float accelX = a.acceleration.x;
    float accelY = a.acceleration.y;
    float accelZ = a.acceleration.z;

    String data = "4- IMU--> Rotation X: " + String(gyroX) + ", Y: " + String(gyroY) + ", Z: " + String(gyroZ) + " rad/s, Acceleration X: " + String(accelX) + ", Y: " + String(accelY) + ", Z: " + String(accelZ) + " m/s^2";
    Serial.println(data);

    TelemetryObject.tilt_x = String(gyroX);
    TelemetryObject.tilt_y = String(gyroY);
    TelemetryObject.accelX = String(accelX);
    TelemetryObject.accelY = String(accelY);
    TelemetryObject.accelZ = String(accelZ);

    vTaskDelay(pdMS_TO_TICKS(200));
  }
}

void TaskGPS(void* pvParameters) {
  (void)pvParameters;

  for (;;) {


    if (SerialGPS.available()) {                         // GPS modülünden veri var mı kontrol et
      String message = SerialGPS.readStringUntil('\n');  // Gelen veriyi oku
      Serial.println(message);
      if (message.substring(0, 6) == "$GNGGA") {  // Eğer GNGGA mesajıysa

        Serial.println("Virgul falan filan");
      }
    }
    Serial.println("5- GPS DataS --> ");
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

void TaskServoControl(void* pvParameters) {
  (void)pvParameters;

  for (;;) {


    //.....

    Serial.println("6- Servo State --> ");
    vTaskDelay(pdMS_TO_TICKS(200));
  }
}

void TaskDcControl(void* pvParameters) {
  (void)pvParameters;

  for (;;) {

    /////........

    Serial.print("7- DC State: ");
    vTaskDelay(pdMS_TO_TICKS(200));
  }
}

void TaskBatteryVoltage(void* pvParameters) {  /////////////////////// Duzenlenecek
  (void)pvParameters;

  // const float r1 = 10000.0;  // R1 direnci değeri
  // const float r2 = 1000.0;   // R2 direnci değeri

  for (;;) {
    //   float vtoplam = 0.0;
    //   int count = 0;

    //   for (int i = 0; i < 75; i++) {
    //     float analogvalue = analogRead(34);       // Analog pinden değer okunuyor
    //     float vout = analogvalue * 2.373 / 4096;  // Analog değeri gerilime dönüştürme (pin gerilimi)
    //     float vin = (((r1 + r2) * vout) / r2);

    //     if (vin != 0.00) {
    //       vtoplam = vtoplam + vin;
    //       count = count + 1;
    //     }
    //     delay(10);
    //   }

    // float voltage = vtoplam / count;

    // TelemetryObject.volt = String(voltage);

    Serial.print("8- Voltage --> ");
    // Serial.print(voltage, 3);
    Serial.println(" Volts");
    vTaskDelay(pdMS_TO_TICKS(200));
  }
}

void TaskTelemeryLoggerSdCard(void* pvParameters) {
  (void)pvParameters;

  for (;;) {


    if (is_sd_card_initialize == true) {

      String tempSdCardData = XBee_Payload_Telemetry + '\n';

      appendFile(SD, "/data.txt", tempSdCardData.c_str());
      Serial.println("Success SD Card Telemetry Write");
    } else {
      Serial.println("Failed SD Card Telemetry Write");
    }

    Serial.println("9- SD Card Logging --> " + is_sd_card_initialize);
    vTaskDelay(800);
  }
}

void setup() {
  Serial.begin(9600);
  SerialGPS.begin(9600, SERIAL_8N1, 14, 13);  // RX - TX
  servo.attach(servoPin);

  delay(500);
  analogReadResolution(voltagePin);
  delay(500);

  unsigned status = bmp.begin(0x76);


  bool rtc_find_state = newRTC.begin();
  if (!rtc_find_state) {
    Serial.println("Couldn't find RTC");
  }

  bool rtc_running_state = newRTC.isrunning();
  if (!rtc_running_state) {
    Serial.println("RTC is NOT running!");
  }

  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    delay(1000);
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  Serial.print("Accelerometer range set to: ");
  switch (mpu.getAccelerometerRange()) {
    case MPU6050_RANGE_2_G:
      Serial.println("+-2G");
      break;
    case MPU6050_RANGE_4_G:
      Serial.println("+-4G");
      break;
    case MPU6050_RANGE_8_G:
      Serial.println("+-8G");
      break;
    case MPU6050_RANGE_16_G:
      Serial.println("+-16G");
      break;
  }

  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  Serial.print("Gyro range set to: ");
  switch (mpu.getGyroRange()) {
    case MPU6050_RANGE_250_DEG:
      Serial.println("+- 250 deg/s");
      break;
    case MPU6050_RANGE_500_DEG:
      Serial.println("+- 500 deg/s");
      break;
    case MPU6050_RANGE_1000_DEG:
      Serial.println("+- 1000 deg/s");
      break;
    case MPU6050_RANGE_2000_DEG:
      Serial.println("+- 2000 deg/s");
      break;
  }

  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  Serial.print("Filter bandwidth set to: ");
  switch (mpu.getFilterBandwidth()) {
    case MPU6050_BAND_260_HZ:
      Serial.println("260 Hz");
      break;
    case MPU6050_BAND_184_HZ:
      Serial.println("184 Hz");
      break;
    case MPU6050_BAND_94_HZ:
      Serial.println("94 Hz");
      break;
    case MPU6050_BAND_44_HZ:
      Serial.println("44 Hz");
      break;
    case MPU6050_BAND_21_HZ:
      Serial.println("21 Hz");
      break;
    case MPU6050_BAND_10_HZ:
      Serial.println("10 Hz");
      break;
    case MPU6050_BAND_5_HZ:
      Serial.println("5 Hz");
      break;
  }

  // Initialize SD card
  SD.begin(SD_CS);
  if (!SD.begin(SD_CS)) {
    Serial.println("Card Mount Failed");
    is_sd_card_mounted = false;
    is_sd_card_attached = false;
    is_sd_card_initialize = false;
  }
  if (is_sd_card_mounted == true) {
    uint8_t cardType = SD.cardType();
    if (cardType == CARD_NONE) {
      Serial.println("No SD card attached");
      is_sd_card_attached = false;
      is_sd_card_initialize = false;
    }
    Serial.println("Initializing SD card...");
    if (!SD.begin(SD_CS)) {
      Serial.println("ERROR - SD card initialization failed!");
      is_sd_card_initialize = false;  // init failed
    }
  }
  // If the data.txt file doesn't exist
  // Create a file on the SD card and write the data labels
  if (is_sd_card_initialize == true) {
    File file = SD.open("/data.txt");
    if (!file) {
      Serial.println("File doens't exist");
      Serial.println("Creating file...");
      writeFile(SD, "/data.txt", "\r\n");
    } else {
      Serial.println("File already exists");
    }
    file.close();
  }

  xTaskCreatePinnedToCore(
    TaskRTC,
    "TaskRTC",
    4096,
    NULL,
    2,
    NULL,
    ARDUINO_RUNNING_CORE);

  xTaskCreatePinnedToCore(
    TaskBMP280,
    "TaskBMP",
    4096,
    NULL,
    2,
    NULL,
    ARDUINO_RUNNING_CORE);

  xTaskCreatePinnedToCore(
    TaskIMU,
    "TaskIMU",
    4096,
    NULL,
    2,
    NULL,
    ARDUINO_RUNNING_CORE);

  xTaskCreatePinnedToCore(
    TaskXBeeSend,
    "TaskXBeeSend",
    4096,
    NULL,
    2,
    NULL,
    ARDUINO_RUNNING_CORE);

  xTaskCreatePinnedToCore(
    TaskXBeeReceive,
    "TaskXBeeReceive",
    4096,
    NULL,
    2,
    NULL,
    ARDUINO_RUNNING_CORE);

  xTaskCreatePinnedToCore(
    TaskGPS,
    "TaskGPS",
    4096,
    NULL,
    2,
    NULL,
    ARDUINO_RUNNING_CORE);

  xTaskCreatePinnedToCore(
    TaskServoControl,
    "TaskServoControl",
    4096,
    NULL,
    2,
    NULL,
    ARDUINO_RUNNING_CORE);

  xTaskCreatePinnedToCore(
    TaskDcControl,
    "TaskDcControl",
    4096,
    NULL,
    2,
    NULL,
    ARDUINO_RUNNING_CORE);

  xTaskCreatePinnedToCore(
    TaskBatteryVoltage,
    "TaskBatteryVoltage",
    4096,
    NULL,
    2,
    NULL,
    ARDUINO_RUNNING_CORE);

  xTaskCreatePinnedToCore(
    TaskTelemeryLoggerSdCard, 
    "TaskTelemeryLoggerSdCard", 4096  // Stack size
    ,
    NULL, 2  // Priority
    ,
    NULL, ARDUINO_RUNNING_CORE);
}

void loop() {}

void appendFile(fs::FS& fs, const char* path, const char* message) {
  Serial.printf("Appending to file: %s\n", path);

  File file = fs.open(path, FILE_APPEND);
  if (!file) {
    Serial.println("Failed to open file for appending");
    return;
  }
  if (file.print(message)) {
    Serial.println("Message appended");
  } else {
    Serial.println("Append failed");
  }
  file.close();
}

// Write to the SD card (DON'T MODIFY THIS FUNCTION)
void writeFile(fs::FS& fs, const char* path, const char* message) {
  Serial.printf("Writing file: %s\n", path);

  File file = fs.open(path, FILE_WRITE);
  if (!file) {
    Serial.println("Failed to open file for writing");
    return;
  }
  if (file.print(message)) {
    Serial.println("File written");
  } else {
    Serial.println("Write failed");
  }
  file.close();
}
