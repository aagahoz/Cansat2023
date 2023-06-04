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
// IMU
#include <Adafruit_BNO055.h> 
#include <utility/imumaths.h>
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

struct TelemetryStruct {
  const String team_id = "1007";
  String mission_time = "0";
  String packet_count = "0";
  String mode = "F";
  String state = "LAUNCH_WAIT";
  String altitude = "0";
  String hs_deployed = "N";
  String pc_deployed = "N";
  String mast_raised = "N";
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


float seaLevelPressure = 1013.25;  // deniz seviyesi basıncı (hektopaskal)
float currentAltitude = 0.0;      // bulunduğunuz yerin yüksekliği (metre)


// Kütüphaneler
Adafruit_BMP280 bmp;
Adafruit_BNO055 bno = Adafruit_BNO055(57);

HardwareSerial XBee(1);

Servo servo;


// Global Degerler
TelemetryStruct TelemetryObject;
String XBee_Incoming_Packet[10];

int getCommaCount(String str);
void parseTelemetryObject(String str, TelemetryStruct &TelemetryObject);

bool ServoState = false;
bool DcState = false;
const int servoPin = 25;
const int voltagePin = 34;

bool is_sd_card_mounted = true;
bool is_sd_card_attached = true;
bool is_sd_card_initialize = true;

bool telemetry_state = true ;   // true -> Real , false -> Simulation
bool telemetry_switch = true ; // true -> On , false -> Off

String XBee_Payload_Telemetry;




void TaskBMP280(void* pvParameters) {
  (void)pvParameters;

  float prevTemperature = 0.0;
  float prevPressure = 0.0;
  float prevAltitude = 0.0;

  for (;;) {
    //if (telemetry_state == true) {
      float temperature = bmp.readTemperature();
      float pressure = bmp.readPressure() / 100.0F;
      float altitude = bmp.readAltitude(1013.25);

      if (temperature != prevTemperature || pressure != prevPressure || altitude != prevAltitude) {
        TelemetryObject.temp = String(temperature);
        TelemetryObject.pressure = String(pressure);
        TelemetryObject.altitude = String(altitude);

        Serial.println("1- BMP280--> " + TelemetryObject.pressure + "," + TelemetryObject.temp + "," + TelemetryObject.altitude);

        // Önceki verileri güncelle
        prevTemperature = temperature;
        prevPressure = pressure;
        prevAltitude = altitude;
      }
      else {
        bmp.begin(0x76);
      }
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

void TaskXBeeSend(void* pvParameters) {
  (void)pvParameters;

  for (;;) {
    // if (telemetry_switch) {
    //     if(telemetry_state == false){
    //         Serial.println("Simulation Mode");
    //     }
        XBee_Payload_Telemetry = TelemetryObject.team_id + "," + TelemetryObject.mission_time + "," + TelemetryObject.packet_count + "," + TelemetryObject.mode + "," + TelemetryObject.state + "," + TelemetryObject.altitude + "," + TelemetryObject.hs_deployed + "," + TelemetryObject.pc_deployed + "," + TelemetryObject.mast_raised + "," + TelemetryObject.temp + "," + TelemetryObject.pressure + "," + TelemetryObject.volt + ","  + TelemetryObject.gps_time + "," + TelemetryObject.gps_altitude + "," + TelemetryObject.gps_latitude + "," + TelemetryObject.gps_longitude + "," + TelemetryObject.gps_sats + "," + TelemetryObject.tilt_x + "," + TelemetryObject.tilt_y + "," + TelemetryObject.cmd_echo + "," + TelemetryObject.accelX + "," + TelemetryObject.accelY + "," + TelemetryObject.accelZ;
        Serial.println("2- XBEE SEND PAYLOAD -->" + XBee_Payload_Telemetry);
        XBee.println(XBee_Payload_Telemetry);
    // }
    vTaskDelay(pdMS_TO_TICKS(1000)); 
  }
}

void TaskXBeeReceive(void* pvParameters) {
  (void)pvParameters;

    for (;;) {

        if (XBee.available()) {
            String incoming_data = XBee.readStringUntil('\n');

            Serial.println("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!Gelen Ham Veri -->  " + incoming_data + "   ;  " + "Boyutu -->  " + incoming_data.length() + "   ;  " + "Son -2 Char -->" + incoming_data[incoming_data.length() - 1]);

                if (incoming_data.length() == 12) {
                    if (incoming_data[0] == '!') {
                      if (incoming_data[incoming_data.length() - 1] == '+') {
                        Serial.println("Alınan Fix Data : " + incoming_data);
                      }
                    }
                }

                else if (incoming_data == "DC")  // 12
                {
                    digitalWrite(12, HIGH); // D13 pimini yüksek seviyeye ayarlama
                    Serial.println("****DC");
                    Serial.println();
                    Serial.println();

                    Serial.println();


                }
                else if (incoming_data == "PARASUT") // 0
                {
                    digitalWrite(0, HIGH); // D13 pimini yüksek seviyeye ayarlama
                    Serial.println("****PARASUT");
                    Serial.println();
                    Serial.println();
                    Serial.println();


                }
                else if (incoming_data == "UPRIGHT")  // 26
                {
                    digitalWrite(26, HIGH); // D13 pimini yüksek seviyeye ayarlama
                    Serial.println("****UPRIGHT");
                    Serial.println();
                    Serial.println();
                    Serial.println();



                }
                else if (incoming_data == "BUZZER")  // 33
                {
                    digitalWrite(33, HIGH); // D13 pimini yüksek seviyeye ayarlama
                    Serial.println("****BUZZER");
                    Serial.println();
                    Serial.println();
                    Serial.println();


                }

                else if (incoming_data == "DCOFF")  // 12
                {
                    digitalWrite(12, LOW); // D13 pimini yüksek seviyeye ayarlama
                    Serial.println("/////DCOFF");
                    Serial.println();
                    Serial.println();
                    Serial.println();



                }
                else if (incoming_data == "PARASUTOFF") // 0
                {
                    digitalWrite(0, LOW); // D13 pimini yüksek seviyeye ayarlama
                    Serial.println("/////PARASUTOFF");
                    Serial.println();
                    Serial.println();
                    Serial.println();



                }
                else if (incoming_data == "UPRIGHTOFF")  // 26
                {
                    digitalWrite(26, LOW); // D13 pimini yüksek seviyeye ayarlama
                    Serial.println("/////UPRIGHTOFF");
                    Serial.println();
                    Serial.println();
                    Serial.println();



                }
                else if (incoming_data == "BUZZEROFF")  // 33
                {
                    digitalWrite(33, LOW); // D13 pimini yüksek seviyeye ayarlama
                    Serial.println("/////BUZZEROFF");
                    Serial.println();
                    Serial.println();
                    Serial.println();


                }
                else if (incoming_data == "SERVO")  // 33
                {
                    servo.write(90);
                    Serial.println("/////SERVO");
                    Serial.println();
                    Serial.println();
                    Serial.println();


                }



                else if (incoming_data == "CMD,1007,SIM,ENABLE")
                {
                    TelemetryObject.mode= "S";  // Real pressure sending but when ACTIVE command comes, it will send fake datas.
                }
                
                else if (incoming_data == "CMD,1007,SIM,ACTIVE" && TelemetryObject.mode=="S")
                {
                    telemetry_state = false; // Take fake datas with SIMP commands 
                }

                else if (incoming_data == "CMD,1007,SIM,DISABLE" && TelemetryObject.mode=="S")
                {
                    TelemetryObject.mode= "F";; // Real pressure sending.
                    telemetry_state = true;
                }

                else if (incoming_data == "CMD,1007,CX,ON")
                {
                    telemetry_switch = true;
                }

                else if (incoming_data == "CMD,1007,CX,OFF")
                {
                    telemetry_switch = false;
                }

                // buraya ST EKLENECEK

                else if (incoming_data == "CMD,1007,CAL")
                {
                    
                }

                else if (incoming_data.substring(0, 14) == "CMD,1007,SIMP," && telemetry_state == false)
                {
                    String command = incoming_data.substring(14, incoming_data.length()-1); // bu satır kontrol edilecek
                    TelemetryObject.pressure= command;       // Fake pressure
                    Serial.println("Fake pressure OK");

                }
                else
                    Serial.println("CHECK ERROR");
        }
   


    Serial.println("3- XBEE Receive PAYLOAD -->");


    vTaskDelay(pdMS_TO_TICKS(900));
    }
}

void TaskIMU(void* pvParameters) {
  (void)pvParameters;

  for (;;) {
    sensors_event_t event;
    bno.getEvent(&event);

    float gyroX = event.orientation.x;
    float gyroY = event.orientation.y;
    float gyroZ = event.orientation.z;
    float accelX = event.acceleration.x;
    float accelY = event.acceleration.y;
    float accelZ = event.acceleration.z;
    

    String data = "4- IMU--> Rotation X: " + String(gyroX) + ", Y: " + String(gyroY) + ", Z: " + String(gyroZ) + " rad/s, Acceleration X: " + String(accelX) + ", Y: " + String(accelY) + ", Z: " + String(accelZ) + " m/s^2";
    Serial.println(data);

    TelemetryObject.tilt_x = String(gyroX);
    TelemetryObject.tilt_y = String(gyroY);
    TelemetryObject.accelX = String(accelX);
    TelemetryObject.accelY = String(accelY);
    TelemetryObject.accelZ = String(accelZ);

    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

void TaskGPS(void* pvParameters) {
  (void)pvParameters;

  for (;;) {

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

    Serial.println("7- DC State: ");
    vTaskDelay(pdMS_TO_TICKS(200));
  }
}

void TaskBatteryVoltage(void* pvParameters) {  /////////////////////// Duzenlenecek
  (void)pvParameters;

   const float r1 = 1000.0;  // R1 direnci değeri
   const float r2 = 1000.0;   // R2 direnci değeri

  for (;;) {
      float vtoplam = 0.0;
      int count = 0;

      for (int i = 0; i < 75; i++) {
        float analogvalue = analogRead(34);       // Analog pinden değer okunuyor
        float vout = analogvalue * 3.7 / 4096;  // Analog değeri gerilime dönüştürme (pin gerilimi)
        float vin = (((r1 + r2) * vout) / r2);

        if (vin != 0.00) {
          vtoplam = vtoplam + vin;
          count = count + 1;
        }
        delay(10);
      }

    float voltage = vtoplam / count;

    TelemetryObject.volt = String(voltage);

    Serial.print("8- Voltage --> ");
    // Serial.print(voltage, 3);
    Serial.println(" Volts");
    vTaskDelay(pdMS_TO_TICKS(50));
  }
}

void TaskTelemeryLoggerSdCard(void* pvParameters) {
  (void)pvParameters;

  for (;;) {


    if (is_sd_card_initialize == true) {

      String tempSdCardData = XBee_Payload_Telemetry + '\n';

      appendFile(SD, "/data.csv", tempSdCardData.c_str());
      Serial.println("Success SD Card Telemetry Write");
    } else {
      Serial.println("Failed SD Card Telemetry Write");
    }

    Serial.println("9- SD Card Logging --> " + is_sd_card_initialize);
    vTaskDelay(800);
  }
}

// void TaskCutRopes(void* pvParameters) {
//   (void)pvParameters;

//   for (;;) {
//     vTaskDelay(800);
//   }
// }

// void TaskMain(void* pvParameters) {
//   (void)pvParameters;

//   for (;;) {
//     vTaskDelay(800);
//   }
// }

void setup() {
  Serial.begin(9600);
  XBee.begin(57600, SERIAL_8N1, 14, 13);

  servo.attach(servoPin);


  pinMode(12, OUTPUT); // dc motor
  pinMode(32, OUTPUT);  // paraşüt
  pinMode(26, OUTPUT); // uprigth
  pinMode(33, OUTPUT); // buzzer

  delay(500);
  analogReadResolution(voltagePin);
  delay(500);

  unsigned status = bmp.begin(0x76);
  bno.begin();
  bno.setExtCrystalUse(true);


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
  // If the data.csv file doesn't exist
  // Create a file on the SD card and write the data labels
  if (is_sd_card_initialize == true) {
    File file = SD.open("/data.csv");
    if (!file) {
      Serial.println("File doens't exist");
      Serial.println("Creating file...");
      writeFile(SD, "/data.csv", "\r\n");
    } else {
      Serial.println("File already exists");
    }
    file.close();
  }



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
    "TaskTelemeryLoggerSdCard", 
    4096 ,
    NULL, 
    2,
    NULL, 
    ARDUINO_RUNNING_CORE);

    // xTaskCreatePinnedToCore(
    // TaskCutRopes, 
    // "TaskCutRopes", 
    // 4096 ,
    // NULL, 
    // 2,
    // NULL, 
    // ARDUINO_RUNNING_CORE);

    // xTaskCreatePinnedToCore(
    // TaskMain, 
    // "TaskMain", 
    // 4096 ,
    // NULL, 
    // 2,
    // NULL, 
    // ARDUINO_RUNNING_CORE);
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
  } 
  else {
    Serial.println("Write failed");
  }
  file.close();
}

int getCommaCount(String str){
    int sayac = 0;
    for (int i = 0; i < str.length(); i++)
    {
        if (str.charAt(i) == ',')
        {
            sayac++;
        }
    }
    return sayac;
}

void parseTelemetryObject(String str, TelemetryStruct &TelemetryObject){
    int pos = 0;
    for (int i = 0; i < 10; i++)
    {
        int nextPos = str.indexOf(',', pos);
        String valueStr = str.substring(pos, nextPos);
        pos = nextPos + 1;
        switch (i)
        {
        case 0:
            //strcpy(TelemetryObject.SENTENCEIDENTIFIER, valueStr.c_str());
            break;
        case 1:
            TelemetryObject.gps_time = valueStr.toFloat();
            break;
        case 2:
            TelemetryObject.gps_latitude = valueStr.toFloat();
            break;
        case 3:
            //TelemetryObject.LATUTE_DIR = valueStr.charAt(0);
            break;
        case 4:
            TelemetryObject.gps_longitude = valueStr.toFloat();
            break;
        case 5:
            //TelemetryObject.LONGTITUDE_DIR = valueStr.charAt(0);
            break;
        case 6:
            //TelemetryObject.FIX_QUALITY = valueStr.charAt(0);
            break;
        case 7:
            TelemetryObject.gps_sats = valueStr.toInt();
            break;
        case 8:
            //TelemetryObject.HORIZANTAL_DILUTION = valueStr.toFloat();
            break;
        case 9:
            TelemetryObject.gps_altitude = valueStr.toFloat();
            break;
        }
    }
}
