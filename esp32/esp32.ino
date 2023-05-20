#if CONFIG_FREERTOS_UNICORE
#define ARDUINO_RUNNING_CORE 0
#else
#define ARDUINO_RUNNING_CORE 1
#endif

#ifndef LED_BUILTIN
#define LED_BUILTIN 13
#endif

#include <Wire.h>
#include <Adafruit_BMP280.h>
#include "RTClib.h"

#include <HardwareSerial.h>
HardwareSerial XBee(1);  //if using UART1


Adafruit_BMP280 bmp;
RTC_DS1307 newRTC;

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
Adafruit_MPU6050 mpu;

int TEAM_ID = 1007;
int PACKETCOUNT, PRESSURE;
String MODE, STATE, HS_DEPLOYED, PC_DEPLOYED, MAST_RAISED, CMD_ECHO;
float ALTITUDE, TEMPERATURE, VOLTAGE, TILT_X, TILT_Y;
char MISSION_TIME[32];


void TaskRTC( void *pvParameters );
void TaskBMP280( void *pvParameters );
void TaskLoRa( void *pvParameters );

bool rtc_find_state = false;
bool rtc_running_state = false;

////////////////////////////////////////
const String p1_takimNo = "389590";
String p2_paketNumarasi = "0";
String p3_gondermeSaati = ",,,,,";
String p4_basinc1 = "";
String p5_basinc2 = "";
String p6_yukseklik1 = "";
String p7_yukseklik2 = "";
String p8_irtifaFarki = "";
String p9_inisHizi = "";
String p10_sicaklik = "";
String p11_pilGerilimi = "";
String p12_gps1Latitude = "";
String p13_gps2Latitude = "";
String p14_gps1Altitude = "";
String p15_gps2Altitude = "";
String p16_gps1Longtitude = "";
String p17_gps2Longtitude = "";
String p18_uyduStatusu = "";
String p19_pitch = "";
String p20_roll = "";
String p21_yaw = "";
String p22_donusSayisi = "";
String p21_videoAktarimBilgisi = "";
////////////////////////////////////////


void setup() {
  
  Serial.begin(115200);
  XBee.begin(9600, SERIAL_8N1, 4, 2);

  unsigned status;
  status = bmp.begin(0x76);
  
   delay(500);
  
  rtc_find_state = newRTC.begin();
  if (!rtc_find_state) 
  {
    Serial.println("Couldn't find RTC");
  }

  rtc_running_state = newRTC.isrunning();
  if (!rtc_running_state)
  {
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
  
  
  // Now set up two tasks to run independently.
  xTaskCreatePinnedToCore(
    TaskRTC
    ,  "TaskRTC"   // A name just for humans
    ,  4096  // This stack size can be checked & adjusted by reading the Stack Highwater
    ,  NULL
    ,  2  // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
    ,  NULL 
    ,  ARDUINO_RUNNING_CORE);

  xTaskCreatePinnedToCore(
    TaskBMP280
    ,  "TaskBMP"
    ,  4096  // Stack size
    ,  NULL
    ,  2  // Priority
    ,  NULL 
    ,  ARDUINO_RUNNING_CORE);

  xTaskCreatePinnedToCore(
    TaskLoRa
    ,  "TaskBMP"
    ,  4096  // Stack size
    ,  NULL
    ,  2  // Priority
    ,  NULL 
    ,  ARDUINO_RUNNING_CORE);

    xTaskCreatePinnedToCore(
    TaskIMU
    ,  "TaskIMU"
    ,  4096  // Stack size
    ,  NULL
    ,  2  // Priority
    ,  NULL 
    ,  ARDUINO_RUNNING_CORE);


    
}

void loop(){}

void TaskRTC(void *pvParameters)
{
  (void) pvParameters;

  for (;;) 
  {    
    rtc_find_state = newRTC.begin();
    rtc_running_state = newRTC.isrunning();
    if (rtc_find_state && rtc_running_state)
    {
      DateTime now = newRTC.now();
      p3_gondermeSaati = String(now.day()) + "," + String(now.month()) + "," + String(now.year()) + ";" + String(now.hour()) + "," + String(now.minute()) + "," + String(now.second());
      Serial.println("RTC--> " + p3_gondermeSaati);
    }
    else
    {
      Serial.println("RTC ERROR!");
      p3_gondermeSaati = ",,;,,";
    }
    vTaskDelay(800); 
  }
}

void TaskBMP280(void *pvParameters) 
{
  (void) pvParameters;

  for (;;)
  {
    p10_sicaklik = String(bmp.readTemperature());
    p4_basinc1 = String(bmp.readPressure());
    p6_yukseklik1 = String(bmp.readAltitude(1017.5));
    Serial.println("BMP280--> " + p4_basinc1 + "," + p10_sicaklik + "," + p6_yukseklik1);
    vTaskDelay(800);  
  }
}

void TaskLoRa(void *pvParameters) 
{
  (void) pvParameters;

  for (;;)
  {
  //  Serial.println("Xbee--> " + String(bmp.readTemperature()) + "," + String(bmp.readPressure()) + "," + String(bmp.readAltitude(1017.5)));
  //  XBee.println("Xbee--> " + String(bmp.readTemperature()) + "," + String(bmp.readPressure()) + "," + String(bmp.readAltitude(1017.5)));
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    
      String DATA_ALL = String(TEAM_ID) + "," + String(MISSION_TIME) + "," + String(PACKETCOUNT) + "," + MODE + "," + STATE + "," + String(bmp.readAltitude(1017.5)) + "," + HS_DEPLOYED + "," + PC_DEPLOYED + "," + MAST_RAISED + "," + String(bmp.readTemperature()) + "," + String(bmp.readPressure()) + "," + String(0.0) + "," + String(0.00) + "," + String(0.00) + "," + String(0.00) + "," + String(0.0) + "," + String(0) + "," + String(g.gyro.x) + "," + String(g.gyro.y) + "," + CMD_ECHO  + "," + String(a.acceleration.x) + "," + String(a.acceleration.y) + "," + String(a.acceleration.y);
        Serial.println(DATA_ALL);
  XBee.println(DATA_ALL);

vTaskDelay(800);  
  }
}

void TaskIMU(void *pvParameters) 
{
  (void) pvParameters;

  for (;;)
  {
    Serial.println("IMU--> " );
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
 
    Serial.print("Rotation X: ");
    Serial.print(g.gyro.x);
    Serial.print(", Y: ");
    Serial.print(g.gyro.y);
    Serial.print(", Z: ");
    Serial.print(g.gyro.z);
    Serial.println(" rad/s");

//    String tempX = map(g.gyro.x, 

//    p19_pitch = String(g.gyro.x);
//    p20_roll = String(g.gyro.y);
//    p21_yaw = String(g.gyro.z);
//    
    p19_pitch = String(0);
    p20_roll = String(0);
    p21_yaw = String(0);  

    vTaskDelay(800);  
  }
}
