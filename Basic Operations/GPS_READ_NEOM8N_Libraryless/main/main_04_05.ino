#include <IntervalTimer.h>
#include <SoftwareSerial.h>
#include <Wire.h>             // BMP280
#include <Adafruit_BMP280.h>  // BMP280
#include <RTClib.h>           // RTC
#include <Adafruit_Sensor.h>  // BNO055
#include <Adafruit_BNO055.h>  // BNO055
#include <utility/imumaths.h> // BNO055
#include <Servo.h>

// Sensors Variables Declaration
int TEAM_ID        = 1007           ;
int PACKETCOUNT    = 1              ;
int PRESSURE       = 0              ;
String MODE        = "F"            ; //‘F’ for flight mode and ‘S’ for simulation mode.
String STATE       = "LAUNCH_WAIT"  ; // LAUNCH_WAIT, ASCENT, ROCKET_SEPARATION, DESCENT, HS_RELEASE, LANDED
String HS_DEPLOYED = "N"            ; // ‘P’ indicates the Probe with heat shield is deployed, ‘N’ otherwise.
String PC_DEPLOYED = "N"            ; // ‘C’ indicates the Probe parachute is deployed (at 200 m), ‘N’ otherwise.
String MAST_RAISED = "N"            ; // ‘M’ indicates the flag mast has been raised after landing, ‘N’ otherwise.
String CMD_ECHO     ;
float ALTITUDE, TEMPERATURE, VOLTAGE, TILT_X, TILT_Y;
char MISSION_TIME[32];

// EXTRAS
int TELEMETRY_STATE  = 1  ;  // 0 : SIM PRESSURE , 1 : REAL PRESSURE 
int TELEMETRY_SWITCH = 0  ;  // 0 : OFF , 1 : ONN

// GPS
typedef struct gpgga
{
    char SENTENCEIDENTIFIER[7]; // 1
    float GPS_TIME;             // 2
    double GPS_LATITUDE;        // 3  // resolution of 0.0001 degrees North.
    char LATUTE_DIR;            // 4
    double GPS_LONGITUDE;       // 5  // resolution of 0.0001 degrees West.
    char LONGTITUDE_DIR;        // 6
    char FIX_QUALITY;           // 7
    int GPS_SATS;               // 8
    float HORIZANTAL_DILUTION;  // 9
    double GPS_ALTITUDE;        // 10
} GPGGA;

GPGGA gpsDatas;

IntervalTimer GpsTimer, MainTimer, ComminicationTimer, SensorReadTimer;

Adafruit_BMP280 bmp;
RTC_DS3231 rtc;
Adafruit_BNO055 bno = Adafruit_BNO055(55);
SoftwareSerial xbee     (7, 8);
SoftwareSerial gpsSerial(0, 1); //

int getCommaCount(String str);
void parseGPGGA(String str, GPGGA &gpgga);

void setup()
{
    Wire.begin();
    bmp.begin(0x76);
    rtc.begin();
    bno.begin();
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    xbee.begin(9600);
    gpsSerial.begin(9600);
    ComminicationTimer.begin(ComminicationTimer_ISR, 1000000);
    GpsTimer.begin(GpsTimer_ISR, 1000000);
    MainTimer.begin(MainTimer_ISR, 1000000);
    SensorReadTimer.begin(SensorReadTimer_ISR, 1000000);
}

void loop()
{
    //    Serial.println("Loop_Func - Main Loop is running");
}

void GpsTimer_ISR()
{
    Serial.println("GpsTimer_ISR - gps is running");
    if (gpsSerial.available())
    {                                                     
        String message = gpsSerial.readStringUntil('\n'); 
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
            }
        }
    }
}

void MainTimer_ISR()
{
    Serial.println("MainTimer_ISR - i2c is running");

    String DATA_ALL = String(TEAM_ID) + "," + String(MISSION_TIME) + "," + String(PACKETCOUNT) + "," + MODE + "," + STATE + "," + String(ALTITUDE) + "," + HS_DEPLOYED + "," + PC_DEPLOYED + "," + MAST_RAISED + "," + String(TEMPERATURE) + "," + String(PRESSURE) + "," + String(VOLTAGE) + "," + String(gpsDatas.GPS_TIME) + "," + String(gpsDatas.GPS_ALTITUDE) + "," + String(gpsDatas.GPS_LATITUDE) + "," + String(gpsDatas.GPS_LONGITUDE) + "," + String(gpsDatas.GPS_SATS) + "," + String(TILT_X) + "," + String(TILT_Y) + "," + CMD_ECHO;
    if(TELEMETRY_SWITCH==1)
    {
        xbee.println(DATA_ALL);
        Serial.println("2_Timer - Data Sended");  
        PACKETCOUNT++; 
    }
      
}

void ComminicationTimer_ISR()
{
    Serial.println("ComminicationTimer_ISR - send is running");

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

        else if (incoming_data == "CMD,1007,SIM,ENABLE")
        {
            MODE= "S";  // Real pressure sending but when ACTIVE command comes, it will send fake datas.
        }
        
        else if (incoming_data == "CMD,1007,SIM,ACTIVE")
        {
            TELEMETRY_STATE = 0; // Take fake datas with SIMP commands 
        }

        else if (incoming_data == "CMD,1007,SIM,DISABLE")
        {
            MODE= "F";; // Real pressure sending.
            TELEMETRY_STATE = 1;
        }

        else if (incoming_data == "CMD,1007,CX,ON")
        {
            TELEMETRY_SWITCH = 1;
        }

        else if (incoming_data == "CMD,1007,CX,OFF")
        {
            TELEMETRY_SWITCH = 0;
            PACKETCOUNT = 1;
        }

        else if (incoming_data == "CMD,1007,CAL")
        {
            
        }

        else if (incoming_data.substring(0, 14) == "CMD,1007,SIMP," && TELEMETRY_STATE == 0)
        {
            String command = incoming_data.substring(14, incoming_data.length()-1); // bu satır kontrol edilecek
            PRESSURE = command.toInt();       // Fake pressure
        }
        else
            Serial.println("CHECK ERROR");
    }
}

void SensorReadTimer_ISR()
{
    Serial.println("SensorReadTimer_ISR - timer4 is running");
    if (TELEMETRY_STATE == 1) // REAL PRESSURE
    {
        // BMP280
        TEMPERATURE = bmp.readTemperature();
        PRESSURE = bmp.readPressure() / 100.0F;
        ALTITUDE = bmp.readAltitude(1013.25); // 1013.25 Referance pressure
    }
    
    Serial.print("TEMPERATURE : ");
    Serial.println(TEMPERATURE);
    Serial.print("PRESSURE : ");
    Serial.println(PRESSURE);
    Serial.print("ALTITUDE : ");
    Serial.println(ALTITUDE);

    // RTC
    DateTime now = rtc.now();
    sprintf(MISSION_TIME, "%02d:%02d:%02d %02d/%02d/%02d", now.hour(), now.minute(), now.second());

    Serial.print(F("MISSION TIME: "));
    Serial.println(MISSION_TIME);

    // BNO055
    sensors_event_t event;
    bno.getEvent(&event);
    TILT_X = event.orientation.x;
    TILT_Y = event.orientation.y;

    Serial.print("X: ");
    Serial.print(TILT_X);
    Serial.print("\tY: ");
    Serial.print(TILT_Y);
    Serial.println("");
}

int getCommaCount(String str)
{
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


