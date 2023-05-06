#include <IntervalTimer.h>
#include <SoftwareSerial.h>

// Sensors Variables Declaration
int TEAM_ID = 1007;
int PACKETCOUNT, PRESSURE ;
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
GPGGA gpsDatas;


IntervalTimer GpsTimer, I2CTimer, SendDataTimer, Timer4;
SoftwareSerial xbee(Xbee_rx, Xbee_tx);


void setup() {
  GpsTimer.begin(GpsTimer_ISR             , 1000000); 
  I2CTimer.begin(I2CTimer_ISR             , 1000000);  
  SendDataTimer.begin(SendDataTimer_ISR   , 1000000); 
  Timer4.begin(Timer4_ISR                 , 1000000);

  xbee.begin(9600);

}

void loop() {
  // put your main code here, to run repeatedly:

}

void GpsTimer_ISR(){
    Serial.println("1_Timer - gps");
    


}

void I2CTimer_ISR(){
  Serial.println("2_Timer - i2c");

}

void SendDataTimer_ISR(){
  Serial.println("3_Timer - send");
   String DATA_ALL = String(TEAM_ID) + "," + String(MISSION_TIME) + "," + String(PACKETCOUNT) + "," + MODE+ "," + STATE+ "," + String(ALTITUDE) + "," + HS_DEPLOYED + "," + PC_DEPLOYED + "," + MAST_RAISED + "," + String(TEMPERATURE) + "," + String(PRESSURE) + "," + String(VOLTAGE) + "," + String(gpsDatas.GPS_TIME) + "," + String(gpsDatas.GPS_ALTITUDE) + "," + String(gpsDatas.GPS_LATITUDE) + "," + String(gpsDatas.GPS_LONGITUDE) + "," + String(gpsDatas.GPS_SATS) + "," + String(TILT_X) + "," + String(TILT_Y) + "," + CMD_ECHO;

  if (xbee.available()) {
    String incoming_data = xbee.readStringUntil('\n');
            Serial.println("Gelen Ham Veri -->  " + incoming_data + "   ;  " + "Boyutu -->  " + incoming_data.length());


  if (incoming_data.length() == 12) {
      if (incoming_data[0] == '!') {
        Serial.println("Alınan Fix Data : " + incoming_data);
         
         }
     }
}

    

  xbee.println(DATA_ALL);

}

void Timer4_ISR(){
  Serial.println("4_Timer - timer4");

}
