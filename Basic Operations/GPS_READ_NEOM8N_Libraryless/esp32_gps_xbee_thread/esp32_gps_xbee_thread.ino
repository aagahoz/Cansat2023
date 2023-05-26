#include <HardwareSerial.h>
HardwareSerial SerialGPS(1);  // RX: GPIO RX, TX: GPIO TX


void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  SerialGPS.begin(9600, SERIAL_8N1, 14, 13); // RX - TX



  xTaskCreatePinnedToCore(
    TaskXBeeSend,
    "TaskXBeeSend",
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
}

void TaskXBeeSend(void* pvParameters) {
  (void)pvParameters;

  for (;;) {


    Serial.println("XBee_Payload_Telemetry");



    Serial.print("2- XBEE SEND PAYLOAD -->");
    Serial.println("XBee_Payload_Telemetry");

    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

void TaskGPS(void* pvParameters) {
  (void)pvParameters;

  for (;;) {


    if (SerialGPS.available()) {                         // GPS modülünden veri var mı kontrol et
      String message = SerialGPS.readStringUntil('\n');  // Gelen veriyi oku
      Serial.println(message);
      if (message.substring(0, 6) == "$GNGGA") {         // Eğer GNGGA mesajıysa

      Serial.println("Virgul falan filan");

       
      }
    }
    Serial.println("5- GPS DataS --> ");
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

void loop() {
  // put your main code here, to run repeatedly:
}
