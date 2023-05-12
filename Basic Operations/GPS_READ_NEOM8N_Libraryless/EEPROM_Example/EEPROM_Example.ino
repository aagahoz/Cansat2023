#include <EEPROM.h>

String str;
int number;
String readStr;
int readNumber;

void setup() {
  
//  EEPROM'a int veri yazma
  int number = 1234;
//  EEPROM.put(0, number);

//  EEPROM'a String veri yazma
  str = "Hello";
  for(int i = 0; i < str.length(); i++) {
    EEPROM.write(10 + i, str[i]); // 10 numaralı adresten başlayarak yazıyoruz.
  }

  
}

void loop() {
// EEPROM'dan String veri okuma
  readStr = "";
  char readChar;
  for(int i = 0; i < str.length(); i++) {
    readChar = EEPROM.read(10 + i);
    readStr += readChar;
  }
  Serial.println(readStr);    
  
  
  //  EEPROM'dan int veri okuma
  EEPROM.get(0, readNumber);
  Serial.println(readNumber);

}
