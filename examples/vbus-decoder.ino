#include <SoftwareSerial.h>
#include "vbusdecoder.h"

SoftwareSerial vbusSerial(8,9);  // RX, TX
VBUSDecoder vbus(&vbusSerial);

uint32_t lastMillis = millis();

void setup() {
  vbusSerial.begin(9600);
  vbus.begin();
  
  Serial.begin(115200);
  Serial.println(F("Starting test of VBUS code.")); 
}

void loop() {
  vbus.loop();

  if ((millis() - lastMillis) > (5 * 1000UL)) {
    char * vbusStat = (vbus.getVbusStat())?"Ok":"Error";
    Serial.print(F("VBUS communication status: "));
    Serial.println(vbusStat);

    char * vbusReady = (vbus.isReady())?"Yes":"No";
    Serial.print(F("VBUS data ready: "));
    Serial.println(vbusReady);

    if (vbus.isReady()) {
      uint8_t tempNum = vbus.getTempNum();
      uint8_t relayNum = vbus.getRelayNum();
      uint8_t pumpNum = vbus.getPumpNum();
      
      Serial.print(F("Temperature sensors: "));
      for (uint8_t i = 0; i < tempNum; i++) {
        Serial.print(vbus.getTemp(i));
        Serial.print(F(", "));
      }
      Serial.println();

      Serial.print(F("Pump power: "));
      for (uint8_t i = 0; i < pumpNum; i++) {
        Serial.print(vbus.getPump(i));
        Serial.print(F(", "));
      }
      Serial.println();

      Serial.print(F("Relay Status: "));
      for (uint8_t i = 0; i < relayNum; i++) {
        Serial.print(vbus.getRelay(i)?"ON": "OFF");
        Serial.print(F(", "));
      }
      Serial.println();          
    }
    lastMillis = millis();
  }
}
