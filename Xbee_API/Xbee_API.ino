#include <XBee.h>
#include <string.h>

XBee xbee = XBee();
Tx16Request tx;
Rx16Response rx = Rx16Response();

long int cont = 0;
unsigned long last_time = 0;


void setup() {
  Serial.begin(38400);
  xbee.setSerial(Serial);
}

void loop() {  
  receivingSerial();
  
  if (millis() - last_time > 1000){
    Serial.println(cont);
    last_time = millis();  
  }
}

void receivingSerial() {
  xbee.readPacket();

  if (xbee.getResponse().isAvailable()) {

    if (xbee.getResponse().getApiId() == RX_16_RESPONSE) {

      xbee.getResponse().getRx16Response(rx);
      
      cont = cont + rx.getDataLength();

    }
  }
}

