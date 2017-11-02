#include <XBee.h>
#include <string.h>

long int cont = 0;

void setup() {
  Serial.begin(38400);
}

void loop() {  
  receivingSerial();
  
  if (millis() - last_time > 1000){
    Serial.println(cont);
    last_time = millis();  
  }
}

void receivingSerial() {
  if (Serial.available() > 0){
    int buffer = Serial.read();
    cont++;
  }
}

