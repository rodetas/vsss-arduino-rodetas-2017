// A SER PEROJETADO
// timer 0 - 5 e 6
// timer 1 - 9 e 10 (Interrupção do controle PID - Portas Livres) 
// timer 2 - 3 e 11 (Motorees por causa da frequência mínima)

#include <XBee.h>
#include <string.h>

XBee xbee = XBee();
Tx16Request tx;
Rx16Response rx = Rx16Response();

const byte AIN2 = 4;
const byte AIN1 = 5;  
const byte STBY = 6;
const byte BIN1 = 7;
const byte BIN2 = 8;
const byte PWM_MOTORB = 9;
const byte PWM_MOTORA = 10;

int direction;
int pwm1 = 0;
int pwm2 = 0;

void setup() {

  pinMode(AIN2, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(STBY, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(PWM_MOTORB, OUTPUT);
  pinMode(PWM_MOTORA, OUTPUT);

  // set timer 1 PWM frequency of 31372.55 Hz
  TCCR1B = (TCCR1B & 0b11111000) | B00000001; 

  Serial.begin(19200);
  xbee.setSerial(Serial);
}

void loop() {
  
  receivingSerial();

  analogWrite(PWM_MOTORA, pwm1);
  analogWrite(PWM_MOTORB, pwm2);

  switch (direction) {
    case 'F': {
        forward();
      } break;
    case 'B': {
        back();
      } break;
    case 'R': {
        turnRight();
      } break;
    case 'L': {
        turnLeft();
      } break;
    case 'S': {
        stopped();
      } break;
  }
}

void receivingSerial() {
  xbee.readPacket();

  if (xbee.getResponse().isAvailable()) {

    if (xbee.getResponse().getApiId() == RX_16_RESPONSE) {

      xbee.getResponse().getRx16Response(rx);

      direction = int(rx.getData(0));
      pwm1 = (rx.getData(1) - '0') * 100 + (rx.getData(2) - '0') * 10 + (rx.getData(3) - '0');
      pwm2 = (rx.getData(4) - '0') * 100 + (rx.getData(5) - '0') * 10 + (rx.getData(6) - '0');
    }
  }
}

void forward() {
  digitalWrite(AIN2, HIGH);
  digitalWrite(AIN1, LOW);
  digitalWrite(BIN1, HIGH);
  digitalWrite(BIN2, LOW);
  digitalWrite(STBY, HIGH);
}

void back() {
  digitalWrite(AIN2, LOW);
  digitalWrite(AIN1, HIGH);
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, HIGH);
  digitalWrite(STBY, HIGH);
}

void turnLeft() {
  digitalWrite(AIN2, LOW);
  digitalWrite(AIN1, HIGH);
  digitalWrite(BIN1, HIGH);
  digitalWrite(BIN2, LOW);
  digitalWrite(STBY, HIGH);
}

void turnRight() {
  digitalWrite(AIN2, HIGH);
  digitalWrite(AIN1, LOW);
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, HIGH);
  digitalWrite(STBY, HIGH);
}

void stopped() {
  digitalWrite(STBY, LOW);
}
