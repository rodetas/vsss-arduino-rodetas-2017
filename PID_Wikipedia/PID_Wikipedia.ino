#include <XBee.h>
#include <string.h>

// COMMUNICATION
XBee xbee = XBee();
Tx16Request tx;
Rx16Response rx = Rx16Response();

// MOTOR CONFIG
const byte AIN2 = 4;
const byte AIN1 = 5;  
const byte STBY = 6;
const byte BIN1 = 7;
const byte BIN2 = 8;
const byte PWM_MOTOR_1 = 9;
const byte PWM_MOTOR_2 = 10;

// HZ
const double frequency = 400;

// PID PARAMETERS
double KP = 5, KI = 200, KD = 0;

// SECONDS
const double dt = (1 / frequency);

// MICROSECONDS
const double timer = (1 / frequency) * 1000000;

// RADIUS (1.6), ENCODER READINGS (375), CONVERTO TO SECONDS (0.000001)
const double speed_calc = ( (2 * PI * 1.6) / 375 ) / 0.000001;

// PID
double integral_1 = 0;
double last_error_1 = 0;
double integral_2 = 0;
double last_error_2 = 0;

int direction;

int pwm_1 = 0;
int cont_1 = 0;
int speed_1 = 0;
int speed_output_1 = 0;
int speed_average_1[10] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

int pwm_2 = 0;
int cont_2 = 0;
int speed_2 = 0;
int speed_output_2 = 0;
int speed_average_2[10] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

// TIMER
unsigned long last_micros = 0;
unsigned long last_millis = 0;
unsigned long last_border_micros_1 = 0;
unsigned long last_border_micros_2 = 0;

void setup() {

  pinMode(AIN2, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(STBY, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(PWM_MOTOR_1, OUTPUT);
  pinMode(PWM_MOTOR_2, OUTPUT);

  // SET TIMER 1 PWM FREQUENCY TO 31372.55 HZ
  TCCR1B = (TCCR1B & 0b11111000) | B00000001;
  
  attachInterrupt(0, encoder_1, FALLING);
  attachInterrupt(1, encoder_2, FALLING);

  Serial.begin(19200);
  xbee.setSerial(Serial);
}

void loop() {

  timerInterrupt();
  
  receivingSerial();

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
  
  if ((millis() - last_millis) > 30){
    Serial.print(speed_1);
    Serial.print(" - ");
    Serial.print(pwm_1);
    Serial.print(" | ");    
    Serial.print(speed_2);
    Serial.print(" - ");
    Serial.println(pwm_2);
    
    last_millis = millis();
  } 
  
}

void receivingSerial() {
  xbee.readPacket();

  if (xbee.getResponse().isAvailable()) {
    if (xbee.getResponse().getApiId() == RX_16_RESPONSE) {
      xbee.getResponse().getRx16Response(rx);
      direction = int(rx.getData(0));
      pwm_1 = (rx.getData(1) - '0') * 100 + (rx.getData(2) - '0') * 10 + (rx.getData(3) - '0');
      pwm_2 = (rx.getData(4) - '0') * 100 + (rx.getData(5) - '0') * 10 + (rx.getData(6) - '0');

      speed_output_1 = convertPWM_1(pwm_1);
      speed_output_2 = convertPWM_2(pwm_2);
    }
  }
}

void timerInterrupt(){

  if ((micros() - last_micros) >= timer){

    pwm_1 = PID_1(speed_output_1, speed_1);
    pwm_2 = PID_2(speed_output_2, speed_2);
    
    analogWrite(PWM_MOTOR_1, pwm_1);
    analogWrite(PWM_MOTOR_2, pwm_2);
     
    forward();

    // VERIFICANDO MOTORES PARADOS
    if (cont_1 == 0) speed_1 = 0;
    if (cont_2 == 0) speed_2 = 0;
    cont_1 = cont_2 = 0;
    
    last_micros = micros(); 
  } 
}

int PID_1(double setpoint, double input) {

  double error = setpoint - input;
  double derivative = (error - last_error_1) / dt;
  
  integral_1 = integral_1 + (error * dt);  
  last_error_1 = error;

  double output = KP * (error) + KI * (integral_1) + KD * (derivative);
    
  if (output > 255) output = 255;
  if (output < 0) output = 0;
  
  return output;
}

int PID_2(double setpoint, double input) {

  double error = setpoint - input;
  double derivative = (error - last_error_2) / dt;
  
  integral_2 = integral_2 + (error * dt);  
  last_error_2 = error;

  double output = KP * (error) + KI * (integral_2) + KD * (derivative);
    
  if (output > 255) output = 255;
  if (output < 0) output = 0;

  return output;
}

int convertPWM_1(int pwm){
  return 1 * pwm_1 + 1;
}

int convertPWM_2(int pwm){
  return 2 * pwm_2 + 2;
}

void encoder_1() {
  unsigned long t = micros() - last_border_micros_1;
  speed_1 = speedAverage_1(speed_calc / t);
  last_border_micros_1 = micros();
  cont_1++;
}

void encoder_2() {
  unsigned long t = micros() - last_border_micros_2;
  speed_2 = speedAverage_2(speed_calc / t);
  last_border_micros_2 = micros();
  cont_2++;
}

int speedAverage_1(int num) {
  speed_average_1[0] = speed_average_1[1];
  speed_average_1[1] = speed_average_1[2];
  speed_average_1[2] = speed_average_1[3];
  speed_average_1[3] = speed_average_1[4];
  speed_average_1[4] = speed_average_1[5];
  speed_average_1[5] = speed_average_1[6];
  speed_average_1[6] = speed_average_1[7];
  speed_average_1[7] = speed_average_1[8];
  speed_average_1[8] = speed_average_1[9];
  speed_average_1[9] = num;
  
  return (speed_average_1[0] + speed_average_1[1] + speed_average_1[2] + 
          speed_average_1[3] + speed_average_1[4] + speed_average_1[5] +
          speed_average_1[6] + speed_average_1[7] + speed_average_1[8] +
          speed_average_1[9]) / 10;
}

int speedAverage_2(int num) {
  speed_average_2[0] = speed_average_2[1];
  speed_average_2[1] = speed_average_2[2];
  speed_average_2[2] = speed_average_2[3];
  speed_average_2[3] = speed_average_2[4];
  speed_average_2[4] = speed_average_2[5];
  speed_average_2[5] = speed_average_2[6];
  speed_average_2[6] = speed_average_2[7];
  speed_average_2[7] = speed_average_2[8];
  speed_average_2[8] = speed_average_2[9];
  speed_average_2[9] = num;
  
  return (speed_average_2[0] + speed_average_2[1] + speed_average_2[2] + 
          speed_average_2[3] + speed_average_2[4] + speed_average_2[5] + 
          speed_average_2[6] + speed_average_2[7] + speed_average_2[8] + 
          speed_average_2[9]) / 10;
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

// A SER PROJETADO
// timer 0 - 5 e 6
// timer 1 - 9 e 10 (Interrupção do controle PID - Portas Livres) 
// timer 2 - 3 e 11 (Motorees por causa da frequência mínima)
