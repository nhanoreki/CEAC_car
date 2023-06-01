#include "carMazeOfOz.h"

#define STEP_TIME_64 4e-6
#define TIMER1_STEP_CYCLE 65536

const byte trig = 8;
const float speedSetPoint = 70;
const float distanceSetPoint = 9;
const float rateSetPoint = 0;

struct countPulse {
  byte value = 0;
  bool status = false;
};

float PWM_LEFT_SAMPLE[11];
float PWM_RIGHT_SAMPLE[11];
bool getSample = true;
volatile countPulse turn;
bool turnRight = false;
bool turnFinish = true;

carMazeOfOz car;

volatile float speedValueLeft, speedValueRight;
volatile float speedValueLeft_SAMPLE[11], speedValueRight_SAMPLE[11];
volatile unsigned long timerPoint = 0, currentEncoderLeft = 0, currentEncoderRight = 0;

// Your variables is in the area below
//--------------------------------------------------//
byte setSpeedLeft;
byte setSpeedRight;
float setDistance;
float setRate;


//--------------------------------------------------//

void setup() {
  Serial.begin(9600);
  car.setPin();
  car.setInterrupt();
  attachInterrupt(0, ENC_LEFT_ISR, RISING);
  attachInterrupt(1, ENC_RIGHT_ISR, RISING);

  // while (millis() < 5000) {
  //   static byte x = 0;
  //   speedLeftPID();
  //   car.setMotorLeft(setSpeedLeft, 1);
  //   PWM_LEFT_SAMPLE[10] -= PWM_LEFT_SAMPLE[x];
  //   PWM_LEFT_SAMPLE[x] = setSpeedLeft;
  //   PWM_LEFT_SAMPLE[10] += PWM_LEFT_SAMPLE[x];
  //   delay(10);
    
  //   speedRightPID();
  //   car.setMotorRight(setSpeedRight, 1);
  //   PWM_RIGHT_SAMPLE[10] -= PWM_RIGHT_SAMPLE[x];
  //   PWM_RIGHT_SAMPLE[x] = setSpeedRight;
  //   PWM_RIGHT_SAMPLE[10] += PWM_RIGHT_SAMPLE[x];
  //   delay(10);
  //   x = (x + 1) % 10;
  // }
  // setSpeedLeft = PWM_LEFT_SAMPLE[10] / 10;
  // setSpeedRight = PWM_RIGHT_SAMPLE[10] / 10;
  // car.setMotorLeft(0, 1);
  // car.setMotorRight(0, 1);
  // delay(1000);
}

void ENC_LEFT_ISR() {
  static byte i = 0;
  speedValueLeft_SAMPLE[10] -= speedValueLeft_SAMPLE[i];
  speedValueLeft_SAMPLE[i] = (TCNT1 - currentEncoderLeft + TIMER1_STEP_CYCLE) % TIMER1_STEP_CYCLE;
  speedValueLeft_SAMPLE[10] += speedValueLeft_SAMPLE[i];
  speedValueLeft = 255254.4 / (speedValueLeft_SAMPLE[10] / 10);
  car.setSpeedLeft(speedValueLeft);
  i = (i + 1) % 10;
  currentEncoderLeft = TCNT1;
  if (turn.status) {
    turn.value++;
  }
}

void ENC_RIGHT_ISR() {
  static byte i = 0;
  speedValueRight_SAMPLE[10] -= speedValueRight_SAMPLE[i];
  speedValueRight_SAMPLE[i] = (TCNT1 - currentEncoderRight + TIMER1_STEP_CYCLE) % TIMER1_STEP_CYCLE;
  speedValueRight_SAMPLE[10] += speedValueRight_SAMPLE[i];
  speedValueRight = 255254.4 / (speedValueRight_SAMPLE[10] / 10);
  car.setSpeedRight(speedValueRight);
  i = (i + 1) % 10;
  currentEncoderRight = TCNT1;
  if (turn.status) {
    turn.value++;
  }
}

// Your functions is in the area below
//--------------------------------------------------//
void speedLeftPID() {
  float Kp = 100, Ki = 1;
  float e = speedSetPoint - car.getSpeedLeft();
  float P = Kp * e;
  static float I = 0;
  I += Ki * e;
  setSpeedLeft = constrain(P + I, 0, 255);
}

void speedRightPID() {
  float Kp = 1, Ki = 0.2;
  float e = speedSetPoint - car.getSpeedRight();
  float P = Kp * e;
  static float I = 0;
  I += Ki * e;
  setSpeedRight = constrain(P + I, 0, 255);
}

void distancePID() {
  float Kp = 2, Ki = 0.25;
  float e = distanceSetPoint - car.getDistanceLeft();
  float P = Kp * e;
  static float I = 0;
  if (e > -0.5 && e < 0.5) {
    I = 0;
  } else {
    I += Ki * e;
  }
  setDistance = constrain(P + I, -50, 50);
}

void rateOfChangeDistancePID() {
  float Kp = 0.1, Ki = 0.1;
  float e = rateSetPoint - car.getRateOfChangeDistanceLeft();
  float P = Kp * e;
  static float I = 0;
  if (e > -0.5 && e < 0.5) {
    I = 0;
  } else {
    I += Ki * e;
  }
  setRate = constrain(P + I, -50, 50);
}

void stopMotor() {
  car.setMotorLeft(0, 1);
  car.setMotorRight(0, 1);
}


//--------------------------------------------------//

void loop() {
  digitalWrite(trig, LOW);
  car.setSpeedLeft(speedValueLeft);
  car.setSpeedRight(speedValueRight);
  car.configureSpeed(speedValueLeft, speedValueRight);

  // Your code is in the area below
  //--------------------------------------------------//
  car.setMotorRight(setSpeedRight, 1);
  speedRightPID();
  car.setMotorLeft(255, 1);
  speedLeftPID();
//  Serial.print(car.getSpeedRight());
//  Serial.print(" ");
  Serial.println(car.getSpeedLeft());
  
  //--------------------------------------------------//
}
