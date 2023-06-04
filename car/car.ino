#include "carMazeOfOz.h"

#define STEP_TIME_64 4e-6
#define TIMER1_STEP_CYCLE 65536



const byte trig = 8;
const float BaseSpeed = 70;
const float distanceSetPoint = 7;
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
float PID;
float pre_error = 0;

//--------------------------------------------------//

//----PID set-----------//

float Kp = 12, Ki = 0, Kd = 0;
float P;
float I = 0;
float D;


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
//void speedLeftPID() {
//  float Kp = 1, Ki = 0.2;
//  float e = speedSetPoint - car.getSpeedLeft();
//  float P = Kp * e;
//  static float I = 0;
//  I += Ki * e;
//  setSpeedLeft = constrain(P + I, 0, 255);
//}
//
//void speedRightPID() {
//  float Kp = 1, Ki = 0.2;
//  float e = speedSetPoint - car.getSpeedRight();
//  float P = Kp * e;
//  static float I = 0;
//  I += Ki * e;
//  setSpeedRight = constrain(P + I, 0, 255);
//}

void distancePID() {
 
  float e = distanceSetPoint - car.getDistanceLeft();
  P = Kp * e;
  if (e > -0.5 && e < 0.5) {
    I = 0;
  } else {
    I += Ki * e;
  }
  D = pre_error * Kd;
  pre_error = e;
  // setDistance = constrain(P + I, -50, 50);
  PID = constrain(P + I + D, -70, 70);
  int left = BaseSpeed + PID;
  int right =  BaseSpeed - PID; 
  if (left < 0)
  {
      left = - 1 * left;
      car.setMotorLeft(left, 0);
  }
  else 
    car.setMotorLeft(left, 1);
  if (right < 0)
  {
      right = - 1 * right;
      car.setMotorRight(right, 0);
  }
  else 
    car.setMotorRight(right, 1);
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
  distancePID(); 
//  car.setMotorLeft(150, 1);
  
  Serial.print(car.getSpeedLeft());
  Serial.print(" ");
  Serial.println(car.getSpeedRight());
//  Serial.print(" ");
//  Serial.print(car.getDistanceHead());
//  Serial.print(" ");
//  Serial.println(car.getDistanceRight());
  
  //--------------------------------------------------//
}