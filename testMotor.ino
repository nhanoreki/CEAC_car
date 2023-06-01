#include "carMazeOfOz.h"


carMazeOfOz car;

void setup()
{
  car.setPin();
}

void loop()
{
    car.setMotorLeft(setSpeedLeft, 1);
    car.setMotorRight(setSpeedRight, 1);
    delay(1000);
}