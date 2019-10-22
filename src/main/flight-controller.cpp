#include <Arduino.h>
#include "duet.cpp"
//#include "Iaircraft.cpp"

Duet plane;

void takeoff() {
  plane.updateThrottle(100);

}

void setup() {
  // put your setup code here, to run once:
}

void loop() {
  // put your main code here, to run repeatedly:
  takeoff();
}

