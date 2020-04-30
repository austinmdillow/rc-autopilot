#if defined(ARDUINO) && ARDUINO >= 100
  #include "Arduino.h"
  #else
  #include "WProgram.h"
  #endif
#include "Flight_Navigation.h"
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_FXAS21002C.h>
#include <Adafruit_FXOS8700.h>



Flight_Navigation::Flight_Navigation() {
}

void Flight_Navigation::setup() {
  Serial.println("Flight Navigation module Setup Complete");
}