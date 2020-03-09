#ifndef Flight_Navigation_h
#define Flight_Navigation_h

#include "Arduino.h"
#include <Adafruit_Sensor.h>
#include <Adafruit_FXAS21002C.h>
#include <Adafruit_FXOS8700.h>


class Flight_Navigation {
  public:
    Flight_Navigation();
    void displaySensorDetails();
    void sensorDebug();
    void setup();

  private:

    int temp;
    Adafruit_FXAS21002C gyro;
    Adafruit_FXOS8700 accelmag;

};

#endif