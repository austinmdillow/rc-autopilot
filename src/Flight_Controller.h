#ifndef Flight_Controller_h
#define Flight_Controller_h

#include "Arduino.h"
#include <SD.h>
#include <SPI.h>

#define PRINTLN(a) (Serial.println(a))
#define PRINT(a) (Serial.print(a))

class Flight_Controller {
  public:
    Flight_Controller();
    void begin();


  private:

};

#endif