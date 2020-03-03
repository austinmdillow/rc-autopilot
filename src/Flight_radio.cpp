#if defined(ARDUINO) && ARDUINO >= 100
  #include "Arduino.h"
  #else
  #include "WProgram.h"
  #endif

#include <Flight_radio.h>
#include <RF24.h>

Flight_radio::Flight_radio(int pinA_in, int pinB_in) : transceiver(pinA_in, pinB_in) {
  //radioSetup();
}

void Flight_radio::radio_setup() {
  transceiver.begin();
  transceiver.setPALevel(RF24_PA_LOW);
  transceiver.openWritingPipe(masterAddress[0]);
  transceiver.openReadingPipe(1,slaveAddress[1]);
  transceiver.startListening();
}

void Flight_radio::readRadio() {
  if(transceiver.available()){
    // Variable for the received timestamp
    while (transceiver.available()) {                                   // While there is data ready
      transceiver.read( &data_rx, sizeof(data_rx) );             // Get the payload
    }
  }
}