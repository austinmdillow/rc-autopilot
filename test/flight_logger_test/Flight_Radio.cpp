#if defined(ARDUINO) && ARDUINO >= 100
  #include "Arduino.h"
  #else
  #include "WProgram.h"
  #endif
#include "Flight_Radio.h"
#include <RF24.h>

Flight_Radio::Flight_Radio(int pinA_in, int pinB_in) : transceiver(pinA_in, pinB_in) {
  //radioSetup();
}

void Flight_Radio::begin() {
  transceiver.begin();
  transceiver.setPALevel(RF24_PA_LOW);
  transceiver.openWritingPipe(masterAddress[0]);
  transceiver.openReadingPipe(1,slaveAddress[1]);
  transceiver.startListening();
  transceiver.printDetails();
  PRINTLN("Radio Setup Complete");
  delay(250);
}

bool Flight_Radio::rxRadio() {
  if(transceiver.available()){
    // Variable for the received timestamp
    while (transceiver.available()) {                                   // While there is data ready
      transceiver.read( &data_rx, sizeof(data_rx) );             // Get the payload
    }
    return true;
  }
  return false; // if nothing was read
}

bool Flight_Radio::txRadio(radio_t* tx) {
  bool success;
  transceiver.stopListening();
  if (!transceiver.write( &tx, sizeof(tx) )){
    PRINTLN("tx failed."); 
    success = false;     
  } else {
    PRINTLN("Tx Sucess!");
    success = true;
  }
  transceiver.startListening();
  return success;
}
