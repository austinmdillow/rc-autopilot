/*
#include <Arduino.h>
#include <RF24.h>

//uint64_t addresses[][6] = {0xF0F0F0F066};
const byte slaveAddress[5] = {'R','x','A','A','A'};
const byte masterAddress[5] = {'T','X','a','a','a'};

struct Remote {
  byte target_id = 1;
  byte pot_1;
  byte pot_2;
  byte stick_L_x;
  byte stick_L_y;
  bool stick_L_b;
  byte stick_R_x;
  byte stick_R_y;
  bool stick_R_b;
  bool estop;
  bool switch_1;
  bool switch_2;
  bool trim_U;
  bool trim_D;
  bool trim_L;
  bool trim_R;
};

struct Remote data_rx;

void setupRadio() {
  flightRadio.begin();
  flightRadio.setPALevel(RF24_PA_LOW);
  flightRadio.openWritingPipe(masterAddress[0]);
  flightRadio.openReadingPipe(1,slaveAddress[1]);
  flightRadio.startListening();
}

void readRadio() {
  if(flightRadio.available()){
    // Variable for the received timestamp
    while (flightRadio.available()) {                                   // While there is data ready
      flightRadio.read( &data_rx, sizeof(data_rx) );             // Get the payload
    }
  }
}
*/