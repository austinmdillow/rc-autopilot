/**
 * This is the main brain of the operation that talks to the Aircraft interface and 
 * receives information from the navigation module.
 */

#include <Arduino.h>
#include "duet.cpp"
#include "AcPID.cpp"
//#include <PID_v1.h>
#include "Flight_radio.h"
#include <PID_Em.h>
#include <RF24.h>
#include <SPI.h>
//#include <RF24.h>
const String VERSION = "1.0";
byte addressesQ[][6] = {"1Node","2Node"};

/*
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
*/

//Remote data;
//RF24 flightRadio(9,10);
//extern RF24 flightRadio(9,10);

/**
void setupRadio() {
  flightRadio.begin();
  flightRadio.setPALevel(RF24_PA_LOW);
  flightRadio.openWritingPipe(addressesQ[0]);
  flightRadio.openReadingPipe(1,addressesQ[1]);
  flightRadio.startListening();
}

void readRadio() {
  if(flightRadio.available()){
    // Variable for the received timestamp
    while (flightRadio.available()) {                                   // While there is data ready
      flightRadio.read( &data_in, sizeof(data_in) );             // Get the payload
    }
  }
}
*/

void manualControl();
void pidSetup();


Duet plane;
Flight_radio flight_radio(8,9);

// Define PID parameters and IO for PID control
double roll_input, roll_output;
double roll_setpoint;
double roll_kp=5, roll_ki=1, roll_kd=1;

//PID rollControl2(&roll_input, &roll_output, roll_setpoint, roll_kp, roll_ki, roll_kd, DIRECT);
PID_Em rollControl(&roll_input, &roll_output, &roll_setpoint, roll_kp, roll_ki, roll_kd);


enum Flying_states {
  Manual,
  Landed,
  Takeoff,
  Cruise,
  Stall,
  Test
};

Flying_states flying_state = Test;

struct Error {
  int id;
  String description;
  unsigned long time_occurred;
};

void test_controller() {
  static int count = 0;
  plane.commandElevator(0);
  plane.commandAileron(5);
  plane.commandThrottle(count);
  delay(1000);
  Serial.println(plane.fc.throttle);
  count++;

  int num_ops = 10000;
  volatile float out_num;
  unsigned long time_start = micros();
  for (int i = 0; i < num_ops; i++) {
    out_num = 284 * i;
  }
  unsigned long time_end = micros();
  Serial.print("Time to perform operations = ");
  Serial.println(time_end-time_start);
  Serial.print("Ops per second = ");
  Serial.println(num_ops/(float((time_end-time_start)/1000000.0)));

}

void manualControl() {
  //Serial.print(data_rx.target_id);
}

void performTakeoff() {
  plane.updateThrottle(100);
  plane.updateElevator(20);
  delay(500);
  plane.updateElevator(0);

}

void scheduled_tasks() {
  //flight_radio.readRadio();
}

double getRoll() {
 return -1;
}

//! Does nothing
void performCruise(int altitude, int speed) {

}

void maintainBankAngle(int bank_angle) {
  rollControl.newSetpoint(bank_angle);
  roll_input = getRoll();
  rollControl.compute();
  plane.commandAileron(roll_output);
}

void errorHandler() {

}

void pidSetup() {
  rollControl.setOutputContraints(-90, 90);
  rollControl.setIntegralWindup(10, 20);
}

void debuggingSetup() {
  Serial.begin(9600);
  Serial.println("===== rc-autopilot =====");
  Serial.print("Version: ");
  Serial.println(VERSION);
  delay(3000);
  Serial.print(45);
}

void setup() {
  delay(1000);
  // put your setup code here, to run once:
  pidSetup();
  flight_radio.radio_setup();
  debuggingSetup();
  Serial.println("end of setup");
}

void loop() {
  Serial.println("top of loop");
  scheduled_tasks();
  /*
  if (flying_state == Landed) {
    performTakeoff();
  } else {
    performCruise(5, 3);
  }
  */
  
  switch (flying_state) {
    case Manual: 
      manualControl();
      break;
    case Test:
      test_controller();
      break;
    default:
      manualControl();
  }
  
}

