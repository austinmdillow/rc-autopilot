/**
 * This is the main brain of the operation that talks to the Aircraft interface and 
 * receives information from the navigation module.
 */

#include <Arduino.h>
#include "duet.cpp"
#include "AcPID.cpp"
//#include <PID_v1.h>
#include <PID_Em.h>
//#include "Iaircraft.cpp"

void manualControl();
void pidSetup();


Duet plane;

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
  Stall
};
Flying_states flying_state = Landed;

struct Error {
  int id;
  String description;
  unsigned long time_occurred;
};

void manualControl() {
  
}

void performTakeoff() {
  plane.updateThrottle(100);
  plane.updateElevator(20);
  delay(500);
  plane.updateElevator(0);

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

void setup() {
  // put your setup code here, to run once:
  pidSetup();
}

void pidSetup() {
  rollControl.setOutputContraints(-90, 90);
  rollControl.setIntegralWindup(10, 20);
}

void loop() {
  // put your main code here, to run repeatedly:
  if (flying_state == Landed) {
    performTakeoff();
  } else {
    performCruise(5, 3);
  }

  switch (flying_state) {
    case Manual: 
      manualControl();
      break;
    default:
      manualControl();
  }

  
}

