/**
 * This is the main brain of the operation that talks to the Aircraft interface and 
 * receives information from the navigation module.
 */

#include <Arduino.h>
#include "duet.cpp"
#include "AcPID.cpp"
//#include "Iaircraft.cpp"

Duet plane;
PID yawControl;

enum Flying_states {
  Landed,
  TAKEOFF,
  CRUISE
};
Flying_states flying_state = Landed;

void performTakeoff() {
  plane.updateThrottle(100);
  plane.updateElevator(20);
  delay(500);
  plane.updateElevator(0);

}

//! Does nothing
void performCruise(int altitude, int speed) {

}

void setup() {
  // put your setup code here, to run once:
}

void loop() {
  // put your main code here, to run repeatedly:
  if (flying_state == Landed) {
    performTakeoff();
  } else {
    performCruise(5, 3);
  }

  
}
