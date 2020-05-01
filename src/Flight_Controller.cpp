#if defined(ARDUINO) && ARDUINO >= 100
  #include "Arduino.h"
  #else
  #include "WProgram.h"
  #endif
#include "Flight_Controller.h"
#include "aeroscout.cpp"


Flight_Controller::Flight_Controller(flight_sensors_t* sensor_data_in) {
  //PID_Em rollControl = PID_Em(&roll_input, &roll_output, &roll_setpoint, roll_kp, roll_ki, roll_kd);
  sensor_data = sensor_data_in;
}

void Flight_Controller::begin() {
  plane.begin();
  surfaceTest();
}

void Flight_Controller::maintainBankAngle(int bank_angle) {
  
  //rollControl.newSetpoint(bank_angle);
  //roll_input = getRoll();
  //rollControl.compute();
  plane.commandAileron(roll_output);
}


bool Flight_Controller::performCruise(int altitude_target, int speed) {
  float altitude_error = sensor_data->bme280.altitude - altitude_target;
  return true;
}

void errorHandler() {

}

void Flight_Controller::pidSetup() {
  rollControl.setOutputContraints(-90, 90);
  rollControl.setIntegralWindup(10, 20);
}

void Flight_Controller::surfaceTest() {
  plane.commandElevator(-5.0);
}