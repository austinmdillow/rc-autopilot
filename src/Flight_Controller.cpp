#if defined(ARDUINO) && ARDUINO >= 100
  #include "Arduino.h"
  #else
  #include "WProgram.h"
  #endif
#include "Flight_Controller.h"
#include "aeroscout.cpp"


Flight_Controller::Flight_Controller(flight_sensors_t* sensor_data_in) : rollControl(&roll_input, &roll_output, roll_setpoint, roll_kp, roll_ki, roll_kd) {
  sensor_data = sensor_data_in;
}

void Flight_Controller::begin() {
  plane.begin();
  surfaceTest();
  //PID_Em rollControl = PID_Em(&roll_input, &roll_output, &roll_setpoint, roll_kp, roll_ki, roll_kd);
}


void Flight_Controller::maintainBankAngle(int bank_angle) {
  
  //rollControl.newSetpoint(bank_angle);
  //roll_input = getRoll();
  //rollControl.compute();
  plane.commandAileron(roll_output);
}


bool Flight_Controller::performCruise(int altitude_target, int speed) {
  float altitude_error = sensor_data->bme280.altitude - altitude_target;
  roll_input = sensor_data->imu.rpy[0];
  rollControl.newSetpoint(4);
  rollControl.compute();
  if (abs(altitude_error) > 5) {
    Serial.println("not at right altitude");
  }

  Serial.print("roll_output function val: ");Serial.println(roll_output);
  Serial.print("roll_input function val: ");Serial.println(roll_input);

  plane.commandAileron(roll_output);
  return true;
}

void errorHandler() {

}

void Flight_Controller::pidSetup() {
  rollControl.setOutputContraints(-100, 100);
  rollControl.setIntegralWindup(10, 20);
}

void Flight_Controller::surfaceTest() {
  Serial.println("Beginning Surface Test");
  plane.commandElevator(-5.0);
  delay(100);
  plane.commandElevator(0);
}