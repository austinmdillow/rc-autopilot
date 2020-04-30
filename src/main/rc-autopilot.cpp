/**
 * This is the main brain of the operation that talks to the Aircraft interface and 
 * receives information from the navigation module.
 */

#include <Arduino.h>
#include "aeroscout.cpp"
#include "AcPID.cpp"
//#include <PID_v1.h>
#include <PID_Em.h>
#include "Flight_Telemetry.h"
#include "Flight_Logger.h"
#include "Flight_Radio.h"
#include "Flight_Controller.h"

const String VERSION = "0.2";

const int internal_led_pin = 13;
bool internal_led_state = true;


void manualControl();
void pidSetup();


Aeroscout plane;
Flight_Controller pilot;
Flight_Radio flight_radio(8,9);
Flight_Telemetry flight_tel;
flight_sensors_t sensor_data;
//Adafruit_FXAS21002C gyro = Adafruit_FXAS21002C(0x0021002C);


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
  Test,
  Wait
};

Flying_states flying_state = Test;

struct Error {
  int id;
  String description;
  unsigned long time_occurred;
};

void sensorDebug() {
  //flight_nav.displaySensorDetails();
  unsigned long start_time = millis();
  int loop_time = 1000;
  unsigned long count;
  while(millis() - start_time < loop_time){
    flight_tel.getSensors(&sensor_data);
    count++;
    flight_tel.printRateMonitors();
  }
  flight_tel.printRateMonitors();
  Serial.println(count);
  Serial.print("IMU "); Serial.println(flight_tel.imu_count);
  Serial.print("GPS "); Serial.println(flight_tel.gps_count);
  Serial.print("BME "); Serial.println(flight_tel.bme280_count);
  flying_state = Wait;
}

void sensorDump() {
  unsigned long start_time = millis();
  int loop_time = 10000;
  unsigned long count;
  while(millis() - start_time < loop_time) {
    flight_tel.getSensors(&sensor_data);
    //flight_tel.printBME280();
    flight_tel.printTelemetry();
  }
  flying_state = Wait;
}

void test_controller() {
  
  static int count = 0;
  plane.commandElevator(0);
  plane.commandAileron(5);
  plane.commandThrottle(count);
  Serial.println(plane.fc.throttle);
  count++;
  //flight_nav.sensorDebug();
  //flight_nav.displaySensorDetails();

  delay(300);
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
  static unsigned long last_blink_time = millis();
  int blink_interval = 700;
  //flight_radio.readRadio();

  if (millis() - last_blink_time > blink_interval) {
    internal_led_state = !internal_led_state;
    digitalWrite(internal_led_pin, internal_led_state);
    last_blink_time = millis();
  }

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
  flight_tel.begin();
  pidSetup();
  debuggingSetup();
  pinMode(internal_led_pin, OUTPUT); // set the mode of the built in LED
  Serial.println("end of setup");
  
}

void loop() {
  //Serial.println("top of loop");
  delay(1000);
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
      //test_controller();
      //sensorDebug();
      sensorDump();
      break;
    case Wait:
    Serial.println("Done");
      delay(10000);
      break;
    default:
      manualControl();
  }
  
}

