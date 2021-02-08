/**
 * This is the main brain of the operation that talks to the Aircraft interface and 
 * receives information from the navigation module.
 */

#include <Arduino.h>
#include "Flight_Telemetry.h"
#include "Flight_Logger.h"
#include "Flight_Radio.h"
#include "Flight_Controller.h"


const String VERSION = "0.2";

#define VBATPIN A6

const int internal_led_pin = 13;
bool internal_led_state = true;

const int chipSelect = 11;
bool SD_VALID = false;
const int CE = 13;
const int CSN = 12;


void manualControl();
void pidSetup();
void printBatteryDebug();

flight_sensors_t sensor_data;
Flight_Controller pilot(&sensor_data);
Flight_Radio radio(CE, CSN);
Flight_Logger logger(chipSelect);
Flight_Telemetry flight_tel;

//Adafruit_FXAS21002C gyro = Adafruit_FXAS21002C(0x0021002C);





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
  int loop_time = 20000;
  unsigned long count;
  while(millis() - start_time < loop_time) {
    delay(10);
    flight_tel.getSensors(&sensor_data);
    //flight_tel.printBME280();
    //flight_tel.printTelemetry();
    flight_tel.printAHRS();
  }
  flying_state = Wait;
}

void test_controller() {
  
  static int count = 0;
  count++;
  //flight_nav.sensorDebug();
  //flight_nav.displaySensorDetails();

  delay(300);
}

void manualControl() {
  //Serial.print(data_rx.target_id);
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

  flight_tel.getSensors(&sensor_data);

}

double getRoll() {
 return -1;
}

//! Does nothing
bool performCruise(int altitude_target, int speed) {
  float altitude_error = sensor_data.bme280.altitude - altitude_target;

}



bool climbToAltitude(int alt) {
  static unsigned long function_start = millis();

  pilot.performCruise(100, 10);

  if (millis() - function_start > 20000) {
    return true;

  }
    return false;

}

void debuggingSetup() {
  Serial.begin(9600);
  Serial.println("===== rc-autopilot =====");
  Serial.print("Version: ");
  Serial.println(VERSION);
  printBatteryDebug();

  delay(3000);
}

void setup() {
  delay(2000);
  flight_tel.begin();
  pilot.begin();
  debuggingSetup();
  pinMode(internal_led_pin, OUTPUT); // set the mode of the built in LED
  Serial.println("end of setup");
  
  flying_state = Cruise;
}

void loop() {
  //Serial.println("top of loop");
  delay(10);
  scheduled_tasks();
  Serial.println(sensor_data.imu.rpy[0]);
  /*
  if (flying_state == Landed) {
    performTakeoff();
  } else {
    performCruise(5, 3);
  }
  */
  
  switch (flying_state) {
    case Manual:
      Serial.println("State is manual");
      manualControl();
      break;
    case Test:
      //test_controller();
      sensorDump();
      break;
    case Wait:
      //Serial.println("Done");
      delay(10000);
      break;
    case Cruise:
      Serial.println("State is Cruise");
      if (climbToAltitude(300)) {
         flying_state = Wait;
      }
       break;
    default:
      manualControl();
  }
  
}





/**
 * Board Functions
*/



float boardBatteryVoltage() {
  float battery_voltage = 2 * 3.3 / 1024.0 * analogRead(VBATPIN);
  Serial.print("VBAT: "); Serial.println(battery_voltage);
  return battery_voltage;
}

void printBatteryDebug() {
  Serial.print("Bat V: "); Serial.println(boardBatteryVoltage());
}

