/**
 * This is the main brain of the operation that talks to the Aircraft interface and 
 * receives information from the navigation module.
 */

#include <Arduino.h>
#include "duet.cpp"
#include "AcPID.cpp"
//#include <PID_v1.h>
#include "Flight_Radio.h"
#include "Flight_Navigation.h"
#include <PID_Em.h>
#include <RF24.h>
#include <SPI.h>
//#include <RF24.h>
const String VERSION = "0.2";
byte addressesQ[][6] = {"1Node","2Node"};

const int internal_led_pin = 13;
bool internal_led_state = true;

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
Flight_Radio flight_radio(8,9);
//Adafruit_FXAS21002C gyro = Adafruit_FXAS21002C(0x0021002C);
Flight_Navigation flight_nav;


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

void sensorDebug() {
  //flight_nav.displaySensorDetails();
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
  // put your setup code here, to run once:
  pidSetup();
  flight_radio.begin();
  flight_nav.setup();
  debuggingSetup();
  pinMode(internal_led_pin, OUTPUT); // set the mode of the built in LED
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
      //sensorDebug();
      break;
    default:
      manualControl();
  }
  
}

