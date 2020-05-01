#ifndef Flight_Controller_h
#define Flight_Controller_h

#include "Arduino.h"
#include "aeroscout.cpp"
//#include <PID_v1.h>
#include <PID_Em.h>
#include <Flight_Telemetry.h>

#define PRINTLN(a) (Serial.println(a))
#define PRINT(a) (Serial.print(a))

class Flight_Controller {
  public:
    Flight_Controller(flight_sensors_t* sensor_data_in);
    void begin();
    Aeroscout plane;
    bool performCruise(int altitude_target, int speed);
    void maintainBankAngle(int bank_angle);
    void surfaceTest();


  private:
    void pidSetup();
    flight_sensors_t* sensor_data;
    // Define PID parameters and IO for PID control
    float roll_input = 1, roll_output;
    float roll_setpoint;
    float roll_kp=5, roll_ki=1, roll_kd=1;

    float pitch_input = 0, pitch_output = 0;
    float pitch_setpoint;
    float pitch_kp=5, pitch_ki=1, pitch_kd=1;

    //PID rollControl2(&roll_input, &roll_output, roll_setpoint, roll_kp, roll_ki, roll_kd, DIRECT);
    PID_Em rollControl;
    //PID_Em pitchControl = PID_Em(&pitch_input, &pitch_output, &pitch_setpoint, pitch_kp, pitch_ki, pitch_kd);

};

#endif