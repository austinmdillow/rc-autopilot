/**
 * @file aeroscout.cpp
 * @author My Self
 * @date 9 Sep 2012
 * @brief File containing example of doxygen usage for quick reference.
 *
 * Here typically goes a more extensive explanation of what the header
 * defines. Doxygens tags are words preceeded by either a backslash @\
 * or by an at symbol @@.
 * @see http://www.stack.nl/~dimitri/doxygen/docblocks.html
 * @see http://www.stack.nl/~dimitri/doxygen/commands.html
 */

#ifndef Aeroscout_cpp
#define Aeroscout_cpp

#include "Aircraft_Interface.cpp"
#include <Servo.h>


class Aeroscout : public Aircraft_Interface {

private:
    Servo elevator_servo;
    const int elevator_pin = 3; ///< @bug needs to be set correctly
    const int motor_min_pwm = 0;
    const int motor_max_pwm = 250;
    const int motor_L_pin = 10; ///< @bug needs to be set correctly
    const int motor_R_pin = 9; ///< @bug needs to be set correctly
    int motor_L_pwm;
    int motor_R_pwm;
    int elevator_setpoint;

    const int ELEVATOR_SERVO_MAX = 100;
    const int ELEVATOR_SERVO_MIN = 80;

    const int K_aileron = 2; ///< how much an aileron input contributes to throttle adjustments

    /**
     * Control the motors through an H-bridge.
     * 
     * This function looks at the aileron and throttle setpoints in order to compute an approproate PWM signal
     */
    void writeMotors() {
        int throttle_L_net = constrain(fc.throttle + (fc.aileron + fc.aileron_trim) * K_aileron, 0, MAX_INPUT);
        int throttle_R_net = constrain(fc.throttle - (fc.aileron + fc.aileron_trim) * K_aileron, 0, MAX_INPUT);
        motor_L_pwm = map(throttle_L_net, 0, MAX_INPUT, motor_min_pwm, motor_max_pwm);
        motor_R_pwm = map(throttle_R_net, 0, MAX_INPUT, motor_min_pwm, motor_max_pwm);
        analogWrite(motor_L_pin, motor_L_pwm);
        analogWrite(motor_R_pin, motor_R_pwm);
    }


public:
    
    Flight_controls fc; ///< initialize the flight controls struct

    //! either a 3CH or 4CH @bug not used
    int num_channels;
    

    /** 
     * Initialize all electronic hardware on the aircraft.
     */
    void begin() {
        elevator_servo.attach(elevator_pin);
    }

    /**
     * Updates the throttle to the aircraft
     * @param input ranges from 0 to 100 (in %)
     */
    void updateThrottle(int input) {
        fc.throttle = input * 2;
    }


    void updateAileron(int aileron_in) {
        fc.aileron = aileron_in;
    }

    void updateAileronTrim(int aileron_trim_in) {
        fc.aileron_trim = aileron_trim_in;
    }

    void updateElevator(float elevator_in) {
        fc.elevator = elevator_in;
    }

    void updateElevatorTrim(int elevator_trim_in) {
        fc.elevator_trim = elevator_trim_in;
    }

    void updateRudder(int rudder_in) {
        fc.rudder = rudder_in;
    }

    void updateRudderTrim(int rudder_trim_in) {
        fc.rudder_trim = rudder_trim_in;
    }

    void updateflaps(int flaps_in) {
        fc.flaps = flaps_in;
    }

    void updateControlSurfaces() {

    }


    void commandThrottle(int throttle_in) {
        updateThrottle(throttle_in);
        writeMotors();
    }

    void commandElevator(float elevator_in) {
        updateElevator(elevator_in);
        int elevator_net = constrain(fc.elevator + fc.elevator_trim, MIN_INPUT, MAX_INPUT);
        elevator_servo.write(map(elevator_net, MIN_INPUT, MAX_INPUT, ELEVATOR_SERVO_MIN, ELEVATOR_SERVO_MAX)); 
    }

    void commandAileron(int aileron_in) {
        updateAileron(aileron_in);
        writeMotors();
    }

    void resetTrim() {
        fc.aileron_trim = 0;
        fc.elevator_trim = 0;
        fc.rudder_trim = 0;
    }

};

#endif