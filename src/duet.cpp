#include "Iaircraft.cpp"
//#include <Servo.h>

class Duet : public Iaircraft {
public:
    
    Flight_controls fc;
    char aircraft_name;
    int num_channels;
    void updateThrottle(int input) {
        fc.throttle = input * 2;
    }

    void updateAileron(int aileron_in) {
        fc.aileron = aileron_in;
    }

    void updateAileronTrim(int aileron_trim_in) {
        fc.aileron_trim = aileron_trim_in;
    }

    void updateElevator(int elevator_in) {
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

private:
    void writeThrottle() {
        //Serial.print(fc.throttle);
    }
};