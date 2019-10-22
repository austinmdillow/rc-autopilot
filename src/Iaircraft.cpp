
class Iaircraft {
    public:
    
    struct Flight_controls {
        int throttle;
        int aileron;
        int aileron_trim;
        int elevator;
        int elevator_trim;
        int rudder;
        int rudder_trim;
        int flaps;
        int spoiler;
        bool landing_gear;
        bool nav_lights;
        bool landing_lights;
    };

    virtual void updateThrottle(int) = 0;
    virtual void updateAileron(int aileron_in) = 0;
    virtual void updateAileronTrim(int aileron_trim_in) = 0;
    virtual void updateElevator(int elevator_in) = 0;
    virtual void updateElevatorTrim(int elevator_trim_in) = 0;
    virtual void updateRudder(int rudder_in) = 0;
    virtual void updateRudderTrim(int rudder_trim_in) = 0;
    virtual void updateflaps(int flaps_in) = 0;
    virtual void updateControlSurfaces() = 0;
    //void updatePose();
    //~Iaircraft();
    
};