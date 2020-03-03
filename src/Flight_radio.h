#ifndef Flight_radio_h
#define Flight_radio_h
#include <RF24.h>

class Flight_radio {
public:
  Flight_radio(int pinA_in, int pinB_in);

  void radio_setup();
  void readRadio();

  struct Remote {
    byte target_id = 1;
    byte pot_1;
    byte pot_2;
    byte stick_L_x;
    byte stick_L_y;
    bool stick_L_b;
    byte stick_R_x;
    byte stick_R_y;
    bool stick_R_b;
    bool estop;
    bool switch_1;
    bool switch_2;
    bool trim_U;
    bool trim_D;
    bool trim_L;
    bool trim_R;
  };

private:
  double kp, ki, kd;
  RF24 transceiver;
  struct Remote data_rx;
  //uint64_t addresses[][6] = {0xF0F0F0F066};
  const byte slaveAddress[5] = {'R','x','A','A','A'};
  const byte masterAddress[5] = {'T','X','a','a','a'};
};

#endif