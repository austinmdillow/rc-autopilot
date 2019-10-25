#include <Arduino.h>
#include <RF24.h>
byte addresses[][6] = {"1Node","2Node"};

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

Remote data_rx;
