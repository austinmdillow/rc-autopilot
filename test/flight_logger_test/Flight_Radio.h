#ifndef Flight_Radio_h
#define Flight_Radio_h
#include <SPI.h>
#include <RF24.h>

#define PRINTLN(a) (Serial.println(a))
#define PRINT(a) (Serial.print(a))

typedef struct {
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
  } radio_t;

class Flight_Radio {
public:
  Flight_Radio(int pinA_in, int pinB_in);
  void begin();
  bool rxRadio();
  bool txRadio(radio_t* tx);

private:
  double kp, ki, kd;
  RF24 transceiver;
  radio_t data_rx;
  //uint64_t addresses[][6] = {0xF0F0F0F066};
  const byte slaveAddress[5] = {'R','x','A','A','A'};
  const byte masterAddress[5] = {'T','X','a','a','a'};
};

#endif
