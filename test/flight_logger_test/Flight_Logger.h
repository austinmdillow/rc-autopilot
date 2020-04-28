#ifndef Flight_Logger_h
#define Flight_Logger_h

#include "Arduino.h"
#include <SD.h>
#include <SPI.h>
#include "Flight_Telemetry.h"

#define PRINTLN(a) (Serial.println(a))
#define PRINT(a) (Serial.print(a))

class Flight_Logger {
  public:
    Flight_Logger(int chip_select);
    bool begin();
    bool end();
    void logData(char* saved_data);
    bool logGPS(gps_t* gps);
    bool logIMU(telemetry_t* tel);
    void setGPSLoggingRate(int hz);
    void setIMULoggingRate(int hz);
    void loggerDumpSerial();

  protected:
    byte status = 0;
    unsigned long num_logs;

  private:
    int _imu_rate = 10; // default 10Hz logging
    int _gps_rate = 10;
    int _imu_period = 1000 / 10;
    int _gps_period = 1000 / 10;
    bool getFilename();
    void error(); // TODO needs to me implemented
    int _chipSelect;
    bool SD_VALID = false;
    unsigned long _last_flush_time = millis();
    File _dataFile;
    char _filename[13];
};




#endif
