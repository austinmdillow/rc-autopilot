#ifndef Flight_Telemetry_h
#define Flight_Telemetry_h

//#include "Arduino.h"
#include <Adafruit_Sensor.h>
#include <Adafruit_FXAS21002C.h>
#include <Adafruit_FXOS8700.h>
#include <Adafruit_AHRS.h> // for heading filters
#include <Adafruit_GPS.h>

#define GPSSerial Serial1

typedef struct {
  sensors_vec_t gyro;
  sensors_vec_t acceleration;
  sensors_vec_t magnetic;
  float rpy[3];
} telemetry_t;

typedef struct {
  float latitude;
  float longitude;
  float altitude;
  byte year;
  byte month;
  byte day;
  byte hour;
  byte minute;
  byte second;
} gps_t;

class Flight_Telemetry {
  public:
    Flight_Telemetry();
    bool begin();
    void displaySensorDetails();
    void printTelemetry();
    void updateTelemetry();
    void updateGPS();
    void updateIMU();
    void getGPS(gps_t* gps);
    void getTelemetry(telemetry_t* tel);    
    void printGPS();
 
  private:
    Adafruit_GPS GPS;
    //Adafruit_GPS& GPS = *GPS_init;

    bool gpsSetup();
    telemetry_t _telemetry;
    Adafruit_NXPSensorFusion filter;
    int temp;
    Adafruit_FXAS21002C gyro;
    Adafruit_FXOS8700 accelmag;
    float mag_offsets[3] = { 10.79F, -11.86F, -135.63F };
    // Soft iron error compensation matrix
    float mag_softiron_matrix[3][3] = { {  0.987,  -0.033,  0.013 },
                                    {  -0.033,  0.956,  -0.015 },
                                    {  -0.013,  -0.015,  1.062 } };
    const float mag_field_strength        = 53.88F;
    // Offsets applied to compensate for gyro zero-drift error for x/y/z
    // Raw values converted to rad/s based on 250dps sensitiv iy (1 lsb = 0.00875 rad/s)
    const float rawToDPS = 0.00875F;
    const float dpsToRad = 0.017453293F;
    float gyro_zero_offsets[3]      = { 0.0F * rawToDPS * dpsToRad,
                                       0.0F * rawToDPS * dpsToRad,
                                        0.0F * rawToDPS * dpsToRad };
};

#endif
