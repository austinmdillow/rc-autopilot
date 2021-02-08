#ifndef Flight_Telemetry_h
#define Flight_Telemetry_h

//#include "Arduino.h"
#include <Adafruit_Sensor.h>
#include <Adafruit_FXAS21002C.h>
#include <Adafruit_FXOS8700.h>
#include <Adafruit_AHRS.h> // for heading filters
#include <Adafruit_GPS.h>
#include <Adafruit_BME280.h>
#include <Wire.h>
#include <SPI.h>

#define GPSSerial Serial1

typedef struct {
  sensors_vec_t gyro;
  sensors_vec_t acceleration;
  sensors_vec_t magnetic;
  float rpy[3];
} imu_t;

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

typedef struct {
  float altitude;
  float pressure;
  float temperature;
  float humidity;
} bme280_t;

typedef struct {
  imu_t imu;
  gps_t gps;
  bme280_t bme280;
} flight_sensors_t;

class Flight_Telemetry {
  public:
    Flight_Telemetry();
    bool begin();
    void displaySensorDetails();
    void printTelemetry();
    void printGPS();
    void printBME280();
    void updateSensors();
    void getSensors(flight_sensors_t* s);
    void updateIMU();
    void getIMU(imu_t* imu_in);
    void updateGPS();
    void getGPS(gps_t* gps_in); 
    void updateBME280();
    void getBME280(bme280_t* bme_in);
    void printRateMonitors();
    void printAHRS();
    unsigned long imu_count;
    unsigned long gps_count;
    unsigned long bme280_count;
    
 
  private:
    bool gpsSetup();
    bool bme280Setup();
    flight_sensors_t _sensor_data;
    Adafruit_NXPSensorFusion filter;
    Adafruit_FXAS21002C gyro;
    Adafruit_FXOS8700 accelmag;
    Adafruit_GPS GPS;
    Adafruit_BME280 bme; // I2C

    int _imu_period_set = 10;
    int _gps_period_set = 500;
    int _bme280_period_set = 100;
    float rate_alpha = .2;
    float _imu_rate_monitor = 0;
    float _gps_rate_monitor = 0;
    float _bme280_rate_monitor = 0;
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
