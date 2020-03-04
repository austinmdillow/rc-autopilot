#if defined(ARDUINO) && ARDUINO >= 100
  #include "Arduino.h"
  #else
  #include "WProgram.h"
  #endif
#include "Flight_Navigation.h"
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_FXAS21002C.h>
#include <Adafruit_FXOS8700.h>



Flight_Navigation::Flight_Navigation() {
  
}

Adafruit_FXAS21002C gyro = Adafruit_FXAS21002C(0x0021002C);
Adafruit_FXOS8700 accelmag = Adafruit_FXOS8700(0x8700A, 0x8700B);

void displaySensorDetails() {
  sensor_t gyro_data, accel_data, mag_data;
  gyro.getSensor(&gyro_data);
  accelmag.getSensor(&accel_data, &mag_data);

  Serial.println("------------------------------------");
  Serial.println("ACCELEROMETER");
  Serial.println("------------------------------------");
  Serial.print  ("Sensor:       "); Serial.println(accel_data.name);
  Serial.print  ("Driver Ver:   "); Serial.println(accel_data.version);
  Serial.print  ("Unique ID:    0x"); Serial.println(accel_data.sensor_id, HEX);
  Serial.print  ("Min Delay:    "); Serial.print(accel_data.min_delay); Serial.println(" s");
  Serial.print  ("Max Value:    "); Serial.print(accel_data.max_value, 4); Serial.println(" m/s^2");
  Serial.print  ("Min Value:    "); Serial.print(accel_data.min_value, 4); Serial.println(" m/s^2");
  Serial.print  ("Resolution:   "); Serial.print(accel_data.resolution, 8); Serial.println(" m/s^2");
  Serial.println("------------------------------------");
  Serial.println("");
  Serial.println("------------------------------------");
  Serial.println("GYROSCOPE");
  Serial.println("------------------------------------");
  Serial.print  ("Sensor:       "); Serial.println(gyro_data.name);
  Serial.print  ("Driver Ver:   "); Serial.println(gyro_data.version);
  Serial.print  ("Unique ID:    0x"); Serial.println(gyro_data.sensor_id, HEX);
  Serial.print  ("Max Value:    "); Serial.print(gyro_data.max_value); Serial.println(" rad/s");
  Serial.print  ("Min Value:    "); Serial.print(gyro_data.min_value); Serial.println(" rad/s");
  Serial.print  ("Resolution:   "); Serial.print(gyro_data.resolution); Serial.println(" rad/s");
  Serial.println("------------------------------------");
  Serial.println("");
  Serial.println("------------------------------------");
  Serial.println("MAGNETOMETER");
  Serial.println("------------------------------------");
  Serial.print  ("Sensor:       "); Serial.println(mag_data.name);
  Serial.print  ("Driver Ver:   "); Serial.println(mag_data.version);
  Serial.print  ("Unique ID:    0x"); Serial.println(mag_data.sensor_id, HEX);
  Serial.print  ("Min Delay:    "); Serial.print(accel_data.min_delay); Serial.println(" s");
  Serial.print  ("Max Value:    "); Serial.print(mag_data.max_value); Serial.println(" uT");
  Serial.print  ("Min Value:    "); Serial.print(mag_data.min_value); Serial.println(" uT");
  Serial.print  ("Resolution:   "); Serial.print(mag_data.resolution); Serial.println(" uT");
  Serial.println("------------------------------------");
  Serial.println("");
}