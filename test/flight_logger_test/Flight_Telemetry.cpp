#if defined(ARDUINO) && ARDUINO >= 100
  #include "Arduino.h"
  #else
  #include "WProgram.h"
#endif
#include "Flight_Telemetry.h"

#define DEBUG 1
#define GPSSerial Serial1

Flight_Telemetry::Flight_Telemetry() : GPS(&GPSSerial){
  //Adafruit_GPS GPS(&GPSSerial);
}


bool Flight_Telemetry::begin() {
  
  Serial.println("Flight Telemetry module Setup Starting");
  //Adafruit_GPS GPS = GPS(&GPSSerial);
  Adafruit_FXAS21002C gyro = Adafruit_FXAS21002C(0x0021002C);
  Adafruit_FXOS8700 accelmag = Adafruit_FXOS8700(0x8700A, 0x8700B);
  if(!gyro.begin()) {
    /* There was a problem detecting the FXAS21002C ... check your connections */
    Serial.println("Ooops, no FXAS21002C detected ... Check your wiring!");
    //while(1);
    return false;
  }

  if(!accelmag.begin(ACCEL_RANGE_4G)) {
    /* There was a problem detecting the FXOS8700 ... check your connections */
    Serial.println("Ooops, no FXOS8700 detected ... Check your wiring!");
    return false;
    //while(1);
  }

  if (!this->gpsSetup()) {
    Serial.println("GPS Failed to initialize");
  }

  Serial.println("Flight Telemetry module Setup Complete");
  return true;
}

bool Flight_Telemetry::gpsSetup() {
  // 9600 NMEA is the default baud rate for Adafruit MTK GPS's- some use 4800
  GPS.begin(9600);
  // uncomment this line to turn on RMC (recommended minimum) and GGA (fix data) including altitude
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ); // 1 Hz update rate

  // Request updates on antenna status, comment out to keep quiet
  GPS.sendCommand(PGCMD_ANTENNA);
  //GPS.sendCommand(PMTK_SET_BAUD_57600);
  //GPS.begin(9600);
  return true;
}

/**
 * get telemetry transfers all telemetry information over to the passed struct
 */
void Flight_Telemetry::getTelemetry(telemetry_t* tel) {
  this->updateIMU();
  tel->magnetic.x = _telemetry.magnetic.x;
  tel->magnetic.y = _telemetry.magnetic.y;
  tel->magnetic.z = _telemetry.magnetic.z;

  tel->gyro.x = _telemetry.gyro.x;
  tel->gyro.y = _telemetry.gyro.y;
  tel->gyro.z = _telemetry.gyro.z;

  tel->acceleration.x = _telemetry.acceleration.x;
  tel->acceleration.y = _telemetry.acceleration.y;
  tel->acceleration.z = _telemetry.acceleration.z;

  tel->rpy[0] = _telemetry.rpy[0];
  tel->rpy[1] = _telemetry.rpy[1];
  tel->rpy[2] = _telemetry.rpy[2];
}

void Flight_Telemetry::getGPS(gps_t* gps) {
  this->updateGPS();
  gps->longitude = GPS.longitudeDegrees;
  gps->latitude = GPS.latitudeDegrees;
  gps->altitude = GPS.altitude;
  gps->month = GPS.month;
  gps->day = GPS.day;
  gps->year = GPS.year;
  gps->hour = GPS.hour;
  gps->minute = GPS.minute;
  gps->second = GPS.seconds;
}

void Flight_Telemetry::updateTelemetry() {
  static unsigned long last_gps_update = millis();
  updateIMU();

  if (millis() - last_gps_update > 100) {
    updateGPS();
    last_gps_update = millis();
  }
}

void Flight_Telemetry::updateIMU() {
  
  sensors_event_t gyro_event;
  sensors_event_t accel_event;
  sensors_event_t mag_event;

  
  gyro.getEvent(&gyro_event);
  accelmag.getEvent(&accel_event, &mag_event);
  
  
  // Apply mag offset compensation (base values in uTesla)
  float x = mag_event.magnetic.x - mag_offsets[0];
  float y = mag_event.magnetic.y - mag_offsets[1];
  float z = mag_event.magnetic.z - mag_offsets[2];
  // Apply mag soft iron error compensation
  float mx = x * mag_softiron_matrix[0][0] + y * mag_softiron_matrix[0][1] + z * mag_softiron_matrix[0][2];
  float my = x * mag_softiron_matrix[1][0] + y * mag_softiron_matrix[1][1] + z * mag_softiron_matrix[1][2];
  float mz = x * mag_softiron_matrix[2][0] + y * mag_softiron_matrix[2][1] + z * mag_softiron_matrix[2][2];

  // Apply gyro zero-rate error compensation
  float gx = gyro_event.gyro.x - gyro_zero_offsets[0];
  float gy = gyro_event.gyro.y - gyro_zero_offsets[1];
  float gz = gyro_event.gyro.z - gyro_zero_offsets[2];


  _telemetry.gyro.x = gx;
  _telemetry.gyro.y = gy;
  _telemetry.gyro.z = gz;

  _telemetry.magnetic.x = mx;
  _telemetry.magnetic.y = my;
  _telemetry.magnetic.z = mz;

  _telemetry.acceleration.x = accel_event.acceleration.x;
  _telemetry.acceleration.y = accel_event.acceleration.y;
  _telemetry.acceleration.z = accel_event.acceleration.z;

  // The filter library expects gyro data in degrees/s, but adafruit sensor
  // uses rad/s so we need to convert them first (or adapt the filter lib
  // where they are being converted)
  gx *= 57.2958F;
  gy *= 57.2958F;
  gz *= 57.2958F;

  
  filter.update(gx, gy, gz,
                accel_event.acceleration.x, accel_event.acceleration.y, accel_event.acceleration.z,
                mx, my, mz);
  _telemetry.rpy[0] = filter.getRoll();
  _telemetry.rpy[1] = filter.getPitch();
  _telemetry.rpy[2] = filter.getYaw();
}

void Flight_Telemetry::updateGPS() {
  for (int i = 0; i < 100; i++) {
    char c = GPS.read();
    // if you want to debug, this is a good time to do it!
    if (DEBUG && false) {
      if (c) Serial.print(c);
    }
    // if a sentence is received, we can check the checksum, parse it...
    if (GPS.newNMEAreceived()) {
      if (!GPS.parse(GPS.lastNMEA())) // this also sets the newNMEAreceived() flag to false
        return; // we can fail to parse a sentence in which case we should just wait for another
    }
  }

}



void Flight_Telemetry::printTelemetry() {
  static unsigned long last_print = millis();
  if (millis() - last_print > 250) {
     /* Display the accel results (acceleration is measured in m/s^2) */
    Serial.print("A ");
    Serial.print("X: "); Serial.print(_telemetry.acceleration.x, 4); Serial.print("  ");
    Serial.print("Y: "); Serial.print(_telemetry.acceleration.y, 4); Serial.print("  ");
    Serial.print("Z: "); Serial.print(_telemetry.acceleration.z, 4); Serial.print("  ");
    Serial.println("m/s^2");
  
    /* Display the results (speed is measured in rad/s) */
    Serial.print("G ");
    Serial.print("X: "); Serial.print(_telemetry.gyro.x); Serial.print("  ");
    Serial.print("Y: "); Serial.print(_telemetry.gyro.y); Serial.print("  ");
    Serial.print("Z: "); Serial.print(_telemetry.gyro.z); Serial.print("  ");
    Serial.println("rad/s ");
  
    /* Display the mag results (mag data is in uTesla) */
    Serial.print("M ");
    Serial.print("X: "); Serial.print(_telemetry.magnetic.x, 1); Serial.print("  ");
    Serial.print("Y: "); Serial.print(_telemetry.magnetic.y, 1); Serial.print("  ");
    Serial.print("Z: "); Serial.print(_telemetry.magnetic.z, 1); Serial.print("  ");
    Serial.println("uT");
  
    Serial.println("");

    // Print the orientation filter output
    float roll = filter.getRoll();
    float pitch = filter.getPitch();
    float heading = filter.getYaw();
    Serial.print(millis());
    Serial.print(" - Orientation: ");
    Serial.print(pitch);
    Serial.print(" ");
    Serial.print(heading);
    Serial.print(" ");
    Serial.println(roll);
    last_print = millis();
  }
}

void Flight_Telemetry::printGPS() {

  int timer = 0;
  if (millis() - timer > 2000) {
    timer = millis(); // reset the timer
    Serial.print("\nTime: ");
    if (GPS.hour < 10) { Serial.print('0'); }
    Serial.print(GPS.hour, DEC); Serial.print(':');
    if (GPS.minute < 10) { Serial.print('0'); }
    Serial.print(GPS.minute, DEC); Serial.print(':');
    if (GPS.seconds < 10) { Serial.print('0'); }
    Serial.print(GPS.seconds, DEC); Serial.print('.');
    if (GPS.milliseconds < 10) {
      Serial.print("00");
    } else if (GPS.milliseconds > 9 && GPS.milliseconds < 100) {
      Serial.print("0");
    }
    Serial.println(GPS.milliseconds);
    Serial.print("Date: ");
    Serial.print(GPS.day, DEC); Serial.print('/');
    Serial.print(GPS.month, DEC); Serial.print("/20");
    Serial.println(GPS.year, DEC);
    Serial.print("Fix: "); Serial.print((int)GPS.fix);
    Serial.print(" quality: "); Serial.println((int)GPS.fixquality);
    Serial.print("Time [s] since last fix: ");
    Serial.println(GPS.secondsSinceFix(), 3);
    Serial.print("    since last GPS time: ");
    Serial.println(GPS.secondsSinceTime(), 3);
    if (GPS.fix) {
      Serial.print("Location: ");
      Serial.print(GPS.latitudeDegrees, 4); Serial.print(GPS.lat);
      Serial.print(", ");
      Serial.print(GPS.longitudeDegrees, 4); Serial.println(GPS.lon);
      Serial.print("Speed (knots): "); Serial.println(GPS.speed);
      Serial.print("Angle: "); Serial.println(GPS.angle);
      Serial.print("Altitude: "); Serial.println(GPS.altitude);
      Serial.print("Satellites: "); Serial.println((int)GPS.satellites);
    }
  }
 
}


void Flight_Telemetry::displaySensorDetails(void) {
  sensor_t accel, mag;
  accelmag.getSensor(&accel, &mag);
  Serial.println("------------------------------------");
  Serial.println("ACCELEROMETER");
  Serial.println("------------------------------------");
  Serial.print  ("Sensor:       "); Serial.println(accel.name);
  Serial.print  ("Driver Ver:   "); Serial.println(accel.version);
  Serial.print  ("Unique ID:    0x"); Serial.println(accel.sensor_id, HEX);
  Serial.print  ("Min Delay:    "); Serial.print(accel.min_delay); Serial.println(" s");
  Serial.print  ("Max Value:    "); Serial.print(accel.max_value, 4); Serial.println(" m/s^2");
  Serial.print  ("Min Value:    "); Serial.print(accel.min_value, 4); Serial.println(" m/s^2");
  Serial.print  ("Resolution:   "); Serial.print(accel.resolution, 8); Serial.println(" m/s^2");
  Serial.println("------------------------------------");
  Serial.println("");
  Serial.println("------------------------------------");
  Serial.println("MAGNETOMETER");
  Serial.println("------------------------------------");
  Serial.print  ("Sensor:       "); Serial.println(mag.name);
  Serial.print  ("Driver Ver:   "); Serial.println(mag.version);
  Serial.print  ("Unique ID:    0x"); Serial.println(mag.sensor_id, HEX);
  Serial.print  ("Min Delay:    "); Serial.print(accel.min_delay); Serial.println(" s");
  Serial.print  ("Max Value:    "); Serial.print(mag.max_value); Serial.println(" uT");
  Serial.print  ("Min Value:    "); Serial.print(mag.min_value); Serial.println(" uT");
  Serial.print  ("Resolution:   "); Serial.print(mag.resolution); Serial.println(" uT");
  Serial.println("------------------------------------");
  Serial.println("");
  delay(500);
}
