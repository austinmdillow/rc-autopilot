#if defined(ARDUINO) && ARDUINO >= 100
  #include "Arduino.h"
  #else
  #include "WProgram.h"
#endif
#include "Flight_Telemetry.h"

#define DEBUG 1
#define GPSSerial Serial1
#define SEALEVELPRESSURE_HPA (1013.25)

Flight_Telemetry::Flight_Telemetry() : GPS(&GPSSerial){
  //Adafruit_GPS GPS(&GPSSerial);
}


bool Flight_Telemetry::begin() {
  
  Serial.println("Flight Telemetry module Setup Starting");
  //Adafruit_GPS GPS = GPS(&GPSSerial);
  gyro = Adafruit_FXAS21002C(0x0021002C);
  accelmag = Adafruit_FXOS8700(0x8700A, 0x8700B);
  filter.begin();
  if(!gyro.begin()) {
    Serial.println("Ooops, no FXAS21002C detected ... Check your wiring!");
    //while(1);
    return false;
  }

  if(!accelmag.begin(ACCEL_RANGE_4G)) {
    Serial.println("Ooops, no FXOS8700 detected ... Check your wiring!");
    return false;
    //while(1);
  }

  if (!this->gpsSetup()) {
    Serial.println("GPS Failed to initialize");
  }

  if (!bme280Setup()) {
    Serial.println("BME280 Failed to initialize");
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


bool Flight_Telemetry::bme280Setup() {   
  // indoor navigation
  bme.begin();
  Serial.println("-- Indoor Navigation Scenario --");
  Serial.println("normal mode, 16x pressure / 2x temperature / 1x humidity oversampling,");
  Serial.println("0.5ms standby period, filter 16x");
  bme.setSampling(Adafruit_BME280::MODE_NORMAL,
                  Adafruit_BME280::SAMPLING_X2,  // temperature
                  Adafruit_BME280::SAMPLING_X16, // pressure
                  Adafruit_BME280::SAMPLING_X1,  // humidity
                  Adafruit_BME280::FILTER_X16,
                  Adafruit_BME280::STANDBY_MS_0_5 );
  
  // suggested rate is 25Hz
  // 1 + (2 * T_ovs) + (2 * P_ovs + 0.5) + (2 * H_ovs + 0.5)
  // T_ovs = 2
  // P_ovs = 16
  // H_ovs = 1
  // = 40ms (25Hz)
  // with standby time that should really be 24.16913... Hz
  // delayTime = 41;
  return true;
}





void Flight_Telemetry::updateSensors() {
  static unsigned long last_gps_update = millis();
  updateIMU();
  updateBME280();
  updateGPS();
}

void Flight_Telemetry::getSensors(flight_sensors_t* sensors) {
  this->updateSensors();
  this->getIMU(&(sensors->imu));
  this->getGPS(&(sensors->gps));
  this->getBME280(&(sensors->bme280));
}

void Flight_Telemetry::updateIMU() {

  unsigned long current_millis = millis();
  static unsigned long last_imu_time = millis();
  if (current_millis - last_imu_time > _imu_period_set) {
  
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


    _sensor_data.imu.gyro.x = gx;
    _sensor_data.imu.gyro.y = gy;
    _sensor_data.imu.gyro.z = gz;

    _sensor_data.imu.magnetic.x = mx;
    _sensor_data.imu.magnetic.y = my;
    _sensor_data.imu.magnetic.z = mz;

    _sensor_data.imu.acceleration.x = accel_event.acceleration.x;
    _sensor_data.imu.acceleration.y = accel_event.acceleration.y;
    _sensor_data.imu.acceleration.z = accel_event.acceleration.z;

    // The filter library expects gyro data in degrees/s, but adafruit sensor
    // uses rad/s so we need to convert them first (or adapt the filter lib
    // where they are being converted)
    gx *= 57.2958F;
    gy *= 57.2958F;
    gz *= 57.2958F;

    
    filter.update(gx, gy, gz,
                  accel_event.acceleration.x, accel_event.acceleration.y, accel_event.acceleration.z,
                  mx, my, mz);
    _sensor_data.imu.rpy[0] = filter.getRoll();
    _sensor_data.imu.rpy[1] = filter.getPitch();
    _sensor_data.imu.rpy[2] = filter.getYaw();

    _imu_rate_monitor = _imu_rate_monitor * (1-rate_alpha) + 1000.0/(current_millis - last_imu_time) * rate_alpha;
    last_imu_time = current_millis;
    imu_count++;
  }
}

/**
 * get telemetry transfers all telemetry information over to the passed struct
 */
void Flight_Telemetry::getIMU(imu_t* imu_in) {
  //this->updateIMU();
  imu_in->magnetic.x = _sensor_data.imu.magnetic.x;
  imu_in->magnetic.y = _sensor_data.imu.magnetic.y;
  imu_in->magnetic.z = _sensor_data.imu.magnetic.z;

  imu_in->gyro.x = _sensor_data.imu.gyro.x;
  imu_in->gyro.y = _sensor_data.imu.gyro.y;
  imu_in->gyro.z = _sensor_data.imu.gyro.z;

  imu_in->acceleration.x = _sensor_data.imu.acceleration.x;
  imu_in->acceleration.y = _sensor_data.imu.acceleration.y;
  imu_in->acceleration.z = _sensor_data.imu.acceleration.z;

  imu_in->rpy[0] = _sensor_data.imu.rpy[0];
  imu_in->rpy[1] = _sensor_data.imu.rpy[1];
  imu_in->rpy[2] = _sensor_data.imu.rpy[2];
}


void Flight_Telemetry::updateGPS() {
  unsigned long current_millis = millis();
  static unsigned long last_gps_log_time = millis();
  if (current_millis - last_gps_log_time > _gps_period_set) {
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
    this->_gps_rate_monitor = (_gps_rate_monitor * (1-rate_alpha)) + (1000.0/(current_millis - last_gps_log_time) * rate_alpha);
    gps_count++;
    last_gps_log_time = current_millis;
  }

}

void Flight_Telemetry::getGPS(gps_t* gps_in) {
  //this->updateGPS();
  gps_in->longitude = GPS.longitudeDegrees;
  gps_in->latitude = GPS.latitudeDegrees;
  gps_in->altitude = GPS.altitude;
  gps_in->month = GPS.month;
  gps_in->day = GPS.day;
  gps_in->year = GPS.year;
  gps_in->hour = GPS.hour;
  gps_in->minute = GPS.minute;
  gps_in->second = GPS.seconds;
}

void Flight_Telemetry::updateBME280() {
  unsigned long current_millis = millis();
  static unsigned long last_bme_log_time = millis();
  if (current_millis - last_bme_log_time > _bme280_period_set) {
  _sensor_data.bme280.pressure = bme.readPressure();
  _sensor_data.bme280.temperature = bme.readTemperature();
  _sensor_data.bme280.humidity = bme.readHumidity();
  _sensor_data.bme280.altitude = bme.readAltitude(SEALEVELPRESSURE_HPA);

  _bme280_rate_monitor = _bme280_rate_monitor * (1-rate_alpha) + 1000.0/(current_millis - last_bme_log_time) * rate_alpha;
  last_bme_log_time = current_millis;
  bme280_count++;
  }
}

void Flight_Telemetry::getBME280(bme280_t* bme_in) {
  bme_in->pressure = _sensor_data.bme280.pressure;
  bme_in->altitude = _sensor_data.bme280.altitude;
  bme_in->temperature = _sensor_data.bme280.temperature;
  bme_in->humidity = _sensor_data.bme280.humidity;
}



void Flight_Telemetry::printTelemetry() {
  static unsigned long last_print = millis();
  if (millis() - last_print > 250) {
     /* Display the accel results (acceleration is measured in m/s^2) */
    Serial.print("A ");
    Serial.print("X: "); Serial.print(_sensor_data.imu.acceleration.x, 4); Serial.print("  ");
    Serial.print("Y: "); Serial.print(_sensor_data.imu.acceleration.y, 4); Serial.print("  ");
    Serial.print("Z: "); Serial.print(_sensor_data.imu.acceleration.z, 4); Serial.print("  ");
    Serial.println("m/s^2");
  
    /* Display the results (speed is measured in rad/s) */
    Serial.print("G ");
    Serial.print("X: "); Serial.print(_sensor_data.imu.gyro.x); Serial.print("  ");
    Serial.print("Y: "); Serial.print(_sensor_data.imu.gyro.y); Serial.print("  ");
    Serial.print("Z: "); Serial.print(_sensor_data.imu.gyro.z); Serial.print("  ");
    Serial.println("rad/s ");
  
    /* Display the mag results (mag data is in uTesla) */
    Serial.print("M ");
    Serial.print("X: "); Serial.print(_sensor_data.imu.magnetic.x, 1); Serial.print("  ");
    Serial.print("Y: "); Serial.print(_sensor_data.imu.magnetic.y, 1); Serial.print("  ");
    Serial.print("Z: "); Serial.print(_sensor_data.imu.magnetic.z, 1); Serial.print("  ");
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
    Serial.println(bme.readPressure());
  }
}

void Flight_Telemetry::printBME280() {
  Serial.print("Temperature = ");
  Serial.print(_sensor_data.bme280.temperature);
  Serial.println(" *C");

  Serial.print("Pressure = ");

  Serial.print(_sensor_data.bme280.pressure);
  Serial.println(" hPa");

  Serial.print("Approx. Altitude = ");
  Serial.print(_sensor_data.bme280.altitude);
  Serial.println(" m");

  Serial.print("Humidity = ");
  Serial.print(_sensor_data.bme280.humidity);
  Serial.println(" %");

  Serial.println();
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

void Flight_Telemetry::printRateMonitors() {
  Serial.print("IMU, GPS, BME rate: "); Serial.print(this->_imu_rate_monitor); Serial.print(" "); Serial.print(this->_gps_rate_monitor); Serial.print(" "); Serial.println(this->_bme280_rate_monitor);
}

void Flight_Telemetry::printAHRS() {
  float roll = filter.getRoll();
    float pitch = filter.getPitch();
    float heading = filter.getYaw();
    Serial.print(roll);
    Serial.print(" ");
    Serial.print(pitch);
    Serial.print(" ");
    Serial.println(heading);
}



