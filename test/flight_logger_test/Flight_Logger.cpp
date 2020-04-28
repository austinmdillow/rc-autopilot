#if defined(ARDUINO) && ARDUINO >= 100
  #include "Arduino.h"
  #else
  #include "WProgram.h"
#endif
#include "Flight_Logger.h"

const float rate_alpha = 0.25;

Flight_Logger::Flight_Logger(int chip_select) {
  _chipSelect = chip_select;
}


bool Flight_Logger::begin() {
  Serial.print("Initializing SD card...");
  if (!SD.begin(_chipSelect)) {
    Serial.println("Card failed, or not present");
    // don't do anything more:
    SD_VALID = false;
  } else {
    Serial.println("card initialized.");
    getFilename();
    _dataFile = SD.open(_filename, FILE_WRITE);
    SD_VALID = true;
  }
  setIMULoggingRate(_imu_rate_set);
  setGPSLoggingRate(_gps_rate_set);
}

bool Flight_Logger::end() {
  _dataFile.close();
  return true;
}

bool Flight_Logger::getFilename() {
  char tmp_filename[13];
  PRINT("Size of filename: ");
  PRINTLN(sizeof(tmp_filename));
  PRINTLN(sizeof("log00000.txt"));
  
  for (int log_number = 0; log_number < 100; log_number++) {
    sprintf(tmp_filename, "log%05d.txt", log_number);
    if (!SD.exists(tmp_filename)) {
      strcpy(_filename, tmp_filename);
      Serial.print("Logging to ");
      Serial.println(_filename);
      return true;
    }
  }
  return false;
}


void Flight_Logger::logData(char* saved_data) {
  const unsigned long flush_interval = 250;
  if (SD_VALID && _dataFile) {
    _dataFile.println(saved_data);
  } else {
     Serial.print("error opening ");
     Serial.println(_filename);
  }

  if (millis() - _last_flush_time > flush_interval) {
    //Serial.print(millis());
    //Serial.println(" Flushed");
    _dataFile.flush();
    _last_flush_time = millis();
  }
}

bool Flight_Logger::logGPS(gps_t* gps_data) {
  unsigned long current_millis = millis();
  static unsigned long last_gps_log_time = millis();
  if (current_millis - last_gps_log_time > _gps_period) {
    char gps_str[125];
    sprintf(gps_str, "%d;GPS;LLA:%.6f,%.6f,%.1f;T:%02d-%02d-%02d_%02d-%02d-%02d",current_millis, gps_data->latitude, gps_data->longitude, gps_data->altitude, gps_data->year, gps_data->month, gps_data->day, gps_data->hour, gps_data->minute, gps_data->second);
    this->logData(gps_str);
    num_logs++;
    _gps_rate_monitor = _gps_rate_monitor * (1-rate_alpha) + 1000.0/(current_millis - last_gps_log_time) * rate_alpha;
    last_gps_log_time = current_millis;
   }
}

bool Flight_Logger::logIMU(telemetry_t* telemetry_data) {
  unsigned long current_millis = millis();
  static unsigned long last_imu_log_time = millis();
  if (current_millis - last_imu_log_time > _imu_period) {
    char telemetry_str[125];
    sprintf(telemetry_str, "%d;%s;A;%03.3f,%.3f,%.3f;G;%03.3f,%.3f,%.3f;M;%03.3f,%.3f,%.3f", current_millis, String("IMU").c_str(), telemetry_data->acceleration.x, telemetry_data->acceleration.y, telemetry_data->acceleration.z, telemetry_data->gyro.x, telemetry_data->gyro.y, telemetry_data->gyro.z, telemetry_data->magnetic.x, telemetry_data->magnetic.y, telemetry_data->magnetic.z);
    this->logData(telemetry_str);
    num_logs++;
    _imu_rate_monitor = _imu_rate_monitor * (1-rate_alpha) + 1000.0/(current_millis - last_imu_log_time) * rate_alpha;
    last_imu_log_time = current_millis;
   }
}

void Flight_Logger::loggerDumpSerial() {
  Serial.print("===Dumping SD Card ");
  Serial.print(_filename);
  Serial.println("===");
  if (_dataFile) {
    _dataFile.close();
    _dataFile = SD.open(_filename);
  }
  if (_dataFile) {
    PRINT("Reading from ");
    PRINTLN(_filename);
    while (_dataFile.available()) {
      Serial.write(_dataFile.read());
    }
    _dataFile.close();
    _dataFile = SD.open(_filename, FILE_WRITE);
  }
}

void Flight_Logger::setGPSLoggingRate(int hz) {
  if (hz <= 0) {
    PRINTLN("Logging rate too slow");
    return;
  } else if (hz > 1000) {
    PRINTLN("Logging rate too fast");
    return;
  } else {
    this->_gps_rate_set = hz;
    this->_gps_period = 1000 / hz;
  }
}

void Flight_Logger::setIMULoggingRate(int hz) {
  if (hz <= 0) {
    PRINTLN("Logging rate too slow");
    return;
  } else if (hz > 1000) {
    PRINTLN("Logging rate too fast");
    return;
  } else {
    this->_imu_rate_set = hz;
    this->_imu_period = 1000 / hz;
  }
}

void Flight_Logger::printRateMonitors() {
  Serial.print("IMU, GPS rate: "); Serial.print(_imu_rate_monitor); Serial.print(" "); Serial.println(_gps_rate_monitor);
}

void Flight_Logger::error() {
  
}
