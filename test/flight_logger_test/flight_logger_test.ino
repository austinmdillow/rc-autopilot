#include <Adafruit_GPS.h>
#include "Flight_Telemetry.h"
#include "Flight_Logger.h"
#include "Flight_Radio.h"

// what's the name of the hardware serial port?



const int chipSelect = 11;
bool SD_VALID = false;
const int CE = 13;
const int CSN = 12;

#define VBATPIN A6

Flight_Logger logger(chipSelect);
Flight_Telemetry flt;
Flight_Radio radio(CE, CSN);

telemetry_t telemetry_data; // store IMU data
gps_t gps_data; // store GPS data
radio_t radio_data;


void setup() {
  while (!Serial);  // uncomment to have the sketch wait until Serial is ready

  // connect at 115200 so we can read the GPS fast enough and echo without dropping chars
  // also spit it out
  Serial.begin(115200);
  Serial.println("Adafruit GPS library basic test!");

  logger.begin();
  logger.setGPSLoggingRate(10);
  logger.setIMULoggingRate(50);
  flt.begin();
  flt.displaySensorDetails();
  flt.printTelemetry();

  radio.begin();
  delay(3000);
}

float boardBatteryVoltage() {
  float battery_voltage = 2 * 3.3 / 1024.0 * analogRead(VBATPIN);
  Serial.print("VBAT: "); Serial.println(battery_voltage);
  return battery_voltage;
}


void sdLogForTime() {
  unsigned long start_logging = millis();
  unsigned long last_gps_log_time = millis();
  unsigned long num_logs = 0;
  const int logging_time = 10000;

  while (millis() - start_logging < logging_time) {
    unsigned long current_time_millis = millis();
    unsigned long tel_start = micros();
    flt.getTelemetry(&telemetry_data);
    unsigned long tel_end = micros();
    unsigned long log_start = micros();
    logger.logIMU(&telemetry_data);
    unsigned long log_end = micros();

    flt.getGPS(&gps_data);
    logger.logGPS(&gps_data);
    
    //logger.printRateMonitors();
    //Serial.println();
    //Serial.print("update Telemetry");
    //Serial.println(tel_end - tel_start);
    //Serial.print("time to log");
    //Serial.println(log_end - log_start);
  }
  Serial.print("Logs recorded = ");
  Serial.println(logger.num_logs);
}

void radioTest() {
  for (int i = 0; i < 10; i++) {
    radio.txRadio(&radio_data);
  }
}


void loop() {
  flt.updateGPS();
  flt.printGPS();
  Serial.println("Starting Logging");
  unsigned long test_start = micros();
  sdLogForTime();
  unsigned long test_end = micros();
  Serial.println("Done Logging");
  //logger.loggerDumpSerial();
  Serial.println(test_end - test_start);
  logger.end();
  radioTest();
  while(1) {
   boardBatteryVoltage();
   delay(1000);
  }

}


void sdWriteTest() {
  const int num_data = 10;
  Serial.println("Starting Write Test");
  
  for (int i = 0; i < num_data; i++) {
    flt.getTelemetry(&telemetry_data);
    flt.getGPS(&gps_data);
    char gps_str[100];
    sprintf(gps_str, "GPS;LLA;%f,%f,%f",gps_data.latitude, gps_data.longitude, gps_data.altitude);
    char telemetry_str[100];
    sprintf(telemetry_str, "%s;A;%03.3f,%.3f,%.3f;G;%03.3f,%.3f,%.3f;M;%03.3f,%.3f,%.3f", String("IMU").c_str(), telemetry_data.acceleration.x, telemetry_data.acceleration.y, telemetry_data.acceleration.z, telemetry_data.gyro.x, telemetry_data.gyro.y, telemetry_data.gyro.z, telemetry_data.magnetic.x, telemetry_data.magnetic.y, telemetry_data.magnetic.z);
    logger.logData(gps_str);
    logger.logData(telemetry_str);
    Serial.println(gps_str);
    Serial.println(telemetry_str);
  }
  Serial.println("End Write Test");
}
