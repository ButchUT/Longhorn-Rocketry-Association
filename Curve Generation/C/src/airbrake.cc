/**
 * @file airbrake.cc
 *
 * @brief Entrypoint for the airbraking code 
 * 
 */

#include <memory>
#include <string>

#ifdef ARDUINO
  #include <Arduino.h>
#endif

#include "constant_area_drag_calculator.h"
#include "density_calculator.h"
#include "acceleration_calculator.h"

#include "barometer.h"

#ifdef GROUND_TEST
#include "mock_barometer_mpl.h"
#else
#include "barometer_mpl.h"
#endif // GROUND_TEST

#ifdef ARDUINO

// Global flags to configure code
#define ACCEL_NUMBER  1  // a flag that contains the number code for which accelerometer chip we are using 
#define USING_LSM     0
#define USING_ADX     1
void updateAccelReadings();

#define LIFO_LENGTH       200
#define NUM_LIFOS          13
#define ACCEL_X_LIFO        0
#define ACCEL_Y_LIFO        1
#define ACCEL_Z_LIFO        2
#define GYRO_X_LIFO         3
#define GYRO_Y_LIFO         4
#define GYRO_Z_LIFO         5
#define MAG_X_LIFO          6
#define MAG_Y_LIFO          7
#define MAG_Z_LIFO          8
#define BARO_ALT_LIFO       9
#define BARO_PRESS_LIFO    10
#define BARO_TEMP_LIFO     11
#define STRATO_LIFO        12

float lifos[NUM_LIFOS][LIFO_LENGTH]; // One Lifo for each sensor
uint32_t timeStamps[NUM_LIFOS][LIFO_LENGTH];
std::string dataTags[NUM_LIFOS] = { "ACCEL_X", "ACCEL_Y", "ACCEL_Z", "GYRO_X", "GYRO_Y", "GYRO_Z", "MAG_X", "MAG_Y", "MAG_Z", "ALTITUDE", "PRESSURE", "TEMPERATURE", "STRATO" };
int lifoTops[NUM_LIFOS];

void getMostRecentReadings(int numReadings, float *values[NUM_LIFOS], uint32_t *timeStamps[NUM_LIFOS]);

bool getLifo(int lifoNum, float* elem, uint32_t* timeStamp);
bool putLifo(int lifoNum, float newElem);

#define LAUNCH_THRESHOLD 10000

#include <SPI.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>

// LSM9DS1 Includes and Variables Definitions
#include <SparkFunLSM9DS1.h>
LSM9DS1 imu;
#define LSM9DS1_M  0x1E // Wosesuld be 0x1C if SDO_M is LOW
#define LSM9DS1_AG  0x6B // Would be 0x6A if SDO_AG is LOW
#define DECLINATION 3.44 // http://www.ngdc.noaa.gov/geomag-web/#declination

// ADX Includes and Variable Definitions
int scale = 200; //can read from -200 to 200 G's
#define ADX_X_PIN A0
#define ADX_Y_PIN A1
#define ADX_Z_PIN A2

std::shared_ptr<Barometer> kBaro = std::shared_ptr<Barometer>(
#ifdef GROUND_TEST
  new MockBarometerWrapper()
#else
  new BarometerWrapper()
#endif
);

void updateBaroReadings();

// Stratologer Includes and Variable Definitions
//#include <SoftwareSerial.h>
//#define STRATO_SERIAL_RX 0
//#define STRATO_SERIAL_TX 1
//SoftwareSerial stratoSerial(STRATO_SERIAL_RX, STRATO_SERIAL_TX); // RX, TX
//Stratologger uses the normal serial lines so don't need to do anything

// SD card Includes and Variable Defintions
#include <SD.h>
#define WRITE_BACK_THRESHOLD 240
int dataPointsSaved = 0;

File dataFile;

// Timer Includes and Definitions
#include <avr/io.h>
#include <avr/interrupt.h>

IntervalTimer stratoTimer;
IntervalTimer accelTimer;
IntervalTimer baroTimer;

void updateGyroReadings();
void updateMagReadings();
void updateStratoReadings();

void updateSensorReadingslog()
{
  updateAccelReadings();
  updateGyroReadings();
  updateMagReadings();
  updateBaroReadings();
  updateStratoReadings();
}

// Lifo functions
bool getLifoElements(int lifoNum, int numReadings, float elems[LIFO_LENGTH], uint32_t tStamps[LIFO_LENGTH])
{
  if(numReadings > LIFO_LENGTH) {numReadings = LIFO_LENGTH;}
  for(int i = 0; i < numReadings; i++)
  {
    elems[i] = lifos[lifoNum][i];
    tStamps[i] = timeStamps[lifoNum][i];
  }
  return 1;
    
} 

bool getLifo(int lifoNum, float* elem, uint32_t *timeStamp)
{
  *elem = lifos[lifoNum][0];
  *timeStamp = timeStamps[lifoNum][0];
  return 1;
}

bool putLifo(int lifoNum, float newElem)
{
  cli();
  dataPointsSaved += 1;
  for(int i = LIFO_LENGTH - 1; i > 0; i--)
  {
    lifos[lifoNum][i] = lifos[lifoNum][i - 1];
    timeStamps[lifoNum][i] = timeStamps[lifoNum][i - 1];
  }

  lifos[lifoNum][0] = newElem;
  timeStamps[lifoNum][0] = millis();
  sei();

  return true;
}

// Accel functions
void accelHandler()
{
  updateAccelReadings();
  updateGyroReadings();
  updateMagReadings();
}

void updateLSMreadings();
void updateADXreadings();

void updateAccelReadings()
{
  switch(ACCEL_NUMBER)
  {
   case USING_LSM:
   updateLSMreadings();
   break;

   case USING_ADX:
   updateADXreadings();   
   break;
  }
}

void logDataPoint(int lifoNum, float elem, uint32_t tStamp)
{
      dataFile = SD.open("datalog.txt", FILE_WRITE);
      dataFile.print(dataTags[lifoNum].c_str());
      dataFile.print(',');   
      dataFile.print(elem);
      dataFile.print(',');     
      dataFile.println(tStamp); 
      dataFile.close();
}

void updateLSMreadings()
{
   if (imu.accelAvailable())
   {
    imu.readAccel();
    putLifo(ACCEL_X_LIFO, imu.ax);
    putLifo(ACCEL_Y_LIFO, imu.ay);
    putLifo(ACCEL_Z_LIFO, imu.az);
    uint32_t timeStamp = millis();
    logDataPoint(ACCEL_X_LIFO, imu.ax, timeStamp);
    logDataPoint(ACCEL_Y_LIFO, imu.ay, timeStamp);
    logDataPoint(ACCEL_Z_LIFO, imu.az, timeStamp);
   } 
}

void updateADXreadings()
{
  // Get raw accelerometer data for each axis
  int rawX = analogRead(ADX_X_PIN);
  int rawY = analogRead(ADX_Y_PIN);
  int rawZ = analogRead(ADX_Z_PIN);

  float ax = map(rawX, 0, 1023, -scale, scale); // maps readings from 0 to 5v to -200 to 200 G's 
  float ay = map(rawY, 0, 1023, -scale, scale);
  float az = map(rawZ, 0, 1023, -scale, scale);
  putLifo(ACCEL_X_LIFO, ax);
  putLifo(ACCEL_Y_LIFO, ay);
  putLifo(ACCEL_Z_LIFO, az);
  uint32_t timeStamp = millis();
  logDataPoint(ACCEL_X_LIFO, ax, timeStamp);
  logDataPoint(ACCEL_Y_LIFO, ay, timeStamp);
  logDataPoint(ACCEL_Z_LIFO, az, timeStamp);
}

void updateGyroReadings()
{
  if (imu.gyroAvailable())
  {
    imu.readGyro();
    putLifo(GYRO_X_LIFO, imu.gx);
    putLifo(GYRO_Y_LIFO, imu.gy);
    putLifo(GYRO_Z_LIFO, imu.gz);
    uint32_t timeStamp = millis();
    logDataPoint(GYRO_X_LIFO, imu.gx, timeStamp);
    logDataPoint(GYRO_Y_LIFO, imu.gy, timeStamp);
    logDataPoint(GYRO_Z_LIFO, imu.gz, timeStamp);
  }
}

void updateMagReadings()
{
  if (imu.magAvailable())
  {
    imu.readMag();
    putLifo(MAG_X_LIFO, imu.mx);
    putLifo(MAG_Y_LIFO, imu.my);
    putLifo(MAG_Z_LIFO, imu.mz);
    uint32_t timeStamp = millis();
    logDataPoint(MAG_X_LIFO, imu.mx, timeStamp);
    logDataPoint(MAG_Y_LIFO, imu.my, timeStamp);
    logDataPoint(MAG_Z_LIFO, imu.mz, timeStamp);
  }
}

// Barometer functions
void baroHandler()
{
  updateBaroReadings();
}

void updateBaroReadings()
{
  struct BarometerData data = kBaro->Read();
  putLifo(BARO_ALT_LIFO, data.altitude);
  putLifo(BARO_PRESS_LIFO, data.pressure);
  putLifo(BARO_TEMP_LIFO, data.temperature);

  uint32_t timeStamp = millis();
  logDataPoint(BARO_ALT_LIFO, data.altitude, timeStamp);
  logDataPoint(BARO_PRESS_LIFO, data.pressure, timeStamp);
  logDataPoint(BARO_TEMP_LIFO, data.temperature, timeStamp);
}

void getMostRecentReadings(int numReadings, float *values[NUM_LIFOS], uint32_t *timeStamps[NUM_LIFOS])
{
  for(int i = 0; i < NUM_LIFOS; i++)
  {
    float elems[numReadings];
    uint32_t tStamps[numReadings];
    getLifoElements(i, numReadings, elems, tStamps);
    values[i] = elems;
    timeStamps[i] = tStamps;
  }
}

// Strato functions
void updateStratoReadings()
{
    String message;
    char lastChar = 'a';
    while(lastChar != '\n')
    {
      char lastChar = Serial.read();
      message += (char)lastChar;     
    }
    float value = message.toFloat();
    putLifo(STRATO_LIFO, value);
    logDataPoint(STRATO_LIFO, value, millis());
}

void stratoHandler()
{
  updateStratoReadings(); 
}

// SD card functions
void setupDatalog()
{
    dataFile = SD.open("datalog.txt", FILE_WRITE);
    dataFile.println("");
    dataFile.println("__New Launch__");
    dataFile.close();
}

void logData()
{
  noInterrupts();
  dataPointsSaved -= WRITE_BACK_THRESHOLD;
  dataFile = SD.open("datalog.txt", FILE_WRITE);
  for(int lifoNum = 0; lifoNum < NUM_LIFOS; lifoNum++)
  {
    float elems[LIFO_LENGTH];
    uint32_t tStamps[LIFO_LENGTH];
    int pointsToWriteBack = (WRITE_BACK_THRESHOLD / 180) * 20;
    getLifoElements(lifoNum, pointsToWriteBack, elems, tStamps);
    for(int i = 0; i < pointsToWriteBack; i++)
    {
      dataFile.print(dataTags[lifoNum].c_str());
      dataFile.print(',');   
      dataFile.print(elems[i]);
      dataFile.print(',');     
      dataFile.println(tStamps[i]); 
    }
  }
  dataFile.close();
  interrupts();
}


void setup() 
{
  Serial.begin(9600);
  //stratoSerial.begin(9600);

  SD.begin(BUILTIN_SDCARD);
  setupDatalog();// set up data log file
  

  imu.settings.device.commInterface = IMU_MODE_I2C;
  imu.settings.device.mAddress = LSM9DS1_M;
  imu.settings.device.agAddress = LSM9DS1_AG;
  imu.begin();

  kBaro->Initialize();

  // This is prototype code for waiting to log data until the rocket is ready to take off
  // Please leave it in
  /*updateAccelReadings(); 

  float elem;
  uint32_t timeStamp;
  getLifo(ACCEL_Z_LIFO, &elem, &timeStamp);
  while(elem < LAUNCH_THRESHOLD)
  {
   updateAccelReadings(); 
   getLifo(ACCEL_Z_LIFO, &elem, &timeStamp);
  }*/
  
  stratoTimer.begin(stratoHandler, 50000);
  accelTimer.begin(accelHandler, 12500);
  baroTimer.begin(baroHandler, 12500);
  interrupts();
}

void loop() 
{

}

#else

int main(int argc, char* argv[]) {

  std::shared_ptr<Barometer> kBaro = std::shared_ptr<Barometer>(
#ifdef GROUND_TEST
    new MockBarometerWrapper());
#else
    new BarometerWrapper());
#endif // GROUND_TEST

  kBaro->Initialize();
}

#endif
