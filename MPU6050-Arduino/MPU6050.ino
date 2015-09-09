#include <Wire.h>
#include <Transformation.h>
#include <KalmanFilter.h>
#include <system_configuration.h>
#include <StandardCplusplus.h>
#include <utility.h>
#include <unwind-cxx.h>

// respect gyro restrictions
// #define RESTRICT_PITCH 

/////////////////////////////////// I2C Methods /////////////////////////////////////////

const uint8_t IMUAddress = 0x68;
const uint16_t I2C_TIMEOUT = 1000;

uint8_t i2cWrite(uint8_t reg, uint8_t data, bool sendStop) {
  return i2cWrite(reg, &data, 1, sendStop); // returns 0 on success
}

uint8_t i2cWrite(uint8_t reg, uint8_t *data, uint8_t len, bool sendStop) {
  Wire.beginTransmission(IMUAddress);
  Wire.write(reg);
  Wire.write(data, len);
  // TODO: add switch to elaborate on error codes
  uint8_t rcode = Wire.endTransmission(sendStop); // returns 0 on success
  if (rcode) {
    Serial.print(F("I2C-Write failed: "));
    Serial.println(rcode);
  }
  return rcode;
}

uint8_t i2cRead(uint8_t reg, uint8_t *data, uint8_t nbytes) {
  uint32_t timeOutTimer;
  Wire.beginTransmission(IMUAddress);
  Wire.write(reg);
  // TODO: add switch to elaborate on error codes
  uint8_t rcode = Wire.endTransmission(false); // keep bus on hold
  if (rcode) {
    Serial.print(F("I2C-Read failed: "));
    Serial.println(rcode);
    return rcode;
  }
  Wire.requestFrom(IMUAddress, nbytes, (uint8_t)true); // release bus after successful retry
  for (uint8_t i = 0; i < nbytes; i++) {
    if (Wire.available())
      data[i] = Wire.read();
    else {
      timeOutTimer = micros();
      while (((micros() - timeOutTimer) < I2C_TIMEOUT) && !Wire.available());
      if (Wire.available())
        data[i] = Wire.read();
      else {
        Serial.println(F("I2C-Read Timeout"));
        return 5;
      }
    }
  }
  return 0;
}

//////////////////////////////////////////////////////////////////////////////////

// global instances
KalmanFilter kalmanX;
KalmanFilter kalmanY;

Transformation trans;

// sensor data
double accX, accY, accZ;
double gyroX, gyroY, gyroZ;
int16_t tempRaw;

double gyroXangle, gyroYangle; // raw gyro angle
// double compAngleX, compAngleY; // gyro angle with complementary filter
double kalAngleX, kalAngleY; // gyro angle with Kalman filter

uint32_t timer;
uint8_t i2cData[14]; // I2C data buffer

// TODO: make calibration routine

void setup() {
  Serial.begin(115200);
  Wire.begin();
#if ARDUINO >= 157
  Wire.setClock(400000UL); // I2C frequency <- 400kHz
#else
  TWBR = ((F_CPU / 400000UL) - 16) / 2; // I2C frequency <- 400kHz
#endif

  i2cData[0] = 7; // sample rate <- 1000Hz
  i2cData[1] = 0x00; // acc filter frequency <- 260Hz; gyro filter frequency <- 256Hz; sampling <- 8kHz
  i2cData[2] = 0x00; // gyro range <- ±250deg/s
  i2cData[3] = 0x00; // acc range <- ±2g
  while (i2cWrite(0x19, i2cData, 4, false));
  while (i2cWrite(0x6B, 0x01, true)); // disable sleep mode

  while (i2cRead(0x75, i2cData, 1));
  if (i2cData[0] != 0x68) { // check self register
    Serial.print(F("Error reading sensor"));
    while (1);
  }

  delay(100); // sensor stabilizing

  // set Kalman and gyro starting angles
  while (i2cRead(0x3B, i2cData, 6));
  accX = (i2cData[0] << 8) | i2cData[1];
  accY = (i2cData[2] << 8) | i2cData[3];
  accZ = (i2cData[4] << 8) | i2cData[5];
  
  tempRaw = (i2cData[6] << 8) | i2cData[7];
  gyroX = (i2cData[8] << 8) | i2cData[9];
  gyroY = (i2cData[10] << 8) | i2cData[11];
  gyroZ = (i2cData[12] << 8) | i2cData[13];

#ifdef RESTRICT_PITCH
  double roll  = atan2(accY, accZ) * RAD_TO_DEG;
  double pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
#else
  double roll  = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
  double pitch = atan2(-accX, accZ) * RAD_TO_DEG;
#endif

  // set starting angles
  kalmanX.setStartingAngle(roll);
  kalmanY.setStartingAngle(pitch);
  gyroXangle = roll;
  gyroYangle = pitch;
//  compAngleX = roll;
//  compAngleY = pitch;

  timer = micros();
}

void loop() {
  // update values
  while (i2cRead(0x3B, i2cData, 14));
  accX = ((i2cData[0] << 8) | i2cData[1]);
  accY = ((i2cData[2] << 8) | i2cData[3]);
  accZ = ((i2cData[4] << 8) | i2cData[5]);
  tempRaw = (i2cData[6] << 8) | i2cData[7];

  double dt = (double)(micros() - timer) / 1000000;
  timer = micros();

#ifdef RESTRICT_PITCH
  double roll  = atan2(accY, accZ) * RAD_TO_DEG;
  double pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
#else 
  double roll  = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
  double pitch = atan2(-accX, accZ) * RAD_TO_DEG;
#endif

  // convert from rad/s to deg/s
  double gyroXrate = gyroX / 131.0; 
  double gyroYrate = gyroY / 131.0; 

#ifdef RESTRICT_PITCH
  // fix acc jumps between ±180deg
  if ((roll < -90 && kalAngleX > 90) || (roll > 90 && kalAngleX < -90)) {
    kalmanX.setStartingAngle(roll);
//    compAngleX = roll;
    kalAngleX = roll;
    gyroXangle = roll;
  } else
    kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // apply Kalman filter

  if (abs(kalAngleX) > 90)
    gyroYrate = -gyroYrate; // keep within acc range
  kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt);
#else
  // fix acc jumps between ±180deg
  if ((pitch < -90 && kalAngleY > 90) || (pitch > 90 && kalAngleY < -90)) {
    kalmanY.setStartingAngle(pitch);
 //   compAngleY = pitch;
    kalAngleY = pitch;
    gyroYangle = pitch;
  } else
    kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt); // apply Kalman filter

  if (abs(kalAngleY) > 90)
    gyroXrate = -gyroXrate; // keep within acc range
  kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // apply Kalman filter
#endif

  //gyroXangle += gyroXrate * dt; // calculate raw gyro angle
  //gyroYangle += gyroYrate * dt;
  gyroXangle += kalmanX.getUnbiasedRate() * dt; // calculate gyro angle with the unbiased rate
  gyroYangle += kalmanY.getUnbiasedRate() * dt;

//  compAngleX = 0.93 * (compAngleX + gyroXrate * dt) + 0.07 * roll; // calculate gyro angle with complementary filter
//  compAngleY = 0.93 * (compAngleY + gyroYrate * dt) + 0.07 * pitch;

  // reset gyro drifts
  if (gyroXangle < -180 || gyroXangle > 180) {
    gyroXangle = kalAngleX;
  }
  if (gyroYangle < -180 || gyroYangle > 180) {
    gyroYangle = kalAngleY;
  }
/*   
 double posx = trans.getAbsolutePosition(kalAngleX, kalAngleY, 0, gyroXangle, gyroYangle, 0, dt, 1) * 0.001;
  Serial.print(posx); Serial.print("\t");
  double posy = trans.getAbsolutePosition(kalAngleX, kalAngleY, 0, gyroXangle, gyroYangle, 0, dt, 2) * 0.001;
 Serial.print(posy); Serial.print("\t");
 double posz = trans.getAbsolutePosition(kalAngleX, kalAngleY, 0, gyroXrate, gyroYrate, 0, dt, 3) * 0.001;
 Serial.print(posz); Serial.print("\t");


  double velx = trans.getAbsoluteVelocity(kalAngleX, kalAngleY, 40, gyroXangle, gyroYangle, 40, dt, 1) * 0.01;
  Serial.print(velx); Serial.print("\t");
  double vely = trans.getAbsoluteVelocity(kalAngleX, kalAngleY, 40, gyroXangle, gyroYangle, 40, dt, 2) * 0.01;
  Serial.print(vely); Serial.print("\t");
*/  
  
  /* Print Data */

#if 0 // Set to 1 to activate
  Serial.print(accX); Serial.print("\t");
  Serial.print(accY); Serial.print("\t");
  Serial.print(accZ); Serial.print("\t");

  Serial.print(gyroX); Serial.print("\t");
  Serial.print(gyroY); Serial.print("\t");
  Serial.print(gyroZ); Serial.print("\t");

  Serial.print("\t");
#endif

//  Serial.print(roll); Serial.print("\t");
//  Serial.print(gyroXangle); Serial.print("\t");
//  Serial.print(compAngleX); Serial.print("\t");
  Serial.print(kalAngleX); Serial.print("\t");

// Serial.print("\t");

//  Serial.print(pitch); Serial.print("\t");
//  Serial.print(gyroYangle); Serial.print("\t");
//  Serial.print(compAngleY); Serial.print("\t");
  Serial.print(kalAngleY); Serial.print("\t");

  Serial.print("\r\n");
  delay(30);
}
