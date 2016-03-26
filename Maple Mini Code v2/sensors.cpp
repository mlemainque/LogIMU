// Interface avec les capteurs I²C : ADXL345, ITG3200, MAG3100 & BMP085
// Matthias Lemainque 2013

#include "sensors.h"

//  * * * * * * * * * * *
// C O N S T R U C T E U R
//  * * * * * * * * * * *

SENSORS::SENSORS() {
  // La classe s'initialise d'elle-même au premier loop()
  I2C_err = I2C_NOT_SETUP;

  // Référence de pression/altitude
  refAlt = 0;
  refPress = 10132.5;
  
  // Par défaut, on utilise les zéros
  enableZeros = true;
}


//  * * * * * * * * * * * * * *
// I N I T I A L I S A T I O N S
//  * * * * * * * * * * * * * *

void SENSORS::setup() {
  setupI2C();
  setupADXL345();
  setupMAG3110();
  setupITG3200();
  setupBMP085();
  lastLoop = micros();
}

void SENSORS::setupI2C() {
  i2c_master_enable(I2C, 0);
  delay(500);
  this->I2C_err = 0;
}

void SENSORS::setupADXL345() {
  this->write(ADXL_ADDR, POWER_CTL, MEASURE);
  this->write(ADXL_ADDR, DATA_FORMAT, 0); // +/- 2g
  rangeADXL345 = 2*9.81; // m/s² / lb
}

void SENSORS::setupMAG3110() {
  this->write(MAG_ADDR, MAG_CTRL_REG2, (1 << MAG_AUTO_MRST_EN)); // enabled auto reset
  this->write(MAG_ADDR, MAG_CTRL_REG1, (1 << MAG_AC)); // active mode
  rangeMAG3110 = 512*0.1; // uT / lb
}

void SENSORS::setupITG3200() {
  this->write(ITG_ADDR, DLPF_FS, DLPF_FS_SEL_0|DLPF_FS_SEL_1|DLPF_CFG_0); // +/- 2000°/s
  this->write(ITG_ADDR, SMPLRT_DIV, 9);
  this->write(ITG_ADDR, INT_CFG, INT_CFG_RAW_RDY_EN | INT_CFG_ITG_RDY_EN);
  this->write(ITG_ADDR, PWR_MGM, PWR_MGM_CLK_SEL_0);
  rangeITG3200 = 512/14.375/CDR; // rad/s / lb
}

void SENSORS::setupBMP085(uint8 mode) {
  if (mode > BMP085_ULTRAHIGHRES) mode = BMP085_ULTRAHIGHRES;
  oversampling = mode;
  ac1 = this->read(BMP085_ADDR, BMP085_CAL_AC1, 2, true); // La conversion uint16->int16 rétablie les bonnes valeurs
  ac2 = this->read(BMP085_ADDR, BMP085_CAL_AC2, 2, true);
  ac3 = this->read(BMP085_ADDR, BMP085_CAL_AC3, 2, true);
  ac4 = this->read(BMP085_ADDR, BMP085_CAL_AC4, 2);
  ac5 = this->read(BMP085_ADDR, BMP085_CAL_AC5, 2);
  ac6 = this->read(BMP085_ADDR, BMP085_CAL_AC6, 2);
  b1  = this->read(BMP085_ADDR, BMP085_CAL_B1,  2, true);
  b2  = this->read(BMP085_ADDR, BMP085_CAL_B2,  2, true);
  mb  = this->read(BMP085_ADDR, BMP085_CAL_MB,  2, true);
  mc  = this->read(BMP085_ADDR, BMP085_CAL_MC,  2, true);
  md  = this->read(BMP085_ADDR, BMP085_CAL_MD,  2, true);
}


//  * * * * * * * * * * * * * *
// E N V O I / R E C E P T I O N
//  * * * * * * * * * * * * * *

void SENSORS::write(uint16 I2Caddr, uint8 reg, uint8 data) {
  /* all i2c transactions send and receive arrays of i2c_msg objects */
  i2c_msg msgs[1]; // we dont do any bursting here, so we only need one i2c_msg object
  uint8 msg_data[2];

  msg_data = { reg, data };
  msgs[0].addr = I2Caddr;
  msgs[0].flags = 0; // write
  msgs[0].length = 2; 
  msgs[0].data = msg_data;
  if (this->I2C_err == 0) this->I2C_err = i2c_master_xfer(I2C, msgs, 1, I2C_TIMEOUT);
}

int32 SENSORS::read(uint16 I2Caddr, uint8 reg, uint8 n, boolean UintToInt, boolean dec) {
  if (READ_MAX_LENGTH > 2) return 0;
  i2c_msg msgs[READ_MAX_LENGTH]; 
  uint8 msg_data[READ_MAX_LENGTH];

  msg_data = { 0x00 };
  msg_data[0] = reg;
  msgs[0].addr = I2Caddr;
  msgs[0].flags = 0; 
  msgs[0].length = 1; // just one byte for the address to read, 0x00
  msgs[0].data = msg_data;
  if (this->I2C_err == 0) this->I2C_err = i2c_master_xfer(I2C, msgs, 1, I2C_TIMEOUT);

  msgs[0].addr = I2Caddr;
  msgs[0].flags = I2C_MSG_READ; 
  msgs[0].length = READ_MAX_LENGTH;
  msgs[0].data = msg_data;
  if (this->I2C_err == 0) this->I2C_err = i2c_master_xfer(I2C, msgs, 1, I2C_TIMEOUT);

  int32 data = 0;
  if (dec == READ_HB_FIRST) {
    for (uint8 i=0 ; i<n ; i++)
      data = (data<<8) | msg_data[i];
  }
  else {
    for (int8 i=n-1 ; i>=0 ; i--) // ATTENTION : si i est un uint8, il ne passe jamais négatif
      data = (data<<8) | msg_data[i];
  }
  if (UintToInt && (data >= 1<<(8*n-1))) data -= 1<<(8*n);
  return data;
}

int32 SENSORS::read2(uint16 I2Caddr, uint8 regH, uint8 regL, boolean UintToInt) {
  int32 data = ( this->read(I2Caddr, regH) << 8 ) | this->read(I2Caddr, regL);
  if (UintToInt && (data >= 1<<15)) data -= 1<<16;
  return data;
}


//  * * * * * * *
// L E C T U R E S
//  * * * * * * *

void SENSORS::readADXL345() {
  float fact = 2*rangeADXL345 / (1<<10);
  measureADXL345[0] = fact * this->read(ADXL_ADDR, DATAX0, 2, true, READ_LB_FIRST);
  measureADXL345[1] = fact * this->read(ADXL_ADDR, DATAY0, 2, true, READ_LB_FIRST);
  measureADXL345[2] = fact * this->read(ADXL_ADDR, DATAZ0, 2, true, READ_LB_FIRST);
  if (enableZeros) AddA( measureADXL345, 1, zeroADXL345, -1, 3 );
}

void SENSORS::readITG3200() {
  float fact = 2*rangeITG3200 / (1<<10);
  measureITG3200[0] = fact * this->read2(ITG_ADDR, GYRO_XOUT_H, GYRO_XOUT_L, true);
  measureITG3200[1] = fact * this->read2(ITG_ADDR, GYRO_YOUT_H, GYRO_YOUT_L, true);
  measureITG3200[2] = fact * this->read2(ITG_ADDR, GYRO_ZOUT_H, GYRO_ZOUT_L, true);
}

void SENSORS::readMAG3110() {
  // Le MAG3110 n'est pas orienté comme les autres capteurs : on effectue donc l'opération X=Y et Y=-X
  float fact = 2*rangeMAG3110 / (1<<10);
  measureMAG3110[0] =  fact * this->read2(MAG_ADDR, MAG_OUT_Y_MSB, MAG_OUT_Y_LSB, true);
  measureMAG3110[1] = -fact * this->read2(MAG_ADDR, MAG_OUT_X_MSB, MAG_OUT_X_LSB, true);
  measureMAG3110[2] =  fact * this->read2(MAG_ADDR, MAG_OUT_Z_MSB, MAG_OUT_Z_LSB, true);
  if (enableZeros) AddA( measureMAG3110, 1, zeroMAG3110, -1, 3 );
}

boolean SENSORS::readBMP085() {
  uint32 lagTime = 0;
  if (this->BMPstate == BMP085_READ_TEMP) lagTime = 5;
  if (this->BMPstate == BMP085_READ_PRESS) {
    if (oversampling == BMP085_ULTRALOWPOWER) lagTime = 5;
    else if (oversampling == BMP085_STANDARD) lagTime = 8;
    else if (oversampling == BMP085_HIGHRES) lagTime = 14;
    else lagTime = 26;
  }
  if (millis()-BMPlastTime<lagTime) return false;

  if (this->BMPstate == BMP085_ASK_TEMP) this->write(BMP085_ADDR, BMP085_CONTROL, BMP085_READTEMPCMD);
  if (this->BMPstate == BMP085_READ_TEMP) {
    UT = this->read(BMP085_ADDR, BMP085_TEMPDATA, 2);
    X1 = ((UT - (int32)ac6) * (int32)ac5) >> 15;
    X2 = ((int32)mc << 11) / (X1 + (int32)md);
    B5 = X1 + X2;
    this->temperature = (B5 + 8) >> 4;
    this->temperature /= 10;
  }
  if (this->BMPstate == BMP085_ASK_PRESS) this->write(BMP085_ADDR, BMP085_CONTROL, BMP085_READPRESSURECMD + (oversampling << 6));
  if (this->BMPstate == BMP085_READ_PRESS) {
    UP = this->read(BMP085_ADDR, BMP085_PRESSUREDATA, 2);
    UP <<= 8;
    UP |= this->read(BMP085_ADDR, BMP085_PRESSUREDATA+2, 1); // Précision supplémentaire éventuelle
    UP >>= (8 - oversampling);

    B6 = B5 - 4000;

    // Calcule B3
    X1 = ((int32)b2 * ( (B6 * B6)>>12 )) >> 11;
    X2 = ((int32)ac2 * B6) >> 11;
    X3 = X1 + X2;
    B3 = ((((int32)ac1*4 + X3) << oversampling) + 2) >> 2;

    // Calcule B4
    X1 = ((int32)ac3 * B6) >> 13;
    X2 = ((int32)b1 * ((B6 * B6) >> 12)) >> 16;
    X3 = ((X1 + X2) + 2) >> 2;
    B4 = ((uint32)ac4 * (uint32)(X3 + 32768)) >> 15;

    B7 = ((uint32)UP - B3) * (uint32)( 50000UL >> oversampling );
    if (B7 < 0x80000000) p = (B7 * 2) / B4;
    else p = (B7 * 2) / B3;

    X1 = (p >> 8) * (p >> 8);
    X1 = (X1 * 3038) >> 16;
    X2 = (-7357 * p) >> 16;

    p += ((X1 + X2 + (int32)3791)>>4); // Résultat en pascals
    this->pressure = (float)p;
  }

  this->BMPlastTime = millis();
  this->BMPstate ++;
  if (this->BMPstate == 4) {
    this->BMPstate = 0;
    return true;
  }
  else return false;
}

float SENSORS::altitude() {
  return 44330 * (1 - (1-this->refAlt/44330) * pow(this->pressure/this->refPress, 1/5.255) );
}

boolean SENSORS::loop() {
  if (this->I2C_err != 0) this->setup();
  else {
    readADXL345();
    readITG3200();
    readMAG3110();
    readBMP085();
    if (this->I2C_err == 0) {
      uint32 time = micros();
      dt = (float)(time-lastLoop)/1000000;
      if (lastLoop>time) dt += pow(256,4)/1000000; // Overflow de micros()
      lastLoop = time;
      return true;
    }
  }
  return false;
}


