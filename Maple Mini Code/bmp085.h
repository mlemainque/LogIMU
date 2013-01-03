// Interface avec le baromètre BMP085
// Inspiré de https://github.com/misenso/BMP085-Arduino-Library
// Matthias Lemainque 2012

#ifndef _BMP085_H_
#define _BMP085_H_

#include "wirish.h"
#include "i2c.h"
#include "log.h"


#define BMP085_ADDR		0x77

#define BMP085_ULTRALOWPOWER	0
#define BMP085_STANDARD		1
#define BMP085_HIGHRES		2
#define BMP085_ULTRAHIGHRES	3
#define BMP085_CAL_AC1		0xAA	// R   Calibration data (16 bits)
#define BMP085_CAL_AC2		0xAC	// R   Calibration data (16 bits)
#define BMP085_CAL_AC3		0xAE	// R   Calibration data (16 bits)    
#define BMP085_CAL_AC4		0xB0	// R   Calibration data (16 bits)
#define BMP085_CAL_AC5		0xB2	// R   Calibration data (16 bits)
#define BMP085_CAL_AC6		0xB4	// R   Calibration data (16 bits)
#define BMP085_CAL_B1		0xB6	// R   Calibration data (16 bits)
#define BMP085_CAL_B2		0xB8	// R   Calibration data (16 bits)
#define BMP085_CAL_MB		0xBA	// R   Calibration data (16 bits)
#define BMP085_CAL_MC		0xBC	// R   Calibration data (16 bits)
#define BMP085_CAL_MD		0xBE	// R   Calibration data (16 bits)

#define BMP085_CONTROL		0xF4 
#define BMP085_TEMPDATA		0xF6
#define BMP085_PRESSUREDATA	0xF6
#define BMP085_READTEMPCMD	0x2E
#define BMP085_READPRESSURECMD	0x34

#define BMP085_ASK_TEMP		0
#define BMP085_READ_TEMP	1
#define BMP085_ASK_PRESS	2
#define BMP085_READ_PRESS	3

class BMP085 {
private:
  uint8 readUInt8(uint8 addr);
  uint16 readUInt16(uint8 addr);
  void write(uint8 reg, uint8 data);

  uint8 oversampling;

  int16 ac1, ac2, ac3, b1, b2, mb, mc, md;
  uint16 ac4, ac5, ac6;
  int32 X1, X2, X3, B3, B5, B6, p;
  uint32 B4, B7;
  uint32 UT, UP;

  uint32 lastTime;
  uint8 state;
public:
  float refAlt, refPress;

  void begin(uint8 mode = BMP085_ULTRAHIGHRES);  // by default go highres
  boolean read();
  void readForce();

  float temperature;
  float pressure;
  float altitude();
};

uint8 BMP085::readUInt8(uint8 reg) {
  i2c_msg msgs[1]; // we dont do any bursting here, so we only need one i2c_msg object
  uint8 msg_data[1];

  msg_data = { 
    0x00           };
  msg_data[0] = reg; 

  msgs[0].addr = BMP085_ADDR;
  msgs[0].flags = 0; 
  msgs[0].length = 1; // just one byte for the address to read, 0x00
  msgs[0].data = msg_data;
  i2c_master_xfer(I2C, msgs, 1,50);

  msgs[0].addr = BMP085_ADDR;
  msgs[0].flags = I2C_MSG_READ; 
  msgs[0].length = 1;
  msgs[0].data = msg_data;
  i2c_master_xfer(I2C, msgs, 1,50);

  return msg_data[0];
}

uint16 BMP085::readUInt16(uint8 reg) {
  i2c_msg msgs[2]; 
  uint8 msg_data[2];

  msg_data = { 
    0x00           };
  msg_data[0] = reg;

  msgs[0].addr = BMP085_ADDR;
  msgs[0].flags = 0; 
  msgs[0].length = 1; // just one byte for the address to read, 0x00
  msgs[0].data = msg_data;
  i2c_master_xfer(I2C, msgs, 1,0);

  msgs[0].addr = BMP085_ADDR;
  msgs[0].flags = I2C_MSG_READ; 
  msgs[0].length = 2;
  msgs[0].data = msg_data;
  i2c_master_xfer(I2C, msgs, 1,0);

  return ( msg_data[0]<<8 | msg_data[1] );
}

void BMP085::write(uint8 reg, uint8 data) {
  /* all i2c transactions send and receive arrays of i2c_msg objects */
  i2c_msg msgs[1]; // we dont do any bursting here, so we only need one i2c_msg object
  uint8 msg_data[2];

  msg_data = { 
    reg, data           };
  msgs[0].addr = BMP085_ADDR;
  msgs[0].flags = 0; // write
  msgs[0].length = 2; 
  msgs[0].data = msg_data;
  i2c_master_xfer(I2C, msgs, 1,0);
}


void BMP085::begin(uint8 mode) {
  Log.print("BMP085 ... ");
  if (mode > BMP085_ULTRAHIGHRES) mode = BMP085_ULTRAHIGHRES;
  oversampling = mode;

  ac1 = this->readUInt16(BMP085_CAL_AC1); // La conversion uint16->int16 rétablie les bonnes valeurs
  ac2 = this->readUInt16(BMP085_CAL_AC2);
  ac3 = this->readUInt16(BMP085_CAL_AC3);
  ac4 = this->readUInt16(BMP085_CAL_AC4);
  ac5 = this->readUInt16(BMP085_CAL_AC5);
  ac6 = this->readUInt16(BMP085_CAL_AC6);
  b1  = this->readUInt16(BMP085_CAL_B1);
  b2  = this->readUInt16(BMP085_CAL_B2);
  mb  = this->readUInt16(BMP085_CAL_MB);
  mc  = this->readUInt16(BMP085_CAL_MC);
  md  = this->readUInt16(BMP085_CAL_MD);

  /*
  ac1 = 408;
   ac2 = -72;
   ac3 = -14383;
   ac4 = 32741;
   ac5 = 32757;
   ac6 = 23153;
   b1  = 6190;
   b2  = 4;
   mb  = -32768;
   mc  = -8711;
   md  = 2868;
   oversampling = BMP085_ULTRALOWPOWER;
  */

  this->refAlt = 0;
  this->refPress = 10132.5;

  Log.println("ok!");
}

void BMP085::readForce() {
  while (!this->read()) { }
}

boolean BMP085::read() {
  uint32 lagTime = 0;
  if (this->state == BMP085_READ_TEMP) lagTime = 5;
  if (this->state == BMP085_READ_PRESS) {
    if (oversampling == BMP085_ULTRALOWPOWER) lagTime = 5;
    else if (oversampling == BMP085_STANDARD) lagTime = 8;
    else if (oversampling == BMP085_HIGHRES) lagTime = 14;
    else lagTime = 26;
  }
  if (millis()-this->lastTime<lagTime) return false;

  if (this->state == BMP085_ASK_TEMP) this->write(BMP085_CONTROL, BMP085_READTEMPCMD);
  if (this->state == BMP085_READ_TEMP) {
    UT = this->readUInt16(BMP085_TEMPDATA);
    X1 = ((UT - (int32)ac6) * (int32)ac5) >> 15;
    X2 = ((int32)mc << 11) / (X1 + (int32)md);
    B5 = X1 + X2;
    this->temperature = (B5 + 8) >> 4;
    this->temperature /= 10;
  }
  if (this->state == BMP085_ASK_PRESS) this->write(BMP085_CONTROL, BMP085_READPRESSURECMD + (oversampling << 6));
  if (this->state == BMP085_READ_PRESS) {
    UP = this->readUInt16(BMP085_PRESSUREDATA);
    UP <<= 8;
    UP |= this->readUInt8(BMP085_PRESSUREDATA+2); // Précision supplémentaire éventuelle
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

  this->lastTime = millis();
  this->state ++;
  if (this->state == 4) {
    this->state = 0;
    return true;
  }
  else return false;
}

float BMP085::altitude() {
  return 44330 * (1 - (1-this->refAlt/44330) * pow(this->pressure/this->refPress, 1/5.255) );
}

#endif // _BMP085_H_





