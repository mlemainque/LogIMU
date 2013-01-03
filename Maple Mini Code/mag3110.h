// Interface avec le magnétomètre MAG3110
// Inspiré de https://github.com/dpellegrino93/MAG3110
// Matthias Lemainque 2012

#ifndef _MAG3110_H_
#define _MAG3110_H_

#include "wirish.h"
#include "i2c.h"
#include "maths.h"
#include "log.h"


#define MAG_FILTER_CONST	0.05	// Constante de temps du filtre passe-bas appliqué (sec)
#define MAG_FILTER_CONST_SLOW	0.5	// Une deuxième mesure, plus amortie, est disponible

#define MAG_ADDR		0x0E

#define MAG_DR_STATUS		0x00

#define MAG_OUT_X_MSB		0x01
#define MAG_OUT_X_LSB		0x02
#define MAG_OUT_Y_MSB		0x03
#define MAG_OUT_Y_LSB		0x04
#define MAG_OUT_Z_MSB		0x05
#define MAG_OUT_Z_LSB		0x06

#define MAG_WHO_AM_I		0x07
#define MAG_SYSMOD		0x08

#define MAG_OFF_X_MSB		0x09
#define MAG_OFF_X_LSB		0x0A
#define MAG_OFF_Y_MSB		0x0B
#define MAG_OFF_Y_LSB		0x0C
#define MAG_OFF_Z_MSB		0x0D
#define MAG_OFF_Z_LSB		0x0E

#define MAG_DIE_TEMP		0x0F

#define MAG_CTRL_REG1		0x10
#define MAG_CTRL_REG2		0x11

//DR_STATUS
#define MAG_ZYXOW		7
#define MAG_ZOW			6
#define MAG_YOW			5
#define MAG_XOW			4
#define MAG_ZYXDR		3
#define MAG_ZDR			2
#define MAG_YDR			1
#define MAG_XDR			0

//WHO_AM_I
#define MAG_WHO_AM_I_VALUE	0xC4

//SYSMOD
#define MAG_SYSMOD1		1
#define MAG_SYSMOD0		0

//CTRL_REG1
#define MAG_DR2			7
#define MAG_DR1			6
#define MAG_DR0			5
#define MAG_OS1			4
#define MAG_OS0			3
#define MAG_FR			2
#define MAG_TM			1
#define MAG_AC			0

//CTRL_REG2
#define MAG_AUTO_MRST_EN	7
#define MAG_RAW			5
#define MAG_Mag_RST		4

class MAG3110 {
private:
  void write(uint8 reg, uint8 data);
  uint8 read1byte(uint8 reg);
public:
  Vector zero;
  Vector measure;
  Vector measureSlow;
  Vector dMeasure;
  void begin();
  uint16 read(Axis axis);
  Vector read(Vector mesInt);
  Vector read();
};

void MAG3110::write(uint8 reg, uint8 data) {
  /* all i2c transactions send and receive arrays of i2c_msg objects */
  i2c_msg msgs[1]; // we dont do any bursting here, so we only need one i2c_msg object
  uint8 msg_data[2];

  msg_data = { 
    reg, data   };
  msgs[0].addr = MAG_ADDR;
  msgs[0].flags = 0; // write
  msgs[0].length = 2; 
  msgs[0].data = msg_data;
  i2c_master_xfer(I2C, msgs, 1,0);
}

void MAG3110::begin() {
  Log.print("MAG3110 ... ");
  this->write(MAG_CTRL_REG2, (1 << MAG_AUTO_MRST_EN)); // enabled auto reset
  delay(15);
  this->write(MAG_CTRL_REG1, (1 << MAG_AC)); // active mode
  Log.println("ok!");
}

uint8 MAG3110::read1byte(uint8 reg) {
  i2c_msg msgs[1]; // we dont do any bursting here, so we only need one i2c_msg object
  uint8 msg_data[1];

  msg_data = { 
    0x00   };
  msg_data[0] = reg; 

  msgs[0].addr = MAG_ADDR;
  msgs[0].flags = 0; 
  msgs[0].length = 1; // just one byte for the address to read, 0x00
  msgs[0].data = msg_data;
  i2c_master_xfer(I2C, msgs, 1,50);

  msgs[0].addr = MAG_ADDR;
  msgs[0].flags = I2C_MSG_READ; 
  msgs[0].length = 1; // just one byte for the address to read, 0x00
  msgs[0].data = msg_data;
  i2c_master_xfer(I2C, msgs, 1,50);

  return msg_data[0];
}

uint16 MAG3110::read(Axis axis) {
  uint8 regH=0;
  uint8 regL=0;
  switch(axis) {
  case axis_X:
    regH = MAG_OUT_X_MSB;
    regL = MAG_OUT_X_LSB;
    break;
  case axis_Y:
    regH = MAG_OUT_Y_MSB;
    regL = MAG_OUT_Y_LSB;
    break;
  case axis_Z:
    regH = MAG_OUT_Z_MSB;
    regL = MAG_OUT_Z_LSB;
    break;
  }
  uint8 dataH = this->read1byte(regH);
  uint8 dataL = this->read1byte(regL);
  return ((dataH << 8) | dataL);
}

Vector MAG3110::read(Vector mesInt) {
  Vector v = convertVector(mesInt, 0.1, this->zero);
  Vector lastMeasure = this->measure;
  lowPassVect(&(this->measure), v, dt/MAG_FILTER_CONST);
  this->dMeasure = Comb2( this->measure, 1/dt, lastMeasure, -1/dt );
  lowPassVect(&(this->measureSlow), v, dt/MAG_FILTER_CONST_SLOW);
  return this->measure;
}

Vector MAG3110::read() {
  // Attention au signe : la base doit être la même que pour l'ADXL345+ITG3200
  return this->read(newVector(+this->read(axis_Y), -this->read(axis_X), +this->read(axis_Z)));
}

#endif // _MAG3110_H_

