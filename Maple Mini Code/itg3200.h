// Interface avec le gyromètres ITG3200
// Inspiré de http://pastebin.com/Z6UAdpvM et
// https://github.com/henryeherman/ee180ucla_maple_demo
// Matthias Lemainque 2012

#ifndef _ITG3200_H_
#define _ITG3200_H_

#include "wirish.h"
#include "i2c.h"
#include "maths.h"
#include "log.h"


#define ITG_FILTER_CONST	0.0001	// Constante de temps du filtre passe-bas appliqué (sec)
#define ITG_FILTER_CONST_SLOW	0.01	// Une deuxième mesure, plus amortie, est disponible

#define ITG_ADDR	(0xD0 >> 1) //0xD0 if tied low, 0xD2 if tied high

//ITG-3200 Register Map
//Interrupt Enable/Interrupt Map/Interrupt Source Register Bits
#define	OVERRUN		(1<<0)
#define	WATERMARK	(1<<1)
#define FREE_FALL	(1<<2)
#define	INACTIVITY	(1<<3)
#define	ACTIVITY	(1<<4)
#define DOUBLE_TAP	(1<<5)
#define	SINGLE_TAP	(1<<6)
#define	DATA_READY	(1<<7)

//Data Format Bits
#define RANGE_0		(1<<0)
#define	RANGE_1		(1<<1)
#define JUSTIFY		(1<<2)
#define	FULL_RES	(1<<3)

#define	INT_INVERT	(1<<5)
#define	SPI		(1<<6)
#define	SELF_TEST	(1<<7)

#define WHO_AM_I	0x00
#define SMPLRT_DIV	0x15
#define	DLPF_FS		0x16
#define INT_CFG		0x17
#define INT_STATUS	0x1A
#define	TEMP_OUT_H	0x1B
#define	TEMP_OUT_L	0x1C
#define GYRO_XOUT_H	0x1D
#define	GYRO_XOUT_L	0x1E
#define GYRO_YOUT_H	0x1F
#define GYRO_YOUT_L	0x20
#define GYRO_ZOUT_H	0x21
#define GYRO_ZOUT_L	0x22
#define	PWR_MGM		0x3E

//Sample Rate Divider
//Fsample = Fint / (divider + 1) where Fint is either 1kHz or 8kHz
//Fint is set to 1kHz
//Set divider to 9 for 100 Hz sample rate

//DLPF, Full Scale Register Bits
//FS_SEL must be set to 3 for proper operation
//Set DLPF_CFG to 3 for 1kHz Fint and 42 Hz Low Pass Filter
#define DLPF_CFG_0	(1<<0)
#define DLPF_CFG_1	(1<<1)
#define DLPF_CFG_2	(1<<2)
#define DLPF_FS_SEL_0	(1<<3)
#define DLPF_FS_SEL_1	(1<<4)

//Power Management Register Bits
//Recommended to set CLK_SEL to 1,2 or 3 at startup for more stable clock
#define PWR_MGM_CLK_SEL_0	(1<<0)
#define PWR_MGM_CLK_SEL_1	(1<<1)
#define PWR_MGM_CLK_SEL_2	(1<<2)
#define PWR_MGM_STBY_Z		(1<<3)
#define PWR_MGM_STBY_Y		(1<<4)
#define PWR_MGM_STBY_X		(1<<5)
#define PWR_MGM_SLEEP		(1<<6)
#define PWR_MGM_H_RESET		(1<<7)

//Interrupt Configuration Bist
#define INT_CFG_ACTL		(1<<7)
#define INT_CFG_OPEN		(1<<6)
#define INT_CFG_LATCH_INT_EN	(1<<5)
#define INT_CFG_INT_ANYRD	(1<<4)
#define INT_CFG_ITG_RDY_EN	(1<<2)
#define INT_CFG_RAW_RDY_EN	(1<<0)

class ITG3200 {
private:
  void write(uint8 reg, uint8 data);
  uint8 read1byte(uint8 reg);
public:
  Vector zero;
  Vector measure;
  Vector measureSlow;
  void begin();
  int read(Axis axis);
  Vector read(Vector mesInt);
  Vector read();
};

void ITG3200::write(uint8 reg, uint8 data) {
  /* all i2c transactions send and receive arrays of i2c_msg objects */
  i2c_msg msgs[1]; // we dont do any bursting here, so we only need one i2c_msg object
  uint8 msg_data[2];

  msg_data = { 
    reg, data   };
  msgs[0].addr = ITG_ADDR;
  msgs[0].flags = 0; // write
  msgs[0].length = 2; 
  msgs[0].data = msg_data;
  i2c_master_xfer(I2C, msgs, 1,0);
}

void ITG3200::begin() {
  Log.print("ITG3200 ... ");
  this->write(DLPF_FS, DLPF_FS_SEL_0|DLPF_FS_SEL_1|DLPF_CFG_0); // +/- 2000°/s
  this->write(SMPLRT_DIV, 9);
  this->write(INT_CFG, INT_CFG_RAW_RDY_EN | INT_CFG_ITG_RDY_EN);
  this->write(PWR_MGM, PWR_MGM_CLK_SEL_0);
  Log.println("ok!");
}

uint8 ITG3200::read1byte(uint8 reg) {
  i2c_msg msgs[1]; // we dont do any bursting here, so we only need one i2c_msg object
  uint8 msg_data[1];

  msg_data = { 
    0x00   };
  msg_data[0] = reg; 

  msgs[0].addr = ITG_ADDR;
  msgs[0].flags = 0; 
  msgs[0].length = 1; // just one byte for the address to read, 0x00
  msgs[0].data = msg_data;
  i2c_master_xfer(I2C, msgs, 1,50);

  msgs[0].addr = ITG_ADDR;
  msgs[0].flags = I2C_MSG_READ; 
  msgs[0].length = 1; // just one byte for the address to read, 0x00
  msgs[0].data = msg_data;
  i2c_master_xfer(I2C, msgs, 1,50);

  return msg_data[0];
}

int ITG3200::read(Axis axis) {
  uint8 regH=0;
  uint8 regL=0;
  switch(axis) {
  case axis_X:
    regH = GYRO_XOUT_H;
    regL = GYRO_XOUT_L;
    break;
  case axis_Y:
    regH = GYRO_YOUT_H;
    regL = GYRO_YOUT_L;
    break;
  case axis_Z:
    regH = GYRO_ZOUT_H;
    regL = GYRO_ZOUT_L;
    break;
  }
  uint8 dataH = this->read1byte(regH);
  uint8 dataL = this->read1byte(regL);
  return ((dataH << 8) | dataL);
}

Vector ITG3200::read(Vector mesInt) {
  Vector v = convertVector(mesInt, 1/14.375, this->zero);
  lowPassVect(&(this->measure), v, dt/ITG_FILTER_CONST);
  lowPassVect(&(this->measureSlow), v, dt/ITG_FILTER_CONST_SLOW);
  return this->measure;
}

Vector ITG3200::read() {
  return this->read(newVector(this->read(axis_X), this->read(axis_Y), this->read(axis_Z)));
}



#endif // _ITG3200_H_


