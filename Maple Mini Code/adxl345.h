// Interface avec l'accéléromètre ADXL345
// Inspiré de http://pastebin.com/Z6UAdpvM et
// https://github.com/henryeherman/ee180ucla_maple_demo
// Matthias Lemainque 2012

#ifndef _ADXL345_H_
#define _ADXL345_H_

#include "wirish.h"
#include "i2c.h"
#include "maths.h"
#include "log.h"


#define ADXL_FILTER_CONST	0.05	// Constante de temps du filtre passe-bas appliqué (sec)
#define ADXL_FILTER_CONST_SLOW	0.5	// Une deuxième mesure, plus amortie, est disponible

#define ADXL_ADDR	(0xA6 >> 1)

//ADXL Register Map
#define	DEVID		0x00	// Device ID Register
#define THRESH_TAP	0x1D	// Tap Threshold
#define	OFSX		0x1E	// X-axis offset
#define	OFSY		0x1F	// Y-axis offset
#define	OFSZ		0x20	// Z-axis offset
#define	DUR		0x21	// Tap Duration
#define	Latent		0x22	// Tap latency
#define	Window		0x23	// Tap window
#define	THRESH_ACT	0x24	// Activity Threshold
#define	THRESH_INACT	0x25	// Inactivity Threshold
#define	TIME_INACT	0x26	// Inactivity Time
#define	ACT_INACT_CTL	0x27	// Axis enable control for activity and inactivity detection
#define	THRESH_FF	0x28	// free-fall threshold
#define	TIME_FF		0x29	// Free-Fall Time
#define	TAP_AXES	0x2A	// Axis control for tap/double tap
#define ACT_TAP_STATUS	0x2B	// Source of tap/double tap
#define	BW_RATE		0x2C	// Data rate and power mode control
#define POWER_CTL	0x2D	// Power Control Register
#define	INT_ENABLE	0x2E	// Interrupt Enable Control
#define	INT_MAP		0x2F	// Interrupt Mapping Control
#define	INT_SOURCE	0x30	// Source of interrupts
#define	DATA_FORMAT	0x31	// Data format control
#define DATAX0		0x32	// X-Axis Data 0
#define DATAX1		0x33	// X-Axis Data 1
#define DATAY0		0x34	// Y-Axis Data 0
#define DATAY1		0x35	// Y-Axis Data 1
#define DATAZ0		0x36	// Z-Axis Data 0
#define DATAZ1		0x37	// Z-Axis Data 1
#define	FIFO_CTL	0x38	// FIFO control
#define	FIFO_STATUS	0x39	// FIFO status

//Power Control Register Bits
#define WU_0		(1<<0)	// Wake Up Mode - Bit 0
#define	WU_1		(1<<1)	// Wake Up mode - Bit 1
#define SLEEP		(1<<2)	// Sleep Mode
#define	MEASURE		(1<<3)	// Measurement Mode
#define AUTO_SLP	(1<<4)	// Auto Sleep Mode bit
#define LINK		(1<<5)	// Link bit
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

class ADXL345 {
private:
  void write(uint8 reg, uint8 data);
  void read2bytes(uint8 reg, uint8 *msg_data);
public:
  Vector zero;
  Vector measure;
  Vector measureSlow;
  Vector dMeasure;
  void begin();
  int read(Axis axis);
  Vector read(Vector mesInt);
  Vector read();
};

void ADXL345::write(uint8 reg, uint8 data) {
  /* all i2c transactions send and receive arrays of i2c_msg objects */
  i2c_msg msgs[1]; // we dont do any bursting here, so we only need one i2c_msg object
  uint8 msg_data[2];

  msg_data = { 
    reg, data   };
  msgs[0].addr = ADXL_ADDR;
  msgs[0].flags = 0; // write
  msgs[0].length = 2; 
  msgs[0].data = msg_data;
  i2c_master_xfer(I2C, msgs, 1,0);
}

void ADXL345::begin() {
  Log.print("ADXL345 ... ");
  this->write(POWER_CTL, MEASURE);
  this->write(DATA_FORMAT, 0); // +/- 2g
  Log.println("ok!");
}

void ADXL345::read2bytes(uint8 reg, uint8 *msg_data) {
  i2c_msg msgs[1]; 
  msg_data[0] = reg;

  msgs[0].addr = ADXL_ADDR;
  msgs[0].flags = 0; 
  msgs[0].length = 1; // just one byte for the address to read, 0x00
  msgs[0].data = msg_data;
  i2c_master_xfer(I2C, msgs, 1,0);

  msgs[0].addr = ADXL_ADDR;
  msgs[0].flags = I2C_MSG_READ; 
  msgs[0].length = 2;
  msgs[0].data = msg_data;
  i2c_master_xfer(I2C, msgs, 1,0);
}

int ADXL345::read(Axis axis) {
  uint8 reg=0;
  switch(axis) {
  case axis_X:
    reg = DATAX0;
    break;
  case axis_Y:
    reg = DATAY0;
    break;
  case axis_Z:
    reg = DATAZ0;
    break;
  }
  uint8 temp[2] = {
    0    };
  this->read2bytes(reg, temp);
  return ((temp[1] << 8) | temp[0]);
}

Vector ADXL345::read(Vector mesInt) {
  Vector v = convertVector(mesInt, 4*9.81/1024, this->zero);
  Vector lastMeasure = this->measure;
  lowPassVect(&(this->measure), v, dt/ADXL_FILTER_CONST);
  this->dMeasure = Comb2( this->measure, 1/dt, lastMeasure, -1/dt );
  lowPassVect(&(this->measureSlow), v, dt/ADXL_FILTER_CONST_SLOW);
  return this->measure;
}

Vector ADXL345::read() {
  return this->read(newVector(this->read(axis_X), this->read(axis_Y), this->read(axis_Z)));
}



#endif // _ADXL345_H_


