// Interface avec les capteurs I²C : ADXL345, ITG3200, MAG3100 & BMP085
// Matthias Lemainque 2013

#ifndef _SENSORS_H_
#define _SENSORS_H_

#include "pins.h"
#include "wirish.h"
#include "maths.h"
#include "i2c.h"


// Paramètres I2C
#define I2C_TIMEOUT	50

// Constantes
#define I2C_NOT_SETUP	1


// * * * * * * * *
//  A D X L 3 4 5
// * * * * * * * *

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


// * * * * * * *
//  B M P 0 8 5
// * * * * * * *

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


// * * * * * * * *
//  I T G 3 2 0 0
// * * * * * * * *

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


// * * * * * * * *
//  M A G 3 1 1 0
// * * * * * * * *

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



//  * * * * * * * * * * * * *
// C L A S S E   S E N S O R S
//  * * * * * * * * * * * * *

// Paramètres
#define READ_MAX_LENGTH	2	// Nombre maxi d'octets lus par la méthode read(...)

// Constantes
#define READ_HB_FIRST	false
#define READ_LB_FIRST	true

class SENSORS {
private:
  void write(uint16 I2Caddr, uint8 reg, uint8 data);
  int32 read(uint16 I2Caddr, uint8 reg, uint8 n=1, boolean UintToInt=false, boolean dec=READ_HB_FIRST);
  int32 read2(uint16 I2Caddr, uint8 regH, uint8 regL, boolean UintToInt=false);

  void setupI2C();
  void setupADXL345();
  void setupITG3200();
  void setupBMP085(uint8 mode=BMP085_ULTRAHIGHRES);
  void setupMAG3110();

  void readADXL345();
  void readITG3200();
  void readMAG3110();
  boolean readBMP085();

  uint32 lastLoop;

  // Variables associées au BMP085
  uint8 oversampling;
  int16 ac1, ac2, ac3, b1, b2, mb, mc, md;
  uint16 ac4, ac5, ac6;
  int32 X1, X2, X3, B3, B5, B6, p;
  uint32 B4, B7;
  uint32 UT, UP;
  uint32 BMPlastTime;
  uint8 BMPstate;

public:
  SENSORS();
  
  void setup();
  boolean loop();

  int32 I2C_err;

  // Amplitudes extremes
  float rangeADXL345;
  float rangeITG3200;
  float rangeMAG3110;

  // Zéros
  boolean enableZeros;
  float zeroADXL345[3];
  float zeroMAG3110[3];

  // Mesures
  float dt;
  float measureADXL345[3];
  float measureITG3200[3];
  float measureMAG3110[3];
  float refAlt, refPress;
  float temperature;
  float pressure;
  float altitude();

};

#endif // _SENSORS_H_
