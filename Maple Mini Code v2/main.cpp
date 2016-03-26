// LogIMU v3
// Matthias Lemainque 2013

#include "pins.h"
#include "wirish.h"
#include "maths.h"

#include "store.h"
#include "sensors.h"
#include "kalman.h"
#include "calib.h"
#include "log.h"
#include "pcd8544.h"
#include "interface.h"

FLASH myFlash;
SENSORS mySensors;
KALMAN myKalman(&mySensors);
CALIB myCalib(&mySensors, &myFlash);
HardwareSPI mySpi(NUM_SPI);
LOG myLog(&mySensors, &myKalman, &mySpi);
pcd8544 myLcd(PIN_LCD_DC, PIN_LCD_RST, PIN_LCD_SS, &mySpi);
INTERFACE myInterface(&mySensors, &myKalman, &myCalib, &myLog, &myLcd);

void setup() {
  mySpi.begin(SPI_9MHZ, MSBFIRST, 0);
  
  //myFlash.setup();
  myLog.setup();
  mySensors.setup();
  myKalman.setup();
  myLcd.begin();
  myInterface.setup();

  mySensors.zeroADXL345 = { 0.46, 0.27, 0.03 };
  mySensors.zeroMAG3110 = { 57, -6499, -55 };
  
  //myFlash.readTf( mySensors.zeroADXL345, FLASH_ZERO_ADXL, 3, mySensors.rangeADXL345 );
  //myFlash.readTf( mySensors.zeroMAG3110, FLASH_ZERO_MAG,  3, mySensors.rangeMAG3110 );
  //myFlash.readT8( myLog.maskMain, FLASH_MAIN_MASK, MAIN_MASK_LENGTH );
  //myFlash.readT8( myLog.maskAux,  FLASH_AUX_MASK,  AUX_MASK_LENGTH );
}

void loop() {
  //myLog.printTab("ADXL", mySensors.measureADXL345, 1, 3);
  //myLog.printTab("ITG", mySensors.measureITG3200, 1, 3);
  //myLog.printTab("MAG", mySensors.measureMAG3110, 1, 3);
  myLog.printTab("Orient", myKalman.Cardan, 1, 3);
  myLog.printTab("ADXL_0", myKalman.measureADXL345_0, 1, 3);
  Serial.println();

  mySensors.loop();
  if (myCalib.state != CALIB_OFF) myCalib.loop();
  else myKalman.loop();
  //myLog.loop();
  myInterface.loop();
}

