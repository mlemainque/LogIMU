// Contrôle des LEDs, Lcd et boutons
// Matthias Lemainque 2013

#ifndef _INTERFACE_H_
#define _INTERFACE_H_

#include "wirish.h"
#include "maths.h"
#include "sensors.h"
#include "kalman.h"
#include "calib.h"
#include "pcd8544.h"

class INTERFACE {
private:
  pcd8544 *Lcd;
  SENSORS *Sensors;
  KALMAN *Kalman;
  CALIB *Calib;
  LOG *Log;
  
  void underlineBmp(uint8 *bmp, uint8 width);
  
  void clearLcd();
  void actuLcd();
  void actuLcdParam();
  void actuLcdCalib();
  uint32 lastLcd;
  boolean lastLcdCalib;
  
  uint16 state;
  uint32 lastState;
  
  uint8 cursor_pos;
  
public:
  INTERFACE(SENSORS *newSensors, KALMAN *newKalman, CALIB *newCalib, LOG *newLog, pcd8544 *newLcd);
  
  void setup();
  void loop();
  void pause();
  
};

#endif // _USER_H_
