// Contrôle des LEDs, Lcd et boutons
// Matthias Lemainque 2013

#include "interface.h"

INTERFACE::INTERFACE(SENSORS *newSensors, KALMAN *newKalman, CALIB *newCalib, LOG *newLog, pcd8544 *newLcd) {
  Sensors = newSensors;
  Kalman = newKalman;
  Calib = newCalib;
  Log = newLog;
  Lcd = newLcd;
  
  lastLcdCalib = false;
}

void INTERFACE::setup() {
  pinMode(PIN_LCD_LED, OUTPUT);
  digitalWrite(PIN_LCD_LED, HIGH);

  clearLcd();
}

void INTERFACE::clearLcd() {

  (*Lcd).clear();
  (*Lcd).negative = true;
  (*Lcd).setCursor(0, 0);
  (*Lcd).clearRestOfLine();

  // BITMAP générés avec http://www.liggitt.net/arduino/
  uint8 bmp_tangage[12] = {
    0x00, 0x00, 0x00, 0x00, 0xBD, 0xFF, 0xC3, 0xE7, 0x00, 0x00, 0x00, 0x00     };  
  uint8 bmp_roulis[12] = {
    0x3C, 0x38, 0x38, 0x2C, 0x04, 0x06, 0x06, 0x04, 0x2C, 0x38, 0x38, 0x3C     };
  uint8 bmp_lacet[12] = {
    0x00, 0x01, 0x3F, 0x67, 0x4F, 0x40, 0x40, 0x4F, 0x67, 0x3F, 0x01, 0x00     };

  (*Lcd).setCursor(3, 0);
  (*Lcd).bitmap(bmp_lacet, 1, 12);
  (*Lcd).setCursor(7, 0);
  (*Lcd).bitmap(bmp_roulis, 1, 12);
  (*Lcd).setCursor(11, 0);
  (*Lcd).bitmap(bmp_tangage, 1, 12);
  (*Lcd).negative = false;

  (*Lcd).setCursor(5, 2);
  (*Lcd).print("g");
  (*Lcd).setCursor(11, 2);
  (*Lcd).print("uT");

}

void INTERFACE::actuLcd() {
  (*Lcd).printFLoc((*Kalman).Cardan[0], 4,  1, 4); // Lacet
  (*Lcd).printFLoc((*Kalman).Cardan[1], 8,  1, 3); // Roulis
  (*Lcd).printFLoc((*Kalman).Cardan[2], 12, 1, 3); // Tangage

  (*Lcd).printFLoc(NormV((*Kalman).measureADXL345_0), 4,  2, 4); // Accélération
  (*Lcd).printFLoc(NormV((*Kalman).measureMAG3110_0), 10, 2, 4); // Champ magnétique

  (*Lcd).printFLoc((*Sensors).temperature, 3, 3, 3); // Température
  (*Lcd).printFLoc((*Sensors).altitude(), 10, 3, 5); // Altitude
}

void INTERFACE::underlineBmp(uint8 *bmp, uint8 width) {
  for (uint8 i=1 ; i<width-1 ; i++) bmp[i] |= 1<<7;
}

void INTERFACE::actuLcdParam() {
  
  uint8 bmp[12];
  
  // Activation du log SD
  bmp = {
    0x00, 0x2E, 0x2A, 0x2A, 0x3A, 0x00, 0x00, 0x3E, 0x22, 0x22, 0x1C, 0x00 };
  (*Lcd).setCursor(1, 5);
  (*Lcd).negative = (cursor_pos == 0);
  if ((*Log).file.isOpen()) underlineBmp(bmp, 12);
  (*Lcd).bitmap(bmp, 1, 12);
  
  // Activation de la fonction Listen
  bmp[12] = {
    0x00, 0x2E, 0x2A, 0x2A, 0x3A, 0x00, 0x00, 0x3E, 0x22, 0x22, 0x1C, 0x00 };
  (*Lcd).setCursor(4, 5);
  (*Lcd).negative = (cursor_pos == 1);
  if ((*Log).enableListen) underlineBmp(bmp, 12);
  (*Lcd).bitmap(bmp, 1, 12);

  // Inhibition des capteurs ADXL345 & MAG3110
  bmp[12] = {
    0x00, 0x18, 0x3C, 0x7E, 0x42, 0x42, 0x52, 0x52, 0x76, 0x36, 0x24, 0x00 };
  (*Lcd).setCursor(4, 5);
  (*Lcd).negative = (cursor_pos == 1);
  if ((*Log).enableListen) underlineBmp(bmp, 12);
  (*Lcd).bitmap(bmp, 1, 12);
  
  // Calibration des capteurs
  bmp[12] = {
    0x00, 0x08, 0x2A, 0x14, 0x00, 0x1C, 0x1C, 0x00, 0x14, 0x2A, 0x08, 0x00 };
  (*Lcd).setCursor(8, 5);
  (*Lcd).negative = (cursor_pos == 2);
  (*Lcd).bitmap(bmp, 1, 12);

  // Verouillage
  bmp[12] = {
    0x00, 0x10, 0x18, 0x10, 0x18, 0x10, 0x18, 0x10, 0x3C, 0x42, 0x3C, 0x00 };
  (*Lcd).setCursor(12, 5);
  (*Lcd).negative = (cursor_pos == 4);
  // (underlineBmp(bmp, 12);
  (*Lcd).bitmap(bmp, 1, 12);
  
}

void INTERFACE::actuLcdCalib() {
  (*Lcd).clear();
  (*Lcd).negative = true;
  (*Lcd).setCursor(0, 0);
  (*Lcd).clearRestOfLine();
  (*Lcd).setCursor(1, 0);
  (*Lcd).print("Calibration");
  (*Lcd).negative = false;
    
  if ((*Calib).state < CALIB_CALCUL_ADXL) {
    (*Lcd).setCursor(1, 1);
    (*Lcd).print("Mesure ");
    (*Lcd).print((*Calib).num_measure);
    if ((*Calib).state == CALIB_MEASURE) {
      uint8 bmp_check[12] = {
        0x00, 0x10, 0x30, 0x70, 0xE0, 0x70, 0x18, 0x0C, 0x06, 0x03, 0x01, 0x00 };
      (*Lcd).setCursor(10, 1);
      (*Lcd).bitmap(bmp_check, 1, 12);
    }
  }
  else {
    (*Lcd).setCursor(5, 1);
    (*Lcd).print(state%1==0 ? "ADXL" : " MAG");
    
    (*Lcd).setCursor(2, 2);
    (*Lcd).print("X");
    (*Lcd).setCursor(2, 3);
    (*Lcd).print("Y");
    (*Lcd).setCursor(2, 4);
    (*Lcd).print("Z");
    
    for (uint8 i=0 ; i<3 ; i++) (*Lcd).printFLoc(state%1==0 ? (*Calib).zeroADXL345[i] : (*Calib).zeroMAG3110[i], 8, 2+i, 6);
    (*Lcd).printFLoc(state%1==0 ? (*Calib).calc_ET
  }
}

void INTERFACE::loop() {
  if (millis()-lastLcd > 100) {
    lastLcd = millis();
    if ((*Calib).state == CALIB_OFF) {
      if (lastLcdCalib) clearLcd();
      actuLcd();
    }
    else actuLcdCalib();
  }
}

void INTERFACE::pause() {
  
}


