// Matthias Lemainque 2012
// Log LCD

#ifndef _LCD_H_
#define _LCD_H_

#define DELAY_ACTU_LCD	250	// (ms)

#define PIN_LED_LCD	19

pcd8544 lcd(12, 13, 14, &spi);
uint32 timeLCD;

void refreshAff(boolean force) {
  if (!force && millis()-timeLCD<DELAY_ACTU_LCD) return;
  timeLCD = millis();

  lcd.clear();

  if (state == IsOff) {
    if (stateSec == IsOff_Splash) {
      lcd.setCursor(0,0);
      lcd.print("**************");
      lcd.setCursor(0,1);
      lcd.print("  LogIMU v2");
      lcd.setCursor(0,3);
      lcd.print(" par Matthias");
      lcd.setCursor(0,4);
      lcd.print("  Lemainque");
      lcd.setCursor(0,5);
      lcd.print("**************");
    }
    else {
      lcd.negative = true;
      lcd.setCursor(0,0);
      lcd.print("* LogIMU  v2 *");

      lcd.setCursor(0,1);
      lcd.negative = (stateSec == IsOff_MenuHotLunch);
      lcd.print("Lancer chaud");

      lcd.setCursor(0,2);
      lcd.negative = (stateSec == IsOff_MenuLunch);
      lcd.print("Lancer");

      lcd.setCursor(0,3);
      lcd.negative = (stateSec == IsOff_MenuCalibAM);
      lcd.print("Calib Acc/Mag");

      lcd.setCursor(0,4);
      lcd.negative = (stateSec == IsOff_MenuConfig);
      lcd.print("Configuration");

      lcd.negative = false;
    }
    led.setColor(ledGREEN, 1000, 20);
  }

  if (state == IsCalibAM) {
    lcd.setCursor(0,0);
    lcd.negative = true;
    lcd.print("Calib Acc/Mag");
    lcd.negative = false;
    if ((stateSec == IsCalibAM_Measure) || (stateSec == IsCalibAM_MeasureOk)) {
      lcd.setCursor(0,1);
      lcd.print("Mesure  ");
      lcd.print(imu.iVectCalibAM+1);
      lcd.print("/14");
      lcd.setCursor(0,2);
      lcd.print("Erreur:");
      lcd.printF(imu.ecartTypeAG, 2);
      lcd.setCursor(0,3);
      lcd.print("Angle: ");
      lcd.printF(imu.errDirA);
      lcd.printDeg();
      if (imu.jVectCalibX!=9999) {
        lcd.setCursor(0,4);
        lcd.print("Etat:   ");
        int i = imu.iVectCalibX;
        if (i<=imu.jVectCalibX) i += CalibX_Length;
        lcd.print((float)(100*(i+1-imu.jVectCalibX)/CalibX_Length),0);
        lcd.print("%");
      }
      if (stateSec == IsCalibAM_Measure)
        if (imu.errDirA >= CalibAM_AngleDiff) led.setColor(ledRED);
        else if (imu.jVectCalibX == 9999) led.setColor(ledPURPLE);
        else led.setColor(ledTURQUOISE);
      else led.setColor(ledGREEN);
    }
    if ((stateSec == IsCalibAM_Converg) || (stateSec == IsCalibAM_ConvergOk)) {
      lcd.setCursor(0,1);
      lcd.print("   Acc   Mag");
      lcd.setCursor(0,2);
      lcd.print("X:");
      lcd.printF(acc.zero.X,2);
      lcd.setCursor(8,2);
      lcd.printF(mag.zero.X,0);

      lcd.setCursor(0,3);
      lcd.print("Y:");
      lcd.printF(acc.zero.Y,2);
      lcd.setCursor(8,3);
      lcd.printF(mag.zero.Y,0);

      lcd.setCursor(0,4);
      lcd.print("Z:");
      lcd.printF(acc.zero.Z,2);
      lcd.setCursor(8,4);
      lcd.printF(mag.zero.Z,0);

      lcd.setCursor(0,5);
      lcd.print("e:");
      lcd.printF(100*imu.ecartTypeAG,1);
      lcd.print("%");
      lcd.setCursor(8,5);
      lcd.printF(100*imu.ecartTypeM,1);
      lcd.print("%");

      if (stateSec == IsCalibAM_ConvergOk) led.setColor(ledBLUE, 1000, 20);
      else led.setColor(ledBLUE);
    }
  }

  if (state == IsCalibG) {
    lcd.setCursor(0,0);
    lcd.negative = true;
    lcd.print("Calib Gyro");
    lcd.negative = false;
    lcd.setCursor(0,1);
    lcd.print("Etat: ");
    lcd.print((float)(100*imu.iVectCalibX/CalibX_Length),0);
    lcd.print("%");
    if (imu.ecartTypeAG!=0) {
      lcd.setCursor(0,2);
      lcd.print("Ecart:");
      lcd.print(imu.ecartTypeAG);
    }
    if (stateSec == IsCalibG_Measure) led.setColor(ledBLUE, 500, 500);
    if (stateSec == IsCalibG_MeasureOk) led.setColor(ledGREEN);
    if (stateSec == IsCalibG_MeasureWrong) led.setColor(ledRED);
  }

  if (state == IsReady) {
    if (stateSec == IsReady_Page2) {
      lcd.setCursor(0,0);
      lcd.negative = true;
      lcd.print(" Lac  Rou Tan ");
      lcd.negative = false;

      Orient orient = imu.base.orient();

      lcd.setCursor(0,1);
      lcd.printF(orient.lacet);
      lcd.printDeg();

      lcd.setCursor(5,1);
      lcd.printF(orient.roulis);
      lcd.printDeg();

      lcd.setCursor(9,1);
      lcd.printF(orient.tangage);
      lcd.printDeg();

      if (imu2.enabled) {
        orient = imu2.base.orient();

        lcd.setCursor(0,2);
        lcd.printF(orient.lacet);
        lcd.printDeg();

        lcd.setCursor(5,2);
        lcd.printF(orient.roulis);
        lcd.printDeg();

        lcd.setCursor(9,2);
        lcd.printF(orient.tangage);
        lcd.printDeg();
      }

      lcd.setCursor(0,3);
      lcd.negative = true;
      lcd.print("Stab Acc  Mag ");
      lcd.negative = false;

      lcd.setCursor(0,4);
      lcd.print("ang. ");
      lcd.print(imu.errDirA, 0);
      lcd.printDeg();
      lcd.setCursor(10,4);
      lcd.print(imu.errDirM, 0);
      lcd.printDeg();

      lcd.setCursor(0,5);
      lcd.print("conf ");
      lcd.print((float)(100*imu.confA), 0);
      lcd.print("%");
      lcd.setCursor(10,5);
      lcd.print((float)(100*imu.confM), 0);
      lcd.print("%");
    }

    if (stateSec == IsReady_Page3) {
      lcd.setCursor(0,0);
      lcd.negative = true;
      lcd.print("   Acc0  Mag  ");
      lcd.negative = false;

      lcd.setCursor(0,1);
      lcd.print("   m.s-2 uT");

      lcd.setCursor(0,2);
      lcd.print("X:");
      lcd.printF(imu.dataAcc0.X,2);
      lcd.setCursor(8,2);
      lcd.printF(mag.measure.X,1);

      lcd.setCursor(0,3);
      lcd.print("Y:");
      lcd.printF(imu.dataAcc0.Y,2);
      lcd.setCursor(8,3);
      lcd.printF(mag.measure.Y,1);

      lcd.setCursor(0,4);
      lcd.print("Z:");
      lcd.printF(imu.dataAcc0.Z,2);
      lcd.setCursor(8,4);
      lcd.printF(mag.measure.Z,1);

      lcd.setCursor(0,5);
      lcd.print("n:");
      lcd.printF(Norm(imu.dataAcc0),1);
      lcd.setCursor(8,5);
      lcd.printF(Norm(mag.measure),1);
    }

    if (stateSec == IsReady_Page1) {
      lcd.setCursor(0,0);
      lcd.negative = true;
      lcd.print("              ");
      lcd.setCursor(0,0);
      lcd.print(" SD:");
      if (file.isOpen()) lcd.print("yes");
      else lcd.print("no");
      lcd.setCursor(9,0);
      if (Log.type==tsUSB) lcd.print("USB");
      if (Log.type==ts1) lcd.print("Ser1");
      if (Log.type==ts2) lcd.print("Ser2");
      if (Log.type==ts3) lcd.print("Ser3");
      lcd.negative = false;

      lcd.setCursor(1,1);
      int t = (millis()-startTime)/1000;
      int m = t/60;
      int s = t%60;
      if (m != 0) {
        lcd.print(m);
        lcd.print("min ");
      }
      lcd.print(s);
      lcd.print("s");

      lcd.setCursor(1,2);
      lcd.print(cps, 0);
      lcd.print("cps");


      lcd.setCursor(0,3);
      lcd.negative = true;
      lcd.print("Environnement ");
      lcd.negative = false;

      lcd.setCursor(0,4);
      lcd.print("Temp.:");
      lcd.printF(bar.temperature,1);
      lcd.printDeg();
      lcd.print("C");

      lcd.setCursor(0,5);
      lcd.print("Alti.: ");
      lcd.print(bar.altitude(), 0);
      lcd.print("m");
    }

    led.setColor( {
      1-imu.confA, imu.confA, 0         } 
    );
  }

  if (state == IsConfig) {
    lcd.setCursor(0,0);
    lcd.negative = true;
    lcd.print("Configuration ");

    if (stateSec<=4) {
      lcd.setCursor(0,1);
      lcd.negative = (stateSec == IsConfig_LogSerial);
      lcd.print("Log Serie");
      lcd.negative = false;
      lcd.setCursor(10,1);
      uint16 param = readParam(CONFIG_LOGSERIAL_FIRST, CONFIG_LOGSERIAL_LAST);
      if (param == tsDisabled) lcd.print("off");
      if (param == ts1) lcd.print("1  ");
      if (param == ts2) lcd.print("2  ");
      if (param == ts3) lcd.print("3  ");
      if (param == tsUSB) lcd.print("USB");

      lcd.setCursor(0,2);
      lcd.negative = (stateSec == IsConfig_SD);
      lcd.print("Log SD");
      lcd.negative = false;
      lcd.setCursor(10,2);
      param = readParam(CONFIG_SD_FIRST, CONFIG_SD_LAST);
      if (param == LOG_TYPE_OFF) lcd.print("off");
      if (param == LOG_TYPE_BIN) lcd.print("BIN");
      if (param == LOG_TYPE_CSV) lcd.print("CSV");

      lcd.setCursor(0,3);
      lcd.negative = (stateSec == IsConfig_IMUgyro);
      lcd.print("IMU gyro");
      lcd.negative = false;
      lcd.setCursor(10,3);
      param = readParam(CONFIG_IMUGYRO_POS, CONFIG_IMUGYRO_POS);
      if (param == false) lcd.print("off");
      if (param == true) lcd.print("YES");

      lcd.setCursor(0,4);
      lcd.negative = (stateSec == IsConfig_Listen);
      lcd.print("Listen PC");
      lcd.negative = false;
      lcd.setCursor(10,4);
      param = readParam(CONFIG_LISTENPC_POS, CONFIG_LISTENPC_POS);
      if (param == false) lcd.print("no");
      if (param == true) lcd.print("YES");

      lcd.setCursor(0,5);
      lcd.negative = (stateSec == IsConfig_Posit);
      lcd.print("Position");
      lcd.negative = false;
      lcd.setCursor(10,5);
      param = readParam(CONFIG_POSIT_POS, CONFIG_POSIT_POS);
      if (param == false) lcd.print("no");
      if (param == true) lcd.print("YES");
    }
    else {
      lcd.setCursor(0,1);
      lcd.negative = (stateSec == IsConfig_Init);
      lcd.print("Réinitialiser");
      lcd.negative = false;

      lcd.setCursor(0,2);
      lcd.negative = (stateSec == IsConfig_back);
      lcd.print("Retour");
    }

    lcd.negative = false;
    led.setColor(ledGREEN, 1000, 20);
  }
}

#endif // _LCD_H_



