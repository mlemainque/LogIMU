// Matthias Lemainque 2012-2013
// TIPE PSI* - matthias.lemainque@gmail.com
// Centrale inertielle

#define _DEBUG_
#define LOG_FPS 24
#define I2C I2C2

#include "stdint.h"
#include "stdlib.h"
#include "string.h"
#include "wirish.h"

#include "led.h"
LED_RGB led(15,16,17);

HardwareSPI spi(1);

#include "SdFat.h"
#include "pcd8544.h"


void refreshAff(boolean force=false);

uint32 startTime;
uint32 lastLog;
#define LOG_SEND_acc true;
#define LOG_SEND_mag false;
#define LOG_SEND_err false;
#define LOG_SEND_bar true;

// Force une pause dans l'envoi des paquets, de temps en temps
#define INTERVAL_LISTEN 1000
#define PAUSE_LISTEN 100
boolean listen = false;
boolean send_logs = true;
uint32 lastListen;

boolean simulIMU = false;
byte usbBuffState = 254;
uint8 usbBuff[18];
byte usbPing = 0;

void actionMenu(byte am);
void loadParams();
#include "maths.h"
#include "log.h"

#include "store.h"
#define ADXL345_FlashRange	19.62	// Plage de mesure des accéléromètres
#define ITG3200_FlashRange	2000	// Plage de mesure des gyromètres
#define MAG3110_FlashRange	1000	// Plage de mesure des magnétomètres

#include "state.h"

#include "adxl345.h"
ADXL345 acc;

#include "itg3200.h"
ITG3200 gyr;

#include "mag3110.h"
MAG3110 mag;

#include "IMU.h"
#include "IMUCALIB.h"
void lunchIMUs();
IMUCALIB imu(true,true,true);
IMU imu2(false,false,true);

#include "bmp085.h"
BMP085 bar;

#include "lcd.h"
#include "param.h"

void setup() {
  pinMode(BOARD_LED_PIN, OUTPUT);	// Activation du voyant clignotant sur la Maple
  pinMode(PIN_LED_LCD, OUTPUT);		// Activation rétroéclairage de l'écran LCD
  digitalWrite(PIN_LED_LCD, HIGH);
  pinMode(PIN_EXT_LED, OUTPUT);		// Activation du voyant clignotant externe
  pinMode(PIN_APC220_EN, OUTPUT);	// Activation de la radio (30 mA)
  togglePin(PIN_EXT_LED);		// Par défaut, voyant éteint
  spi.begin(SPI_9MHZ, MSBFIRST, 0);	// Interface SPI (sd et lcd)
  i2c_master_enable(I2C, 0);		// Interface I²C
  seedRand(millis());

  // LCD
  lcd.begin();
  lcd.clear();

  // Flash et paramètres
  flash_setup();
  //resetFlash();
  page_read();
  loadParams();

  // Splash screen
  changeState(IsOff, IsOff_Splash);

  // Lancement capteurs
  gyr.begin();
  XtoggleLED();
  acc.begin();
  mag.begin();
  bar.begin();
  gyr.zero = flashToVect(0, ITG3200_FlashRange);
  acc.zero = flashToVect(3, ADXL345_FlashRange);
  mag.zero = flashToVect(6, MAG3110_FlashRange);
  
  bar.readForce();
  Log.println(bar.pressure);

  // Affichage menu principal
  process(1500);
  changeState(IsOff, IsOff);
}

void loop() {
  process();

  if ((!simulIMU) || (Log.type==tsUSB)){
    gyr.read();
    acc.read();
    mag.read();
  }
  else {
    while (SerialUSB.available()) { // vaut 0 ou 1, seulement ... (bug de SerialUSB)
      byte b = SerialUSB.read();
      if (b==0) usbPing+=1;
      else usbPing=0;
      if (usbPing==10) {
        Log.write(255);
        Log.write(3);
      }
      if (usbBuffState>=128) { // On doit avoir une amorce de 2 bytes Null
        if (b==0) usbBuffState++;
        else usbBuffState = 254;
      }
      else {
        usbBuff[usbBuffState++] = b;
        if (usbBuffState==18) { // On a reçu le paquet complet : on décode
          Vector V;
          for (byte i=0 ; i<3 ; i++) {
            V.X = usbBuff[i*6+0]*256+usbBuff[i*6+1];
            V.Y = usbBuff[i*6+2]*256+usbBuff[i*6+3];
            V.Z = usbBuff[i*6+4]*256+usbBuff[i*6+5];
            if (i==0) gyr.read(V);
            if (i==1) acc.read(V);
            if (i==2) mag.read(V);
          }
          XtoggleLED();
          usbBuffState = 254;
        }
      }
    }
  }
  bar.read();
  if (state == IsReady) {
    imu.procIMU();
    imu2.procIMU();
  }
  if (state == IsReady) logDATA();
  if (state == IsCalibG) imu.procCalibG();
  if (state == IsCalibAM) imu.procCalibAM();

  readSerial();
  dTime(true);
}

void logDATA() {
  // On force éventuellement, de temps en temps, une période sans envoi de données
  // Ceci permet de laisser la parole à l'opérateur
  if (listen) {
    if (send_logs) {
      if (millis()-lastListen>=INTERVAL_LISTEN) {
        send_logs = false;
        lastListen = millis();
      }
    } 
    else {
      if (millis()-lastListen>=PAUSE_LISTEN) {
        send_logs = true;
        lastListen = millis();
      }
      else return;
    }
  }

  if (millis()-lastLog<1000/LOG_FPS) return;
  uint32 newLastLog = millis();

  // LOG BINAIRE (toujours sur port série et éventuellement carte SD)
  Log.printInSD = (logType == LOG_TYPE_BIN);
  Log.write(251 + imu2.enabled + 2*imu.posit);
  Log.write(min(250,(millis()-lastLog)/2)); // Permet de gérer jusqu'à 500 ms d'écart (2 cps) entre chaque log
  Log.writevab(imu.base.uX, -1, 1);
  Log.writevab(imu.base.uY, -1, 1);
  Log.writevab(imu.dataAcc0, -2*9.81, 2*9.81);

  Log.writefab(imu.confA, 0, 1);
  Log.writefab(imu.confM, 0, 1);

  Log.writefab(bar.temperature, -20, 42);
  Log.writefa16(bar.altitude(), -500, 7000);

  if (imu2.enabled) {
    Log.writevab(imu2.base.uX, -1, 1);
    Log.writevab(imu2.base.uY, -1, 1);
  }
  if (imu.posit) Log.writeva16(imu.dataPos0, -3125, 3125);

  // LOG CSV (uniquement sur carte SD)
  if (logType == LOG_TYPE_CSV) {
    Orient orient = imu.base.orient();
    Log.printInLog = false;
    Log.printInSD = true;
    Log.print((float)(millis()-startTime)/1000);
    Log.print(";");
    Log.print(orient.lacet);
    Log.print(";");
    Log.print(orient.roulis);
    Log.print(";");
    Log.print(orient.tangage);
    Log.print(";");
    Log.print(imu.dataAcc0);
    Log.print(";");
    Log.print((float)imu.confA*100);
    Log.print(";");
    Log.print((float)imu.confM*100);
    Log.print(";");
    Log.print((uint16)bar.altitude());
    if (imu2.enabled) {
      orient = imu2.base.orient();
      Log.print(";");
      Log.print(orient.lacet);
      Log.print(";");
      Log.print(orient.roulis);
      Log.print(";");
      Log.print(orient.tangage);
    }
    Log.println("");
  }
  Log.printInSD = false;
  Log.printInLog = true;

  lastLog = newLastLog;
}

void actionMenu(byte am) {
  if (am==amHotLunch) lunchIMUs();
  if (am==amLunch) imu.beginCalibG();
  if (am==amCalibAM) imu.beginCalibAM();
}

void lunchIMUs() {
  changeState(IsReady);
  startTime = millis();
  refreshAff(true);
  setupSD();
  if (logType == LOG_TYPE_CSV) {
    file.print("Horloge (s);Lacet (deg);Roulis (deg);Tangage (deg);Acc X (m.s-2);Acc Y (m.s-2);Acc Z (m.s-2);confA (%);confM (%);Altitude (m)");
    if (imu2.enabled) file.print(";Lacet gyr (deg);Roulis gyr (deg);Tangage gyr (deg)");
    file.println("");
  }
  logSendParams();
  imu.lunch();
  imu2.lunch();
  startTime = millis();
  lastLog = millis();
}




