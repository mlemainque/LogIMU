// Matthias Lemainque 2012
// Gère le statut du programme, le temps, l'interface utilisateur

#ifndef _STATE_H_
#define _STATE_H_

void changeState(byte prim, byte sec=0, uint32 duration=0);
float dTime(boolean inc=false);
void process(uint32 duration = 0);
void process(uint32 duration, RGB forcedCol);
void pause();



// ******************************************
//   Statut du programme

// Etats
#define IsOff			0	// A l'arrêt
#define IsCalibAM		1	// Calibration accéléromètres & magnétomètres
#define IsCalibG		2	// Calibration gyromètres
#define IsReady			3	// Fonctionnement normal
#define IsConfig		4	// Ecran de configuration

// Etats secondaires
#define IsOff_MenuHotLunch	0	// Option lancement à chaud de l'IMU séléctionnée
#define IsOff_MenuLunch		1	// Option lancement IMU sélectionnée (avec calibration gyromètres)
#define IsOff_MenuCalibAM	2	// Option lancement calibration sélectionnée
#define IsOff_MenuConfig	3	// Option configuration sélectionnée
#define IsOff_Splash		4	// Affiche le splash screen de lancement

#define IsCalibAM_Measure	0	// Mesures en cours ou non
#define IsCalibAM_MeasureOk	1	// Mesure validée
#define IsCalibAM_Converg	2	// Algorithme de convergence
#define IsCalibAM_ConvergOk	3	// Convergence terminée

#define IsCalibG_Measure	0	// Mesure en cours
#define IsCalibG_MeasureOk	1	// Mesure ok
#define IsCalibG_MeasureWrong	2	// Mesure non valide

#define IsReady_Page1		0	// Affichage orientation
#define IsReady_Page2		1	// Affichage mesures accéléromètres/magnétomètres
#define IsReady_Page3		2	// Affichage horloge et mesures température/altitude

#define IsConfig_LogSerial	0	// Choix du mode du Log (disabled, usb, 1, 2, 3)
#define IsConfig_SD		1	// Choix du mode de la carte SD (off, csv, bin)
#define IsConfig_IMUgyro	2	// Activation ou non de l'IMU gyromètre seule
#define IsConfig_Listen		3	// Forçage de l'écoute du pc
#define IsConfig_Posit		4	// Intégration de la position (inutile mais pour montrer que ça ne marche pas)
#define IsConfig_Init		5	// Réinitilisation des paramètres
#define IsConfig_back		6	// Retour au menu principal

byte state;
byte stateSec;

// Change le statut du programme et force éventuellement un delay
void changeState(byte prim, byte sec, uint32 duration) {
  state = prim;
  stateSec = sec;
  process(duration);
}




// ******************************************
//   Intervalle de temps

#define DELAY_CALC_CPS	1000

uint32 lastTime;
uint32 lastTimecps;
float longerLag;
uint32 nLoop;
float cps;
float dt;

// Renvoie le temps écoulé depuis la dernière execution de cette procédure (en sec)
float dTime(boolean inc) {
  uint32 time = micros();
  dt = (float)(time-lastTime)/1000000;
  if (lastTime>time) dt += pow(256,4)/1000000;		// Overflow de micros()

  // On veut démarrer un nouveau chrono
  if (inc) {
    lastTime = time;	// On se souvient de l'heure
    process();		// On actualise les led au passage
    nLoop++;
    if (nLoop==50) {	// On fait clignoter la led témoin
      XtoggleLED();
      nLoop = 0;
    }
  }

  // On calcule le nombre d'itérations par secondes (minoration)
  longerLag = max( longerLag, dt );
  if (millis()-lastTimecps>=DELAY_CALC_CPS) {
    lastTimecps = millis();
    if (longerLag != 0) cps = 1 / longerLag;
    longerLag = 0;
  }

  return dt;
}




// ******************************************
//    Gère les pauses et les boutons

#define DELAY_BUTTON	700	// Délai avant première répétition (menus) (ms)
#define INTERVAL_BUTTON	300	// Intervale entre répétitions (menus) (ms)
#define DELAY_LG_BUTTON	2500	// Délai pour un "long" bouton (hors menus) (ms)

// Actions menu principal
#define amHotLunch	0
#define amLunch		1
#define amCalibAM	2

#define ORIENT_HORIZ	0
#define ORIENT_VERTI	1

#define CONFIG_LOGSERIAL_FIRST	0
#define CONFIG_LOGSERIAL_LAST	2
#define CONFIG_LOGSERIAL_MAXI	tsUSB
#define CONFIG_SD_FIRST		3
#define CONFIG_SD_LAST		4
#define CONFIG_SD_MAXI		LOG_TYPE_CSV
#define CONFIG_LISTENPC_POS	5		// Ecoute de temps en temps s'il y a des paquets en provenance du pc (à haute vitesse, la radio n'est pas bidirectionnelle)
#define CONFIG_IMUGYRO_POS	6
#define CONFIG_LOGACC_POS	7
#define CONFIG_LOGMAG_POS	8
#define CONFIG_LOGERR_POS	9
#define CONFIG_LOGBAR_POS	10
#define CONFIG_POSIT_POS	11

uint32 timePress;
uint32 lastClick;
boolean nRepeat;
boolean haveBeenLow = true;

void resetFlash() {
  page_clear();
  vectToFlash(newVector(-1.1,-0.8,1.0),  0, ITG3200_FlashRange);
  vectToFlash(newVector(0.46,0.27,0.03), 3, ADXL345_FlashRange);
  vectToFlash(newVector(-210,-233,-110), 6, MAG3110_FlashRange);
  writeParam(CONFIG_LOGSERIAL_FIRST, CONFIG_LOGSERIAL_LAST, ts2, false);
  writeParam(CONFIG_SD_FIRST,        CONFIG_SD_LAST,        LOG_TYPE_OFF, false);
  writeParam(CONFIG_IMUGYRO_POS,     CONFIG_IMUGYRO_POS,    false, false);
  writeParam(CONFIG_LISTENPC_POS,    CONFIG_LISTENPC_POS,   true, false);
  writeParam(CONFIG_POSIT_POS,       CONFIG_POSIT_POS,      false, false);
  floatToFlash(1.0, FLASH_POS_CONST_A, 0, 120);
  floatToFlash(1.5, FLASH_POS_CONST_M, 0, 120);
  floatToFlash(0.1, FLASH_POS_ERRNORM_A, 0, 10);
  floatToFlash(0.1, FLASH_POS_ERRNORM_M, 0, 10);
  floatToFlash(10, FLASH_POS_ERRDIR_A, 0, 180);
  floatToFlash(20, FLASH_POS_ERRDIR_M, 0, 180);
  floatToFlash(2, FLASH_POS_DIFFCST_A, 0, 20);
  floatToFlash(0.01, FLASH_POS_DIFFCST_M, 0, 20);
  floatToFlash(0, FLASH_POS_REFALT, -500, 5750);
  floatToFlash(1.01325, FLASH_POS_REFPRESS, .5, 1.125);
  page_write_quick();
}  

void buttonShort() {
  timePress = 0;
  if (state == IsOff) {
    if (stateSec == IsOff_MenuHotLunch) actionMenu(amHotLunch);
    else if (stateSec == IsOff_MenuLunch) actionMenu(amLunch);
    else if (stateSec == IsOff_MenuCalibAM) actionMenu(amCalibAM);
    else if (stateSec == IsOff_MenuConfig) changeState(IsConfig);
  }
  else if (state == IsReady) {
    if (stateSec == IsReady_Page1) changeState(IsReady, IsReady_Page2);
    else if (stateSec == IsReady_Page2) changeState(IsReady, IsReady_Page3);
    else if (stateSec == IsReady_Page3) changeState(IsReady, IsReady_Page1);
  }
  else if (state == IsConfig) {
    if (stateSec == IsConfig_LogSerial)    nextParam(CONFIG_LOGSERIAL_FIRST, CONFIG_LOGSERIAL_LAST, CONFIG_LOGSERIAL_MAXI);
    else if (stateSec == IsConfig_SD)      nextParam(CONFIG_SD_FIRST,        CONFIG_SD_LAST,        CONFIG_SD_MAXI);
    else if (stateSec == IsConfig_IMUgyro) nextParam(CONFIG_IMUGYRO_POS,     CONFIG_IMUGYRO_POS,    true);
    else if (stateSec == IsConfig_Listen)  nextParam(CONFIG_LISTENPC_POS,    CONFIG_LISTENPC_POS,   true);
    else if (stateSec == IsConfig_Posit)   nextParam(CONFIG_POSIT_POS,       CONFIG_POSIT_POS,      true);
    else if (stateSec == IsConfig_Init) {
      resetFlash();
      changeState(IsConfig);
    }
    else if (stateSec == IsConfig_back) {
      loadParams();
      changeState(IsOff);
    }
  }
  refreshAff(true);
  lastClick = millis();
  haveBeenLow = false;
}

void buttonLong() {
  timePress = 0;
  nRepeat++;
  if (state == IsOff) {
    stateSec += 1;
    if (stateSec > IsOff_MenuConfig) stateSec = 0;
  }
  else if (state == IsReady) changeState(IsOff);
  else if (state == IsCalibG) changeState(IsOff);
  else if (state == IsCalibAM) changeState(IsOff);
  else if (state == IsConfig) {
    stateSec += 1;
    if (stateSec > IsConfig_back) stateSec = 0;
  }
  refreshAff(true);
  lastClick = millis();
  if (!((state == IsOff) || (state == IsConfig))) haveBeenLow = false;
}

boolean isButtonLong() {
  uint32 delayPress = millis()-timePress;
  if ((state==IsOff) || (state==IsConfig)) {
    if (nRepeat!=1 && (delayPress>=INTERVAL_BUTTON)) return true;
    if (nRepeat==1 && (delayPress>=DELAY_BUTTON)) return true;
  }
  else if (delayPress>=DELAY_LG_BUTTON) return true;
  return false;
}

void processWhile() {
  // Actualise l'écran
  refreshAff(false);

  // Actualise les LEDs
  led.refresh();

  // Gère les boutons
  if (digitalRead(BOARD_BUTTON_PIN)==LOW) haveBeenLow = true;
  if (haveBeenLow && (millis()-lastClick>=200)) {
    if (digitalRead(BOARD_BUTTON_PIN)==HIGH) {
      if (timePress == 0) timePress = millis();
      if (isButtonLong()) buttonLong();
    }
    else if (timePress != 0) { // Le bouton vient d'être relâché
      if (isButtonLong()) buttonLong();
      else if (nRepeat == 0) buttonShort();
      timePress = 0;
      nRepeat = 0;
    }
  }
}

void process(uint32 duration) {
  uint32 begin = millis();
  do {
    processWhile();
  }
  while (millis()-begin<=duration);
}

void process(uint32 duration, RGB forcedCol) {
  led.setColor(forcedCol);
  process(duration);
}

void pause() {
  do {
    processWhile();
    timePress = 0; // Sinon buttonShort() est déclenché lorsqu'on presse le bouton
  }
  while (!isButtonPressed());
}




#endif // _STATE_H_





