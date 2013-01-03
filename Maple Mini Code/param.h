// Matthias Lemainque 2012
// Paramètres de la centrale

void loadParams() {
  Log.begin(readParam(CONFIG_LOGSERIAL_FIRST, CONFIG_LOGSERIAL_LAST));
  //Log.begin(ts2);
  logType = readParam(CONFIG_SD_FIRST, CONFIG_SD_LAST);
  imu2.enabled = (readParam(CONFIG_IMUGYRO_POS, CONFIG_IMUGYRO_POS) == 1);
  imu.posit = (readParam(CONFIG_POSIT_POS, CONFIG_POSIT_POS) == 1);
  Gravitee = Gravitee_0;
  NordMagn = NordMagn_0;
  listen = readParam(CONFIG_LISTENPC_POS, CONFIG_LISTENPC_POS);

  // Paramètres de correction avec les accéléromètres
  CorrectA_Const = flashToFloat(FLASH_POS_CONST_A, 0, 120);
  CorrectA_ENorm = flashToFloat(FLASH_POS_ERRNORM_A, 0, 10);
  CorrectA_EDir = flashToFloat(FLASH_POS_ERRDIR_A, 0, 180);
  CorrectA_Diff = flashToFloat(FLASH_POS_DIFFCST_A, 0, 20);

  // Paramètres de correction avec les magnétomètres
  CorrectM_Const = flashToFloat(FLASH_POS_CONST_M, 0, 120);
  CorrectM_ENorm = flashToFloat(FLASH_POS_ERRNORM_M, 0, 10);
  CorrectM_EDir = flashToFloat(FLASH_POS_ERRDIR_M, 0, 180);
  CorrectM_Diff = flashToFloat(FLASH_POS_DIFFCST_M, 0, 20);
  
  bar.refAlt = flashToFloat(FLASH_POS_REFALT, -500, 5750);
  bar.refPress = 100000 * flashToFloat(FLASH_POS_REFPRESS, .3, 1.2);
}

#define SERIAL_PARAM_FIRST	96	// Premier emplacement de la mémoire flash synchronisé
#define SERIAL_PARAM_LAST	105	// Premier emplacement de la mémoire flash synchronisé

void logSendParams() {
  Log.write(255);
  Log.write((byte)0);
  for (uint8 i=SERIAL_PARAM_FIRST ; i<=SERIAL_PARAM_LAST ; i++)
    Log.write16(store_data[i]);
}

void readSerial() {
  if (!Log.available()) return;
  byte b = Log.read();
  Log.printInSD = false;
  Log.printInLog = true;
  if (b==253) { // On reçoit les nouveaux paramètres
    for (uint8 i=SERIAL_PARAM_FIRST ; i<=SERIAL_PARAM_LAST ; i++)
      store_data[i] = Log.read16();
    page_write_quick();
    loadParams();
    logSendParams();
  }
  else if (b==254) // On nous demande les paramètres actuels
    logSendParams();
  else if (b==252) { // On demande une simulation
    simulIMU = true;
    loadParams();
    Log.write(255); // Accusé de réception
    Log.write(1);
  }
  else if (b==251) { // On arrête la simulation
    simulIMU = false;
    loadParams();
    Log.write(255); // Accusé de réception
    Log.write(1);
  }
  else if (b==255) { // Demande pression
    Log.write(255);
    Log.write(4);
    Log.writefa16(bar.pressure/100000, .3, 1.2);
  }
}

