// Communication série des mesures
// Matthias Lemainque 2013

#include "log.h"


// * * * * * * * * * * * * * * *
//  I N I T I A L I S A T I O N
// * * * * * * * * * * * * * * *

LOG::LOG(SENSORS *newSensors, KALMAN *newKalman, HardwareSPI *newSpi) {
  Sensors = newSensors;
  Kalman = newKalman;
  Spi = newSpi;
  key = 0;
  nSync = 0;
}

void LOG::setup() {
  Serial.begin(BAUD_RATE);
  Serial.println("truc");
  return;
  
  if (!setupSD()) return;
  newFile();
}

boolean LOG::setupSD() {
  if (!Root.isOpen()) {
    if (!Card.init(Spi)) return false;
    delay(100);
    if (!Volume.init(&Card)) return false;
    if (!Root.openRoot(&Volume)) return false;
  }
  return true;
}

boolean LOG::newFile() {
  if (File.isOpen()) {
    if (!File.close()) return false;
  }
  char Buffer[12] = "LOG_000.BIN";
  for (uint8 i=0 ; i<100 ; i++) {
    Buffer[5] = '0' + i/100;
    Buffer[6] = '0' + (i/10)%10;
    Buffer[7] = '0' + i%100;
    if (File.open(&Root,Buffer,O_CREAT|O_EXCL|O_WRITE)) break;
    if (i == 999) return false;
  }
  return true;
}


//  * * * * * * * * * * * * * *
// E C R I T U R E   S I M P L E
//  * * * * * * * * * * * * * *

uint8 LOG::write(const uint8 data) {
  key = key*13 + data;
  Serial.write( data );
  if (File.isOpen()) {
    if (nSync ++ >= 512) {
      File.sync();
      nSync = 0;
    }
    File.write(data);
  }
  return 1;
}

uint8 LOG::write(const uint64 data, boolean useBuffer, uint8 n) {
  if (useBuffer) {
    // On insère data en haut du buffer
    Buffer |= data << nBuffer;
    nBuffer += n;
  }
  else {
    // On écrit chaque octet en partant du haut de data, et on calcule la clé
    for (uint8 i=n-1 ; i>=0 ; i--)
      write( (data>>(8*i)) % 256 );
  }
  return n; // Renvoit le nombre d'octets écrits
}

uint8 LOG::writeBuffer(uint8 n) {
  for (uint8 i=n-1 ; i>=0 ; i--)
    write( (Buffer>>(8*i)) % 256 );
  nBuffer -= n;
  return n; // Renvoit le nombre d'octets écrits
}

uint8 LOG::writeTab(const float* data, uint8 len, float min, float max, boolean useBuffer, uint8 n) {
  for (uint8 i=0 ; i<len ; i++)
    write( ftoi( data[i], min, max, 8*n ), useBuffer, n );
  return len*n;
}


// * * * * * * * * * * * * * *
//  E N V O I   D O N N E E S
// * * * * * * * * * * * * * *

uint8 LOG::writeData(uint8 b, boolean useBuffer) {
  uint64 data = 0;
  // Même si le switch n'est pas pratique, il représente un bon gain de performance par rapport à une énumération de if
  switch(b) {
  case MASK_END : return false;
  case MASK_KEY :
    // ATTENTION : la clé est mis à jour pendant l'envoi donc ne doit faire que 1 octet
    write(key, useBuffer);
    key = 0;
    return 1;
  case MASK_DT :
    data = constrain( millis()-lastDt, 0, 255 );
    lastDt = millis();
    return write(data, useBuffer);
  case MASK_QUAT_5 :
    data = 0;
    for (uint8 i=0 ; i<3 ; i++) data = (data<<13) | ftoi((*Kalman).X[i], -1, 1, 13);
    data = (data<<13) | ((*Kalman).X[3] > 0);
    return write(data, useBuffer, 5);
  case MASK_TEMP  : return write( ftoi((*Sensors).temperature, -5, 45), useBuffer );
  case MASK_PRESS : return write( ftoi((*Sensors).pressure, 30000, 1200000, 2*8), useBuffer, 2);
  case MASK_ALTI  : return write( ftoi((*Sensors).altitude(), -500, 7000, 2*8), useBuffer, 2);
  case MASK_ACC_6  : return writeTab( (*Sensors).measureADXL345, 3, -(*Sensors).rangeADXL345, (*Sensors).rangeADXL345, useBuffer, 2);
  case MASK_ACC0_3 : return writeTab( (*Kalman).measureADXL345_0, 3, -(*Sensors).rangeADXL345, (*Sensors).rangeADXL345, useBuffer, 1);
  case MASK_ACC0_6 : return writeTab( (*Kalman).measureADXL345_0, 3, -(*Sensors).rangeADXL345, (*Sensors).rangeADXL345, useBuffer, 2);
  case MASK_MAG_6  : return writeTab( (*Sensors).measureMAG3110, 3, -(*Sensors).rangeMAG3110, (*Sensors).rangeMAG3110, useBuffer, 2);
  case MASK_MAG0_3 : return writeTab( (*Kalman).measureMAG3110_0, 3, -100, 100, useBuffer, 1);
  case MASK_MAG0_6 : return writeTab( (*Kalman).measureMAG3110_0, 3, -(*Sensors).rangeMAG3110, (*Sensors).rangeMAG3110, useBuffer, 2);
  }
}

void LOG::loop() {
  
  // Ecoute de l'ordinateur client
  if (!isListening && (millis()-lastListen > 2000)) {
    isListening = true;
    lastListen = millis();
  }
  if (isListening && (millis()-lastListen > 100)) {
    isListening = false;
    lastListen = millis();
  }
  if ((isListening) && (enableListen)) return;
  
  // Respect du nombre de paquets /sec
  if (millis()-lastPacket < 1000/24) return;
  lastPacket = millis();
  
  // *************************
  // Envoi du paquet principal
  for (uint8 i=0 ; i<MAIN_MASK_LENGTH ; i++)
    if (!writeData(maskMain[i], false)) break;
  
  // **************************
  // Envoi du paquet auxiliaire
  
  // On vérifie que le masque est valide
  if (maskMain[0] == 0) {
    for (uint8 i=0 ; i<lenAux ; i++) write((uint8)0);
    return;
  }
  
  uint8 len = 0;
  len += writeBuffer( min(nBuffer,lenAux) ); // On vide le buffer

  // S'il reste de la place dans le paquet auxiliaire, on remplit le buffer avec l'information suivante
  while (len < lenAux) {
    // Si on arrive en bout de masque, on recommence du début
    if ((writeData(maskMain[posAux], true)==0) || (posAux==AUX_MASK_LENGTH-1)) posAux = 0;
    else posAux++;
    // On vide le buffer, dans la limite de longueur du paquet
    len += writeBuffer( min(nBuffer,lenAux-len) );
  }

  // ***************  
  // Envoi de la clé
  writeData(MASK_KEY, false);

}

void LOG::printTab(const char *str, const float* data, uint8 m, uint8 n) {
  Serial.println();
  Serial.print(str);
  Serial.println(" :");
  for (uint8 i=0 ; i<m ; i++) {
    for (uint8 j=0 ; j<n ; j++) {
      Serial.print(data[i*n+j], 5);
      if (j<n-1) Serial.print("  ");
    }
    Serial.println();
    delay(50);
  }
}

