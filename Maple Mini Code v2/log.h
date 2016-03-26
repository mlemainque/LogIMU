// Communication série des mesures
// Matthias Lemainque 2013

#ifndef _LOG_H_
#define _LOG_G_

#include "wirish.h"
#include "maths.h"
#include "sensors.h"
#include "kalman.h"
#include "SdFat.h"

// Paramètres
#define BAUD_RATE		19200

#define LISTEN_ENABLE		true
#define LISTEN_INTERVAL		2000

#define MAIN_MASK_LENGTH	10
#define AUX_MASK_LENGTH		20

// Constantes
// Masque : chaque information ne peut faire que 8 octets (limitation du buffer)
#define MASK_END		255
#define MASK_KEY		0	// 1 octet
#define MASK_DT			1	// 1 octet
#define MASK_QUAT_5		2	// 5 octets : 13 bits par coordonnées X,Y,Z et 1 bit de signe pour W (le quaternion est normé)
#define MASK_QUAT_8		3	// 8 octets : 21 bits     "     "     "     et 1 bit     "     "    "    "    "    "    "
#define MASK_TEMP		4	// 1 octet
#define MASK_PRESS		5	// 2 octets
#define MASK_ALTI		6	// 2 octets
#define MASK_ACC_6		7
#define MASK_ACC0_3		8
#define MASK_ACC0_6		9
#define MASK_MAG_6		10
#define MASK_MAG0_3		11
#define MASK_MAG0_6		12
#define MASK_GYRZ_6		13

class LOG {
private:
  SENSORS *Sensors;
  KALMAN *Kalman;
  HardwareSPI *Spi;

  uint8 nBuffer;
  uint64 Buffer; // Buffer de type FIFO
  uint8 posAux; // Position actuelle dans le masque auxiliaire
  
  uint8 write(const uint8 data);
  uint8 write(const uint64 data, boolean useBuffer, uint8 n=1);
  uint8 writeBuffer(uint8 n);
  uint8 writeTab(const float *data, uint8 len, float min, float max, boolean useBuffer, uint8 n=1);
  uint8 writeData(uint8 b, boolean useBuffer);
  
  uint16 nSync;
  Sd2Card Card;
  SdVolume Volume;
  SdFile Root;
  SdFile File;

  uint8 key;

  boolean enableListen;  
  boolean isListening;
  uint32 lastListen;
  uint32 lastPacket;
  uint32 lastDt;

public:
  LOG(SENSORS *newSensors, KALMAN *newKalman, HardwareSPI *newSpi);
  
  void setup();
  boolean setupSD();
  boolean newFile();
  void loop();
  
  void printTab(const char *str, const float* data, uint8 m, uint8 n);

  uint8 maskMain[MAIN_MASK_LENGTH];
  uint8 maskAux[AUX_MASK_LENGTH];
  uint8 lenAux;
  
};

#endif // _LOG_H_
