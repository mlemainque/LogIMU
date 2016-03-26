// Gestion de la mémoire Flash
// Matthias Lemainque 2013

#ifndef _STORE_H_
#define _STORE_H_

#include "wirish.h"
#include "flash.h"
#include "maths.h"

// Paramètres
#define STORE_PAGE		125		// Doit être compris entre 120 et 127
#define DATA_LENGTH		24		// Nombre de variables de 16 bits (entre 0 et 512)

// Constantes
// Addresses paramètres
#define FLASH_ZERO_ADXL		0
#define FLASH_ZERO_MAG		3
#define FLASH_PARAM		6
#define FLASH_MAIN_MASK		7
#define FLASH_AUX_MASK		7

// Flash
#define FLASH_BASE_ADDRESS	0x08000000	// Début de la mémoire flash 134217728
#define FLASH_LOWER_ADDRESS	0x801E000	// Lower safety page limit for the demo Page 120
#define FLASH_UPPER_ADDRESS	0x0801FFFF	// Flash upper address. The upper page (127) limit

class FLASH {
private:
  boolean lock();
  boolean unlock();
  void wait();

public:
  uint16 data[DATA_LENGTH];
  
  void setup(boolean clear = false);
  void read();
  boolean erase();
  boolean write();
  boolean write(uint8 i, uint16 value);

  void readT8(uint8 *tab, uint8 pos, uint8 len);
  boolean writeT8(const uint8 *tab, uint8 pos, uint8 len);
  
  void readTf(float *tab, uint8 pos, uint8 len, float range);
  boolean writeTf(const float *tab, uint8 pos, uint8 len, float range);

};

#endif // _STORE_H_
