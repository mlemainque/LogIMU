// Interface avec la mémoire flash (mémoire permanente)
// Tiré du travail de Samtal : http://pastebin.com/h0CxGRQ7
// Matthias Lemainque 2012

#ifndef _STORE_H_
#define _STORE_H_

#include "stdlib.h"	//Included explicitly for proper data printing. 
#include "flash.h"	//Only contains 2 function (latency and prefetch) that are implemented here 

#include "maths.h"

#define STORE_PAGE		125		// Doit être compris entre 120 et 127
#define STORE_LENGTH_DATA	105 + 1		// Nombre de variables de 16 bits (entre 0 et 512)
#define FLASH_BASE_ADDRESS	0x08000000	// Début de la mémoire flash 134217728
#define FLASH_LOWER_ADDRESS	0x801E000	// Lower safety page limit for the demo Page 120
#define FLASH_UPPER_ADDRESS	0x0801FFFF	// Flash upper address. The upper page (127) limit

uint16 store_data[STORE_LENGTH_DATA];		// Tableau de valeurs à stocker

// 0   - 2   : GYR ZERO
// 3   - 5   : ACC ZERO
// 6   - 8   : MAG ZERO
// 9   - 50  : ACC SEED (pack de 3)
// 51  - 92  : MAC SEED (pack de 3)
// 93  - 95  : PARAMS BOOLEANS
// 96  - 103 : PARAMS CORRECTION

#define FLASH_POS_CONST_A	96
#define FLASH_POS_ERRNORM_A	97
#define FLASH_POS_ERRDIR_A	98
#define FLASH_POS_DIFFCST_A	99
#define FLASH_POS_CONST_M	100
#define FLASH_POS_ERRNORM_M	101
#define FLASH_POS_ERRDIR_M	102
#define FLASH_POS_DIFFCST_M	103
#define FLASH_POS_REFALT	104
#define FLASH_POS_REFPRESS	105

float flashToFloat(uint16 first, float min, float max) {
  return ((float) store_data[first]*(max-min)/62500 + min);
}

void floatToFlash(float value, uint16 first, float min, float max) {
  store_data[first] = ( value-min )/(max-min)*62500;
}

Vector flashToVect(uint16 first, float range) {
  Vector result;
  result.X = ((float) store_data[first+0]/31250 -1 ) * range;
  result.Y = ((float) store_data[first+1]/31250 -1 ) * range;
  result.Z = ((float) store_data[first+2]/31250 -1 ) * range;
  Log.print("load:");
  Log.println(result);
  return result;
}

void vectToFlash(Vector v, uint16 first, float range) {
  store_data[first+0] = ( v.X/range +1 ) * 31250;
  store_data[first+1] = ( v.Y/range +1 ) * 31250;
  store_data[first+2] = ( v.Z/range +1 ) * 31250;
  Log.print("store:");
  Log.println(v);
}


void flash_setup() {
  FLASH_BASE->ACR |= 1<<4;  //ACR bit 4, Prefetch enable.
  FLASH_BASE->ACR |= 1<<1;  //ACR bit 1, Set latency 010 Two wait states, if 48 MHz < SYSCLK ≤ 72 MHz
}


boolean flash_unlock() {
  FLASH_BASE->KEYR = 0x45670123;  //The STM Key1
  FLASH_BASE->KEYR = 0xCDEF89AB;  //The STM Key2
  if (FLASH_BASE->CR & 1 << 7) return false;  //Check if unlocked
  else return true;
}


boolean flash_lock() {
  FLASH_BASE->CR = 0x80;  //Reset and lock.
  if (FLASH_BASE->CR & 1<<7) return false;  //Check if locked
  else return true;
}


void flash_not_used() { // Attend que la flash soit inutilisée
  while (FLASH_BASE->SR & (1<<0)) {  
  }
}


void page_read() {
  byte* read_address;
  for (uint16 i=0 ; i<STORE_LENGTH_DATA ; i++) {
    flash_not_used();
    //Reading the data, combining 2 bytes into one half-word (16 bits). The odd bytes are most significant.
    read_address  = (byte*)( FLASH_BASE_ADDRESS + STORE_PAGE*1024 + i*2 );   // Calculating fist byte address.
    store_data[i] = *read_address;                                           // Loading first byte to the array. 
    read_address  = (byte*)( FLASH_BASE_ADDRESS + STORE_PAGE*1024 + i*2+1 ); // Reading second byte
    store_data[i] = store_data[i] + (*read_address << 8);                    // Combining 2 bytes to one 16 bit half-word.   
  }
}


boolean page_erase() {
  if (FLASH_BASE->CR & (1 << 7)) {
    Log.println("Error in flash_erase() : flash must be unlocked");
    return false;
  }

  flash_not_used();
  FLASH_BASE->CR |= 1<<1;                //PER Single Page Erase bit selected.
  FLASH_BASE->AR  = ((int) FLASH_BASE_ADDRESS + STORE_PAGE*1024);    // The page address
  FLASH_BASE->CR |= 1<<6;                         //Set the Erase Start bit

  uint16 test_data;
  byte* read_address;
  flash_not_used();
  for (uint16 i=0 ; i<1024 ; i++) {
    flash_not_used();
    read_address = (byte*)( FLASH_BASE_ADDRESS + STORE_PAGE*1024 + i );   
    test_data    = *read_address;
    if ((uint8)test_data != 0xFF) {
      Log.println("Error in flash_erase() : page not erased");
      flash_lock();
      return false;
    }
  }
  flash_lock();
  return true;
}


boolean page_write() {
  if (FLASH_BASE->CR & (1 << 7)) {
    Log.println("Error in flash_write() : flash must be unlocked");
    return false;  //Check if unlocked.
  }

  //Prior to writing, must read cells to check if all cells are erased (if all set to 'FF')
  byte test_data;
  boolean error=false;
  for (uint16 i=0 ; i<STORE_LENGTH_DATA ; i++) {
    flash_not_used();
    test_data = *((byte*)( FLASH_BASE_ADDRESS + STORE_PAGE*1024 + i*2 ));   
    if (test_data != 0xFF) {
      error=true;
      break;
    }
  }
  if (error) {
    Log.println("Error in flash_write() : page must be erased before writing");
    flash_lock();
    return false;
  }

  // Writing
  uint16* write_address;
  flash_not_used();
  FLASH_BASE->CR |= 1<<0;
  for (uint16 i=0 ; i<STORE_LENGTH_DATA ; i++) {
    write_address=(uint16*)( FLASH_BASE_ADDRESS + STORE_PAGE*1024 + i*2 );   
    if ((int)write_address<FLASH_LOWER_ADDRESS || (int)write_address>FLASH_UPPER_ADDRESS) {
      Log.println("Error in flash_write() : address out of allowed limits");
      flash_lock();
      return false;
    }
    flash_not_used();
    *(uint16*)write_address = store_data[i]; // writing
  }
  flash_lock();
  return true;
}

boolean page_write_quick() {
  flash_unlock();
  page_erase();
  flash_unlock();
  return page_write();
}

boolean page_clear() {
  for (uint16 i=0 ; i<STORE_LENGTH_DATA ; i++) store_data[i] = 0;
  return page_write_quick();
}


void writeParam(uint16 first, uint16 last, uint16 value, boolean doWrite=false) {
  if (last >= 3*16) return;
  for (uint8 i=0 ; i<last-first+1 ; i++) bitWrite(store_data[93+((i+first)/16)], (i+first)%16, bitRead(value, i));
  if (doWrite) page_write_quick();
}

uint16 readParam(uint16 first, uint16 last) {
  if (last >= 3*16) return 0;
  uint16 value = 0;
  for (uint8 i=0 ; i<last-first+1 ; i++) bitWrite(value, i, bitRead(store_data[93+((i+first)/16)], (i+first)%16));
  return value;
}

void nextParam(uint16 first, uint16 last, uint16 maxi) {
  if (last >= 3*16) return;
  uint16 param = readParam(first, last);
  param = (param+1)%(maxi+1);
  writeParam(first, last, param);
}

#endif // _STORE_H_

