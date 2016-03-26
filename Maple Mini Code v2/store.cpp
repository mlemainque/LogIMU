// Gestion de la mémoire Flash
// Matthias Lemainque 2013

#include "store.h"

void FLASH::setup(boolean clear) {
  FLASH_BASE->ACR |= 1<<4;	//ACR bit 4, Prefetch enable.
  FLASH_BASE->ACR |= 1<<1;	//ACR bit 1, Set latency 010 Two wait states, if 48 MHz < SYSCLK ≤ 72 MHz
  if (clear) erase();
  read();
}

boolean FLASH::lock() {
  FLASH_BASE->CR = 0x80;			//Reset and lock.
  if (FLASH_BASE->CR & 1<<7) return false;	//Check if locked
  else return true;
}

boolean FLASH::unlock() {
  FLASH_BASE->KEYR = 0x45670123;		//The STM Key1
  FLASH_BASE->KEYR = 0xCDEF89AB;		//The STM Key2
  if (FLASH_BASE->CR & 1 << 7) return false;	//Check if unlocked
  else return true;
}

void FLASH::wait() { // Attend que la flash soit inutilisée
  while (FLASH_BASE->SR & (1<<0)) {  
  }
}

void FLASH::read() {
  byte* read_address;
  for (uint16 i=0 ; i<DATA_LENGTH ; i++) {
    wait();
    //Reading the data, combining 2 bytes into one half-word (16 bits). The odd bytes are most significant.
    read_address = (byte*)( FLASH_BASE_ADDRESS + STORE_PAGE*1024 + i*2 );	// Calculating first byte address.
    data[i]      = *read_address;						// Loading first byte to the array. 
    read_address = (byte*)( FLASH_BASE_ADDRESS + STORE_PAGE*1024 + i*2+1 );	// Reading second byte
    data[i]     += *read_address << 8;						// Combining 2 bytes to one 16 bit half-word.   
  }
}

boolean FLASH::erase() {
  if (!unlock()) return false;
  wait();

  // Efface la page
  FLASH_BASE->CR |= 1<<1;						//PER Single Page Erase bit selected.
  FLASH_BASE->AR  = ((int) FLASH_BASE_ADDRESS + STORE_PAGE*1024);	// The page address
  FLASH_BASE->CR |= 1<<6;						//Set the Erase Start bit

  // Vérifie que la page a été effacée
  uint16 test_data;
  byte* read_address;
  wait();
  for (uint16 i=0 ; i<1024 ; i++) {
    wait();
    read_address = (byte*)( FLASH_BASE_ADDRESS + STORE_PAGE*1024 + i );   
    test_data    = *read_address;
    if ((uint8)test_data != 0xFF) return 0*lock();
  }

  return lock();
}

boolean FLASH::write() {
  // Efface la page avant l'écriture
  if (!erase()) return false;

  // Ecrit la page
  if (!unlock()) return false;
  uint16* write_address;
  wait();
  FLASH_BASE->CR |= 1<<0;
  for (uint16 i=0 ; i<DATA_LENGTH ; i++) {
    write_address = (uint16*)( FLASH_BASE_ADDRESS + STORE_PAGE*1024 + i*2 );   
    if ((int)write_address<FLASH_LOWER_ADDRESS || (int)write_address>FLASH_UPPER_ADDRESS) return 0*lock();
    wait();
    *(uint16*)write_address = data[i]; // writing
  }
  return lock();
}

boolean FLASH::write(uint8 i, uint16 value) {
  if (i >= DATA_LENGTH) return false;
  data[i] = value;
  return write();
}

void FLASH::readT8(uint8 *tab, uint8 pos, uint8 len) {
  for (uint8 i=0 ; i<len ; i++) {
    tab[2*i]   = data[i+pos] >> 8;
    tab[2*i+1] = data[i+pos] % (1<<8);
  }
}

boolean FLASH::writeT8(const uint8 *tab, uint8 pos, uint8 len) {
  for (uint8 i=0 ; i<len ; i++) data[i+pos] = tab[2*i]<<8 | tab[2*i+1];
  return write();
}

void FLASH::readTf(float *tab, uint8 pos, uint8 len, float range) {
  for (uint8 i=0 ; i<len ; i++) tab[i] = itof( data[i+pos], -range, range, 16 );
}

boolean FLASH::writeTf(const float *tab, uint8 pos, uint8 len, float range) {
  for (uint8 i=0 ; i<len ; i++) data[i+pos] = ftoi( tab[i], -range, range, 16 );
  return write();
}



