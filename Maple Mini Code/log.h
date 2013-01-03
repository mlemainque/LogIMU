// Log Serie et SD
// Matthias Lemainque 2012

#ifndef _LOG_H_
#define _LOG_H_

#define PIN_APC220_EN	20

#include "maths.h"
#include "Print.h"
#include "string.h"
#include "wirish.h"
#include "nvic.h"

#include "pcd8544.h"



// Log SD

#define LOG_TYPE_OFF	0
#define LOG_TYPE_BIN	1
#define LOG_TYPE_CSV	2

byte logType = LOG_TYPE_BIN;

Sd2Card card;
SdVolume volume;
SdFile root;
SdFile file;

uint16 nSync = 0;

void setupSD() {
  if (logType == LOG_TYPE_OFF) return;
  if (!card.init(&spi)) return;
  delay(100);
  if (!volume.init(&card)) return;
  if (!root.openRoot(&volume)) return;
  char buffer[10] = "LOG00.BIN";
  if (logType == LOG_TYPE_CSV) {
    buffer[6] = 'C';
    buffer[7] = 'S';
    buffer[8] = 'V';
  }
  if (file.isOpen()) file.close();
  for (byte i=0; i<100; i++) {
    buffer[3] = '0' + i/10;
    buffer[4] = '0' + i%10;
    if (file.open(&root,buffer,O_CREAT|O_EXCL|O_WRITE)) break;
  }
}




// Log Serie

class XPrint :
public Print {
public:
  void print(Vector v);
  void println(Vector v);
  void writefab(float, float, float);
  void writevab(Vector, float, float);
  void write16(uint16);
  void writefa16(float, float, float);
  void writeva16(Vector, float, float);
  using Print::print;
  using Print::println;
};

void XPrint::print(Vector v) {
  this->print(v.X);
  this->print(";");
  this->print(v.Y);
  this->print(";");
  this->print(v.Z);
}

void XPrint::println(Vector v) {
  this->print(v);
  this->println();
}

void XPrint::writefab(float value, float min, float max) {
  byte b = (byte)((value-min)*(250/(max-min))+.5); // 0 ... 250 !
  this->write(b & 0xFF);
}

void XPrint::writevab(Vector v, float min, float max) {
  this->writefab(v.X, min, max);
  this->writefab(v.Y, min, max);
  this->writefab(v.Z, min, max);
}

void XPrint::writefa16(float value, float min, float max) {
  this->write16((uint16)((value-min)*(250*250/(max-min))));
}

void XPrint::writeva16(Vector v, float min, float max) {
  this->writefa16(v.X, min, max);
  this->writefa16(v.Y, min, max);
  this->writefa16(v.Z, min, max);
}

void XPrint::write16(uint16 value) { // max = 251²-1 = 63000
  byte b1 = value/250;
  byte b2 = value - b1*250;
  this->write(b1);
  this->write(b2);
  //this->write((value >> 8) & 0xFF);
  //this->write(value & 0xFF);
}




#define tsDisabled	0
#define ts1		1
#define ts2		2
#define ts3		3
#define tsUSB		4

class XSerial : 
public XPrint {
public:
  XSerial(byte);
  byte type;
  boolean printInSD;
  boolean printInLog;
  void begin(byte, unsigned int=9600);
  uint32 available(void);
  uint32 available16(void);
  uint8 read(void);
  uint16 read16(void);
  void write(uint8);
  void write(const char *str);
  void write(const void*, uint32);
};

XSerial::XSerial(byte type) {
  this->type = type;
  this->printInSD = false;
  this->printInLog = true;
}

void XSerial::begin(byte type, unsigned int baud) {
  this->type = type;
  if ((this->type!=tsUSB) && (!simulIMU)) { // Désactive complètement le port usb lorsqu'il n'est pas utilisé, sous peine de ralentissements importants
    SerialUSB.end();
    nvic_irq_disable(NVIC_USB_LP_CAN_RX0);
  }
  else {
    nvic_irq_enable(NVIC_USB_LP_CAN_RX0);
    SerialUSB.begin();
  }
  digitalWrite(PIN_APC220_EN, (this->type!=tsUSB) && (this->type!=tsDisabled));
  switch(this->type) {
  case tsDisabled:
    return;
  case tsUSB:
    return;
  case ts1:
    return Serial1.begin(baud);
  case ts2:
    return Serial2.begin(baud);
  case ts3:
    return Serial3.begin(baud);
  }
}

uint32 XSerial::available(void) { // ! SerialUSB.available() bugue et ne renvoit que 0 ou 1 ....
  switch(this->type) {
  case tsDisabled:
    return 0;
  case tsUSB:
    return SerialUSB.available();
  case ts1:
    return Serial1.available();
  case ts2:
    return Serial2.available();
  case ts3:
    return Serial3.available();
  }
}

uint8 XSerial::read(void) {
  switch(this->type) {
  case tsDisabled:
    return 0;
  case tsUSB:
    return SerialUSB.read();
  case ts1:
    return Serial1.read();
  case ts2:
    return Serial2.read();
  case ts3:
    return Serial3.read();
  }
}

uint16 XSerial::read16(void) { // ! SerialUSB.available() bugue et ne renvoit que 0 ou 1 ....
  while (!this->available()) { }
  uint16 b1 = this->read();
  while (!this->available()) { }
  return (b1*250) + this->read();
}

void XSerial::write(uint8 ch) {
  const uint8 buf[] = {
    ch                              };
  this->write(buf, 1);
}

void XSerial::write(const char *str) {
  this->write(str, strlen(str));
}

void XSerial::write(const void *buf, uint32 len) {
  if (this->printInSD && file.isOpen()) {
    if (nSync+len >= 512) {
      file.sync();
      nSync = 0;
    }
    file.write(buf, len);
    nSync += len;
  }
  if (this->printInLog) {
    switch(this->type) {
    case tsDisabled:
      return;
    case tsUSB:
      return SerialUSB.write(buf, len);
    case ts1:
      return Serial1.write(buf, len);
    case ts2:
      return Serial2.write(buf, len);
    case ts3:
      return Serial3.write(buf, len);
    }
  }
}

XSerial Log(tsUSB);





#endif // _lOG_H_








