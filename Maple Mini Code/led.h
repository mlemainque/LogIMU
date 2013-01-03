// Matthias Lemainque 2012
// Interface avec une LED RGB

#ifndef _LED_H_
#define _LED_H_

typedef struct {
  float R;
  float G;
  float B;
}
RGB;

RGB ledNONE		= { 
  0, 0, 0 };
RGB ledRED		= { 
  1, 0, 0 };
RGB ledGREEN		= { 
  0, 1, 0 };
RGB ledBLUE		= { 
  0, 0, 1 };
RGB ledORANGE		= { 
  1, 1, 0 };
RGB ledPURPLE		= { 
  1, 0, 1 };
RGB ledTURQUOISE	= { 
  0, 1, 1 };
RGB ledWHITE		= { 
  1, 1, 1 };

boolean idRGB(RGB c1, RGB c2) {
  if (c1.R!=c2.R) return false;
  if (c1.G!=c2.G) return false;
  if (c1.B!=c2.B) return false;
  return true;
}


boolean isPWM(uint8 pin) {
  for (uint8 i=0 ; i<BOARD_NR_PWM_PINS ; i++) if(boardPWMPins[i]==pin) return true;
  return false;    
}

class LED {
public:
  LED();
  LED(uint8 Pin);
  uint8 Pin;
  float lastIntensity;
  float intensity;
  uint16 blink_low;
  uint16 blink_high;
  void setIntensity(float intensity, uint16 blink_low=0, uint16 blink_high=50, boolean reset=false);
  void refresh();
private:
  void init();
  boolean initialized;
  boolean pwm;
  boolean state;
  uint32 lastTime;
};

LED::LED() {
}

LED::LED(uint8 Pin) {
  this->Pin = Pin;
}

void LED::init() {
  this->intensity  = 0;
  this->lastIntensity = 0;
  this->blink_low  = 0;
  this->blink_high = 50;
  this->state = HIGH;
  this->pwm = isPWM(this->Pin);
  this->initialized = true;
  if(this->pwm) pinMode(this->Pin, PWM);
  else pinMode(this->Pin, OUTPUT);
}

void LED::setIntensity(float intensity, uint16 blink_low, uint16 blink_high, boolean reset) {
  if (this->Pin==0) return;
  if (!this->initialized) this->init();
  if (intensity!=this->intensity) reset = true;
  if (blink_low!=this->blink_low) reset = true;
  if (blink_high!=this->blink_high) reset = true;
  if (reset) {
    this->intensity  = intensity;
    this->blink_low  = blink_low;
    this->blink_high = blink_high;
    this->lastTime = millis();
    this->state = HIGH;
  }
  this->refresh();
}

void LED::refresh() {
  if (this->Pin==0) return;
  boolean lastState = this->state;
  if (this->blink_low==0) this->state = HIGH;
  else if ((this->state==HIGH && millis()-this->lastTime>=this->blink_high) || (this->state==LOW && millis()-this->lastTime>=this->blink_low)) {
    this->lastTime = millis();
    this->state = !this->state;
  }
  if ((this->state != lastState) || (this->lastIntensity != this->intensity)) {
    if (!this->pwm) digitalWrite(this->Pin, (this->state && (this->intensity>=.25)));
    else if (this->state==LOW) pwmWrite(this->Pin, 0);
    else pwmWrite(this->Pin, this->intensity*65535);
  }
  this->lastIntensity = this->intensity;
}



class LED_RGB {
public:
  LED_RGB();
  LED_RGB(uint8 PinR, uint8 PinG, uint8 PinB);
  RGB color;
  void setColor(RGB color, uint16 blink_low=0, uint16 blink_high=50, boolean reset=false);
  void refresh();
  LED ledR;
  LED ledG;
  LED ledB;
};

LED_RGB::LED_RGB() {
  this->color = ledNONE;
}

LED_RGB::LED_RGB(uint8 PinR, uint8 PinG, uint8 PinB) {
  this->color = ledNONE;
  this->ledR.Pin = PinR;
  this->ledG.Pin = PinG;
  this->ledB.Pin = PinB;
}

void LED_RGB::setColor(RGB color, uint16 blink_low, uint16 blink_high, boolean reset) {
  if (!idRGB(color,this->color)) reset = true;
  this->color = color;
  this->ledR.setIntensity(color.R, blink_low, blink_high, reset);
  this->ledG.setIntensity(color.G, blink_low, blink_high, reset);
  this->ledB.setIntensity(color.B, blink_low, blink_high, reset);
  this->refresh();
}

void LED_RGB::refresh() {
  this->ledR.refresh();
  this->ledG.refresh();
  this->ledB.refresh();
}




#define PIN_EXT_LED 18

void XtoggleLED() {
  toggleLED();
  togglePin(PIN_EXT_LED);
}

#endif // _LED_H_


