// Algorithme de calibration des capteurs ADXL345 & MAG3110
// Matthias Lemainque 2013

#ifndef _CALIB_H_
#define _CALIB_H_

#include "wirish.h"
#include "maths.h"
#include "sensors.h"
#include "store.h"

// Paramètres
#define NB_CALIB		50
#define DURATION_CALIB		2000

#define ANGLE_MIN_DESACT	8	// angle en dessous duquel on passe à la prise de mesure
#define ET_MIN_DESACT		1	// écart-type (en %)    "     "     "     "     "
#define ANGLE_MAX_ACT		10	// angle au dessus duquel on abandonne la prise de mesure
#define ET_MAX_ACT		2	// écart-type (en %)    "     "     "     "     "

// Constantes
#define CALIB_OFF		0
#define CALIB_WAIT		1
#define CALIB_MEASURE		2
#define CALIB_CALCUL_ADXL	3
#define CALIB_CALCUL_MAG	4

class CALIB {
private:
  SENSORS *Sensors;
  FLASH *Flash;
  
  float exptectedDirADXL345[3];
  
  float measureADXL345_tmp[NB_CALIB][3];
  uint32 lastMeasure;
  uint16 pos_calib, first_measure;
  
  float calc_zero[3];
  float calc_norm_goal;
  float calc_step, calc_step_min;
  float calc_ET_2;
  boolean calc_reset;
  void loopCalcul();
  
  float measureADXL345[14][3];
  float measureMAG3110[14][3];
  void loopMeasure();
  
public:
  CALIB(SENSORS *newSensors, FLASH *newFlash);
  
  void loop();
  
  uint8 state;
  uint8 num_measure;

  float *zeroADXL345;
  float *zeroMAG3110;
  
};

#endif // _CALIB_H_
