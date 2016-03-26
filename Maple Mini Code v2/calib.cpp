// Algorithme de calibration des capteurs ADXL345 & MAG3110
// Matthias Lemainque 2013

#include "calib.h"



CALIB::CALIB(SENSORS *newSensors, FLASH *newFlash) {
  Flash = newFlash;
  Sensors = newSensors;

  pos_calib = 0;
  first_measure = 0;
  num_measure = 0;
  exptectedDirADXL345 = { 0,0,1 };

  calc_step = 0;
  zeroADXL345 = (*Sensors).zeroADXL345;
  zeroMAG3110 = (*Sensors).zeroMAG3110;
}

void CALIB::loopCalcul() {
  
  // Si le pas est nul, on doit initialiser
  if (calc_step == 0) {
    if (state == CALIB_CALCUL_ADXL) {
      calc_step = 0.5;
      calc_step_min = 0.02;
      calc_norm_goal = 9.81;
      zeroADXL345 = calc_zero;
    }
    else {
      calc_step = 200;
      calc_step_min = 0.2;
      calc_norm_goal = 10;
      zeroMAG3110 = calc_zero;
    }
    FillA(calc_zero, 3, 0);
  }

  boolean findBetter = false;
  float measure_tmp[3];
  float diff[3];
  float ET_2;

  for (diff[0]=-1 ; diff[0]<=1 ; diff[0]++) {
    for (diff[1]=-1 ; diff[1]<=1 ; diff[1]++) {
      for (diff[2]=-1 ; diff[2]<=1 ; diff[2]++) {
        if (Norm2V(diff) > 0) {

          // On calcule le nouveau zéro
          AddA(calc_zero, 1, diff, 1, 3);

          ET_2 = 0;
          for (uint16 i=0 ; i<14 ; i++) {
            // On recalcule les mesures en utilisant le nouveau zéro
            if (state == CALIB_CALCUL_ADXL) CopyA(measure_tmp, measureADXL345[i], 3);
            else CopyA(measure_tmp, measureMAG3110[i], 3);
            AddA(measure_tmp, 1, calc_zero, -1, 3);

            // On calcule l'écart-type de l'erreur relative en norme
            ET_2 = ( ET_2*i + sq( NormV(measure_tmp)/calc_norm_goal-1 ) ) / ((float)i+1);
          }

          // Si on a fait strictement mieux qu'avant, on s'en souvient ; sinon, on revient à l'ancien zéro
          if ((ET_2 < calc_ET_2) || (calc_ET_2 == 0)) {
            calc_ET_2 = ET_2;
            findBetter = true;
          }
          else AddA(calc_zero, 1, diff, -1, 3);

        }
      }
    }
  }

  // Si on a pas trouvé mieux, on affine le pas
  if (!findBetter) calc_step /= 3;

  // On a atteint le pas minimum
  if (calc_step < calc_step_min) {

    // On enregistre le nouveau zéro    
    if (state == CALIB_CALCUL_ADXL) {
      CopyA(calc_zero, (*Sensors).zeroADXL345, 3);
      (*Flash).writeTf(calc_zero, FLASH_ZERO_ADXL, 3, (*Sensors).rangeADXL345);
      zeroADXL345 = (*Sensors).zeroADXL345;
    }
    else {
      CopyA(calc_zero, (*Sensors).zeroMAG3110, 3);
      (*Flash).writeTf(calc_zero, FLASH_ZERO_MAG, 3, (*Sensors).rangeMAG3110);
      zeroMAG3110 = (*Sensors).zeroMAG3110;
    }

    // On passe à l'étape suivante (éventuellement l'arrêt de la calibration)
    state ++;
    calc_step = 0;
    if (state > CALIB_CALCUL_MAG) {
      state = 0;
    }
      

  }
}

void CALIB::loopMeasure() {
  // On s'impose une fréquence maximale de mesure
  if (millis()-lastMeasure < DURATION_CALIB / NB_CALIB) return;
  lastMeasure = millis();

  // On enregistre en continu les mesures accélérométriques dans un tableau
  CopyA(measureADXL345_tmp[pos_calib], (*Sensors).measureADXL345, 3);
  pos_calib = (pos_calib+1) % NB_CALIB;
  
  // On a accès à l'angle d'erreur par rapport à la direction demandée
  float COS_2_ANGLE = sq(PrdSV((*Sensors).measureADXL345, exptectedDirADXL345)) / Norm2V((*Sensors).measureADXL345);

  // On a accès à l'écart-type des mesures stockées
  float ET_2 = 0;
  for (uint16 i=0 ; i<NB_CALIB ; i++) ET_2 = ( ET_2*i + Norm2V(measureADXL345_tmp[i]) ) / (i+1);
  ET_2 /= sq( (*Sensors).rangeADXL345 );

  // On attend que l'on descende en dessous d'un seuil d'erreur pour commencer la mesure
  if (state == CALIB_WAIT) {
    if ( (ET_2 <= sq(ET_MAX_ACT)) && (COS_2_ANGLE <= sq(DLcos(ANGLE_MAX_ACT/CDR))) ) {
      CopyA(measureADXL345[num_measure], (*Sensors).measureADXL345, 3);
      CopyA(measureMAG3110[num_measure], (*Sensors).measureMAG3110, 3);
      first_measure = pos_calib;
      state ++;
    }
  }
  
  // On mesure à la fois l'accélération et le champ magnétique, sous réserve que l'on ne redépasse pas un seuil d'erreur
  else if (state == CALIB_MEASURE) {
    if ( (COS_2_ANGLE > sq(DLcos(ANGLE_MIN_DESACT))) || (ET_2 > sq(ET_MIN_DESACT)) ) state --;
    AddA(measureADXL345[num_measure], pos_calib/((float)pos_calib+1), (*Sensors).measureADXL345, 1/((float)pos_calib+1), 3);
    AddA(measureMAG3110[num_measure], pos_calib/((float)pos_calib+1), (*Sensors).measureMAG3110, 1/((float)pos_calib+1), 3);
  
    if (pos_calib == first_measure) {
      // On passe à la mesure suivante, il y en a 14
      if (num_measure < 14) {
        RotV(exptectedDirADXL345, 0,0,1 , 0, 1);
        if (num_measure%4 == 1) RotV(exptectedDirADXL345, 1,0,0 , COSPI4, SINPI4);
        num_measure ++;
      }
      // Après la dernière mesure, on passe au calcul des zéros
      else {
        state ++;
        exptectedDirADXL345 = { 0,0,1 };
      }
    }
  }
}


void CALIB::loop() {
  (*Sensors).enableZeros = (state == CALIB_OFF);
  if (state == CALIB_OFF) return;

  if ((state==CALIB_CALCUL_ADXL) || (state==CALIB_CALCUL_MAG)) loopCalcul();
  else loopMeasure();
}



