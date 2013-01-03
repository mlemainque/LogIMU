// Matthias Lemainque 2012
// Extension du composant IMU permettant de calibrer les capteurs

#ifndef _IMUCALIB_H_
#define _IMUCALIB_H_

#include "itg3200.h"
#include "adxl345.h"
#include "mag3110.h"

#include "maths.h"
#include "state.h"
#include "imu.h"

// Calibrations
#define CalibX_Length		1024	// Nombre d'échantillons pour le calibrage (règle la durée de la calibration des accéléromètres, indirectement)

// Calibration gyromètres
#define CalibG_Duration		2500	// Durée de la calibration (ms)
#define CalibG_EcartType	0.2	// Ecart-type maximum accepté (deg.s-1)

// Calibration accéléro/magnétomètres
// Prise des mesures
#define CalibAM_Pas		32	// Nbres de mesures entre chaque calcul (règle la durée des mesures, indirectement)
#define CalibAM_AngleDiff	10	// Angle maximum avec l'orientation demandée
#define CalibA_EcartType	0.07	// Ecart-type maximum accepté (m.s-2)

// Algorithme de convergence
#define CalibAM_DivDiff		1.3	// Division du déplacement lorsqu'on ne trouve rien de mieux
#define CalibA_MinDiff		0.001	// Déplacement le plus faible autorisé (m.s-2)
#define CalibA_BaseDiff		0.1	// Déplacement initial (m.s-2)
#define CalibA_MaxDiff		3	// Déplacement le plus grand autorisé (m.s-2)
#define CalibM_MinDiff		1	// Déplacement le plus faible autorisé (uT)
#define CalibM_BaseDiff		10	// Déplacement initial (uT)
#define CalibM_MaxDiff		100	// Déplacement le plus grand autorisé (uT)<

class IMUCALIB :
public IMU {
public:
  IMUCALIB(boolean corA=true, boolean corM=true, boolean en=true);

  // Calibration
  Vector vectCalibX[CalibX_Length];
  uint16 iVectCalibX;
  uint16 jVectCalibX;

  void beginCalibG();
  void procCalibG();
  void calcCalibG();

  float ecartTypeAG;
  float ecartTypeM;

  Vector vectCalibA[14];
  Vector vectCalibM[14];
  uint8 iVectCalibAM;
  uint16 jVectCalibM;

  Vector dirAExcepted;

  void beginCalibAM();
  void procCalibAM();
  void calcCalibAM();
  void convergCalib(Vector vectCalib[], Vector* zero, float* e, float goalNorm, float minDiff, float baseDiff, float maxDiff);
  void seedCalibAM();
};


// Initialisation configuration
IMUCALIB::IMUCALIB(boolean corA, boolean corM, boolean en) {
  this->enabled = en;
  this->correctA = corA;
  this->correctM = corM;
}


// ****************************
//  Calibration des gyromètres

void IMUCALIB::beginCalibG() {
  changeState(IsCalibG);
  this->iVectCalibX = 0;
  this->ecartTypeAG = 0;
  gyr.zero = newVector(0,0,0);
  process(500);
}

void IMUCALIB::procCalibG() {
  dTime(false);
  if (dt*1000>=CalibG_Duration/CalibX_Length) {
    dTime(true);
    this->vectCalibX[this->iVectCalibX] = gyr.measure;
    this->iVectCalibX ++;
    if (this->iVectCalibX>=CalibX_Length) this->calcCalibG();
  }
}

void IMUCALIB::calcCalibG() {
  // Calcul de la moyenne puis de l'écart-type des mesures
  Vector zero = newVector(0,0,0);
  for (uint16 i=0 ; i<CalibX_Length ; i++) zero = Comb2( zero, i/((float)i+1), this->vectCalibX[i], 1/((float)i+1) );
  Vector e = newVector(0,0,0);
  for (uint16 i=0 ; i<CalibX_Length ; i++) e = Comb2( e, i/((float)i+1), sqV(Comb2(this->vectCalibX[i],1,zero,-1)), 1/((float)i+1) );
  this->ecartTypeAG = maxV(sqrtV(e));

  // Si l'écart-type est assez faible, on valide la calibration
  if (this->ecartTypeAG<=CalibG_EcartType) {
    changeState(IsCalibG, IsCalibG_MeasureOk, 500);	// Avertit de la validation
    gyr.zero = zero;					// Enregistre
    vectToFlash(zero, 0, ITG3200_FlashRange);		// Sauvegarde dans la mémoire flash
    page_write_quick();					// Ecrit dans la mémoire flash
    lunchIMUs();					// Lance la centrale
  }
  else {
    changeState(IsCalibG, IsCalibG_MeasureWrong, 1000);
    changeState(IsCalibG);
    this->iVectCalibX = 0;
  }
}


// **********************************
// Calibration accéléro/magnétomèrtes

// Principe :	La centrale centrale doit être orientée dans la direction demandée
//		Une fois que c'est le cas, et que la centrale semble au repos, une prise de mesure des accélérations et du champ magnétique est effectuée et sauvegardée
//		14 mesures de ce type sont effectuées, de façon à couvrir toutes les directions possibles
//		Un algorithme de convergence se charge alors de calculer le zéro accélérométrique et magnétométrique, en recherchant les valeurs de ces zéros pour lesquelles
//		toutes les mesures effectuées ont la même norme

void IMUCALIB::beginCalibAM() {
  changeState(IsCalibAM);
  this->iVectCalibX	= 0;
  this->jVectCalibX	= 9999;
  this->iVectCalibAM	= 0;
  this->dirAExcepted	= newVector(0,0,1);
  this->ecartTypeAG	= 0;
  acc.zero		= newVector(0,0,0);
  mag.zero		= newVector(0,0,0);
}

// On ne mesure l'accélération que lorsqu'elle est stable et dans la direction désirée
// Si c'est le cas, on mesure également une moyenne du champ magnétique
void IMUCALIB::procCalibAM() {
  this->iVectCalibX ++;
  if (this->iVectCalibX>=CalibX_Length) this->iVectCalibX = 0;
  this->vectCalibX[this->iVectCalibX] = acc.measure;
  this->vectCalibM[this->iVectCalibAM] = Comb2( this->vectCalibM[this->iVectCalibAM], this->jVectCalibM/((float)this->jVectCalibM+1), mag.measure, 1/((float)this->jVectCalibM+1) );
  this->jVectCalibM ++;
  if (this->iVectCalibX%CalibAM_Pas!=0) return;

  // On calcule le vecteur accélération moyen mesuré et l'écart-type
  Vector mVect = newVector(0,0,0);
  for (uint16 i=0 ; i<CalibX_Length ; i++) mVect = Comb2( mVect, i/((float)i+1), this->vectCalibX[i], 1/((float)i+1) );
  Vector eVect = newVector(0,0,0);
  for (uint16 i=0 ; i<CalibX_Length ; i++) eVect = Comb2( eVect, i/((float)i+1), sqV( Comb2(this->vectCalibX[i], 1, mVect, -1) ), 1/((float)i+1) );
  this->ecartTypeAG = maxV(sqrtV(eVect));
  if (Norm(mVect) == 0) return;

  // On calcule l'angle entre le vecteur accélération moyen mesuré et le vecteur désiré
  float angle = acos( PrdS(this->dirAExcepted,mVect) / Norm(mVect) );
  if (angle*CDR<=CalibAM_AngleDiff && this->ecartTypeAG<=CalibA_EcartType) {	// La mesure est valide
    if (this->jVectCalibX==9999) {						// C'est la première mesure valide
      this->jVectCalibX = this->iVectCalibX;					// on se rappelle de la position actuelle et on refait un tour
      this->jVectCalibM = 0;							// on relance la mesure du champ magnétique moyen
    }
    else if (this->jVectCalibX==this->iVectCalibX) {				// Un tour complet a été refait sans être interrompu
      Log.write(255);								// Accusé de fin de mesure (un bip est émis sur le PC)
      Log.write(2);
      changeState(IsCalibAM, IsCalibAM_MeasureOk, 1000);
      this->vectCalibA[this->iVectCalibAM] = mVect;				// on enregistre la mesure
      this->iVectCalibAM++;							// on passe à la mesure suivante
      if(iVectCalibAM>13) this->calcCalibAM();					// Toutes les mesures ont été faites, on peut lancer l'algo de convergence
      else {									// Sinon, on tourne d'un quart de tour et 1 fois sur 4 on monte d'un huitième de tour
        this->dirAExcepted = RotateVect(this->dirAExcepted, newVector(0,0,1), 0, 1);
        if(iVectCalibAM%4==1) this->dirAExcepted = RotateVect(this->dirAExcepted, newVector(1,0,0), cosPI4, sinPI4);
        changeState(IsCalibAM, IsCalibAM_Measure);
      }
      this->jVectCalibX = 9999;
    }
  }
  else {																	// Mesure non valide
    this->errDirA = acos( PrdS(this->dirAExcepted,this->vectCalibX[this->iVectCalibX]) / Norm(this->vectCalibX[this->iVectCalibX]) ) * CDR;	// On calcule l'angle entre la direction désirée et la direction actuelle
    this->jVectCalibX = 9999;															// On avorte toute mesure en cours
    changeState(IsCalibAM, IsCalibAM_Measure);
  }
}

// Traitement des données receuillies
void IMUCALIB::calcCalibAM() {
  changeState(IsCalibAM, IsCalibAM_Converg);
  for (int i=0 ; i<14 ; i++) {
    vectToFlash(this->vectCalibA[i], (3+i)*3, 2*9.81);
    vectToFlash(this->vectCalibM[i], (17+i)*3, 1000);
  }
  page_write_quick();
  this->convergCalib(this->vectCalibA, &(acc.zero), &(this->ecartTypeAG), Norm(Gravitee), CalibA_MinDiff, CalibA_BaseDiff, CalibA_MaxDiff);
  this->convergCalib(this->vectCalibM, &(mag.zero), &(this->ecartTypeM),  Norm(NordMagn), CalibM_MinDiff, CalibM_BaseDiff, CalibM_MaxDiff);
  changeState(IsCalibAM, IsCalibAM_ConvergOk);
  pause();
  vectToFlash(acc.zero, 3, ADXL345_FlashRange);
  vectToFlash(mag.zero, 6, MAG3110_FlashRange);
  page_write_quick();
  changeState(IsOff);
}

// Algorithme de convergence (vectCalib doit être de dimension 14)
void IMUCALIB::convergCalib(Vector vectCalib[], Vector* zero, float* eAvg, float goalNorm, float minDiff, float baseDiff, float maxDiff) {
  *zero = newVector(0,0,0);
  Vector newZero = *zero;
  float diff = baseDiff;
  *eAvg = 9999;
  float eGoal = 9999;
  boolean reset = false;
  while (true) { // La sortie de la boucle devrait se faire naturellement
    // Recherche du gradient d'erreur dans toutes les directions
    boolean findBetter = false;
    for (int8 x=-1 ; x<=1 ; x++)
      for (int8 y=-1 ; y<=1 ; y++)
        for (int8 z=-1 ; z<=1 ; z++)
          if (x!=0 || y!=0 || z!=0) {
            float newEavg = 0;	// Ecart par rapport à la norme moyenne
            // Si on se réfère à elle pour le gradient, elle risque de faire diverger l'algo vers une solution à l'infini
            // Toutefois, celle-ci donne une bonne idée de la cohérence des mesures, c'est donc elle qu'on affiche
            float newEgoal = 0;	// Ecart par rapport à la norme demandée en paramètres
            // On se réfère plutôt à elle pour le gradient
            Vector vDiff = newVector(x*diff,y*diff,z*diff);
            float avgNorm;
            for (uint16 i=0 ; i<14 ; i++ ) {	// Calcul de la norme moyenne
              Vector tmp = Comb3(vectCalib[i], 1, *zero, -1, vDiff, -1);
              avgNorm  = ( avgNorm*i  + Norm(tmp) ) / ((float)i+1);
            }
            for (uint16 i=0 ; i<14 ; i++ ) {	// Calcul des deux écart-types
              Vector tmp = Comb3(vectCalib[i], 1, *zero, -1, vDiff, -1);
              newEavg  = ( newEavg*i  + sq(Norm(tmp)-avgNorm)  ) / ((float)i+1);
              newEgoal = ( newEgoal*i + sq(Norm(tmp)-goalNorm) ) / ((float)i+1);
            }
            newEgoal = sqrt( newEgoal ) / goalNorm;
            newEavg  = sqrt( newEavg  ) / avgNorm;
            if (newEgoal<eGoal) {	// On se réfère à la norme demandée et non pas la moyenne
              eGoal = newEgoal;
              *eAvg = newEavg;		// Mais on renvoit l'écart à la norme moyenne, ce qui est plus représentatif
              newZero = Add2(*zero, vDiff);
              findBetter = true;
              XtoggleLED();
            }
          }
    *zero = newZero;
    refreshAff();

    // Si on ne trouve rien de mieux à la distance où l'on cherche, on affine la recherche
    if (!findBetter) {
      diff /= CalibAM_DivDiff;
      if (diff<minDiff) { // Rien de mieux très proche, on essaie très loin
        if (reset) break; // Si on s'est déjà éloigné une fois sans rien trouver de mieux, on abandonne
        diff = maxDiff;
        reset = true;
      }
    } 
    else reset = false;
  };
  process();
}

void IMUCALIB::seedCalibAM() {
  this->beginCalibAM();
  page_read();
  for (int i=0 ; i<14 ; i++) {
    this->vectCalibA[i] = flashToVect((3+i)*3, 2*9.81);
    this->vectCalibM[i] = flashToVect((17+i)*3, 1000);
  }
  this->iVectCalibAM = 14;
  this->calcCalibAM();
}


#endif // _IMUCALIB_H_

